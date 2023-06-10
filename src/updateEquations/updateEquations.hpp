/** 
* @file updateEquations.hpp 
* @brief This header deals with the high-level simulation producing procedure. 
* 
* @author Yochai Weissman 
* 
* @date 20/5/2023
*/

#ifndef UPDATE_EQUATIONS
#define UPDATE_EQUATIONS


#include <string>
#include <vector>
#include <iostream>
#include <cmath>
#include <array>

#include <armadillo>

#include "../config.hpp"
#include "../dynamics/framesConversion/framesConversion.hpp"
#include "../dynamics/forcesAndMoments/forcesAndMoments.hpp"
#include "../sensors/sensors.hpp"
#include "../visualization/visualization.hpp"
#include "../interface/interface.hpp"



/** 
* This method will update the drone state based on the previous state. 
* @param quat_before(arma::vec4) - quatenion for the previous time.
* @param position_before(arma::vec3) - position for the previos time for world frame.
* @param velocity_before(arma::vec3) - position for the previos time for world frame.
* @param angular_velocty_before(arma::vec3) - body angular rates.
* @param rotors_velocity(arma::vec4) - the rottors speed command.
* @return std::array<float, 13>, which include the updated: quaternion, position, velocity, angular velocity.
*/
std::array<float, 13> singleTimeUpdate(arma::vec4 quat_before, arma::vec3 position_before, arma::vec3 velocty_before , arma::vec3 angular_velocty_before, arma::vec4 rotors_velocity);


/** 
* This method will accumulates the full simulation data. 
* @param T(float).
* @param rottors_speed_initial(arma::vec4) - the initial rottors speed for the simulation.
* @param controller(std::function<arma::vec4(arma::vec3 referance,
std::array<float,13> current_step,
std::array<float,13> previous_step,
std::array<std::array<float,3>,6> controllers_coefficients,
float * altitude_error_integral)>)
 a function gets states(current and previos), referance and controllers coefficiets at some time and relevant state error integral and produces a control output(rottors_speeds).
* @param referanc(arma::vec3) - referance position.
* @param controllers_coefficients(std::array<std::array<float,3>,6>) - PID coefficients for all 6 controllers, default to all 0.
* @param tuning_controller(int) - detrmine if the call is a part of a automatic controller tuning.
* @return std::vector<std::array<float,13>> represent 13 states(std::vector<float,13>) for each poit in time.
*/
std::vector<std::array<float,13>> droneSimulation(float T, arma::vec4 rottors_speed_initial, std::function<arma::vec4(arma::vec3,\
 std::array<float,13>, std::array<float,13>, std::array<std::array<float,3>,6>, float *)> controller,\
 arma::vec3 referance={0,0,0}, std::array<std::array<float,3>,6> controllers_coefficients={{{0,0,0},\
                                                                    {0,0,0},\
                                                                    {0,0,0},\
                                                                    {0,0,0},\
                                                                    {0,0,0},\
                                                                    {0,0,0}}},
                                                                    int tuning_controller=1);


/** 
* This method will update the oriantation quaternion. 
* @param quaterion_before(arma::vec4)- the quaternion represent the previous orientation.
* @param angular_rate(arma::vec3)- the current angular rate.
* @return arma::vec4 which is the quaternion represent the new orientation.
*/
arma::vec4 updateOrientation(arma::vec4 quaternion_before, arma::vec3 angular_rate);

/** 
* This method will convert the state from array to either: euler angles or position for the controllers. 
* @param state_vector(std::array<float,13>)- full state vector for some point in time.
* @param type(char)- 'e' or 'p'.
* @return arma::vec3 with either euler angles or position.
*/
arma::vec3 convertStateVectorForController(std::array<float,13> state_vector, char type);

#endif