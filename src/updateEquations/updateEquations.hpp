/** 
* @file updateEquations.hpp 
* @brief TBD. 
* 
* @author Yochai Weissman 
* 
* @date 20/5/2023
*/


#include <string>
#include <vector>
#include <armadillo>

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
* @param controller(std::function<arma::vec4(std::array<float,13> current_step, std::array<float,13> previous_step)>) a function gets a states(current and previos) at some time and produces a control output(rottors_speeds).
* @return TBD.
*/
std::vector<std::array<float,13>> droneSimulation(float T, std::function<arma::vec4(std::array<float,13>,std::array<float,13>)> controller);

/** 
* This method will update the oriantation quaternion. 
* @param quaterion_before(arma::vec4)- the quaternion represent the previous orientation.
* @param angular_rate(arma::vec3)- the current angular rate.
* @return arma::vec4 which is the quaternion represent the new orientation.
*/
arma::vec4 updateOrientation(arma::vec4 quaternion_before, arma::vec3 angular_rate);