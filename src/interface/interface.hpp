/** 
* @file interface.hpp 
* @brief this header deals with the command line interface that helps the user create the simulation. 
* 
* @author Yochai Weissman 
* 
* @date 27/5/2023
*/


#include <string>
#include <iostream>
#include <direct.h>
#include <array>
#include <filesystem>

#include <armadillo>

#ifndef INTERFACE
#define INTERFACE

#include "../config.hpp"
#include "../dynamics/framesConversion/framesConversion.hpp"
#include "../dynamics/forcesAndMoments/forcesAndMoments.hpp"
#include "../controllerTuner/controllerTuner.hpp"
#include "../visualization/visualization.hpp"

/** 
* This method get a name for the simulation from the user, and creates a directory in /simulations/ with that name. 
* @return an  std::string with the simulation name.
*/
std::string getSimulationName();

/** 
* This method get a wanted duration for the simulation from the user. 
* @return an float represent the simulation duration in seconds.
*/
float getSimulationDuration();

/** 
* This method gets the wanted initial rottors-speed-velocity. 
* @return an arma::vec4 represent the initial rottors-speed-velocity.
*/
arma::vec4 getSimulationRottorsVelocity();

/** 
* This method present the initial forces and moments due to the initial rottor speed, chosen by the user. 
* @param rottors_velocity(arma::vec4) - the initial rottors_velocity.
* @param quaternion_initial(arma::vec4) - the initial orientation quaternion.
*/
void initialForcesAndMoments(arma::vec4 rottors_velocity, arma::vec4 quaternion_initial);

/** 
* This method let the user pick the altitude controller PID coefficients. 
* @param referance(float) - wanted altitude.
* @return an std::array<float,3> represent (Kp, Ki, Kd).
*/
std::array<float,3> tuneAltitudeController(float referance);

/** 
* This method returns a single state for all times. 
* @param fullVector(std::vector<std::array<float,13>>) - the whole simulation data.
* @param myType(char)-'x'/'y'/'z'/'r'/'p'/'h', represents:x_position/y_position/z_position/roll/pitch/yaw.
* @return an std::vector<float> represen single state for all simulation time.
*/
std::vector<float> getDataFromSimulation(std::vector<std::array<float,13>> fullVector, char myType);


#endif