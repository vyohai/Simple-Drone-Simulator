/** 
* @file sensors.hpp 
* @brief This header deals with sensors errors. 
* 
* @author Yochai Weissman 
* 
* @date 08/06/2023
*/

#ifndef SENSORS
#define SENSORS

#include <random>
#include <cmath>
#include <array>
#include <vector>

#include <armadillo>

#include "../config.hpp"



/** 
* This method will create white noise. 
* @return float  noise sample created.
*/
float gaussianWhiteNoise();

/** 
* This method will create a "measured" state with noises and errors. 
* @param real_state(std::array<float,13>) - the real state of the system.
* @return std::array<float,13>  state measured by the sensors.
*/
std::array<float,13> sensorModel(std::array<float,13> real_state);

#endif