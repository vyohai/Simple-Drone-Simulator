/** 
* @file controller.hpp 
* @brief This file contain the controller and all its helper functions. 
* 
* @author Yochai Weissman 
* 
* @date 27/5/2023
*/




#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>
#include <array>
#include <random>

#include <armadillo>

#ifndef CONTROLLER_TUNER
#define CONTROLLER_TUNER

// #include "../dynamics/forcesAndMoments/forcesAndMoments.hpp"
// #include "../dynamics/framesConversion/framesConversion.hpp"
#include "../updateEquations/updateEquations.hpp"
#include "../controller/controller.hpp"
#include "../interface/interface.hpp"
#include "../config.hpp"


/** 
* This method will evaluate the system output with respect to some referance for some controller.
* the method will return a high value(poor results) for too much overshoot or under shoot.
* the method will return the sum of the abolute values of(output-referance) divided by output length.
* 
* @param system_output(std::vector<float>) - output from the system.
* @param referance(float) - wanted state.
* @param max_boundery(float) - the state values cant be bigger than "max_boundary".
* @param max_boundery(float) - the state values cant be smaller than "min_boundary".
* @return float indicate how good the system performed.
*/
float evaluate_resaults(float referance, std::vector<float> system_output, float max_boundery,float min_boundery);


/** 
* This method will create a random sets of pid's to create a search.
* 
* @param coefficient_boundaries(std::array<float,6>) - [min kp, max kp, min ki, max ki, min kd, max kd].
* @param grid_size(int) -how many triplets to create.
* @return std::vector<std::array<float,3>> represents the "grid_size" number of triplets.
*/
std::vector<std::array<float,3>> create_parameter_grid(std::array<float,6> coefficient_boundaries, int grid_size );

/** 
* This method is the high level method of the search and evaluation.
* 
* @param coefficient_boundaries(std::array<float,6>) - [min kp, max kp, min ki, max ki, min kd, max kd].
* @param grid_size(int) -how many triplets to create.
* @param referance(float) - wanted state.
* @return std::array<float,3> represents the best triplets.
*/
std::array<float,3> search_controller_parameters(std::array<float,6> coefficient_boundaries, int grid_size, float referance);

#endif
