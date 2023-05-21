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
* @param TBD.
* @param TBD.
* @return TBD.
*/
std::vector<float> singleTimeUpdate(std::vector<float> before, std::vector<float> rotors_velocity);

/** 
* This method will accumulates the full simulation data. 
* @param TBD.
* @param TBD.
* @return TBD.
*/
std::vector<std::vector<float>> droneSimulation(float T);

/** 
* This method will update the oriantation quaternion. 
* @param quaterion_before(arma::vec4)- the quaternion represent the previous orientation.
* @param angular_rate(arma::vec3)- the current angular rate.
* @return arma::vec4 which is the quaternion represent the new orientation.
*/
arma::vec4 updateOrientation(arma::vec4 quaternion_before, arma::vec3 angular_rate);