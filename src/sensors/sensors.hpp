/** 
* @file sensors.hpp 
* @brief This header deals with sensors errors. 
* 
* @author Yochai Weissman 
* 
* @date 08/06/2023
*/


#include <string>
#include <vector>
#include <armadillo>

/** 
* This method will create white noise. 
* @param variance(float) - variance for the white noise.
* @return float  noise sample created.
*/
float gaussianWhiteNoise(float variance);

/** 
* This method will create a "measured" state with noises and errors. 
* @param real_state(std::array<float,13>) - the real state of the system.
* @param variance(float) - variance for the noises to add.
* @return std::array<float,13>  state measured by the sensors.
*/
std::array<float,13> sensorModel(std::array<float,13> real_state, float variance);

