/** 
* @file visualization.hpp 
* @brief this header deals visualize the simulation data. 
* 
* @author Yochai Weissman 
* 
* @date 27/5/2023
*/

#ifndef VISUALIZATION
#define VISUALIZATION


#include <vector>
#include <array>

#include <filesystem>
#include <algorithm>


#include <armadillo>

#include "../config.hpp"




/** 
* This method produces a plot for some state and save it in the 'path' loacation. 
* @param myVector(std::vector<float> vector) - the time vector to plot.
* @param myPath(std::string) - wgere to save the figure.
* @param myName(std::string) - the name to save the plot.
*/
void singlePlot(std::vector<float> myVector, std::string myPath, std::string myName);

#endif