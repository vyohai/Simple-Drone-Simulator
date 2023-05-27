/** 
* @file visualization.hpp 
* @brief this header deals visualize the simulation data. 
* 
* @author Yochai Weissman 
* 
* @date 27/5/2023
*/


#include <string>
#include <armadillo>
#include <sciplot/sciplot.hpp>

/** 
* This method returns a single state for all times. 
* @param vector(std::vector<std::array<float,13>>) - the whole simulation data.
* @param type(char)-'x'/'y'/'z'/'r'/'p'/'h', represents:x_position/y_position/z_position/roll/pitch/yaw.
* @return an std::vector<float> represen single state for all simulation time.
*/
std::vector<float> getDataFromSimulation(std::vector<std::array<float,13>> vector, char type);


/** 
* This method produces a plot for some state and save it in the 'path' loacation. 
* @param vector(std::vector<float> vector) - the time vector to plot.
* @param path(std::string) - wgere to save the figure.
* @param name(std::string) - the name to save the plot.
*/
void singlePlot(std::vector<float> vector, std::string path, std::string name);

