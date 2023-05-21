/** 
* @file framesConversion.hpp 
* @brief this header deals with the world and body frames transformation. 
* 
* @author Yochai Weissman 
* 
* @date 20/5/2023
*/


#include <string>
#include <armadillo>

/** 
* This method will return a single axis rotation matrix. 
* @param axis(int) - the rotation axis.
* @param angle(float) - the amount of degress to rotate.
* @return an  arma::mat, sized 3*3, which is the rotation matrix.
*/
arma::mat singleAxisRotationMatrix(int axis,float angle);