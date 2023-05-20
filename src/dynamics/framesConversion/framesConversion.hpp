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

/** 
* This method will implement matrix multiplication of all rotation matrixes. 
* @param Rx(arma::mat) - the rotation axis.
* @param Ry(arma::mat) - the amount of degress to rotate.
* @param Rz(arma::mat) - the amount of degress to rotate.
* @return an arma::mat which is the combined matrix .
* @throw "axis is wrong!" exception massage if the axis is not 1/2/3.
*/
arma::mat rotationMatrix(arma::mat Rx, arma::mat Ry, arma::mat Rz);

/** 
* This method will convert angle from degrees to radian. 
* @param angled (float) angle in degrees.
* @return a float represent angle in radians.
*/
float deg2rad(float deg);