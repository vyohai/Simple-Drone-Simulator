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
* @return an  arma::Mat<float>, sized 3*3, which is the rotation matrix.
*/
arma::mat singleAxisRotationMatrix(int axis,float angle);

/** 
* This method will implement matrix multiplication of all rotation matrixes. 
* @param Rx(arma::Mat<float>) - the rotation axis.
* @param Ry(arma::Mat<float>) - the amount of degress to rotate.
* @param Rz(arma::Mat<float>) - the amount of degress to rotate.
* @return an arma::Mat<float> which is the combined matrix .
*/
arma::mat rotationMatrix(arma::Mat<float> Rx, arma::Mat<float> Ry, arma::Mat<float> Rz);
