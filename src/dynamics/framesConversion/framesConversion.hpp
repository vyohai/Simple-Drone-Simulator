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

/** 
* This method will return the quaternion-mulriplication of 2 quaternions. 
* @param Q1 (arma::vec4) quaternion1.
* @param Q2 (arma::vec4) quaternion2.
* @return a arma::vec4 represent resulted quaternion.
*/
arma::vec4 quaternionMultiplication(arma::vec4 Q1, arma::vec4 Q2);

/** 
* This method will convert rotation quaternion to eulwer angles in degrees. 
* @param Q (arma::vec4) rotation-quaternion.
* @return a arma::vec3 represent the euler sequence.
*/
arma::vec3 quaternionToEuler(arma::vec4 Q);

/** 
* This method will convert initial euler sequence to initial quaternion. 
* @param Euler (arma::vec3) initial euler angles.
* @return a arma::vec4 initial quaternion.
*/
arma::vec4 initialQuaternion(arma::vec3 Euler);