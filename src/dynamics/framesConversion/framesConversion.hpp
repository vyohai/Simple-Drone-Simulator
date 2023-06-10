/** 
* @file framesConversion.hpp 
* @brief this header deals with the world and body frames transformation. 
* 
* @author Yochai Weissman 
* 
* @date 27/5/2023
*/


#ifndef FRAMES_CONVERSION
#define FRAMES_CONVERSION


#include <cmath>

#include <armadillo>

#include "../../config.hpp"

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
* This method will rotate a vector from body frame to world frame. 
* @param Q1 (arma::vec4) quaternion represent the body rotation with respect to earth.
* @param vec (arma::vec3) vetor in body frame.
* @return a arma::vec3 represent the vector in world frame.
*/
arma::vec3 quaternionVectorRotation(arma::vec4 Q1, arma::vec3 vec);

/** 
* This method will convert rotation quaternion to euler angles in degrees - WONT PASS TESTS FOR NOW, DONT USE. 
* @param Q (arma::vec4) rotation-quaternion.
* @return a arma::vec3 represent the euler sequence.
*/
arma::vec3 quaternionToEuler(arma::vec4 Q);

/** 
* This method will convert initial euler sequence to initial quaternion - WONT PASS TESTS FOR NOW, DONT USE.
* @param Euler (arma::vec3) initial euler angles.
* @return a arma::vec4 initial quaternion.
*/
arma::vec4 initialQuaternion(arma::vec3 Euler);

#endif