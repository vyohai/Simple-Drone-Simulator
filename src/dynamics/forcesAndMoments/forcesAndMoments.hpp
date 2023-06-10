/** 
* @file forcesAndMoments.hpp 
* @brief this header deals with forces and moments calculations. 
* 
* @author Yochai Weissman 
* 
* @date 27/5/2023
*/

#ifndef FORCES_AND_MOMENTS
#define FORCES_AND_MOMENTS

#include <armadillo>

#include "../../config.hpp"
#include "../../dynamics/framesConversion/framesConversion.hpp"

/** 
* This method will calculate the thruset of a single rottor based on the rottor speed. 
* @param rottor_speed(float) - the rottor anular speed.
* @return a float represent the rottor thrust.
*/
float rottorThrust( float rottor_speed);

/** 
* This method will calculate the linear acceleration in the world frame. 
* @param quaternion (arma::vec4) - quaternion represent the world to body frame transformation.
* @param rottor_speed(arma::vec4) - rottor speeds.
* @return an arma::vec3 represent the linear accelerations.
*/
arma::vec3 linearAcceleration(arma::vec4 quaternion, arma::vec4 rotors_speed);

/** 
* This method will calculate the angular acceleration in the body frame. 
* @param ang_vel (arma::vec3) - vector size 3 of angular velocities.
* @param Moments(arma::vec3) - vector size 3 of moments.
* @return an arma::vec3 represent the angular accelerations.
*/
arma::vec3 angularAcceleration(arma::vec3 Moments, arma::vec3 ang_vel);

/** 
* This method will calculate the x_body and y_body moments. 
* @param rottor_speeds (arma::vec4) - vector size e of rottor rotations.
* @return an arma::vec2 represent [Mxb,Myb].
*/
arma::vec2 MomentsXY(arma::vec4 rottor_speeds);

/** 
* This method will calculate the x_body and y_body moments. 
* @param rottor_vel (arma::vec4>) - all 4 rottor rotation velocity(only size and not direction).
* @return an float represent Mz.
*/
float MomentZ(arma::vec4 rottor_vel);

#endif

