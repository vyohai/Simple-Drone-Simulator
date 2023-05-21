/** 
* @file forcesAndMoments.hpp 
* @brief this header deals with forces and noments calculations. 
* 
* @author Yochai Weissman 
* 
* @date 20/5/2023
*/


#include <string>
#include <armadillo>

/** 
* This method will calculate the thruset of a single rottor based on the rottor speed. 
* @param kf (float) - the rotor thrust coefficient.
* @param omega(float) - the rottor anular speed.
* @return an float represent the rottor thrust.
*/
float rottorThrust(float kf, float omega);

/** 
* This method will calculate the linear acceleration in the world frame. 
* @param R (arma::Mat<float>) - rotation matrix.
* @param Thrust(float) - total drone thrust.
* @param m(float) - drone mass.
* @return an arma::Mat<float> represent the lineaar accelerations.
*/
arma::mat linearAcceleration(arma::Mat<float> R, float Thrust, float m);

/** 
* This method will calculate the angular acceleration in the body frame. 
* @param ang_vel (arma::Mat<float>) - vector size 3 of angular velocities.
* @param Moments(arma::Mat<float>) - vector size 3 of moments.
* @param r(float) - drone mass.
* @return an arma::Mat<float> represent the angular accelerations.
*/
arma::mat angularAcceleration(arma::mat R, float Thrust, float m);

/** 
* This method will calculate the x_body and y_body moments. 
* @param Thrusts (arma::Mat<float>) - vector size 3 of angular velocities.
* @param L (float) - vector size 3 of moments.
* @return an arma::Mat<float> represent [Mxb,Myb].
*/
arma::mat MomentsXY(arma::Mat<float> Thrusts, float L);

/** 
* This method will calculate the x_body and y_body moments. 
* @param rottor_vel (arma::Mat<float>) - all 4 rottor rotation velocity with direction.
* @param km (float) - moment coefficient.
* @return an float represent Mz.
*/
float MomentsZ(arma::mat rottor_vel, float km);

/** 
* This method will calculate the forces generated by the thrust and the tilt angle. 
* @param roll (float) - all 4 rottor rotation velocity with direction.
* @param pitch (float) - moment coefficient.
* @param Thrusts (float) - sum of all thrust.
* @return an arma::Mat<float> represents the [Fxb,Fyb,Fzw].
*/
float Forces(arma::mat rottor_vel, float km);

