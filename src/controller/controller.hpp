/** 
* @file controller.hpp 
* @brief This file contain the controller and all its helper functions. 
* 
* @author Yochai Weissman 
* 
* @date 27/5/2023
*/


#include <string>
#include <vector>
#include <armadillo>

/** 
* This method will return the control command(rottors_speeds) based on the
  current and previous states,
  each state contains: quaternion, position, velocity, angular velocity,
  for now only the altitude control is relevant.
* @param referance(arma::vec3) - wanted location.
* @param current_step(std::array<float,13>) - body angular rates.
* @param previous_step(std::array<float,13>) - the rottors speed command.
* @param controllers_coefficients(std::array<std::array<float,3>,6>) - (kp,ki,kd) triplets for all 6 PID's.
* @param Integral_altitude(float *) - the integral of the altitude error.
* @return arma::vec4, control output(rottors_speeds).
*/
arma::vec4 controller(arma::vec3 referance,std::array<float,13> current_step,\
                        std::array<float,13> previous_step,\
                         std::array<std::array<float,3>,6> controllers_coefficients,\
                         float *Integral_altitude);

/** 
* This method will implement the mixer which get thrust/yaw/pitch/roll commands and
  convert them to rottors_speeds,the mixer assumes that positive rotation in rottor1 and
  rottor3 will cause positive Mz.
* @param thrust_cmd(float) - wanted thrust.
* @param yaw_cmd(float) - wanted yaw.
* @param pitch_cmd(float) - wanted pitch.
* @param roll_cmd(float) - wanted roll.
* @return arma::vec4, the needed rottors speeds.
*/
arma::vec4 mixer(float thrust_cmd, float yaw_cmd, float pitch_cmd,float roll_cmd);

/** 
* This method will implement PID scalar controller based on previous state and current state.
* @param state_before(float) - the current state.
* @param state_current(float) - the previous state.
* @param Integral(float) - the current value of the integral of the state error.
* @param pid_coefficients(std::array<float,3>) - Kp, Ki, Kd.
* @param referance(float) - the wanted value for the stae. 
* @return arma::vec4, the needed rottors speeds( remember that 1.5811 is the HOVER horizontal speed).
*/
float PID(float state_before,float state_current, float Integral, std::array<float,3> pid_coefficients,float referance);