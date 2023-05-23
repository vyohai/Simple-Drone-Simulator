/** 
* @file updateEquations.hpp 
* @brief TBD. 
* 
* @author Yochai Weissman 
* 
* @date 20/5/2023
*/


#include <string>
#include <vector>
#include <armadillo>

/** 
* This method will retirm the control command(rottors_speeds) based on the current and previous states
each state contains: quaternion, position, velocity, angular velocity.
* @param current_step(std::array<float,13>) - body angular rates.
* @param previous_step(std::array<float,13>) - the rottors speed command.
* @return arma::vec4, control output(rottors_speeds).
*/
arma::vec4 controller(std::array<float,13> current_step,std::array<float,13> previous_step);

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