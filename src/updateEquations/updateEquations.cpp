#include <string>
#include <iostream>
#include <cmath>
#include <vector>
#include <array>

// #include <armadillo>

#include "updateEquations.hpp"

#include "../config.hpp"
#include "../dynamics/framesConversion/framesConversion.hpp"
#include "../dynamics/forcesAndMoments/forcesAndMoments.hpp"

std::array<float, 13> singleTimeUpdate(arma::vec4 quat_before, arma::vec3 position_before, arma::vec3 velocty_before , arma::vec3 angular_velocty_before, arma::vec4 rotors_velocity)
{
    
    // calculate Moments
    arma::vec2 Mxy=MomentsXY(rotors_velocity);
    float Mz=MomentZ(rotors_velocity);
    arma::vec3 Moments={Mxy(0), Mxy(1),Mz};

    // calculate angular acceleartion
    arma::vec3 angular_acc=angularAcceleration(Moments, angular_velocty_before);
    
    // update angular velocity
    arma::vec3 angular_velocty_updated=angular_velocty_before+DT*angular_acc;

    // update quaternion
    arma::vec4 quat_updated=updateOrientation(quat_before,angular_velocty_before);
        
      // calculate linear acceleration
    arma::vec3 linear_acc=linearAcceleration(quat_updated,rotors_velocity);

    // update linear velocity
    arma::vec3 velocty_updated=velocty_before+DT*linear_acc;

    // update positions
    arma::vec3 position_updated=position_before+DT*velocty_before;


    // accumelate all to one container
    std::array<float, 13> output;
    output[0]=quat_updated(0);
    output[1]=quat_updated(1);
    output[2]=quat_updated(2);
    output[3]=quat_updated(3);
    output[4]=position_updated(0);
    output[5]=position_updated(1);
    output[6]=position_updated(2);
    output[7]=velocty_updated(0);
    output[8]=velocty_updated(1);
    output[9]=velocty_updated(2);
    output[10]=angular_velocty_updated(0);
    output[11]=angular_velocty_updated(1);
    output[12]=angular_velocty_updated(2);

    return output;
}



// std::vector<std::vector<float>> droneSimulation(float T)
// {

// }

arma::vec4 updateOrientation(arma::vec4 quaternion_before, arma::vec3 angular_rate)
{
    arma::vec4 S={0,angular_rate(0),angular_rate(1),angular_rate(2)};

    arma::vec4 dQ=0.5*quaternionMultiplication(quaternion_before,S);

    arma::vec4 Q_updated=quaternion_before+dQ*DT;
    
    return Q_updated;
}