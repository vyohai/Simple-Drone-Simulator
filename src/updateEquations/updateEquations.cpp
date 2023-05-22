#include <string>
#include <iostream>
#include <cmath>

#include "updateEquations.hpp"

#include "../config.hpp"
#include "../dynamics/framesConversion/framesConversion.hpp"
#include "../dynamics/forcesAndMoments/forcesAndMoments.hpp"

// arma::vec singleTimeUpdate(arma::vec4 quat_before, arma::vec3 position_before, arma::vec3 velocty_before , arma::vec3 angular_velocty_before, arma::vec4 rotors_velocity)
// {
    

//     // calculate linear acceleration
//     arma::vec3 linear_acc=linearAcceleration(quat_before,rotors_velocity);

//     // calculate angular acceleartion
    

//     // update quaternion
        
//     // update positions

//     // update angular velocity

// }



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