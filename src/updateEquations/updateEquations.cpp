#include <string>
#include <iostream>
#include <cmath>

#include "updateEquations.hpp"

#include "../config.hpp"
#include "../dynamics/framesConversion/framesConversion.hpp"

// std::vector<float> singleTimeUpdate(std::vector<float> before, std::vector<float> rotors_velocity)
// {
//     // each state-->X=[x,y,z,roll,pitch,yaw,x_dot,y_dot,z_dot,p,q,r]
//     // each state-->X_dot=[x_dot,y_dot,z_dot,roll,pitch,yaw,x_dot,y_dot,z_dot,p,q,r]


//     // calculate linear acceleration

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