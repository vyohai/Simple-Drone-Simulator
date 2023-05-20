#include <string>
#include <iostream>
#include <numbers>
#include <cmath>

#include <armadillo>

#include "framesConversion.hpp"


arma::mat singleAxisRotationMatrix(int axis, float angle)
{
    arma::mat A(3,3,arma::fill::zeros);
    float angle_rad=deg2rad(angle);
    if (axis==1)
        {
            A = { {1,0,0},
            {0,std::cos(angle_rad),-std::sin(angle_rad)},
            {0,std::sin(angle_rad),std::cos(angle_rad)}};
        }
    else if(axis==2)
        {
            A = { {std::cos(angle_rad),0,std::sin(angle_rad)},
                {0,1,0},
                {-std::sin(angle_rad),0,std::cos(angle_rad)}};
        }
    else if(axis==3)
        {
            A = { {std::cos(angle_rad),-std::sin(angle_rad),0},
                {std::sin(angle_rad),std::cos(angle_rad),0},
                {0,0,1}};
        }
    else
        {
            const char* msg="axis is wrong!";
            throw msg;
        }
    
    return A;
}


arma::mat rotationMatrix(arma::mat Rx, arma::mat Ry, arma::mat Rz)
{
    arma::mat A=Rz*Ry*Rx;
    return A;
}

float deg2rad(float deg)
{
    float output=deg*std::numbers::pi/180.0;
    return output;
}