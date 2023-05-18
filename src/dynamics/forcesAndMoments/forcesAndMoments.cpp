#include <string>
#include <iostream>
#include <armadillo>


float rottorThrust(float kf, float omega)
{
    arma::mat a=arma::ones(3,3);
    std::cout<<a;
    return 1;
}

arma::mat linearAcceleration(arma::Mat<float> R, float Thrust, float m)
{
    arma::mat a=arma::ones(3,3);
    std::cout<<a;
    return a;
}

arma::mat angularAcceleration(arma::Mat<float> R, float Thrust, float m)
{
    arma::mat a=arma::ones(3,3);
    std::cout<<a;
    return a;
}

arma::mat MomentsXY(arma::Mat<float> Thrusts, float L)
{
    arma::mat a=arma::ones(3,3);
    std::cout<<a;
    return a;
}

float MomentsZ(arma::Mat<float> rottor_vel, float km)
{
    arma::mat a=arma::ones(3,3);
    std::cout<<a;
    return 1;
}

float Forces(arma::mat rottor_vel, float km)
{
    arma::mat a=arma::ones(3,3);
    std::cout<<a;
    return 1;
}