#include <string>
#include <iostream>
// #include <armadillo>
// #include "./dynamics/forcesAndMoments/forcesAndMoments.hpp"

#include <armadillo>
#include "./dynamics/forcesAndMoments/forcesAndMoments.hpp"
#include "./dynamics/framesConversion/framesConversion.hpp"

// using namespace arma;

int main()
{
    arma::mat A1(3, 3, arma::fill::randu);
    arma::mat A2(3, 3, arma::fill::randu);
    arma::mat A3(3, 3, arma::fill::randu);
    arma::mat A=rotationMatrix(A1,A2,A3);
    std::cout<<A<<std::endl;
    return 1;
}
