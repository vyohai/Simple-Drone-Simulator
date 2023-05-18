#include <string>
#include <iostream>
// #include <armadillo>
// #include "./dynamics/forcesAndMoments/forcesAndMoments.hpp"

#include <armadillo>
#include "./dynamics/forcesAndMoments/forcesAndMoments.hpp"
#include "./dynamics/framesConversion/framesConversion.hpp"

using namespace arma;

int main()
{
    mat A(4, 5, fill::randu);
    std::cout<<A<<std::endl;
    return 1;
}
