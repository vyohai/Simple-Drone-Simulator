

#include <string>
#include <iostream>
#include <armadillo>


arma::mat singleAxisRotationMatrix(int axis, float angle)
{
    arma::mat A(4, 5, arma::fill::randu);
    std::cout<<A<<std::endl;
    return A;
}


arma::mat rotationMatrix(arma::Mat<float> Rx, arma::Mat<float> Ry, arma::Mat<float> Rz)
{
    arma::mat A(4, 5, arma::fill::randu);
    std::cout<<A<<std::endl;
    return A;
}
