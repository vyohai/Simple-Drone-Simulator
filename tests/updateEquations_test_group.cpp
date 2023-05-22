#include <vector>
#include <iostream>

#include <armadillo>

#include "../src/dynamics/forcesAndMoments/forcesAndMoments.hpp"
#include "../src/dynamics/framesConversion/framesConversion.hpp"
#include "../src/updateEquations/updateEquations.hpp"
#include "../src/config.hpp"

#include "CppUTest/TestHarness.h"

TEST_GROUP(updateEquationsUpdateOrientationTestGroup)
{
    
};

TEST(updateEquationsUpdateOrientationTestGroup, correctOutput)
{
    arma::vec3 angles_initial={30.0,30.0,30.0};
    arma::vec3 angles_updated_expected={32.0,30.0,30.0};

    arma::vec4 Q_init=initialQuaternion(angles_initial);

    arma::vec3 angular_rate={deg2rad(100),0,0};

    arma::vec4 Q_updated=updateOrientation(Q_init,angular_rate);

    Q_updated=updateOrientation(Q_updated,angular_rate);

    arma::vec3 angles_updated=quaternionToEuler(Q_updated);

    CHECK_TRUE(arma::approx_equal(angles_updated,angles_updated_expected,"absdiff",0.3));
}

TEST_GROUP(updateEquationsSingleTimeUpdateTestGroup)
{
    
};

TEST(updateEquationsSingleTimeUpdateTestGroup, correctOutput)
{
    arma::vec3 angles_initial={30.0,30.0,30.0};
    arma::vec3 angles_updated_expected={32.0,30.0,30.0};

    arma::vec4 Q_init=initialQuaternion(angles_initial);

    arma::vec3 angular_rate={deg2rad(100),0,0};

    arma::vec4 Q_updated=updateOrientation(Q_init,angular_rate);


    Q_updated=updateOrientation(Q_updated,angular_rate);

    arma::vec3 angles_updated=quaternionToEuler(Q_updated);

    CHECK_TRUE(arma::approx_equal(angles_updated,angles_updated_expected,"absdiff",0.3));
}