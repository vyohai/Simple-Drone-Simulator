#include <iostream>
#include <array>
#include <cmath>

#include <armadillo>

#include "../src/dynamics/forcesAndMoments/forcesAndMoments.hpp"
#include "../src/dynamics/framesConversion/framesConversion.hpp"
#include "../src/updateEquations/updateEquations.hpp"
#include "../src/controller/controller.hpp"
#include "../src/config.hpp"

#include "CppUTest/TestHarness.h"

TEST_GROUP(controllerMixerTestGroup)
{
    
};

TEST(controllerMixerTestGroup, correctOutput)
{
    
    //only thrust
    float thrust_cmd=10;
    float yaw_cmd=0;
    float pitch_cmd=0;
    float roll_cmd=0;
    arma::vec4 mixer_output=mixer(thrust_cmd, yaw_cmd, pitch_cmd, roll_cmd);
    arma::vec4 mixer_expected={10,10,10,10};
    CHECK_TRUE(arma::approx_equal(mixer_output,mixer_expected,"absdiff",0.1));

    //only yaw
    thrust_cmd=0;
    yaw_cmd=10;
    pitch_cmd=0;
    roll_cmd=0;
    mixer_output=mixer(thrust_cmd, yaw_cmd, pitch_cmd, roll_cmd);
    mixer_expected={20,0,20,0};
    CHECK_TRUE(arma::approx_equal(mixer_output,mixer_expected,"absdiff",0.1));

    //only pitch
    thrust_cmd=0;
    yaw_cmd=0;
    pitch_cmd=10;
    roll_cmd=0;
    mixer_output=mixer(thrust_cmd, yaw_cmd, pitch_cmd, roll_cmd);
    mixer_expected={20,20,0,0};
    CHECK_TRUE(arma::approx_equal(mixer_output,mixer_expected,"absdiff",0.1));

    //only roll
    thrust_cmd=0;
    yaw_cmd=0;
    pitch_cmd=0;
    roll_cmd=10;
    mixer_output=mixer(thrust_cmd, yaw_cmd, pitch_cmd, roll_cmd);
    mixer_expected={20,0,0,20};
    CHECK_TRUE(arma::approx_equal(mixer_output,mixer_expected,"absdiff",0.1));

}