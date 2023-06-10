#include <iostream>
#include <array>
#include <cmath>

#include <armadillo>


// #include "../src/dynamics/forcesAndMoments/forcesAndMoments.hpp"
// #include "../src/dynamics/framesConversion/framesConversion.hpp"
// #include "../src/updateEquations/updateEquations.hpp"
#include "../src/controller/controller.hpp"
#include "../src/visualization/visualization.hpp"
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
    arma::vec4 mixer_expected={10+HOVER_THRUST, 10+HOVER_THRUST, 10+HOVER_THRUST, 10+HOVER_THRUST};
    CHECK_TRUE(arma::approx_equal(mixer_output,mixer_expected,"absdiff",0.1));

    //only yaw
    thrust_cmd=0;
    yaw_cmd=10;
    pitch_cmd=0;
    roll_cmd=0;
    mixer_output=mixer(thrust_cmd, yaw_cmd, pitch_cmd, roll_cmd);
    mixer_expected={10+HOVER_THRUST,0,10+HOVER_THRUST,0};
    CHECK_TRUE(arma::approx_equal(mixer_output,mixer_expected,"absdiff",0.1));

    //only pitch
    thrust_cmd=0;
    yaw_cmd=0;
    pitch_cmd=10;
    roll_cmd=0;
    mixer_output=mixer(thrust_cmd, yaw_cmd, pitch_cmd, roll_cmd);
    mixer_expected={10+HOVER_THRUST,10+HOVER_THRUST,0,0};
    CHECK_TRUE(arma::approx_equal(mixer_output,mixer_expected,"absdiff",0.1));

    //only roll
    thrust_cmd=0;
    yaw_cmd=0;
    pitch_cmd=0;
    roll_cmd=10;
    mixer_output=mixer(thrust_cmd, yaw_cmd, pitch_cmd, roll_cmd);
    mixer_expected={10+HOVER_THRUST,0,0,10+HOVER_THRUST};
    CHECK_TRUE(arma::approx_equal(mixer_output,mixer_expected,"absdiff",0.1));

}

TEST_GROUP(controllerPIDTestGroup)
{
    
};

TEST(controllerPIDTestGroup, correctOutput)
{
    //altitude for example
    float referance=-1;

    // only P
    float state_before=0;
    float state_current=0;
    float Integral=0;
    std::array<float,3> pid_coefficients={1,0,0};

    float altitude_cmd=PID(state_before, state_current, Integral, pid_coefficients, referance);
    CHECK(altitude_cmd==1);

    //only D - negative drivative
    state_before=0;
    state_current=1;
    Integral=0;
    pid_coefficients={0,0,1};
    altitude_cmd=PID(state_before, state_current, Integral, pid_coefficients, referance);
    CHECK(altitude_cmd==1);

     //only D - positive drivative
    state_before=0;
    state_current=-0.9;
    Integral=0;
    pid_coefficients={0,0,1};
    altitude_cmd=PID(state_before, state_current, Integral, pid_coefficients, referance);
    CHECK(abs(altitude_cmd-(-0.9))<0.001);

    //only I
    state_before=0;
    state_current=1;
    Integral=1;
    pid_coefficients={0,1,0};
    altitude_cmd=PID(state_before, state_current, Integral, pid_coefficients, referance);
    CHECK(abs(altitude_cmd-(0.98))<0.001);

}

