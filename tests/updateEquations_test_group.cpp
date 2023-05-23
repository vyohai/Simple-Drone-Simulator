#include <iostream>
#include <array>
#include <cmath>

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

TEST(updateEquationsSingleTimeUpdateTestGroup, correctOutputFalling)
{
    // no thrust, the drone will just fall down
    arma::vec4 rottor_speeds={0,0,0,0};

    arma::vec3 initial_euler_angles={0,0,0};
    arma::vec4 quaternion=initialQuaternion(initial_euler_angles);
    arma::vec3 position={0,0,0};
    arma::vec3 velocity={0,0,0};
    arma::vec3 angular_velocity={0,0,0};
    std::array<float, 13> new_state=singleTimeUpdate(quaternion,\
                                                    position,\
                                                    velocity,\
                                                    angular_velocity,\
                                                    rottor_speeds);

    // expected only velocity in z due to G, vz=G/DT=0.1
    std::array<float, 13> new_state_expected={0,0,0,1,\
                                                0,0,0,\
                                                0,0,0.1,\
                                                0,0,0};
    float sum_difrences=0.0;
    for (int i = 0; i < 13; i++)
    {
        sum_difrences=sum_difrences+abs(new_state[i]-new_state_expected[i]);
    }

    CHECK_TRUE(sum_difrences< 0.01);
}

TEST(updateEquationsSingleTimeUpdateTestGroup, correctOutputHovering)
{
    // thrust equal to gravity, the drone will just hover
    arma::vec4 rottor_speeds={1.58,1.58,1.58,1.58};

    arma::vec3 initial_euler_angles={0,0,0};
    arma::vec4 quaternion=initialQuaternion(initial_euler_angles);
    arma::vec3 position={0,0,0};
    arma::vec3 velocity={0,0,0};
    arma::vec3 angular_velocity={0,0,0};
    std::array<float, 13> new_state=singleTimeUpdate(quaternion,\
                                                    position,\
                                                    velocity,\
                                                    angular_velocity,\
                                                    rottor_speeds);

    // expected no movement
    std::array<float, 13> new_state_expected={0,0,0,1,\
                                                0,0,0,\
                                                0,0,0,\
                                                0,0,0};
    float sum_difrences=0.0;
    for (int i = 0; i < 13; i++)
    {
        sum_difrences=sum_difrences+abs(new_state[i]-new_state_expected[i]);
    }

    CHECK_TRUE(sum_difrences< 0.01);
}

TEST(updateEquationsSingleTimeUpdateTestGroup, correctOutputRotateX)
{
    // rottors 1 and 4 are dominant, moment in x
    //thrust(if KF=1)=2*10^2+2*0^2-->az=-(200-10)-->vz=-190*DT=-1.9
    //Mx=10^2+10^2-0^2-0^2=200-->angular_acc_x=200/Ixx=200-->angular_vel_x=200*DT=2
    arma::vec4 rottor_speeds={10,0,0,10};

    arma::vec3 initial_euler_angles={0,0,0};
    arma::vec4 quaternion=initialQuaternion(initial_euler_angles);
    arma::vec3 position={0,0,0};
    arma::vec3 velocity={0,0,0};
    arma::vec3 angular_velocity={0,0,0};
    std::array<float, 13> new_state=singleTimeUpdate(quaternion,\
                                                    position,\
                                                    velocity,\
                                                    angular_velocity,\
                                                    rottor_speeds);

    
    
     // expected no movement
    std::array<float, 13> new_state_expected={0,0,0,1,\
                                                0,0,0,\
                                                0,0,-1.9,\
                                                2,0,0};
    float sum_difrences=0.0;
    for (int i = 0; i < 13; i++)
    {
        sum_difrences=sum_difrences+abs(new_state[i]-new_state_expected[i]);
    }

    CHECK_TRUE(sum_difrences< 0.01);
}

TEST(updateEquationsSingleTimeUpdateTestGroup, correctOutputRotateY)
{
    // rottors 1 and 2 are dominant, moment in y
    //thrust(if KF=1)=2*10^2+2*0^2-->az=-(200-10)-->vz=-190*DT=-1.9
    //My=10^2+10^2-0^2-0^2=200-->angular_acc_y=200/Iyy=200-->angular_vel_y=200*DT=2
    arma::vec4 rottor_speeds={10,10,0,0};

    arma::vec3 initial_euler_angles={0,0,0};
    arma::vec4 quaternion=initialQuaternion(initial_euler_angles);
    arma::vec3 position={0,0,0};
    arma::vec3 velocity={0,0,0};
    arma::vec3 angular_velocity={0,0,0};
    std::array<float, 13> new_state=singleTimeUpdate(quaternion,\
                                                    position,\
                                                    velocity,\
                                                    angular_velocity,\
                                                    rottor_speeds);

    
     // expected no movement
    std::array<float, 13> new_state_expected={0,0,0,1,\
                                                0,0,0,\
                                                0,0,-1.9,\
                                                0,2,0};
    float sum_difrences=0.0;
    for (int i = 0; i < 13; i++)
    {
        sum_difrences=sum_difrences+abs(new_state[i]-new_state_expected[i]);
    }

    CHECK_TRUE(sum_difrences< 0.01);
}

TEST(updateEquationsSingleTimeUpdateTestGroup, correctOutputRotateZ)
{
    // rottors 1 and 3 are dominant, moment in z
    //thrust(if KF=1)=2*10^2+2*0^2-->az=-(200-10)-->vz=-190*DT=-1.9
    //Mz=10^2+10^2-0^2-0^2=200-->angular_acc_z=200/Izz=100-->angular_vel_z=100*DT=1
    arma::vec4 rottor_speeds={10,0,10,0};

    arma::vec3 initial_euler_angles={0,0,0};
    arma::vec4 quaternion=initialQuaternion(initial_euler_angles);
    arma::vec3 position={0,0,0};
    arma::vec3 velocity={0,0,0};
    arma::vec3 angular_velocity={0,0,0};
    std::array<float, 13> new_state=singleTimeUpdate(quaternion,\
                                                    position,\
                                                    velocity,\
                                                    angular_velocity,\
                                                    rottor_speeds);

    
    
     // expected no movement
    std::array<float, 13> new_state_expected={0,0,0,1,\
                                                0,0,0,\
                                                0,0,-1.9,\
                                                0,0,1};
    float sum_difrences=0.0;
    for (int i = 0; i < 13; i++)
    {
        sum_difrences=sum_difrences+abs(new_state[i]-new_state_expected[i]);
    }

    CHECK_TRUE(sum_difrences< 0.01);
}

TEST(updateEquationsSingleTimeUpdateTestGroup, correctOutputAccelerateX)
{
    // to accelerate in x_world:
    // initial orientation pitch =-45 (rottors tilted forward)
    // to cancel gravity: T*cos(45)=10-->4*w_rot^2=10/cos(45)--> w=1.88
    // ax=10-->vx=10*DT=0.0ץ1
    
    arma::vec4 rottor_speeds={1.88,1.88,1.88,1.88};

    arma::vec3 initial_euler_angles={0,-45,0};
    arma::vec4 quaternion=initialQuaternion(initial_euler_angles);
    arma::vec3 position={0,0,0};
    arma::vec3 velocity={0,0,0};
    arma::vec3 angular_velocity={0,0,0};
    std::array<float, 13> new_state=singleTimeUpdate(quaternion,\
                                                    position,\
                                                    velocity,\
                                                    angular_velocity,\
                                                    rottor_speeds);

        // expected no movement
    std::array<float, 13> new_state_expected={0,-0.382683,0,0.92388,\
                                                0,0,0,\
                                                0.1,0,0,\
                                                0,0,0};
    float sum_difrences=0.0;
    for (int i = 0; i < 13; i++)
    {
        sum_difrences=sum_difrences+abs(new_state[i]-new_state_expected[i]);
    }

    CHECK_TRUE(sum_difrences< 0.01);
    
}

TEST(updateEquationsSingleTimeUpdateTestGroup, correctOutputAccelerateY)
{
    // to accelerate in x_world:
    // initial orientation roll =45 (rottors tilted right)
    // to cancel gravity: T*cos(45)=10-->4*w_rot^2=10/cos(45)--> w=1.88
    // ay=10-->vy=10*DT=0.0ץ1

    arma::vec4 rottor_speeds={1.88,1.88,1.88,1.88};

    arma::vec3 initial_euler_angles={45,0,0};
    arma::vec4 quaternion=initialQuaternion(initial_euler_angles);
    arma::vec3 position={0,0,0};
    arma::vec3 velocity={0,0,0};
    arma::vec3 angular_velocity={0,0,0};
    std::array<float, 13> new_state=singleTimeUpdate(quaternion,\
                                                    position,\
                                                    velocity,\
                                                    angular_velocity,\
                                                    rottor_speeds);

    // expected no movement
    std::array<float, 13> new_state_expected={0.3827,0,0,0.9239,\
                                                0,0,0,\
                                                0,0.1,0,\
                                                0,0,0};
    float sum_difrences=0.0;
    for (int i = 0; i < 13; i++)
    {
        sum_difrences=sum_difrences+abs(new_state[i]-new_state_expected[i]);
    }

    CHECK_TRUE(sum_difrences< 0.01);
    
}

TEST_GROUP(updateEquationsDroneSimulationTestGroup)
{
    
};

TEST(updateEquationsDroneSimulationTestGroup, correctOutput)
{
    // std::cout<<std::endl<<"-----------falling-----------"<<std::endl;
    // arma::vec4 rottors_speed={0,0,0,0};
    // std::vector<std::array<float,13>> result=droneSimulation(0.1,rottors_speed);

    // std::cout<<std::endl<<"----------hovering-------------"<<std::endl;
    // rottors_speed={1.58,1.58,1.58,1.58};
    // result=droneSimulation(0.1,rottors_speed);

    //std::cout<<std::endl<<"---------rotating in x body direction----------"<<std::endl;
    // rottors_speed={2.236,0,0,2.236};
    
    //std::vector<std::array<float,13>> result=droneSimulation(0.1,controller);




}
