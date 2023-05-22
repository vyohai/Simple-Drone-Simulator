#include <string>
#include <iostream>
#include <armadillo>

#include "../../config.hpp"
#include "../../dynamics/framesConversion/framesConversion.hpp"


float rottorThrust(float omega)
{
    return KF*omega*omega;
}

float rottorMoment(float omega)
{
    return KM*omega*omega;
}

arma::vec3 linearAcceleration(arma::vec4 quaternion, arma::vec4 rotors_speed)
{
    //calc the thrust induced by all 4 rottors
    float thrust=0;
    for(int i=0; i<4;i=i+1)
    {
        thrust=thrust+rottorThrust(rotors_speed(i));
    }
    
    //calculate the linear acceleration in world frame
    arma::vec3 thrust_vector={0,0,-thrust/M};//body frame
    arma::vec3 thrust_vector_rotated=quaternionVectorRotation(quaternion,thrust_vector);
   
    arma::vec3 gravity_vector={0,0,G};//world frame

    arma::vec3 a=gravity_vector+thrust_vector_rotated;

    return a;
}

arma::vec3 angularAcceleration(arma::vec3 Moments, arma::vec3 ang_vel)
{
    arma::vec3 angular_acceleration={0,0,0};

    //dwx=Mx/Ixx-(Iyy-Izz)*wy*wz/Ixx
    float dwx=Moments(0)/Ixx-(Iyy-Izz)*ang_vel(1)*ang_vel(2)/Ixx;

    //dwy=My/Iyy-(Izz-Ixx)*wx*wz/Iyy
    float dwy=Moments(1)/Ixx-(Izz-Ixx)*ang_vel(0)*ang_vel(2)/Iyy;

    //dwz=Mz/Izz-(Ixx-Iyy)*wx*wy/Izz
    float dwz=Moments(2)/Ixx-(Ixx-Iyy)*ang_vel(0)*ang_vel(1)/Izz; 

    // fill the angular acceleration vector
    angular_acceleration(0)=dwx;
    angular_acceleration(1)=dwy;
    angular_acceleration(2)=dwz;

    return angular_acceleration;
}

arma::vec2 MomentsXY(arma::vec4 Thrusts)
{
    arma::vec2 Mxy={0,0};

    // for x configuration
    
    //Mx=(T_front_left+T_back_left-T_front_right-T_back_right)*L
    Mxy(0)=(Thrusts(0)+Thrusts(3)-Thrusts(1)-Thrusts(2))*L;

    //My=(T_front_left+T_front_right-T_back_right-T_back_left)*L
    Mxy(1)=(Thrusts(0)+Thrusts(1)-Thrusts(2)-Thrusts(3))*L;

    return Mxy;
}

float MomentZ(arma::vec4 rottor_vel)
{
    float Mz=0;
    for(int i=0; i<4;i=i+1)
    {
        float sign=pow(-1,i+2);
        Mz=Mz+sign*rottorMoment(rottor_vel(i));
    }
    return Mz;
}


