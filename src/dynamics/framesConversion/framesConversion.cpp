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

arma::vec4 quaternionMultiplication(arma::vec4 Q1, arma::vec4 Q2)
{
    float a1=Q1[0];
    float b1=Q1[1];
    float c1=Q1[2];
    float d1=Q1[3];
    
    float a2=Q2[0];
    float b2=Q2[1];
    float c2=Q2[2];
    float d2=Q2[3];
    
    arma::vec4 Q;
    Q[0]=a1*a2-b1*b2-c1*c2-d1*d2;
    Q[1]=a1*b2+b1*a2+c1*d2-d1*c2;
    Q[2]=a1*c2-b1*d2+c1*a2+d1*b2;
    Q[3]=a1*d2+b1*c2-c1*b2+d1*a2;
    
    return Q;
}


arma::vec3 quaternionToEuler(arma::vec4 Q)
{
    float qx=Q[0];
    float qy=Q[1];
    float qz=Q[2];
    float qw=Q[3];

   arma::mat R={{1-2*(qy*qy+qz*qz),2*(qx*qy-qw*qz),2*(qx*qz+qw*qy)},
                {2*(qx*qy+qw*qz), 1-2*(qx*qx+qz*qz),2*(qy*qz-qw*qx)},
                {2*(qx*qz-qw*qy),2*(qy*qz+qw*qx),1-2*(qx*qx+qy*qy)}};

    float yaw=atan2(R(2,1),R(2,2));
    float pitch=-asin(R(2,0));
    float roll=atan2(R(1,0)/cos(pitch),R(0,0)/cos(pitch));

    arma::vec3 angles={roll*57,pitch*57,57*yaw};

    return angles;
}


arma::vec4 initialQuaternion(arma::vec3 Euler)
{
    arma::mat Rx=singleAxisRotationMatrix(1,Euler(0));
    arma::mat Ry=singleAxisRotationMatrix(2,Euler(1));
    arma::mat Rz=singleAxisRotationMatrix(3,Euler(2));

    arma::mat Rtot=rotationMatrix(Rx,Ry,Rz);

    float qw=0.5*sqrt(1+Rtot(0,0)+Rtot(1,1)+Rtot(2,2));
    float qx=(Rtot(2,1)-Rtot(1,2))/4.0/qw;
    float qy=(Rtot(0,2)-Rtot(2,0))/4.0/qw;
    float qz=(Rtot(1,0)-Rtot(0,1))/4.0/qw;
    arma::vec4 quat={qx,qy,qz,qw};

    return quat;
}