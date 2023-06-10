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
    float output=deg*PI_YOH/180.0;
    return output;
}

arma::vec4 quaternionMultiplication(arma::vec4 Q1, arma::vec4 Q2)
{
    //Q=[qx,qy,qz,qw]
    float qw1=Q1[3];
    float qx1=Q1[0];
    float qy1=Q1[1];
    float qz1=Q1[2];
    
    float qw2=Q2[3];
    float qx2=Q2[0];
    float qy2=Q2[1];
    float qz2=Q2[2];
    
    arma::vec4 Q;

    float qw=qw1*qw2-qx1*qx2-qy1*qy2-qz1*qz2;
    float qx=qw1*qx2+qx1*qw2+qy1*qz2-qz1*qy2;
    float qy=qw1*qy2-qx1*qz2+qy1*qw2+qz1*qx2;
    float qz=qw1*qz2+qx1*qy2-qy1*qx2+qz1*qw2;

    Q[3]=qw;
    Q[0]=qx;
    Q[1]=qy;
    Q[2]=qz;
    
    return Q;
}

arma::vec3 quaternionVectorRotation(arma::vec4 Q1, arma::vec3 vec)
{
    arma::vec4 Q1_conj={-Q1(0), -Q1(1), -Q1(2), Q1(3)};

    arma::vec4 vec_as_quaternion={vec(0), vec(1), vec(2), 0};

    arma::vec4 vec_as_quaternion_rotated= quaternionMultiplication(quaternionMultiplication(Q1,vec_as_quaternion),Q1_conj);

    arma::vec3 vec_rotated={vec_as_quaternion_rotated(0), vec_as_quaternion_rotated(1), vec_as_quaternion_rotated(2)};
    
    return vec_rotated;
}


arma::vec3 quaternionToEuler(arma::vec4 Q)
{
    float qx=Q[0];
    float qy=Q[1];
    float qz=Q[2];
    float qw=Q[3];

   arma::mat R={{pow(qw,2)+pow(qx,2)-pow(qy,2)-pow(qz,2),2*(qx*qy-qw*qz),2*(qx*qz+qw*qy)},
                {2*(qx*qy+qw*qz), pow(qw,2)-pow(qx,2)+pow(qy,2)-pow(qz,2),2*(qy*qz-qw*qx)},
                {2*(qx*qz-qw*qy),2*(qy*qz+qw*qx),pow(qw,2)-pow(qx,2)-pow(qy,2)+pow(qz,2)}};

    float pitch=-asin(R(2,0));
    float roll=atan2(R(2,1)/cos(pitch),R(2,2)/cos(pitch));
    float yaw=atan2(R(1,0)/cos(pitch),R(0,0)/cos(pitch));

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