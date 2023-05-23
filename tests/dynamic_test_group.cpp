#include <vector>
#include <iostream>

#include <armadillo>

#include "../src/dynamics/forcesAndMoments/forcesAndMoments.hpp"
#include "../src/dynamics/framesConversion/framesConversion.hpp"


#include "CppUTest/TestHarness.h"




TEST_GROUP(dynamicFramesConersionSingleAxisRotationMatrixTestGroup)
{
    
};

TEST(dynamicFramesConersionSingleAxisRotationMatrixTestGroup, wrongAxisExceptionTest)
{
    try {
        arma::mat A = singleAxisRotationMatrix(4,30.0);
    } 
    catch (const char* msg)
    {
        STRCMP_EQUAL("axis is wrong!", msg);
    }
    
}

TEST(dynamicFramesConersionSingleAxisRotationMatrixTestGroup, corretOutput)
{
    
    arma::mat A = singleAxisRotationMatrix(3,0.0);
    arma::mat expectedA={{1,0,0},
                        {0,1,0},
                        {0,0,1}};
   
    CHECK_TRUE(arma::approx_equal(A,expectedA,"absdiff",0.001));
}

TEST_GROUP(dynamicFramesConversionRotationMatrixTestGroup)
{
    
};

TEST(dynamicFramesConversionRotationMatrixTestGroup, corretOutput)
{
    
    arma::mat Rz=singleAxisRotationMatrix(3,30.0);
    arma::mat Ry=singleAxisRotationMatrix(2,30.0);
    arma::mat Rx=singleAxisRotationMatrix(1,30.0);

    arma::vec3 vector_body_frame={0,0,1};

    arma::mat Rtotal=rotationMatrix(Rx,Ry,Rz);

    arma::vec3 vector_world_frame=Rtotal*vector_body_frame;
    
    arma::vec3 expected_vector={0.625,-0.2165,0.75};
    CHECK_TRUE(arma::approx_equal(vector_world_frame,expected_vector,"absdiff",0.001));
}

TEST_GROUP(dynamicFramesConersionQuaternionMultiplicationTestGroup)
{
    
};

TEST(dynamicFramesConersionQuaternionMultiplicationTestGroup, correctOutput)
{
    arma::vec4 Q1={0.17677,0.30618,0.17677,0.91855};
    arma::vec4 Q2={0.0,1.75438,0,0};
    arma::vec4 Q3=quaternionMultiplication(Q1,Q2);
    arma::vec4 Q_expected={-0.5371688, 0.31013455, 1.61150641, -0.31013455};
    CHECK_TRUE(arma::approx_equal(Q3,Q_expected,"absdiff",0.001));
}

TEST_GROUP(dynamicFramesConersionQuaternionToEulerTestGroup)
{
    
};

TEST(dynamicFramesConersionQuaternionToEulerTestGroup, correctOutput)
{
    arma::vec4 Q1={0.17677,0.30618,0.17677,0.91855};

    arma::vec3 angles=quaternionToEuler(Q1);
    
    arma::vec3 angles_expected={30.0,30.0,30.0};
    CHECK_TRUE(arma::approx_equal(angles,angles_expected,"absdiff",2));
}

TEST_GROUP(dynamicFramesConersionInitialQuaternionTestGroup)
{
   
};

TEST(dynamicFramesConersionInitialQuaternionTestGroup, correctOutput)
{
    arma::vec4 Q1_expected={0.17677,0.30618,0.17677,0.91855};
    
    arma::vec3 angles_wanted={30.0,30.0,30.0};

    arma::vec4 Q1=initialQuaternion(angles_wanted);

    CHECK_TRUE(arma::approx_equal(Q1,Q1_expected,"absdiff",0.1));
}

TEST_GROUP(dynamicFramesConersionQuaternionVectorRotationTestGroup)
{
 
};

TEST(dynamicFramesConersionQuaternionVectorRotationTestGroup, correctOutput)
{
    arma::vec4 Q1={0.17677,0.30618,0.17677,0.91855};

    arma::vec3 vector_world_expected={0.625, -0.216, 0.75};
    
    arma::vec3 vector_body={0,0,1};

    arma::vec3 vector_world=quaternionVectorRotation(Q1,vector_body);

    CHECK_TRUE(arma::approx_equal(vector_world,vector_world_expected,"absdiff",0.01));
}

TEST_GROUP(dynamicForcesAndMomentsLinearAccelerationTestGroup)
{
 
};

TEST(dynamicForcesAndMomentsLinearAccelerationTestGroup, correctOutput)
{
    
    arma::vec3 euler={45,0,0};

    arma::vec4 Q1=initialQuaternion(euler);
    // i will give such a rottor speeds that the gravity az=0 and ay=10 
    arma::vec4 thrusts={1.88, 1.88, 1.88, 1.88};
    arma::vec3 a=linearAcceleration(Q1,thrusts);
    
    arma::vec3 a_expected={0., 10.0, 0.0};
    
    CHECK_TRUE(arma::approx_equal(a,a_expected,"absdiff",0.1));
}

TEST_GROUP(dynamicForcesAndMomentsMomentZTestGroup)
{
 
};

TEST(dynamicForcesAndMomentsMomentZTestGroup, correctOutput)
{
    
    arma::vec4 rottor_vel={1,1,1,1};

    float Mz=MomentZ(rottor_vel);

    CHECK_TRUE(Mz==0.0);
}

TEST_GROUP(dynamicForcesAndMomentsMomentsXYTestGroup)
{
 
};

TEST(dynamicForcesAndMomentsMomentsXYTestGroup, correctOutput)
{
    //craete x moment (2*2^2-2*1^2)*L, rottors: 1 and 4 faster
    arma::vec4 rottor_vel_for_Mx={2,1,1,2};
    arma::vec2 Mxyx=MomentsXY(rottor_vel_for_Mx);
    arma::vec2 Mxyx_expected={6.0,0.0};

    //craete y moment (2*2^2-2*1^2)*L, rottors: 1 and 2 faster
    arma::vec4 rottor_vel_for_My={2,2,1,1};
    arma::vec2 Mxyy=MomentsXY(rottor_vel_for_My);
    arma::vec2 Mxyy_expected={0.0,6.0};

    CHECK_TRUE(arma::approx_equal(Mxyx,Mxyx_expected,"absdiff",0.1));
    CHECK_TRUE(arma::approx_equal(Mxyy,Mxyy_expected,"absdiff",0.1));
}



TEST_GROUP(dynamicForcesAndMomentsAngularAccelerationTestGroup)
{
 
};

TEST(dynamicForcesAndMomentsAngularAccelerationTestGroup, correctOutput)
{
    //craete x moment (2*2^2-2*1^2)*L, rottors: 1 and 4 faster
    arma::vec4 rottor_vel_for_Mx={2,1,1,2};
    arma::vec2 Mxyx=MomentsXY(rottor_vel_for_Mx);
    float Mzx=MomentZ(rottor_vel_for_Mx);
    arma::vec3 Mx={Mxyx(0), Mxyx(1),Mzx};
   

    arma::vec3 w2={1.0, 0.0, 1.0};
    arma::vec3 angular_acc=angularAcceleration(Mx,w2);

    arma::vec3 angular_acc_expected={6.0,-1.0,0.0};
    
    CHECK_TRUE(arma::approx_equal(angular_acc,angular_acc_expected,"absdiff",0.1));
}
   