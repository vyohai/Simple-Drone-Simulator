#include <vector>
#include <iostream>

#include <armadillo>

#include "../src/dynamics/forcesAndMoments/forcesAndMoments.hpp"
#include "../src/dynamics/framesConversion/framesConversion.hpp"


#include "CppUTest/TestHarness.h"




TEST_GROUP(dynamicFramesConersionSingleAxisRotationMatrixTestGroup)
{
    void setup()
    {

    }

    void teardown()
    {

    }
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
    void setup()
    {

    }

    void teardown()
    {

    }
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
    void setup()
    {

    }

    void teardown()
    {

    }
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
    void setup()
    {

    }

    void teardown()
    {

    }
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
    void setup()
    {

    }

    void teardown()
    {

    }
};

TEST(dynamicFramesConersionInitialQuaternionTestGroup, correctOutput)
{
    arma::vec4 Q1_expected={0.17677,0.30618,0.17677,0.91855};
    
    arma::vec3 angles_wanted={30.0,30.0,30.0};

    arma::vec4 Q1=initialQuaternion(angles_wanted);

    CHECK_TRUE(arma::approx_equal(Q1,Q1_expected,"absdiff",0.1));
}