#include <armadillo>
#include "../src/dynamics/forcesAndMoments/forcesAndMoments.hpp"
#include "../src/dynamics/framesConversion/framesConversion.hpp"
#include "CppUTest/TestHarness.h"
// #include "CppUTest/"

#include <iostream>


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


TEST_GROUP(dynamicFramesConersionrotationMatrixTestGroup)
{
    void setup()
    {

    }

    void teardown()
    {

    }
};


TEST(dynamicFramesConersionrotationMatrixTestGroup, wrongAxisExceptionTest)
{
    // try {
    //     arma::mat A = rotationMatrix(4,30.0);
    // } 
    // catch (const char* msg)
    // {
    //     STRCMP_EQUAL("axis is wrong!", msg);
    // }
    
}

TEST(dynamicFramesConersionrotationMatrixTestGroup, corretOutput)
{
    
    arma::mat Rz=singleAxisRotationMatrix(3,30.0);
    arma::mat Ry=singleAxisRotationMatrix(2,30.0);
    arma::mat Rx=singleAxisRotationMatrix(1,30.0);

    arma::vec3 vector_body_frame={0,0,1};

    arma::mat Rtotal=rotationMatrix(Rx,Ry,Rz);

    arma::vec3 vector_world_frame=Rtotal*vector_body_frame;
    
    arma::vec3 expected_vector={0.625,-0.2165,0.75};
    std::cout<<"expected is"<<expected_vector<<" and the calc is "<<vector_world_frame<<std::endl;
    CHECK_TRUE(arma::approx_equal(vector_world_frame,expected_vector,"absdiff",0.001));
}