#include <armadillo>
#include "../src/dynamics/forcesAndMoments/forcesAndMoments.hpp"
#include "../src/dynamics/framesConversion/framesConversion.hpp"
#include "CppUTest/TestHarness.h"

#include <iostream>
// #include <armadillo>
// #include "../src/dynamics/forcesAndMoments/forcesAndMoments.hpp"
// #include "../src/dynamics/framesConversion/framesConversion.hpp"

TEST_GROUP(dynamicTestGroup)
{
    void setup()
    {

    }

    void teardown()
    {

    }
};


TEST(dynamicTestGroup, ConversionTest)
{

    arma::mat b=singleAxisRotationMatrix(1,1.0);
    CHECK(1==1);
}

TEST(dynamicTestGroup, ForcesTest)
{
    
    CHECK(1.0==rottorThrust(1.0, 1.0));
    //CHECK(1.0==1.0);
}