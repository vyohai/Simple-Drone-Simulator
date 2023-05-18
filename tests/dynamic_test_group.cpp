#include "CppUTest/TestHarness.h"
# include <string>
#include <iostream>
#include "../src/dynamics.hpp"

TEST_GROUP(dynamicTestGroup)
{
    void setup()
    {

    }

    void teardown()
    {

    }
};


TEST(dynamicTestGroup, SecondTest)
{
    std::string Test=myFunctiond(0);
    std::string True="dy";
    CHECK(Test==True);
}