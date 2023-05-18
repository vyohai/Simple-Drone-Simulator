#include "CppUTest/TestHarness.h"
# include <string>
#include "../src/controller.hpp"

TEST_GROUP(controllerTestGroup)
{
};

TEST(controllerTestGroup, FirstTest)
{
   FAIL("Fail me!");
}

TEST(controllerTestGroup, SecondTest)
{
    std::string Test=myFunctionC(0);
    std::string True="Co";
    CHECK(Test==True);
}