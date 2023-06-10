#include <iostream>
#include <array>
#include <cmath>

#include <armadillo>

// #include "../src/dynamics/forcesAndMoments/forcesAndMoments.hpp"
// #include "../src/dynamics/framesConversion/framesConversion.hpp"
// #include "../src/updateEquations/updateEquations.hpp"
#include "../src/controller/controller.hpp"
#include "../src/controllerTuner/controllerTuner.hpp"
// #include "../src/visualization/visualization.hpp"
#include "../src/config.hpp"

#include "CppUTest/TestHarness.h"


TEST_GROUP(evaluate_resaultsTestGroup)
{
    
};

TEST(evaluate_resaultsTestGroup, correctOutput)
{
    
    float referance=-4.0;
    std::vector<float> system_output;
    system_output.push_back(-7.0);
    system_output.push_back(1.0);
    system_output.push_back(-4.0);
    system_output.push_back(-3.0);
    float resault=evaluate_resaults(referance, system_output,2.0, -12.0);
    float expected_resalut=(abs(referance-system_output[0])+abs(referance-system_output[1])+\
                            abs(referance-system_output[2])+abs(referance-system_output[3]))/system_output.size();
    
    CHECK_TRUE(expected_resalut==resault);
}

TEST(evaluate_resaultsTestGroup, correctOutputTooMuchOvershoot)
{
    //output starts lower than referance
    float referance=-5.0;
    std::vector<float> system_output;
    system_output.push_back(-11.0);
    system_output.push_back(7.0);
    float resault=evaluate_resaults(referance, system_output,8.0, -12.0);
    CHECK_TRUE(resault==10000.0);

    //output starts bigger than referance
    std::vector<float> system_output2;
    system_output2.push_back(1.0);
    system_output2.push_back(-17.0);
    resault=evaluate_resaults(referance, system_output,2.0, -18.0);
    CHECK_TRUE(resault==10000.0);

}

TEST(evaluate_resaultsTestGroup, correctOutputBoundaries)
{
    
   float referance=-5.0;
    std::vector<float> system_output;
    system_output.push_back(0.0);
    system_output.push_back(-7.0);
    system_output.push_back(1.0);
    float resault=evaluate_resaults(referance, system_output,0.0, -12.0);
    CHECK_TRUE(resault==10000.0);


    std::vector<float> system_output2;
    system_output2.push_back(0.0);
    system_output2.push_back(-13.0);
    resault=evaluate_resaults(referance, system_output,0.0, -12.0);
    CHECK_TRUE(resault==10000.0);
}

TEST_GROUP(create_parameter_gridTestGroup)
{
    
};

TEST(create_parameter_gridTestGroup, correctOutput)
{
    
    std::array<float,6> coefficient_boundaries={0.0,3.0,0.0,1.0,0.0,1.0};
    int grid_size=50;
    std::vector<std::array<float,3>> grid=create_parameter_grid(coefficient_boundaries, grid_size);    
    
    bool ALL_IN_BOUNDERIES=true;

    for (size_t i = 0; i < grid_size; i++)
    {
        if (grid[i][0]>coefficient_boundaries[1] || grid[i][0]<coefficient_boundaries[0])
        {
            ALL_IN_BOUNDERIES=false;
        }
        if (grid[i][1]>coefficient_boundaries[3] || grid[i][1]<coefficient_boundaries[2])
        {
            ALL_IN_BOUNDERIES=false;
        }
        if (grid[i][2]>coefficient_boundaries[5] || grid[i][2]<coefficient_boundaries[4])
        {
            ALL_IN_BOUNDERIES=false;
        }
    }
    
    CHECK_TRUE(grid.size()==grid_size); 
    CHECK_TRUE(ALL_IN_BOUNDERIES);
}


// TEST_GROUP(search_controller_parametersTestGroup)
// {
    
// };

// TEST(search_controller_parametersTestGroup, correctOutput)
// {

//     std::array<float,6> coefficient_boundaries={0.0,0.5,0.0,0.1,0.0,10.0};
//     int grid_size=250;
//     float referance=-5.0;
//     std::array<float,3> res=search_controller_parameters(coefficient_boundaries, grid_size, referance);

// }

