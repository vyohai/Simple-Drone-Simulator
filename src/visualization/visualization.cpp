#include <string>
#include <iostream>
#include <cmath>
#include <vector>
#include <array>
#include <filesystem>
#include <algorithm>

#include <sciplot/sciplot.hpp>

#include "visualization.hpp"
#include "../config.hpp"
#include "../dynamics/framesConversion/framesConversion.hpp"


void singlePlot(std::vector<float> vector, std::string path, std::string name)
{
    sciplot::Vec x = sciplot::linspace(0.0, vector.size()*DT, vector.size());
    sciplot::Plot2D plot;
    plot.xlabel("time");
    plot.ylabel(name);

    // Set the x and y ranges
    plot.xrange(0.0,  vector.size()*DT);
    auto biggest = std::max_element(std::begin(vector), std::end(vector));
    auto smallest = std::min_element(std::begin(vector), std::end(vector));
    if (*biggest==*smallest)
    {
        plot.yrange(*smallest-1, *biggest+1);    
    }
    else
    {
        plot.yrange(*smallest, *biggest);
    }
    plot.drawCurve(x, vector);
    sciplot::Figure fig = {{plot}};
    // Create canvas to hold figure
    sciplot::Canvas canvas = {{fig}};

    

    // Save the plot to a PDF file
    std::string finall=".pdf";
    std::string name2=name+finall;
    // std::cout<<name2<<std::endl;
    
    canvas.save(name2);
    std::filesystem::copy(name2, path+name2);
    std::filesystem::remove(name2);

}

std::vector<float> getDataFromSimulation(std::vector<std::array<float,13>> vector, char type)
{
    std::vector<float> dataVector;
    int array_index=-1;
    bool position=0;
    if (type=='z')
    {
        array_index=6;
        position=1;
    }
    if (type=='x')
    {
        array_index=4;
        position=1;
    }
    if (type=='y')
    {
        array_index=5;
        position=1;
    }
    if (type=='r')
    {
        array_index=0;
    }
    if (type=='p')
    {
        array_index=1;
    }
    if (type=='h')
    {
        array_index=2;
    }

    if (position)
    {
        for (size_t i = 0; i < vector.size(); i++)
        {
            std::array singleTimeArray=vector[i];
            dataVector.push_back(singleTimeArray[array_index]);
        }
    }
    else
    {
        for (size_t i = 0; i < vector.size(); i++)
        {
            std::array singleTimeArray=vector[i];
            arma::vec4 quaternion= {singleTimeArray[0],\
                                     singleTimeArray[1],\
                                     singleTimeArray[2],\
                                     singleTimeArray[3]};
            arma::vec3 euler=quaternionToEuler(quaternion);
            dataVector.push_back(euler(array_index));
        }
    }
       
    return dataVector;
}

