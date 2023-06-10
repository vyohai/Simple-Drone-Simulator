#include <matplot/matplot.h>
#include "visualization.hpp"



void singlePlot(std::vector<float> myVector, std::string myPath, std::string myName)
{
    
    std::vector<double> x = matplot::linspace(0.0, myVector.size()*DT, myVector.size());
    matplot::plot(x,myVector);
    matplot::title(myName.c_str());
    matplot::xlabel("time");
    matplot::ylabel(myName);

    // Set the x and y ranges
    // matplot::xlim({0.0,  myVector.size()*DT});
    auto biggest = std::max_element(std::begin(myVector), std::end(myVector));
    auto smallest = std::min_element(std::begin(myVector), std::end(myVector));
    if (*biggest==*smallest)
    {
        matplot::ylim({*smallest-1, *biggest+1});  
    }
    else
    {
        matplot::ylim({*smallest, *biggest});  
    }
    
    // Save the plot to a PDF file
    std::string finall=".jpeg";
    std::string name2=myName+finall;
    std::cout<<myPath+name2<<std::endl;
    // matplot::show();
    matplot::save(myPath+name2);
    // std::filesystem::copy(name2, myPath+name2);
    // std::filesystem::remove(name2);

}



