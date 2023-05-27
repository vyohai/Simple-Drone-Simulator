#include <string>
#include <iostream>
#include<filesystem>
#include <direct.h>

#include <armadillo>

#include "../config.hpp"
#include "../dynamics/framesConversion/framesConversion.hpp"
#include "../dynamics/forcesAndMoments/forcesAndMoments.hpp"

std::string getSimulationName()
{
    std::string simulation_name="";
    std::cout << "type your simulation name: "<<std::endl; // Type a number and press enter
    std::cin >> simulation_name; // Get user input from the keyboard
    std::string dirName = "C:\\Users\\User\\Desktop\\GITHUB-REPOS\\Simple-Drone-Simulator\\simulations\\"; 
    std::string simulation_dir=dirName+simulation_name+"\\";
    const char * c = simulation_dir.c_str();
    _mkdir(c);

    return simulation_dir;
}

float getSimulationDuration()
{
    float T; 
    std::cout << "Type a duration: "; // Type a number and press enter
    std::cin >> T; // Get user input from the keyboard

    return T;
}


arma::vec4 getSimulationRottorsVelocity()
{
    arma::vec4 rottors_velocity={0,0,0,0};
    std::cout<<"the Hover-thrust is "<<HOVER_THRUST<<std::endl;
    std::cout <<std::endl<<"Type thrust for thruster1(forward-left): "; // Type a number and press enter
    std::cin >> rottors_velocity(0); // Get user input from the keyboard
    std::cout <<std::endl<<"Type thrust for thruster2(forward-right): "; // Type a number and press enter
    std::cin >> rottors_velocity(1); // Get user input from the keyboard
    std::cout <<std::endl<<"Type thrust for thruster3(backward-right): "; // Type a number and press enter
    std::cin >> rottors_velocity(2); // Get user input from the keyboard
    std::cout <<std::endl<<"Type thrust for thruster4(backward-left): "; // Type a number and press enter
    std::cin >> rottors_velocity(3); // Get user input from the keyboard
    std::cout << "rottors_velocity: " <<std::endl<< rottors_velocity<<std::endl;; // Display the input value

    return rottors_velocity;
}


void initialForcesAndMoments(arma::vec4 rottors_velocity, arma::vec4 quaternion_initial)
{
    arma::vec3 acc=linearAcceleration(quaternion_initial,rottors_velocity);
    arma::vec2 Mxy=MomentsXY(rottors_velocity);
    float Mz=MomentZ(rottors_velocity);
    arma::vec3 Moments={Mxy(0), Mxy(1),Mz};
    arma::vec ang_acc=angularAcceleration(Moments,{0,0,0});
    std::cout<<std::endl<<"acc={"<<acc(0)<<","<<acc(1)<<","<<acc(2)<<"}, ";
    std::cout<<std::endl<<"Moments={"<<Moments(0)<<","<<Moments(1)<<","<<Moments(2)<<"}, ";
    std::cout<<std::endl<<"angular_acc={"<<ang_acc(0)<<","<<ang_acc(1)<<","<<ang_acc(2)<<"}"<<std::endl;

}


arma::vec3 tuneAltitudeController()
{
    
    arma::vec3 altitude_controller_coeff={0.00000f,0.00000f,0.00000f};
    float kp=0.00000f;
    float ki=0.00000f;
    float kd=0.00000f;
    std::cout<<"if you want to tune the altitude controller: type numbers diffrent than 0"<<std::endl;
    std::cout<<"if you type all K's 0 the drone will get command of {1.5811, 1.5811, 1.5811, 1.5811} from the 2nd step."<<std::endl;
    std::cout <<std::endl<<"Type Kp: "; // Type a number and press enter
    std::cin >> kp; // Get user input from the keyboard
    std::cout <<std::endl<<"Type Ki: "; // Type a number and press enter
    std::cin >> ki; // Get user input from the keyboard
    std::cout <<std::endl<<"Type Kd: "; // Type a number and press enter
    std::cin >> kd; // Get user input from the keyboard

    altitude_controller_coeff={kp, ki, kd};
    std::cout << "altitude_controller_coefficients: " <<std::endl<< altitude_controller_coeff<<std::endl;; // Display the input value

    
    return altitude_controller_coeff;

}