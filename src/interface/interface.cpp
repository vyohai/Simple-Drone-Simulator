#include "interface.hpp"

std::string getSimulationName()
{
    std::string simulation_name="";
    std::cout << "type your simulation name: "<<std::endl; // Type a number and press enter
    std::cin >> simulation_name; // Get user input from the keyboard
    std::string dirName = SIMULATION_DIR; 
    std::string simulation_dir=dirName+simulation_name+"/";
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


std::array<float,3> tuneAltitudeController(float referance)
{
    
    float kp=0;
    float ki=0;
    float kd=0;
    bool autotune=false;
    std::cout<<"if you want automatitic tuning type 1 , for manually tuning type 0"<<std::endl;
    std::cin >> autotune; // Get user input from the keyboard
    if (autotune)
    {
        std::array<float,6> coefficient_boundaries;
        int grid_size;
        std::cout <<std::endl<<"Type nomber of triplets(p,i,d) to evaluate: "; // Type a number and press enter
        std::cin >> grid_size; // Get user input from the keyboard
        std::cout <<std::endl<<"Type minimum for Kp: "; // Type a number and press enter
        std::cin >> coefficient_boundaries[0]; // Get user input from the keyboard
        std::cout <<std::endl<<"Type maximum for Kp: "; // Type a number and press enter
        std::cin >> coefficient_boundaries[1]; // Get user input from the keyboard
        std::cout <<std::endl<<"Type minimum for Ki: "; // Type a number and press enter
        std::cin >> coefficient_boundaries[2]; // Get user input from the keyboard
        std::cout <<std::endl<<"Type maximum for Ki: "; // Type a number and press enter
        std::cin >> coefficient_boundaries[3]; // Get user input from the keyboard
        std::cout <<std::endl<<"Type minimum for Kd: "; // Type a number and press enter
        std::cin >> coefficient_boundaries[4]; // Get user input from the keyboard
        std::cout <<std::endl<<"Type maximum for Kd: "; // Type a number and press enter
        std::cin >> coefficient_boundaries[5]; // Get user input from the keyboard
        std::array<float,3> PID_array=search_controller_parameters(coefficient_boundaries, grid_size, referance);
        kp=PID_array[0];
        ki=PID_array[1];
        kd=PID_array[2];
    }
    else
    {
        std::cout<<"if you want to tune the altitude controller manually : type numbers diffrent than 0"<<std::endl;
        std::cout<<"if you type all K's 0 the drone will get command of {1.5811, 1.5811, 1.5811, 1.5811} from the 2nd step."<<std::endl;
        std::cout <<std::endl<<"Type Kp: "; // Type a number and press enter
        std::cin >> kp; // Get user input from the keyboard
        std::cout <<std::endl<<"Type Ki: "; // Type a number and press enter
        std::cin >> ki; // Get user input from the keyboard
        std::cout <<std::endl<<"Type Kd: "; // Type a number and press enter
        std::cin >> kd; // Get user input from the keyboard
    }

    std::array<float,3> altitude_controller_coeff={kp, ki, kd};
    std::cout << "altitude_controller_coefficients: {" <<std::endl<<\
             altitude_controller_coeff[0]<<", "<<altitude_controller_coeff[1]<<\
             ", "<<altitude_controller_coeff[2]<<"} "<<std::endl;; // Display the input value

    
    return altitude_controller_coeff;

}

std::vector<float> getDataFromSimulation(std::vector<std::array<float,13>> fullVector, char myType)
{
    std::vector<float> dataVector;
    int array_index=-1;
    bool position=0;
    if (myType=='z')
    {
        array_index=6;
        position=1;
    }
    if (myType=='x')
    {
        array_index=4;
        position=1;
    }
    if (myType=='y')
    {
        array_index=5;
        position=1;
    }
    if (myType=='r')
    {
        array_index=0;
    }
    if (myType=='p')
    {
        array_index=1;
    }
    if (myType=='h')
    {
        array_index=2;
    }

    if (position)
    {
        for (size_t i = 0; i < fullVector.size(); i++)
        {
            std::array singleTimeArray=fullVector[i];
            dataVector.push_back(singleTimeArray[array_index]);
        }
    }
    else
    {
        for (size_t i = 0; i < fullVector.size(); i++)
        {
            std::array singleTimeArray=fullVector[i];
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