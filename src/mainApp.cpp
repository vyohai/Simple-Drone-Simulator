#include "./interface/interface.hpp"


int main()
{
    
    // remove previous checks plots
    std::filesystem::remove_all( std::string(SIMULATION_DIR));
    std::filesystem::create_directory(std::string(SIMULATION_DIR));
    bool finished=0;
    while (finished!=1)
    {
    // **get user input**
    // get simulation name and create a dir
    std::string simulation_dir=getSimulationName();
    
    //simulation duration
    float T=getSimulationDuration();
    
    //Thrusts
    arma::vec4 rottors_velocity=getSimulationRottorsVelocity();

    //get acc and angular_acc
    arma::vec3 euler_initial={0,0,0};
    arma::vec4 quaternion_initial=initialQuaternion(euler_initial);
    initialForcesAndMoments(rottors_velocity, quaternion_initial);  

    // get controller coefficients
    arma::vec3 position_referance={0,0,-5};
    std::array<float,3> K_altitude=tuneAltitudeController(position_referance(2));

    // **the simulation**
    std::array<std::array<float,3>,6> pid_coeff={K_altitude[0], K_altitude[1], K_altitude[2],\
                                                    0.0f, 0.0f, 0.0f,\
                                                    0.0f, 0.0f, 0.0f,\
                                                    0.0f, 0.0f, 0.0f,\
                                                    0.0f, 0.0f, 0.0f,\
                                                    0.0f, 0.0f, 0.0f};
    std::vector<std::array<float,13>> output=droneSimulation(T, rottors_velocity, controller,position_referance,pid_coeff, 0);
    
    //**create plots**
    std::vector<float> altitude=getDataFromSimulation(output,'z');
    std::string plot_name="alt";
    singlePlot(altitude,simulation_dir,plot_name);

    std::vector<float> x=getDataFromSimulation(output,'x');
    plot_name="x";
    singlePlot(x,simulation_dir,plot_name);

    std::vector<float> y=getDataFromSimulation(output,'y');
    plot_name="y";
    singlePlot(y,simulation_dir,plot_name);

    std::vector<float> roll=getDataFromSimulation(output,'r');
    plot_name="roll";
    singlePlot(roll,simulation_dir,plot_name);
    
    std::vector<float> pitch=getDataFromSimulation(output,'p');
    plot_name="pitch";
    singlePlot(pitch,simulation_dir,plot_name);
    
    std::vector<float> yaw=getDataFromSimulation(output,'h');
    plot_name="yaw";
    singlePlot(yaw,simulation_dir,plot_name);

    

    //check if finished
    std::cout << "if you want another simulation type 0"; // Type a number and press enter
    std::cin >> finished; // Get user input from the keyboard
    }
    
    return 1;
}
