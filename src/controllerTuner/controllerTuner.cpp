#include "controllerTuner.hpp"


float evaluate_resaults(float referance, std::vector<float> system_output, float max_boundery,float min_boundery)
{
    // check for undershoot and overshoot
    float max_output=*std::max_element(system_output.begin(), system_output.end());
    float min_output=*std::min_element(system_output.begin(), system_output.end());

    //check for boundary cross
    if (max_output > max_boundery || min_output < min_boundery)
    {
        return 10000.0;
    }
    
    // check for overshoot
    //// if the output began smaller than the refarance and go up
    if (referance>system_output[0])
    {
        if (max_output>referance+11)
        {
            return 10000.0;
        }    
    }
    //// if the output began bigger than the refarance and go down
    else
    {
        if (min_output<referance-11)
        {
            return 10000.0;
        } 
    }
    
    // calculate normelized rms
    float normelized_rms=0.0;
    for (size_t i = 0; i < system_output.size(); i++)
    {
        normelized_rms=normelized_rms + abs(system_output[i]-referance);
    }
    
    return normelized_rms/system_output.size();
}

std::vector<std::array<float,3>> create_parameter_grid(std::array<float,6> coefficient_boundaries, int grid_size )
{
    std::vector<std::array<float,3>> grid;
    
    std::random_device rd;  // a seed source for the random number engine
    std::mt19937 gen(rd()); // mersenne_twister_engine seeded with rd()
    std::uniform_real_distribution<float> KPdistrib( coefficient_boundaries[0], coefficient_boundaries[1]);
    std::uniform_real_distribution<float> KIdistrib( coefficient_boundaries[2], coefficient_boundaries[3]);
    std::uniform_real_distribution<float> KDdistrib( coefficient_boundaries[4], coefficient_boundaries[5]);
    
    std::array<float,3> PID;
    for (int i = 0; i < grid_size; i++)
    {
        PID={KPdistrib(gen), KIdistrib(gen),KDdistrib(gen)};
        grid.push_back(PID);
    }
    return grid;
}

std::array<float,3> search_controller_parameters(std::array<float,6> coefficient_boundaries, int grid_size, float referance)
{
    arma::vec4 rottors_speed_initial={HOVER_THRUST,HOVER_THRUST,HOVER_THRUST,HOVER_THRUST};
    float T=TUNING_SIMULATION_DURATION;

    // create a grid of controller parameters
    std::vector<std::array<float,3>> grid=create_parameter_grid(coefficient_boundaries, grid_size);

    // iterarte and run simulations and evaluate them
    float min_resault=100;
    int i_min=0;
    std::array<float,3> PID_min={0,0,0};

    for (size_t i = 0; i < grid_size; i++)
    {
        std::array<float,3> PID=grid[i];
        std::array<std::array<float,3>,6> controllers_coefficients={{PID,\
                                                                    {0,0,0},\
                                                                    {0,0,0},\
                                                                    {0,0,0},\
                                                                    {0,0,0},\
                                                                    {0,0,0}}};

        std::vector<std::array<float,13>> simulation_output=droneSimulation(T, rottors_speed_initial,  controller,\
        {0,0,referance},  controllers_coefficients);

        // get altitude from simulation
        std::vector<float> system_output=getDataFromSimulation(simulation_output,'z');
        
        //evaluate
        float max_boundery=0.1;;
        float min_boundery=referance-20;
        float resault=evaluate_resaults(referance, system_output, max_boundery, min_boundery);
        
        
        if (resault<min_resault)
        {
            i_min=i;
            PID_min=PID;
            min_resault=resault;
        }
        
        
    }

    // return best parameters
    return PID_min;

}

