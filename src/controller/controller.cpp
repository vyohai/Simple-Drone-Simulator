#include <array>
#include <armadillo>

#include "controller.hpp"

arma::vec4 controller(std::array<float,13> current_step, std::array<float,13> previous_step)
{
    arma::vec4 rottors_speed={0,0,0,0};
    return rottors_speed;

}

arma::vec4 mixer(float thrust_cmd, float yaw_cmd, float pitch_cmd,float roll_cmd)
{
    arma::vec4 output={0,0,0,0};

    output(0)=thrust_cmd+yaw_cmd+pitch_cmd+roll_cmd;
    output(1)=thrust_cmd-yaw_cmd+pitch_cmd-roll_cmd;
    output(2)=thrust_cmd+yaw_cmd-pitch_cmd-roll_cmd;
    output(3)=thrust_cmd-yaw_cmd-pitch_cmd+roll_cmd;

    //this beacause the commands is responsible for absolute velocity and not for direction.(+/-) 
    float min_output=min(output);
    if(min_output<0)
    {
        output=output-min_output;
    }

    return output;
}