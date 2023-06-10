#include "controller.hpp"



arma::vec4 controller(arma::vec3 referance,std::array<float,13> current_step,std::array<float,13> previous_step, std::array<std::array<float,3>,6> controllers_coefficients, float *Integral_altitude)
{
    
    // get position and euler angles
    arma::vec3 euler_ref={0,0,0};
    arma::vec3 euler_current=convertStateVectorForController(current_step,'e')-euler_ref;
    arma::vec3 pos_current=convertStateVectorForController(current_step,'p')-referance;

    arma::vec3 euler_previous=convertStateVectorForController(previous_step,'e')-euler_ref;
    arma::vec3 pos_previous=convertStateVectorForController(previous_step,'p')-referance;



    //altitude PID
    float altitude_cmd=PID(pos_previous(2),pos_current(2),*Integral_altitude ,controllers_coefficients[0],referance(2));
    *Integral_altitude=*Integral_altitude+pos_current(2)*DT;
    float roll_cmd=0;
    float pitch_cmd=0;
    float yaw_cmd=0;
    // float altitude_cmd=0;

    arma::vec4 rottors_speed=mixer(altitude_cmd,yaw_cmd,pitch_cmd,roll_cmd);
    return rottors_speed;
}

arma::vec4 mixer(float thrust_cmd, float yaw_cmd, float pitch_cmd,float roll_cmd)
{
    arma::vec4 output={0,0,0,0};

    output(0)=HOVER_THRUST+thrust_cmd+yaw_cmd+pitch_cmd+roll_cmd;
    output(1)=HOVER_THRUST+thrust_cmd-yaw_cmd+pitch_cmd-roll_cmd;
    output(2)=HOVER_THRUST+thrust_cmd+yaw_cmd-pitch_cmd-roll_cmd;
    output(3)=HOVER_THRUST+thrust_cmd-yaw_cmd-pitch_cmd+roll_cmd;

    //this beacause the commands is responsible for absolute velocity and not for direction.(+/-) 
    for (size_t i = 0; i < 4; i++)
    {
        if (output(i)<0)
        {
            output(i)=0;
        }
    }

    return output;
}

float PID(float state_before,float state_current, float Integral,std::array<float,3> pid_coefficients,float referance)
{
    float current_error=referance-state_current;
    float previous_error=referance-state_before;
    
    //proportional part
    float P_term=-pid_coefficients[0]*current_error;

    //integral part
    Integral=Integral+current_error*DT;
    float I_term=pid_coefficients[1]*Integral;


    //difrrential part
    float derivative=(current_error-previous_error);
    float D_term=pid_coefficients[2]*derivative;

    
    
    return (P_term + I_term - D_term);
}