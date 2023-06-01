#include <string>
#include <iostream>
#include <cmath>
#include <vector>
#include <array>

// #include <armadillo>

#include "updateEquations.hpp"

#include "../config.hpp"
#include "../dynamics/framesConversion/framesConversion.hpp"
#include "../dynamics/forcesAndMoments/forcesAndMoments.hpp"

std::array<float, 13> singleTimeUpdate(arma::vec4 quat_before, arma::vec3 position_before, arma::vec3 velocty_before , arma::vec3 angular_velocty_before, arma::vec4 rotors_velocity)
{
    
    // calculate Moments
    arma::vec2 Mxy=MomentsXY(rotors_velocity);
    float Mz=MomentZ(rotors_velocity);
    arma::vec3 Moments={Mxy(0), Mxy(1),Mz};

    // calculate angular acceleartion
    arma::vec3 angular_acc=angularAcceleration(Moments, angular_velocty_before);
    
    // update angular velocity
    arma::vec3 angular_velocty_updated=angular_velocty_before+DT*angular_acc;

    // update quaternion
    arma::vec4 quat_updated=updateOrientation(quat_before,angular_velocty_before);
        
      // calculate linear acceleration
    arma::vec3 linear_acc=linearAcceleration(quat_updated,rotors_velocity);

    // update linear velocity
    arma::vec3 velocty_updated=velocty_before+DT*linear_acc;

    // update positions
    arma::vec3 position_updated=position_before+DT*velocty_before;


    // accumelate all to one container
    std::array<float, 13> output;
    output[0]=quat_updated(0);
    output[1]=quat_updated(1);
    output[2]=quat_updated(2);
    output[3]=quat_updated(3);
    output[4]=position_updated(0);
    output[5]=position_updated(1);
    output[6]=position_updated(2);
    output[7]=velocty_updated(0);
    output[8]=velocty_updated(1);
    output[9]=velocty_updated(2);
    output[10]=angular_velocty_updated(0);
    output[11]=angular_velocty_updated(1);
    output[12]=angular_velocty_updated(2);

    return output;
}


std::vector<std::array<float,13>> droneSimulation(float T, arma::vec4 rottors_speed_initial, std::function<arma::vec4(arma::vec3,std::array<float,13>,std::array<float,13>,std::array<std::array<float,3>,6>, float *)> controller, arma::vec3 referance, std::array<std::array<float,3>,6> controllers_coefficients)
{
  //define output
  std::vector<std::array<float,13>> output;

  //definer roll and pitch and yaw referanc shold be ereased
  arma::vec3 euler_ref={0,0,0};
  
  arma::vec4 rottors_speed=rottors_speed_initial;
  
  //define initial condition
  arma::vec3 euler_angles_initial={0,0,0};
  arma::vec4 quaternion_initial=initialQuaternion(euler_angles_initial);

  arma::vec3 position_initial={0,0,0};
  arma::vec3 velocity_initial={0,0,0};
  arma::vec3 angular_velocity_initial={0,0,0};

  arma::vec4 quaternion_previous=quaternion_initial;
  arma::vec3 position_previous=position_initial;
  arma::vec3 velocity_previous=velocity_initial;
  arma::vec3 angular_velocity_previous=angular_velocity_initial;

  
  float Integral_alt=0;
  // float Integral_x=0;
  // float Integral_y=0;
  // float Integral_roll=0;
  // float Integral_pitch=0;
  // float Integral_yaw=0;


  float t=0.0;
  int it=0;
  while (t<T)
  {

    std::array<float,13> new_state=singleTimeUpdate(quaternion_previous,\
                                      position_previous,\
                                      velocity_previous,\
                                      angular_velocity_previous,\
                                      rottors_speed);
    if (it>0)
    {
      // error calculation:x,y,z,roll,pitch,yaw
      arma::vec3 euler_error=convertStateVectorForController(new_state,'e')-euler_ref;
      arma::vec3 pos_error=convertStateVectorForController(new_state,'p')-referance;
      
      //get contrrol command
      rottors_speed=controller(referance,new_state,output[it-1],\
                                controllers_coefficients,&Integral_alt);
                       
    }
                                      
    quaternion_previous={new_state[0],new_state[1],new_state[2],new_state[3]};
    position_previous={new_state[4],new_state[5],new_state[6]};
    velocity_previous={new_state[7],new_state[8],new_state[9]};
    angular_velocity_previous={new_state[10],new_state[11],new_state[12]};

    output.push_back(new_state);
 
  
  t=t+DT;
  it=it+1;
  }
  

  return output;
}

arma::vec4 updateOrientation(arma::vec4 quaternion_before, arma::vec3 angular_rate)
{
    arma::vec4 S={angular_rate(0),angular_rate(1),angular_rate(2),0};

    arma::vec4 dQ=0.5*quaternionMultiplication(quaternion_before,S);

    arma::vec4 Q_updated=quaternion_before+dQ*DT; 
 
    return Q_updated;
}


arma::vec3 convertStateVectorForController(std::array<float,13> state_vector, char type)
{
  arma::vec3 output={-1,-1,-1};

  if (type=='e')
  {
    arma::vec4 quat={state_vector[0], state_vector[1], state_vector[2], state_vector[3]};
    output=quaternionToEuler(quat);  
  }
  else if (type=='p')
  {
    output={state_vector[4], state_vector[5],state_vector[6]};
  }
  else
  {
    std::cout<<"wrong char* in updateEquations.convertStateVectorForController"<<std::endl;
    return {-1,-1,-1};
  }
  
  return output;
  
}