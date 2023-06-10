#include "sensors.hpp"

float gaussianWhiteNoise()
{
    std::random_device rd;
    std::mt19937 gen(rd());
 
    // values near the mean are the most likely
    // standard deviation affects the dispersion of generated values from the mean
    std::normal_distribution<> d(0.0, std::sqrt(ALTITUDE_SENSOR_VARIANCE));
    
    return d(gen);
}


std::array<float,13> sensorModel(std::array<float,13> real_state)
{
    std::array<float,4> quaternion={real_state[0], real_state[1], real_state[2], real_state[3]};
    std::array<float,3> position={real_state[4], real_state[5], real_state[6]};
    std::array<float,3> velocity={real_state[7], real_state[8], real_state[9]};
    std::array<float,3> angular_velocity={real_state[10], real_state[11], real_state[12]};

    position[2]=position[2]+gaussianWhiteNoise();

    std::array<float,13> sensor_state={quaternion[0],quaternion[1],quaternion[2],quaternion[3],\
                                        position[0], position[1], position[2],\
                                        velocity[0], velocity[1], velocity[2],\
                                        angular_velocity[0], angular_velocity[1], angular_velocity[2]};

    return sensor_state;
}