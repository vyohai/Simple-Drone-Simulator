#include <random>
#include <cmath>
#include <array>
#include <vector>

float gaussianWhiteNoise(float variance)
{
    std::default_random_engine generator;
    std::normal_distribution<float> dist(0.0, std::sqrt(variance));
    return dist(generator);
}


std::array<float,13> sensorModel(std::array<float,13> real_state, float variance)
{
    std::array<float,4> quaternion={real_state[0], real_state[1], real_state[2], real_state[3]};
    std::array<float,3> position={real_state[4], real_state[5], real_state[6]};
    std::array<float,3> velocity={real_state[7], real_state[8], real_state[9]};
    std::array<float,3> angular_velocity={real_state[10], real_state[11], real_state[12]};

    // position[2]=position[2]+gaussianWhiteNoise(variance);

    std::array<float,13> sensor_state={quaternion[0],quaternion[1],quaternion[2],quaternion[3],\
                                        position[0], position[1], position[2],\
                                        velocity[0], velocity[1], velocity[2],\
                                        angular_velocity[0], angular_velocity[1], angular_velocity[2]};

    return sensor_state;
}