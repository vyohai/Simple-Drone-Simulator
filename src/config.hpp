
#ifndef CONFIG
#define CONFIG

// GENARAL
#define PI_YOH 3.14

// FOR DRONE DYNAMICS
#define DT 0.01 // sec 
#define KF 1.0 //
#define KM 1.0 //
#define M 1.0 // kg 
#define Ixx 1.0 // kg*m^2
#define Iyy 1.0 // kg*m^2
#define Izz 2.0 // kg*m^2 
#define G 10.0 // m/s^2
#define L 1.0 // m
#define ROTTOR_DIR1 1
#define ROTTOR_DIR2 -1
#define ROTTOR_DIR3 1
#define ROTTOR_DIR4 -1
#define HOVER_THRUST 1.5811

// FOR TESTS
#define CHECKS_DIR_FULL "C:\\Users\\User\\Desktop\\GITHUB-REPOS\\Simple-Drone-Simulator\\tests\\checks\\"
#define CHECKS_DIR "tests/checks/"
#define CREATE_TEST_CHECKS_GRAPHS 1

// FOR APP SIMULATIONS
#define SIMULATION_DIR "simulations/"

// TUNING CONTROLLERS
#define TUNING_SIMULATION_DURATION 50.0

// SENSORS NOISES VARIANCES
#define ALTITUDE_SENSOR_VARIANCE 1.5

#endif