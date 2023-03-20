#include <Arduino.h>


float getCtrl(float setpoint, float pos, float vel, float err_sum);

float Kp = 8;
float Kd = 1;
float Ki = 0;


/*
    - reads desired steering angle from higher level control
    - calculates output from -1 to 1 (max left to max right command to motor in terms of motor speed)
*/ 


// const float SOME_VAR = 0;

// struct CtrlGains

// {
//     float Kp = 8;
//     float Kd = 1;
//     float Ki = 0;
// };


// class CtrlModule : public BaseModule
// {
//     public:
//     /**
//      * @brief Constructor for the CtrlModule class
//      *
//      * This constructor takes in the pin number for the sensor and initializes
//      * the `pin` member variable.
//      *
//      * @param pin Pin number for the sensor.
//      */
//    CtrlModule(float *output, float *pos, float *vel );  // how send pointer to output parameter?

//     /**
//      * @brief Sets up controls for desired actuator, giving output from max neg to max pos (-1 to 1)
//      *
//      * This function takes 
//      *
//      * @return A Status indicating success or failure.
//      */
//     Status setup();

//     /**
//      * @brief Continuously reads data from the sensor
//      *
//      * This function continuously reads data from the sensor and updates the
//      * `currentAngle` member variable.
//      *
//      * @return A Status indicating success or failure.
//      */
//     Status loop();

//     /**
//      * @brief Cleans up after the sensor
//      *
//      * This function cleans up any resources used by the sensor.
//      *
//      * @return A Status indicating success or failure.
//      */
//     Status cleanup();

//     /**
//      * @brief Gets the current steering angle
//      *
//      * This function returns the current steering angle read from the sensor.
//      *
//      * @return The current steering angle.
//      */
//     float getSteeringAngle();

// }
