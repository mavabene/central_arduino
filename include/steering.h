#include "pin.h"

const float LEFT_ANGLE = -30.0; // Minimun distance for actuator
const float RIGHT_ANGLE = 30.0; // Maximum safe distance that the actuator can move (inches)
float measureAngle()
{
    float sensorValue = analogRead(STEERING_ANGLE_SENSOR);
    float angle = (sensorValue - 0) / (1023 - 0) * (RIGHT_ANGLE - LEFT_ANGLE) + LEFT_ANGLE;
    return angle;
}


void setupSteeringControl()
{
  // steering
    Sensor *SteeringPot = new Sensor();
    PidLoop *SteeringPID = new PidLoop();
    Actuator *SteeringMotor = new Actuator();
    float angle_dt = 20; // in milliseconds
    int idxSteeringV = 0;
    float dSumSteeringV = 0;
    float dMeanSteeringV = 0;
    float current_angle = 0;
    float start_angle = 0;
    float min_detectable_angle = -45;  // as measureable by encoder
    float max_detectable_angle = 45;
    float steering_angular_vel = 0;
    unsigned long angle_detect_start_time = 0;
    float last_angle = 0;
    unsigned long currentMillis = 0;
    unsigned long startMillis = 0;


}