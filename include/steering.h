#include "pin.h"

const float LEFT_ANGLE = -30.0; // Minimun distance for actuator
const float RIGHT_ANGLE = 30.0; // Maximum safe distance that the actuator can move (inches)
float measureAngle()
{
    float sensorValue = analogRead(STEERING_ANGLE_SENSOR);
    float angle = (sensorValue - 0) / (1023 - 0) * (RIGHT_ANGLE - LEFT_ANGLE) + LEFT_ANGLE;
    return angle;
}


void setupSteeringControl(Sensor *sens)
{
  // steering
    sens->SensorMin = 350;
    sens->SensorMax = 850;
    sens->MinEngVal = -45;
    sens->MaxEngVal = 45;
    sens->InputPin = STEERING_ANGLE_SENSOR;
    //SteeringPot->InputVarAddress = &steering_command;
    sens->dt = 20;

}