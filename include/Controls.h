#include <Arduino.h>
#include <pin.h>

currentMillis = millis();
SteeringPot->SensorVal = getAnalogInt(STEERING_ANGLE_SENSOR);
SteeringPot->Setpoint = 30;
CtrlLoop(SteeringPID,SteeringPot,SteeringMotor);
current_angle = SensorInputToUnitsFloatPtr(SteeringPot);
SteeringPot->EngVal = current_angle;

if ((currentMillis-angle_detect_start_time) >= 20) // set up time interval for getting dx
{ 
    steering_angular_vel = getDx(angle_detect_start_time, currentMillis, start_angle, current_angle);
    SteeringPot->Vel = UnitConversion(steering_angular_vel,deg_per_ms_TO_deg_per_sec);
    start_angle = current_angle;
    angle_detect_start_time = currentMillis;
}