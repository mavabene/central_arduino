#include <controls.h>

/*
float steeringCtrl::calcOutput()
{
   
    curr_angle = vehicle_state->angle; 
    curr_speedvehicle_state->angular_velocity;
    dt = curr_angle_t - prev_angle_t;
}
*/

// void getCtrl(float setpoint, float pos, float vel, float err_sum)
// {
//     float ctrlOutput;
//     float pos_err;
//     pos_err = setpoint - pos;
//     ctrlOutput = Kp * pos_err - Kd * vel - Ki * err_sum;
//     //ctrlOutput = std::clamp(ctrlOutput, -1, 1);
//     //vehicle_state->current_actuation->steering = ctrlOutput;

//     Serial.print("Calc Output: ");
//     Serial.print(ctrlOutput, 2);
//     Serial.println("\t");

// }
