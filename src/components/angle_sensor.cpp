#include <Arduino.h>

#include "pin.h"
#include "angle_sensor.h"

SteeringAngleSensor::SteeringAngleSensor(int pin, uint32_t interval)
{
    this->pin = pin;
    this->interval = interval; // ms
    this->prev_reading_time = 0;
}

Status SteeringAngleSensor::setup()
{
    if (this->pin)
    {
        return Status::SUCCESS;
    }
    return Status::FAILED;
}

Status SteeringAngleSensor::loop()
{
    if (!this->pin)
    {
        return Status::FAILED;
    }
    if (millis() - this->prev_reading_time < this->interval)
    {
        return Status::SUCCESS;
    } 
    this->prev_reading_time = millis(); // update last read time 

    float sensorValue = analogRead(this->pin);
    float angle = (sensorValue - 0) / (1023 - 0) * (this->RIGHT_ANGLE - this->LEFT_ANGLE) + this->LEFT_ANGLE;
    this->currentAngle = angle;

    this->addReading(this->currentAngle);
    this->currentAngularVelocity = this->calcVelocity();

    float ctrlOut = getCtrl();

    // Serial.print("sensorValue: ");
    // Serial.print(sensorValue, 1);
    // Serial.print("\t");
    Serial.print("Current angle: ");
    Serial.print(currentAngle, 1);
    Serial.print("\t");
    Serial.print("Angular Velocity: ");
    Serial.print(currentAngularVelocity, 1);
    Serial.print("\t");
    Serial.print("Calc Output: ");
    Serial.print(ctrlOut, 2);
    Serial.println("\t");

    return Status::SUCCESS;
}

void SteeringAngleSensor::addReading(float reading)
{
    reading_buffer.push(reading);
    timestamp_buffer.push(millis());
}


float SteeringAngleSensor::calcVelocity()
{
   
    curr_angle_t = millis();
    dt = curr_angle_t - prev_angle_t;

    if (dt >= time_gap) {
    
        curr_angle = this->currentAngle;
       
        if (abs(curr_angle - prev_angle) >= 0.3) {
            
            angular_velocity = 0.5 * (angular_velocity + ((curr_angle - prev_angle)*1000/dt)); // convert deg/ms to deg/s  
            prev_angle = curr_angle;
            prev_angle_t = curr_angle_t;
        }
        else{
            angular_velocity = 0.5 * angular_velocity;
            if (abs(angular_velocity < 1)) {
                angular_velocity = 0;
            }
        }
        
        
        // Serial.print("Angular Velocity: ");
        // Serial.print(currentAngularVelocity, 2);
        // Serial.print("\t");
        // Serial.print("curr_angle: ");
        // Serial.print(curr_angle, 1);
        // Serial.println("\t");
    }
    
    return angular_velocity;

}

    float SteeringAngleSensor::getCtrl()
    {
        float ctrlOutput;
        float Kp = 4;
        float Kd = 1;
        float Ki = 0;
        float pos = DegToRad(this->curr_angle);
        float setpoint = 0.0; // convert setpoint to deg for calc
        // float pos_err = vehicle_state->current_actuation->steering_des - vehicle_state->angle;
        float pos_err = setpoint - pos; 
        
        float vel = this->angular_velocity;
        float err_sum = 0;
        // pos_err = setpoint - pos;
        ctrlOutput = Kp * pos_err - Kd * DegToRad(vel) - Ki * err_sum;
        ctrlOutput = constrain(ctrlOutput, -1, 1);
        // vehicle_state->current_actuation->steering = ctrlOutput;

        // Serial.print("angle: ");
        // Serial.print(vehicle_state->angle, 2);
        // Serial.print("\t");
        // Serial.print("pos_err: ");
        // Serial.print(pos_err, 2);
        // Serial.print("\t");
        
        return ctrlOutput;

    }

    float SteeringAngleSensor::DegToRad(float deg)
        {
            float rad = deg * (22/7)/180;
            return rad;
        }

/*
float SteeringAngleSensor::calcVelocity()
{
    if (reading_buffer.size() != STEERING_ANGLE_BUFFER_LEN || timestamp_buffer.size() != STEERING_ANGLE_BUFFER_LEN) {
        return 0.0;
    }
    // find the earliest timestamp
    uint32_t min_timestamp_index = -1;
    uint32_t min_timestamp = millis();
    for (uint32_t i = 0; i < timestamp_buffer.size(); i ++)
    {
        if (timestamp_buffer[i] < min_timestamp) {
            min_timestamp_index = i;
        }   
    }
    // find average velocity between each pair
    float total = 0;
    for (size_t i = 0; i < STEERING_ANGLE_BUFFER_LEN - 1; i++)
    {
        size_t curr_index = (min_timestamp_index + i) % STEERING_ANGLE_BUFFER_LEN;
        size_t next_index = (min_timestamp + i + 1) % STEERING_ANGLE_BUFFER_LEN;

        float curr_reading = reading_buffer[curr_index];
        float next_reading = reading_buffer[next_index];

        uint32_t curr_time = timestamp_buffer[curr_index];
        uint32_t next_time = timestamp_buffer[next_index];

        float displacement = next_reading - curr_reading;
        uint32_t time_diff = next_time - curr_time;
        float curr_velocity = (displacement / time_diff)*1000; //convert deg/ms to deg/sec
        total += curr_velocity;

        Serial.print("displacement: ");
        Serial.print(displacement, 1);
        Serial.print("\t");
        Serial.print("dt: ");
        Serial.print(time_diff, 1);
        Serial.println("\t");
    }

    float avg_velocity = total / STEERING_ANGLE_BUFFER_LEN;

    return avg_velocity;
}
*/

Status SteeringAngleSensor::cleanup()
{
    return Status::SUCCESS;
}

float SteeringAngleSensor::getSteeringAngle()
{
    return this->currentAngle;
}
float SteeringAngleSensor::getAngularVelocity()
{
    return this->currentAngularVelocity;
}

// void SteeringAngleSensor::test(float reading)
// {
//     Serial.print(reading, 1);
//     Serial.println("\t");
// }