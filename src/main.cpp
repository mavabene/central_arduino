// Copyright 2023 michael. All rights reserved.
// Use of this source code is governed by a BSD-style
// license that can be found in the LICENSE file.

/**
 * By Michael Wu on July 9, 2022.
 * reference from https://www.camelsoftware.com/2015/12/25/reading-pwm-signals-from-an-rc-receiver-with-arduino/
 *
 * Please wire up your PWM Inputs on Digital Pins and revise the *_SOURCE fields to the corresponding pin numbers.
 * only arduino ble nano 33 and arduino due are tested
 *
 **/
#include <main.h>
#include <assert.h>

void setup()
{
  Serial.begin(115200);
  vehicle_state = new VehicleState();
  setupModules();
}

void loop()
{
  // synchronizeModules(); 
  module_manager->loop();
  // actuation_module->actuate(vehicle_state);
  
}

void setupModules()
{
  module_manager = new ModuleManager();
  
  // led_module = new LEDModule(LED_BUILTIN, 500);
  // module_manager->setupModule(led_module);

  steering_angle_sensor = new SteeringAngleSensor(STEERING_ANGLE_SENSOR, 5);
  module_manager->setupModule(steering_angle_sensor);

  // pwm_to_voltage_converter = new PWMVoltageConverterModule(THROTTLE_OUTPUT_PIN);
  // module_manager->setupModule(pwm_to_voltage_converter);

  // radio_link = new RadioLinkModule(THROTTLE_SOURCE, STEERING_SOURCE, BRAKE_SOURCE, BUTTON_SOURCE);
  // module_manager->setupModule(radio_link);

  // steering_limiter = new SteeringLimiter(STEERING_LEFT_LIMITER, STEERING_RIGHT_LIMITER);
  // module_manager->setupModule(steering_limiter);

  // spark_max_module = new SparkMaxModule(STEERING_OUTPUT_PIN);
  // module_manager->setupModule(spark_max_module);

  // serial_communicator = new SerialCommunicator();
  // module_manager->setupModule(serial_communicator);

  // actuation_module = new ActuationModule(steering_limiter, pwm_to_voltage_converter, spark_max_module);
  // module_manager->setupModule(actuation_module);
}

void synchronizeModules()
{
  // // get data from angle sensor, steering limiter and update vehicle state
  // vehicle_state->angle = steering_angle_sensor->getSteeringAngle();
  // vehicle_state->angular_velocity = steering_angle_sensor->getAngularVelocity();
  // vehicle_state->is_left_limiter_ON = steering_limiter->isLeftLimiterON();
  // vehicle_state->is_right_limiter_ON = steering_limiter->isRightLimiterON();

  // // get data from radio link
  // vehicle_state->manual_actuation = radio_link->getRadioLinkActuation();
  // vehicle_state->is_auto = radio_link->isAutoFromButton();
  // vehicle_state->auto_actuation = serial_communicator->getAction();
}