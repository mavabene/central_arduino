/**
 * By Michael Wu on July 9, 2022.
 * reference from https://www.camelsoftware.com/2015/12/25/reading-pwm-signals-from-an-rc-receiver-with-arduino/
 *
 * Please wire up your PWM Inputs on Digital Pins and revise the *_SOURCE fields to the corresponding pin numbers.
 * only arduino ble nano 33 and arduino due are tested
 *
 **/
#include <main.h>

void setup()
{
  Serial.begin(9600);
  setupPwmVoltageConverter();
  setupLED();
  setupSparkMax();
  setupRadioLink();
  setupSteeringLimiters();
  setupSteeringControl();
  //setupBrake(); // this will block program if CAN is not initialized
}

void loop()
{
  // update state
  loopSpeedEstimation();
  runControls();
  vehicleState->speed = getSpeed();
  vehicleState->is_auto = determine_auto();
  vehicleState->angle = SteeringPot->EngVal;
  Serial.println(vehicleState->angle);
  updateLimiterStates(vehicleState);
  // actuation
  // if (vehicleState->is_auto)
  // {
  //   onAutoDrive();
  // }
  // else
  // {
  //   onManualDrive();
  // }
  onManualDrive();
  applyVehicleSafetyPolicy(vehicleState);
  actuate(vehicleState->act);
}

void onAutoDrive()
{
  digitalWrite(LED_BUILTIN, HIGH);
  processSerialCommunication(vehicleState, true);
}

void onManualDrive()
{

  digitalWrite(LED_BUILTIN, LOW);

  // if button is not pressed, serial input is ignored, use controller input
  vehicleState->act->throttle = throttle_pulse_time;
  vehicleState->act->steering = steering_pulse_time;
  vehicleState->act->brake = brake_pulse_time;
  processSerialCommunication(vehicleState, false);
}

void actuate(Actuation *act)
{
  writeToSteering(act->steering);
  writeToBrake(act->brake);
  writeToThrottle(act->throttle);
}
