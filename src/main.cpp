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
  // getCtrl();
  
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

  spark_max_module = new SparkMaxModule(STEERING_OUTPUT_PIN);
  module_manager->setupModule(spark_max_module);

  actuation_module = new ActuationModule(steering_limiter, pwm_to_voltage_converter, spark_max_module);
  module_manager->setupModule(actuation_module);

  serial_communicator = new SerialCommunicator();
  module_manager->setupModule(serial_communicator);
}

void synchronizeModules()
{
    Serial.print("angle: ");
    Serial.print(vehicle_state->angle, 2);
    Serial.println("\t");
  // get data from angle sensor, steering limiter and update vehicle state
  vehicle_state->angle = steering_angle_sensor->getSteeringAngle();
  vehicle_state->angular_velocity = steering_angle_sensor->getAngularVelocity();
  vehicle_state->is_left_limiter_ON = steering_limiter->isLeftLimiterON();
  vehicle_state->is_right_limiter_ON = steering_limiter->isRightLimiterON();

  if (radio_link->isAutoFromButton()) {
    // get data from serial communicator
    vehicle_state->current_actuation->throttle = serial_communicator->getAction()->throttle;
    vehicle_state->current_actuation->brake = serial_communicator->getAction()->throttle;
    vehicle_state->current_actuation->steering = serial_communicator->getAction()->steering;
  } else {
    // get data from radio link
    vehicle_state->current_actuation->throttle = radio_link->getThrottle();
    vehicle_state->current_actuation->brake = radio_link->getBrake();
    vehicle_state->current_actuation->steering = radio_link->getSteering();
  }

    Serial.print("sync mod angle: ");
    Serial.print(vehicle_state->angle, 2);
    Serial.println("\t");
  // vehicle_state->current_actuation->steering = getCtrl(vehicle_state->current_actuation->steering_des, vehicle_state->angle, vehicle_state->angular_velocity, 0);

  serial_communicator->setVehicleState(vehicle_state);
  
}

void getCtrl()
{
    float ctrlOutput;
    float Kp = 8;
    float Kd = 1;
    float Ki = 0;
    // float pos_err = vehicle_state->current_actuation->steering_des - vehicle_state->angle;
    float pos_err = 0.0 - vehicle_state->angle;
    
    float vel = vehicle_state->angular_velocity;
    float err_sum = 0;
    // pos_err = setpoint - pos;
    ctrlOutput = Kp * pos_err - Kd * vel - Ki * err_sum;
    //ctrlOutput = std::clamp(ctrlOutput, -1, 1);
    vehicle_state->current_actuation->steering = ctrlOutput;

    // Serial.print("angle: ");
    // Serial.print(vehicle_state->angle, 2);
    // Serial.print("\t");
    // Serial.print("pos_err: ");
    // Serial.print(pos_err, 2);
    // Serial.print("\t");
    Serial.print("Calc Output: ");
    Serial.print(ctrlOutput, 2);
    Serial.println("\t");

}
