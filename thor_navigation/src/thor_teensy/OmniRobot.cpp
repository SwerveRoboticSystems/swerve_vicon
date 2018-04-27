/**
 * @file OmniRobot.cpp
 * @breif Implementation file for OmniRobot class
 * @author Frederick Wachter - wachterfreddy@gmail.com
 * @date Created: 2018-02-14
 */

#include <Arduino.h>
#include <math.h>

#include "Logger.h"
#include "OmniRobot.h"

/* CONSTRUCTOR FUNCTIONS */
OmniRobot::OmniRobot(bool enable_ros) :
      _led(PIN_LEFT_RED, PIN_LEFT_GREEN, PIN_LEFT_BLUE, PIN_RIGHT_RED, PIN_RIGHT_GREEN, PIN_RIGHT_BLUE),
      _drive_left( MOTOR_TYPE_DC, PIN_PWM_LEFT,  PIN_A_LEFT,  PIN_B_LEFT,  PIN_A_LEFT_ENCODER,  PIN_B_LEFT_ENCODER),
      _drive_right(MOTOR_TYPE_DC, PIN_PWM_RIGHT, PIN_A_RIGHT, PIN_B_RIGHT, PIN_A_RIGHT_ENCODER, PIN_B_RIGHT_ENCODER),
      _drive_tail( MOTOR_TYPE_DC, PIN_PWM_TAIL,  PIN_A_TAIL,  PIN_B_TAIL,  PIN_A_TAIL_ENCODER,  PIN_B_TAIL_ENCODER),
      _imu(SENSOR_TYPE_IMU, PIN_IMU_SCL, PIN_IMU_SDA, PIN_IMU_INT),
      _rc_controller(PIN_CH_1, &controlLoopInterface),
      _imu_publisher(      "/thor/imu",          &_state.imu),
      _imu_calib_publisher("/thor/imu_calib",    &_state.imu_calibration),
      _heading_publisher(  "/thor/imu_heading",  &_state.heading),
      _encoder_publisher(  "/thor/joint_states", &_joint_states),
      _input_publisher(    "/thor/input",        &_state.input), 
      _ros_input_control(  "/thor/ros_control",  &rosInputControlInterface),
      _ros_input_command(  "/thor/cmd_vel",      &rosInputCommandInterface) {

  if (enable_ros) {
    _ros_node_handle.initNode();
    _ros_node_handle.advertise(_imu_publisher);
    _ros_node_handle.advertise(_imu_calib_publisher);
    _ros_node_handle.advertise(_heading_publisher);
    _ros_node_handle.advertise(_encoder_publisher);
    _ros_node_handle.advertise(_input_publisher);
    _ros_node_handle.subscribe(_ros_input_control);
    _ros_node_handle.subscribe(_ros_input_command);

    _joint_states.name_length     = 3;
    _joint_states.position_length = 3;
  } else {
    _ros_enabled = false;
  }

  _imu_interval_timer.begin(    updateIMUStateInterface,   IMU_UPDATE_RATE);
  _encoder_interval_timer.begin(updateMotorStateInterface, ENCODER_UPDATE_RATE);

  _ready = true;
  
  logger::displayInfo("Omni robot has been setup");

}

/* PUBLIC FUNCTIONS */
void OmniRobot::controlLoop(void) {

  if (_ready) {
    _getInput();
    _runRobotModel();
  }
  _updateLED();
  _ros_node_handle.spinOnce();

}

void OmniRobot::rosInputControl(const std_msgs::Bool &ros_control) {

  _ros_controlled = ros_control.data;
  if (_ros_controlled) {
    logger::displayInfo("Robot is set to be controlled by ROS network");
  } else {
    logger::displayInfo("Robot is set to be controlled by RC controller");
  }

}

void OmniRobot::rosInputCommand(const geometry_msgs::Twist &input) {

  if (_ros_controlled) {
    _x_input    = input.linear.x;
    _y_input    = input.linear.y;
    _spin_input = input.angular.z * SPIN_GAIN;

    _applyDeadZone(&_x_input, &_y_input, &_spin_input); 
  }

}

void OmniRobot::updateMotorState(void) {

  _state.wheel_positions[MOTOR_POSITION_LEFT_INDEX]  = _drive_left.readPosition()  * ENCODER_POSITION_TO_RADIANS;
  _state.wheel_positions[MOTOR_POSITION_RIGHT_INDEX] = _drive_right.readPosition() * ENCODER_POSITION_TO_RADIANS;
  _state.wheel_positions[MOTOR_POSITION_TAIL_INDEX]  = _drive_tail.readPosition()  * ENCODER_POSITION_TO_RADIANS;

  _joint_states.header.stamp = _ros_node_handle.now();
  _joint_states.name     = _state.joint_names;
  _joint_states.position = _state.wheel_positions;

  Serial.println(_joint_states.position[0]);

  if (_ros_enabled) {
    _encoder_publisher.publish(&_joint_states);
  }

}

void OmniRobot::updateIMUState(void) {

  _imu.readIMUSensor(_ros_node_handle, &_state.imu, &_state.imu_calibration, &_state.heading, IMU_FRAME_ID);

  if (_ros_enabled) {
    _imu_publisher.publish(&_state.imu);
    _imu_calib_publisher.publish(&_state.imu_calibration);
    _heading_publisher.publish(&_state.heading);
  }

}

void OmniRobot::updateRCChannels(void) {

  _rc_controller.updateChannels();

}

void OmniRobot::displayRobotState(void) {

  logger::displayInfo("Robot State: " + String(_state.wheel_pwm_left) + " (left) | " + 
      String(_state.wheel_pwm_right) + " (right) | " + String(_state.wheel_pwm_tail) + 
      " (tail) | " + String(_state.input_speed) + " (body speed) | " + String(_state.input_direction) 
      + " (body direction) | " + String(_state.input_spin) + " (body spin)");

}

void OmniRobot::displayRCChannels(void) {

  _rc_controller.displayChannels();

}

void OmniRobot::displayEncoderState(void) {

  cli();
  String encoder_states = "Encoder States: ";
  encoder_states += String(_state.wheel_positions[MOTOR_POSITION_LEFT_INDEX]) + " (left) | ";
  encoder_states += String(_state.wheel_positions[MOTOR_POSITION_RIGHT_INDEX]) + " (right) | ";
  encoder_states += String(_state.wheel_positions[MOTOR_POSITION_TAIL_INDEX]) + " (tail)";
  logger::displayInfo(encoder_states);
  sei();

}

void OmniRobot::displayIMUState(void) {

  cli();
  String imu_states = "IMU Calibration: ";
  imu_states += String(_state.imu_calibration.vector.x) + " (system) | ";
  imu_states += String(_state.imu_calibration.vector.x) + " (gyroscope) | ";
  imu_states += String(_state.imu_calibration.vector.x) + " (accelerometer)";
  logger::displayInfo(imu_states);
  imu_states = "IMU Orientation: ";
  imu::Quaternion quaternion(_state.imu.orientation.x, _state.imu.orientation.y, _state.imu.orientation.z, _state.imu.orientation.w);
  imu::Vector<3> orientation = quaternion.toEuler();
  imu_states += String(orientation[0]) + " (x axis) | ";
  imu_states += String(orientation[1]) + " (y axis) | ";
  imu_states += String(orientation[2]) + " (z axis)";
  logger::displayInfo(imu_states);
  imu_states = "IMU Angular Velocity: ";
  imu_states += String(_state.imu.angular_velocity.x) + " (x axis) | ";
  imu_states += String(_state.imu.angular_velocity.y) + " (y axis) | ";
  imu_states += String(_state.imu.angular_velocity.z) + " (z axis)";
  logger::displayInfo(imu_states);
  imu_states = "IMU Linear Acceleration: ";
  imu_states += String(_state.imu.linear_acceleration.x) + " (x axis) | ";
  imu_states += String(_state.imu.linear_acceleration.y) + " (y axis) | ";
  imu_states += String(_state.imu.linear_acceleration.z) + " (z axis)";
  logger::displayInfo(imu_states);
  sei();

}

/* PROTECTED FUNCTIONS */
void OmniRobot::_applyDeadZone(int *input_1, int *input_2, int *input_3) {

  if (abs(*input_1) < DEAD_ZONE) {
    *input_1 = 0;
  }
  if (abs(*input_2) < DEAD_ZONE) {
    *input_2 = 0;
  }
  if (abs(*input_3) < DEAD_ZONE) {
    *input_3 = 0;
  }

}

void OmniRobot::_getInput(void) {

  if (!_ros_controlled) {
    updateRCChannels();

    cli();
    _x_input    =  _rc_controller._state.value_ch_2;
    _y_input    = -_rc_controller._state.value_ch_1;
    _spin_input = -_rc_controller._state.value_ch_4;
    sei();

    _applyDeadZone(&_x_input, &_y_input, &_spin_input);
  }

}

void OmniRobot::_runRobotModel(void) {

  // Initialize variables

  int wheel_speed_left;
  int wheel_speed_right;
  int wheel_speed_tail;

  int max_wheel_speed; // max speed of the three wheels

  // Apply dead zone for controller

  _state.input.vector.x = _x_input;
  _state.input.vector.y = _y_input;
  _state.input.vector.z = _spin_input;

  if (_ros_enabled) {
    _input_publisher.publish(&_state.input);
  }

  // Calculate speed and direction in the body frame
  _state.input_spin      = int(SPIN_GAIN * _spin_input);
  _state.input_speed     = constrain(sqrt(_x_input*_x_input + _y_input*_y_input), SPEED_MIN, SPEED_MAX);
  _state.input_direction = atan2(_y_input, _x_input);

  // Calcualte individual wheel speeds
  wheel_speed_left  = int(_state.input_speed * cos(-150.0*DEG_TO_RAD + _state.input_direction)) + _state.input_spin;
  wheel_speed_right = int(_state.input_speed * cos( -30.0*DEG_TO_RAD + _state.input_direction)) + _state.input_spin;
  wheel_speed_tail  = int(_state.input_speed * cos(-270.0*DEG_TO_RAD + _state.input_direction)) + _state.input_spin;

  // Check if any wheels are maxed out and adjust accordingly to make sure max wheel speed is 100
  max_wheel_speed = max(wheel_speed_left, max(wheel_speed_right, wheel_speed_tail));
  if (max_wheel_speed > SPEED_MAX) {
    double speed_scale = ((SPEED_MAX - _state.input_spin) / double(max_wheel_speed - _state.input_spin));
    wheel_speed_left  = floor((wheel_speed_left  - _state.input_spin) * speed_scale)  + _state.input_spin;
    wheel_speed_right = floor((wheel_speed_right - _state.input_spin) * speed_scale)  + _state.input_spin;
    wheel_speed_tail  = floor((wheel_speed_tail  - _state.input_spin) * speed_scale)  + _state.input_spin;
  }

  // Move motors at desired speeds
  _drive_left.run( wheel_speed_left);
  _drive_right.run(wheel_speed_right);
  _drive_tail.run( wheel_speed_tail);

  _state.wheel_pwm_left  = wheel_speed_left;
  _state.wheel_pwm_right = wheel_speed_right;
  _state.wheel_pwm_tail  = wheel_speed_tail;

}

void OmniRobot::_updateLED(void) {

  if (_ros_controlled) {
    _led.setState(COLOR_GREEN, BLINK_ON);
  } else {
    _led.setState(COLOR_GREEN);
  }

}


