/**
 * @file OmniRobot.h
 * @breif Header file for OmniRobot class
 * @author Frederick Wachter - wachterfreddy@gmail.com
 * @date Created: 2018-02-14
 */

#ifndef OMNI_ROBOT_H_
#define OMNI_ROBOT_H_

#include <Arduino.h>

#include <ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>

#include "LED.h"             // LED status class
#include "Motor.h"           // Motor class
#include "RCController6CH.h" // 6 channel RC controller class
#include "Sensor.h"          // Sensor class

#define PIN_PWM_LEFT  2
#define PIN_PWM_RIGHT 3
#define PIN_PWM_TAIL  4

#define PIN_A_TAIL  33
#define PIN_B_TAIL  34
#define PIN_A_RIGHT 35
#define PIN_B_RIGHT 36
#define PIN_A_LEFT  37
#define PIN_B_LEFT  38

#define PIN_A_TAIL_ENCODER  25
#define PIN_B_TAIL_ENCODER  26
#define PIN_A_RIGHT_ENCODER 11
#define PIN_B_RIGHT_ENCODER 12
#define PIN_A_LEFT_ENCODER   0
#define PIN_B_LEFT_ENCODER   1

#define PIN_CH_1 27

#define PIN_LEFT_BLUE    8
#define PIN_LEFT_RED     9
#define PIN_LEFT_GREEN  10
#define PIN_RIGHT_BLUE  23
#define PIN_RIGHT_RED   22
#define PIN_RIGHT_GREEN 21

#define PIN_IMU_SCL 18
#define PIN_IMU_SDA 19
#define PIN_IMU_INT 20
#define IMU_FRAME_ID "thor_imu"

#define SPEED_MIN   0
#define SPEED_MAX 100
#define DEAD_ZONE  10
#define SPIN_GAIN   0.5

#define IMU_UPDATE_RATE     100000 // imu update rate in microseconds
#define ENCODER_UPDATE_RATE 100000 // encoder update rate in microseconds

#define MOTOR_POSITION_LEFT_INDEX  0
#define MOTOR_POSITION_RIGHT_INDEX 1
#define MOTOR_POSITION_TAIL_INDEX  2

// Interrupt and Callback Interface Functions
void controlLoopInterface(void);
void updateIMUStateInterface(void);
void updateMotorStateInterface(void);
void rosInputControlInterface(const std_msgs::Bool&);
void rosInputCommandInterface(const geometry_msgs::Twist&);

struct OmniRobotState {
  volatile int input_spin;
  volatile double input_speed;
  volatile double input_direction;
  geometry_msgs::Vector3Stamped input;

  volatile int wheel_pwm_left;
  volatile int wheel_pwm_right;
  volatile int wheel_pwm_tail;

  char *joint_names[3]     = {"thor_left_wheel_joint", "thor_right_wheel_joint", "thor_tail_wheel_joint"};
  float wheel_positions[3] = {0, 0, 0};
  sensor_msgs::Imu imu;
  geometry_msgs::Vector3Stamped heading;
  geometry_msgs::Vector3Stamped imu_calibration;

};

class OmniRobot {
public:

  /* Constructor Functions */
  OmniRobot(bool enable_ros = true);
  /** @fn OmniRobot(bool enable_ros = true)
   *  @brief Default constructor
   *  @author Frederick Wachter
   *  @date Created: 2018-03-06
   */

  /* Public Functions */
  void controlLoop(void);
  /** @fn void controlLoop(void)
   *  @brief Run through control loop for omni robot
   *  @author Frederick Wachter
   *  @date Created: 2018-04-25
   */

  void rosInputControl(const std_msgs::Bool&);
  /** @fn void rosInputControl(const std_msgs::Bool&)
   *  @brief Callback function for subscriber to toggle between control from ROS or RC controller
   *  @author Frederick Wachter
   *  @date Created: 2018-04-26
   */

  void rosInputCommand(const geometry_msgs::Twist&);
  /** @fn void rosInputCommand(const geometry_msgs::Twist&)
   *  @brief Callback function for subscriber for control input from ROS
   *  @author Frederick Wachter
   *  @date Created: 2018-04-26
   */

  void updateMotorState(void);
  /** @fn void updateMotorPosition(void)
   *  @brief Update motor states from wheel encoders
   *  @author Frederick Wachter
   *  @date Created: 2018-04-25
   */

  void updateIMUState(void);
  /** @fn void updateIMUState(void)
   *  @brief Update IMU state
   *  @author Frederick Wachter
   *  @date Created: 2018-04-25
   */

  void updateRCChannels(void);
  /** @fn void updateRCChannels(void)
   *  @brief Updates the RC controller values
   *  @author Frederick Wachter
   *  @date Created: 2018-03-06
   */

  void displayRobotState(void);
  /** @fn void displayRobotState(void)
   *  @brief Displays the robot state
   *  @author Frederick Wachter
   *  @date Created: 2018-03-06
   */

  void displayRCChannels(void);
  /** @fn void displayRCChannels(void)
   *  @brief Displays all the values from the RC controller
   *  @author Frederick Wachter
   *  @date Created: 2018-03-06
   */

  void displayEncoderState(void);
  /** @fn void displayEncoderState(void)
   *  @brief Displays wheel encoder states
   *  @author Frederick Wachter
   *  @date Created: 2018-04-25
   */

  void displayIMUState(void);
  /** @fn void displayIMUState(void)
   *  @brief Displays IMU state
   *  @author Frederick Wachter
   *  @date Created: 2018-04-25
   */

protected:

  bool _ready          = false;
  bool _ros_enabled    = true;
  bool _ros_controlled = false;

  int _x_input;    // x input in the body frame of the robot
  int _y_input;    // y input in the body frame of the robot
  int _spin_input; // spin input in the body frame of the robot

  OmniRobotState _state;

  Motor _drive_left;
  Motor _drive_right;
  Motor _drive_tail;
  Sensor _imu;
  LED _led;
  RCController6CH _rc_controller;

  IntervalTimer _imu_interval_timer;
  IntervalTimer _encoder_interval_timer;

  sensor_msgs::JointState _joint_states;

  ros::NodeHandle _ros_node_handle;
  ros::Publisher _imu_publisher;
  ros::Publisher _imu_calib_publisher;
  ros::Publisher _heading_publisher;
  ros::Publisher _encoder_publisher;
  ros::Publisher _input_publisher;
  ros::Subscriber<std_msgs::Bool> _ros_input_control;
  ros::Subscriber<geometry_msgs::Twist> _ros_input_command;

  static void _applyDeadZone(int*, int*, int*);
  /** @fn static void _applyDeadZone(int*, int*, int*)
   *  @brief Applies a dead zone to the controller input
   *  @author Frederick Wachter
   *  @date Created: 2018-03-15
   */

  void _getInput(void);
  /** @fn void _getInput(void)
   *  @brief Get input from either ROS or RC controller
   *  @author Frederick Wachter
   *  @date Created: 2018-04-26
   */

  void _runRobotModel(void);
  /** @fn void _runRobotModel(void)
   *  @brief Executes the robot model based on input from the RC controller
   *  @author Frederick Wachter
   *  @date Created: 2018-03-06
   */

  void _updateLED(void);
  /** @fn void updateLED(void)
   *  @brief Updates LED colors based on robot state
   *  @author Frederick Wachter
   *  @date Created: 2018-03-06
   */

};

#endif /* OMNI_ROBOT_H_ */


