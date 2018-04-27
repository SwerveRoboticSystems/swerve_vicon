/**
 * @file thor_teensy.ino
 * @breif Main file to execute omni robot code
 * @author Frederick Wachter - wachterfreddy@gmail.com
 * @date Created: 2018-02-14
 */

// Includes to allow class file to reference correct libraries
#include <ros.h>
#include <Encoder.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>

#include "OmniRobot.h"

OmniRobot omni_robot;

void setup() {}

void loop() {
  // omni_robot.displayRCChannels();
  // omni_robot.displayRobotState();
  omni_robot.displayEncoderState();
  // omni_robot.displayIMUState();
  delay(100);
}

// Interface functions implementations
void controlLoopInterface(void) {
  omni_robot.controlLoop();
}

void updateIMUStateInterface(void) {
  omni_robot.updateIMUState();
}

void updateMotorStateInterface(void) {
  omni_robot.updateMotorState();
}

void rosInputControlInterface(const std_msgs::Bool &ros_control) {
  omni_robot.rosInputControl(ros_control);
}

void rosInputCommandInterface(const geometry_msgs::Twist &input) {
  omni_robot.rosInputCommand(input);
}


