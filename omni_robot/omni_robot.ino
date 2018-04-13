/**
 * @file omni_robot.ino
 * @breif Main file to execute omni-robot code
 * @author Frederick Wachter - wachterfreddy@gmail.com
 * @date Created: 2018-02-14
 */

/* DEFENSE ROBOT */
#include "OmniRobot.h"
#include "OffenseRobot.h"

void runShooter(Servo&, Servo&);

Servo pusher;
Servo shoot_left;
Servo shoot_right;

int shoot_speed = 0;

OmniRobot omni_robot;

void setup() {
  cli();

  pusher.attach(7);

  shoot_left.attach(6);
  for (int angle = 1500; angle < 1600; angle++) {
    shoot_left.writeMicroseconds(angle);
    delay(10);
    Serial.println(angle);
  }
  for (int angle = 1600; angle > 1500; angle--) {
    shoot_left.writeMicroseconds(angle);
    delay(10);
    Serial.println(angle);
  }
  shoot_left.writeMicroseconds(1500);
  delay(500);

  shoot_right.attach(5);
  for (int angle = 1500; angle < 1600; angle++) {
    shoot_right.writeMicroseconds(angle);
    delay(10);
    Serial.println(angle);
  }
  for (int angle = 1600; angle > 1500; angle--) {
    shoot_right.writeMicroseconds(angle);
    delay(10);
    Serial.println(angle);
  }
  shoot_right.writeMicroseconds(1500);
  delay(500);

  sei();
}

void loop() {
  omni_robot.displayRCChannels();
  // omni_robot.displayRobotState();
  delay(500);
}

void updateController(void) {
  if (omni_robot.ready) {
    omni_robot.updateRCChannels();
    omni_robot.runRobotModel();
    runShooter(shoot_left, shoot_right);
  }
  omni_robot.updateLED();
}

void runShooter(Servo &shoot_left, Servo &shoot_right) {

  // Initialized Variables
  int pusher_input; // input for actuating the linear actuator
  int shoot_input;  // input for actuating the shoot motors

  // Get pusher and shoot input from controller
  cli();
  pusher_input = omni_robot.rc_controller.getChannelVal(3);
  shoot_input  = omni_robot.rc_controller.getChannelVal(5);
  sei();

  // Map pusher input into relevant values
  pusher_input = map(pusher_input, PUSHER_INPUT_MIN, CH_VALUE_MAP_MAX, PUSHER_OUTPUT_MIN, PUSHER_OUTPUT_MAX);
  pusher_input = constrain(pusher_input, PUSHER_OUTPUT_MIN, PUSHER_OUTPUT_MAX);
  pusher_input = (pusher_input/10)*10; // set the last digit to zero to avoid moving actuator too much

  // shooter_state.speed = map(shoot_input, -100, 100, 1200, 1800);
  // Map shoot input into relevant values
  if (shoot_input > SHOOT_INPUT_CAPTURE_THRESH) {
    shoot_input = SHOOT_OUTPUT_CAPTURE;
  } else if (shoot_input < SHOOT_INPUT_SHOOT_THRESH) {
    shoot_input = SHOOT_OUTPUT_SHOOT;
  } else {
    shoot_input = SHOOT_OUTPUT_OFF;
  }

  // Set pusher and shoot motrs to desired location/speed
  pusher.writeMicroseconds(pusher_input);
  shoot_left.writeMicroseconds(shoot_input);
  shoot_right.writeMicroseconds(shoot_input);

}

/* OFFENSE ROBOT */
// #include "OffenseRobot.h"

// #include <Servo.h>

// Servo left_shoot;
// Servo right_shoot;

// int shoot_speed = 0;

// OffenseRobot omni_robot;

// void setup() {

//   Serial.begin(9600);

//   cli();
//   left_shoot.attach(6);
  
//   for (int angle = 1500; angle < 1600; angle++) {
//     left_shoot.writeMicroseconds(angle);
//     delay(10);
//     Serial.println(angle);
//   }
//   for (int angle = 1600; angle > 1500; angle--) {
//     left_shoot.writeMicroseconds(angle);
//     delay(10);
//     Serial.println(angle);
//   }

//   left_shoot.writeMicroseconds(1500);
//   delay(500);

//   right_shoot.attach(5);
  
//   for (int angle = 1500; angle < 1600; angle++) {
//     right_shoot.writeMicroseconds(angle);
//     delay(10);
//     Serial.println(angle);
//   }
//   for (int angle = 1600; angle > 1500; angle--) {
//     right_shoot.writeMicroseconds(angle);
//     delay(10);
//     Serial.println(angle);
//   }

//   right_shoot.writeMicroseconds(1500);
//   delay(500);
//   sei();
// }

// void loop() {
// 	// omni_robot.displayRCChannels();
// 	// omni_robot.displayRobotState();
// 	// omni_robot.displayShooterState();
//   left_shoot.writeMicroseconds(shoot_speed);
//   right_shoot.writeMicroseconds(shoot_speed);
// 	delay(250);
// }

// void updateController(void) {
// 	if (omni_robot.ready) {
// 		omni_robot.updateRCChannels();
// 		omni_robot.runRobotModel();
// 		shoot_speed = omni_robot.runShooter(left_shoot, right_shoot);
// 	}
// 	omni_robot.updateLED();
// }



// #include "RCController6CH.h"

// void updateController();

// RCController6CH rc_controller(27, &updateController);

// void updateController() {
// 	rc_controller.updateChannels();
// 	rc_controller.displayChannels();
// }

// void setup() {

// }

// void loop() {

// }




