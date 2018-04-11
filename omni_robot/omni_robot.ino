/**
 * @file omni_robot.ino
 * @breif Main file to execute omni-robot code
 * @author Frederick Wachter - wachterfreddy@gmail.com
 * @date Created: 2018-02-14
 */

/* DEFENSE ROBOT */
#include "OmniRobot.h"

OmniRobot omni_robot;

void setup() {}

void loop() {
  omni_robot.displayRCChannels();
  // omni_robot.displayRobotState();
  delay(500);
}

void updateController(void) {
  if (omni_robot.ready) {
    omni_robot.updateRCChannels();
    omni_robot.runRobotModel();
  }
  omni_robot.updateLED();
}

/* OFFENSE ROBOT */
// #include "OffenseRobot.h"

// OffenseRobot omni_robot;

// void setup() {}

// void loop() {
// 	omni_robot.displayRCChannels();
// 	// omni_robot.displayRobotState();
// 	// omni_robot.displayShooterState();
// 	delay(500);
// }

// void updateController(void) {
// 	if (omni_robot.ready) {
// 		omni_robot.updateRCChannels();
// 		omni_robot.runRobotModel();
// 		omni_robot.runShooter();
// 	}
// 	omni_robot.updateLED();
// }


