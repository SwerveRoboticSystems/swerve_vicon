/**
 * @file omni_robot.ino
 * @breif Main file to execute omni-robot code
 * @author Frederick Wachter - wachterfreddy@gmail.com
 * @date Created: 2018-02-14
 */

#include "OmniRobot.h"

OmniRobot omni_robot;

void setup() {

}

void loop() {
	// omni_robot.displayRCChannels();
	// omni_robot.displayRobotState();
	omni_robot.displayShooterState();
	delay(100);
}

void updateController(void) {
	omni_robot.updateRCChannels();
	omni_robot.runRobotModel();
	omni_robot.runShooter();
}


