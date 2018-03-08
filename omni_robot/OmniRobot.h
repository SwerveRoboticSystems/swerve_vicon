/**
 * @file OmniRobot.h
 * @breif Header file for OmniRobot class
 * @author Frederick Wachter - wachterfreddy@gmail.com
 * @date Created: 2018-02-14
 */

#ifndef OMNI_ROBOT_H_
#define OMNI_ROBOT_H_

#include <Arduino.h>

#include "LED.h"             // LED status class
#include "Motor.h"           // Motor class
#include "RCController6CH.h" // 6 channel RC controller class
#include "Sensor.h"          // Sensor class

class OmniRobot {
public:
	Motor drive_right();
	Motor drive_left();
	Motor drive_tail();
	Motor shoot_left();
	Motor shoot_right();
	Motor pusher();

	Sensor compass();
	Sensor current(); // current being used by the whole robot
	Sensor ir_ball(); // sensor used to tell if a ball has been captured
	Sensor ir_capture(); // sensor used to tell if a ball is infront of the robot or not

	RCController6CH rc_controller();

	void encoderModel() {
		// Code goes here ...
	}
	void controllerInputModel() { // maybe not needed? maybe define in rc controller class?
		// Code goes here ...
	}
	void robotModel() {
		// Code goes here ...
	}
};

#endif /* OMNI_ROBOT_H_ */


