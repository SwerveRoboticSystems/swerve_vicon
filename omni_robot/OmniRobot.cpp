/**
 * @file OmniRobot.cpp
 * @breif Implementation file for OmniRobot class
 * @author Frederick Wachter - wachterfreddy@gmail.com
 * @date Created: 2018-02-14
 */

#include "OmniRobot.h"

/* PUBLIC FUNCTIONS */
bool OmniRobot::isSetup() {

	return setup;

}

/* PRIVATE FUNCTIONS */
void OmniRobot::_setupMotorPins() {

	pinMode(_pin_pwm_motor_left,  OUTPUT);
	pinMode(_pin_pwm_motor_right, OUTPUT);
	pinMode(_pin_pwm_motor_tail,  OUTPUT);
	pinMode(_pin_ppm_motor_shoot, OUTPUT);
	pinMode(_pin_ppm_motor_push,  OUTPUT);

}




