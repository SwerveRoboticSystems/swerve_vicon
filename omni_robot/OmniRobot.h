/**
 * @file OmniRobot.h
 * @breif Header file for OmniRobot class
 * @author Frederick Wachter - wachterfreddy@gmail.com
 * @date Created: 2018-02-14
 */

#ifndef OMNI_ROBOT_H_
#define OMNI_ROBOT_H_

#include <Arduino.h>

class OmniRobot {
/** @class OmniRobot
 *  @brief Class to control SDC 2018 omni wheeled robot
 *  @author Frederick Wachter - wachterfreddy@gmail.com
 *  @date Created: 2018-02-14
 */

public:
	OmniRobot();

	void setup();
	void changeMotorPins(int, int, int, int, int);

	bool isSetup();


protected:
	bool setup = false;

private:
	const int _pin_pwm_motor_left  = 0;
	const int _pin_pwm_motor_right = 0;
	const int _pin_pwm_motor_tail  = 0;
	const int _pin_ppm_motor_shoot = 0;
	const int _pin_ppm_motor_push  = 0;

	void _setupMotorPins();

};

#endif /* OMNI_ROBOT_H_ */


