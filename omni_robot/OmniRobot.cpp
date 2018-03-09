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
OmniRobot::OmniRobot(void) :
			drive_left( MOTOR_TYPE_DC, PIN_PWM_LEFT,  PIN_A_LEFT,  PIN_B_LEFT),
			drive_right(MOTOR_TYPE_DC, PIN_PWM_RIGHT, PIN_A_RIGHT, PIN_B_RIGHT),
			drive_tail( MOTOR_TYPE_DC, PIN_PWM_TAIL,  PIN_A_TAIL,  PIN_B_TAIL),
			rc_controller(PIN_CH_1, &updateController) {

}

/* PUBLIC FUNCTIONS */
void OmniRobot::runRobotModel(void) {
	
	int x_input; // x input in the body frame from controller
	int y_input; // y input in the body frame from controller
	
	// Get x and y input from controller
	cli();
	x_input =  rc_controller.state.value_ch_2;
	y_input = -rc_controller.state.value_ch_1;
	sei(); 

	// Calculate speed and direction in the body frame
	state.body_speed     = sqrt(x_input*x_input + y_input*y_input);
	state.body_direction = atan2(y_input, x_input);

	// Calcualte individual wheel speeds
	state.wheel_speed_left  = int(state.body_speed * cos(-150.0*DEG_TO_RAD + state.body_direction));
	state.wheel_speed_right = int(state.body_speed * cos( -30.0*DEG_TO_RAD + state.body_direction));
	state.wheel_speed_tail  = int(state.body_speed * cos(-270.0*DEG_TO_RAD + state.body_direction));

	// Move motors at desired speeds
	drive_left.run( state.wheel_speed_left);
	drive_right.run(state.wheel_speed_right);
	drive_tail.run( state.wheel_speed_tail);

}

void OmniRobot::displayRobotState(void) {

	logger::displayInfo("Motor Speeds: " + String(state.wheel_speed_left) + " (left) | " + 
			String(state.wheel_speed_right) + " (right) | " + String(state.wheel_speed_tail) + 
			" (tail) | " + String(state.body_speed) + " (body speed) | " + 
			String(state.body_direction) + " (body direction)");

}

void OmniRobot::displayRCChannels(void) {

	rc_controller.displayChannels();

}

void OmniRobot::updateRCChannels(void) {

	rc_controller.updateChannels();

}


