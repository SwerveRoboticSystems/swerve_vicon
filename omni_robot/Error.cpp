/**
 * @file Sensor.cpp
 * @breif Implementation file for error codes
 * @author Frederick Wachter - wachterfreddy@gmail.com
 * @date Created: 2018-03-07
 */

#include "Error.h"

#define SUCCESS              0
#define MOTOR_TYPE_ERROR    -1 // motor type not recognized
#define MOTOR_DIR_ERROR     -2 // provided and invalid motor direction
#define ENCODER_SETUP_ERROR -3 // encoder has not been setup yet
#define SENSOR_TYPE_ERROR   -4 // sensor type not recognized
#define SENSOR_INT_ERROR    -5 // interrupt function not provided to interruptable sensor

void error::displayError(int error_code, int val) {

	Serial.print("[ERROR] ");

	switch (error_code) {
		case MOTOR_TYPE_ERROR:
			Serial.println("Provided motor type not recognized");
			return MOTOR_TYPE_ERROR;
		case ENCODER_SETUP_ERROR:
			Serial.println("Requesting information from encoder when it has not been setup yet");
			return ENCODER_SETUP_ERROR;
		case SENSOR_TYPE_ERROR:
			Serial.println("Provided sensor type not recognized");
			return SENSOR_TYPE_ERROR;
		case SENSOR_INT_ERROR:
			Serial.println("Sensor is interruptable but no interrupt function was provided");
			return SENSOR_INT_ERROR;
	}

}


