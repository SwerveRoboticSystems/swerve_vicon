/**
 * @file Sensor.cpp
 * @breif Implementation file for Sensor class
 * @author Frederick Wachter - wachterfreddy@gmail.com
 * @date Created: 2018-03-06
 */

#include <Wire.h>

#include "Error.h"
#include "Sensor.h"

/* CONSTRUCTOR FUNCTIONS */
Sensor::Sensor(int type, int pin, bool interruptable, 
		void (*interrupt_func)(void)) {

	// Store pin location
	pins.PIN_DATA = pin;

	// Setup interrupts and store communication type
	setupSensor(type, interruptable, interrupt_func);

}

Sensor::Sensor(int type, int pin_scl, int pin_sda, bool interruptable, 
		void (*interrupt_func)(void)) :
			_magnetometer = MAG3110() {

	if (type == SENSOR_TYPE_MAG3110) {
		_magnetometer.initialize(); // initialize magnetometer

		// Calibrate if not calibrated
		if (!_magnetometer.isCalibrated()) {
			if (!_magnetometer.isCalibrating()) {
				_magnetometer.enterCalMode();
			} else {
				_magnetometer.calibrate();
			}
		}
	} else {
		Serial.println("[WARN] Sensor class isn't made to work with I2C devices other than the MAG3110");
	}

	// Store pin locations
	pins.PIN_DATA  = pin_sda;
	pins.PIN_CLOCK = pin_scl;

	// Setup interrupts and store communication type
	setupSensor(type, interruptable, interrupt_func);

}

/* PUBLIC FUNCTIONS */
int Sensor::setupSensor(int type, bool interruptable, 
		void (*interrupt_func)(void)) {
/** @return error code
 */

	// Setup interrupt if sensor is interruptable
	if (interruptable) {
		INTERRUPTABLE      = true;
		pins.PIN_INTERRUPT = pin;

		if (interrupt_func != NULL) {
			setupDataInterrupt(interrupt_func);
		} else {
			return error::displayError(SENSOR_INT_ERROR);
		}
	}

	// Check sensor type
	if ((type < SENSOR_TYPE_MIN) || (type > SENSOR_TYPE_MAX)) {
		return error::displayError(SENSOR_TYPE_ERROR);
	} else {
		state.type = type;
	}

}

void Sensor::setupDataInterrupt(void (*interrupt_func)(void)) {

	attachInterrupt(pins.PIN_INTERRUPT, interrupt_func, CHANGE);
	INTERRUPT_SETUP = true;

}

void Sensor::setDataScalar(double data_scalar) {

	DATA_SCALAR = data_scalar;

}

double Sensor::updateData(void) {

	switch (state.type) {
		case SENSOR_TYPE_CURRENT:
			state.data = analogRead(pins.PIN_DATA) * DATA_SCALAR;
			return state.data;
		case SENSOR_TYPE_DIGITAL:
			state.data = digitalRead(pins.PIN_DATA);
			return state.data;
		case SENSOR_TYPE_MAG3110:
			state.data = _magnetometer.readHeading(void);
			return state.data;
	}

}


