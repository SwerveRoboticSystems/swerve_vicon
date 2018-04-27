/**
* @file Motor.cpp
* @breif Implementation file for Motor class
* @author Frederick Wachter - wachterfreddy@gmail.com
* @date Created: 2018-03-06
*/

#include "Logger.h"
#include "Motor.h"

/* CONSTRUCTOR FUNCTIONS */
Motor::Motor(int type, int pin_pwm, int pin_a, int pin_b) :
      _encoder(NO_ENCODER_PIN, NO_ENCODER_PIN) {

  setupMotor(type, pin_pwm, pin_a, pin_b);

}

Motor::Motor(int type, int pin_pwm, int pin_a, int pin_b, 
    int encoder_pin_a, int encoder_pin_b) :
      _encoder(encoder_pin_a, encoder_pin_b) {

  _encoder.write(0);
  ENCODER_SETUP = true;

  setupMotor(type, pin_pwm, pin_a, pin_b);

}

/* PUBLIC FUNCTIONS */
int Motor::setupMotor(int type, int pin_pwm, int pin_a, int pin_b) {
/** @return error code
*/

  // Initialize motor state
  _state.speed     = 0; // set initial speed to zero
  _state.direction = MOTOR_FORWARD; // set initial direction to forward

  // Store motor pins
  _pins.PIN_PWM = pin_pwm;

  // Setup motor pins
  pinMode(_pins.PIN_PWM, OUTPUT);

  // Check motor type
  if ((type < MOTOR_TYPE_MIN) || (type > MOTOR_TYPE_MAX)) {
    logger::displayError("Motor type not recognized");
    return MOTOR_SETUP_FAILED;
  } else {
    _state.type = type;
  }

  // Setup motor
  if (_state.type == MOTOR_TYPE_DC) {
    _setupDCMotor(pin_a, pin_b);
  } if (_state.type == MOTOR_TYPE_BLDC) {
    _setupBLDCMotor();
  }

  logger::displayInfo(_getMotorTypeName() + " motor has been setup");
  return MOTOR_SETUP_SUCCESS;

}

void Motor::run(int speed) {
/** @return error code
*/

  if (speed > 0) {
    _state.direction = MOTOR_FORWARD;
  } else if (speed < 0) {
    _state.direction = MOTOR_REVERSE;
  } else {
    _state.direction = MOTOR_BRAKE;
  }

  if (_state.type == MOTOR_TYPE_DC) {
    return _runDCMotor(speed);
  } else if (_state.type == MOTOR_TYPE_BLDC) {
    return _runBLDCMotor(speed);
  } else if (_state.type == MOTOR_TYPE_SERVO) {
    return _runServoMotor(speed);
  }

}

long Motor::readPosition(void) {
/** @return position of the motor
*/

  if (!ENCODER_SETUP) {
    logger::displayError("Cannot read from encoder since it has not been setup yet");
    return ENCODER_SETUP_FAILED;
  }

  _state.position = _encoder.read(); // get encoder position

  return _state.position;

}

/* PUBLIC FUNCTIONS */
void Motor::_setupDCMotor(int pin_a, int pin_b) {

// Store motor _pins
  _pins.PIN_A = pin_a;
  _pins.PIN_B = pin_b;

// Setup motor _pins
  pinMode(_pins.PIN_A, OUTPUT);
  pinMode(_pins.PIN_B, OUTPUT);

}

void Motor::_setupBLDCMotor(void) {

  // Setup BLDC
  _servo.attach(_pins.PIN_PWM, MOTOR_PPM_MIN, MOTOR_PPM_MAX);

  for (int angle = 1500; angle < 1600; angle++) {
    _servo.writeMicroseconds(angle);
    delay(10);
    Serial.println(angle);
  }
  for (int angle = 1600; angle > 1500; angle--) {
    _servo.writeMicroseconds(angle);
    delay(10);
    Serial.println(angle);
  }

  _state.speed = MOTOR_PPM_OFF; // set initial speed to zero

}

void Motor::_runDCMotor(int speed) {

  _state.speed = abs(speed);

  if (_state.direction == MOTOR_FORWARD) {
    digitalWrite(_pins.PIN_A, HIGH);
    digitalWrite(_pins.PIN_B, LOW);
  } else if (_state.direction == MOTOR_BRAKE) {
    digitalWrite(_pins.PIN_A, HIGH);
    digitalWrite(_pins.PIN_B, HIGH);
  } else if (_state.direction == MOTOR_REVERSE) {
    digitalWrite(_pins.PIN_A, LOW);
    digitalWrite(_pins.PIN_B, HIGH);
  }

  speed = map(_state.speed, MOTOR_SPEED_MIN, MOTOR_SPEED_MAX, MOTOR_PWM_MIN, MOTOR_PWM_MAX);
  speed = constrain(speed, MOTOR_PWM_MIN, MOTOR_PWM_MAX);
  analogWrite(_pins.PIN_PWM, speed);

  logger::displayDebug("Running DC motor at speed: " + String(_state.speed));

}

void Motor::_runBLDCMotor(int speed) {

  _servo.writeMicroseconds(speed);

}

void Motor::_runServoMotor(int position) {

  // Constrain position and store position
  position = max(min(position, MOTOR_PPM_MAX), MOTOR_PPM_MIN);
  _state.speed = position;

  // Set position of the motor
  cli();
  digitalWrite(_pins.PIN_PWM, HIGH);
  delayMicroseconds(_state.speed);
  digitalWrite(_pins.PIN_PWM, LOW);
  sei();

}

String Motor::_getMotorTypeName(void) {
/** @return string of the motor type name
*/

  switch (_state.type) {
    case MOTOR_TYPE_DC:
    return "DC";
    case MOTOR_TYPE_BLDC:
    return "Brushless DC";
    case MOTOR_TYPE_SERVO:
    return "Servo";
  }

  return "";

}


