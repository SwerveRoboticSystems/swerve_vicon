/**
 * @file Motor.h
 * @breif Header file for Motor class
 * @author Frederick Wachter - wachterfreddy@gmail.com
 * @date Created: 2018-03-06
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include <Encoder.h> // Teensy Encoder library
#include <Servo.h>   // Teensy Servo library
// #include <PWMServo.h>

// #include "Sensor.h"  // Sensor class

#define MOTOR_SETUP_SUCCESS  0
#define MOTOR_SETUP_FAILED   1
#define ENCODER_SETUP_FAILED 0

#define MOTOR_TYPE_DC    1
#define MOTOR_TYPE_BLDC  2
#define MOTOR_TYPE_SERVO 3

#define MOTOR_TYPE_MIN 1
#define MOTOR_TYPE_MAX 3

#define MOTOR_FORWARD  1
#define MOTOR_BRAKE    0
#define MOTOR_REVERSE -1

#define MOTOR_SPEED_MIN   0
#define MOTOR_SPEED_MAX 100

#define MOTOR_PWM_MIN    0
#define MOTOR_PWM_MAX  255
#define MOTOR_PPM_MIN 1200
#define MOTOR_PPM_MAX 1800
#define MOTOR_PPM_OFF 1500

#define ENCODER_POSITION_TO_RADIANS (2*3.1415/1440.0)

#define ESC_ARM_DELAY 10

#define NO_ENCODER_PIN -1 // pin to supply encoder class if not using it

struct MotorState {
  int type;  // motor type
  int speed; // value between 0 and 100
  long position;
  int direction;
};

struct MotorPins {
  int PIN_PWM;
  int PIN_A;
  int PIN_B;
};

class Motor {
public:

  /* Constructor Functions */
  Motor(int, int, int pin_a = 0, int pin_b = 0);
  /** @fn Motor(int, int, int pin_a = 0, int pin_b = 0)
   *  @brief Default constructor
   *  @author Frederick Wachter
   *  @date Created: 2018-03-06
   */

  Motor(int, int, int, int, int, int);
  /** @fn Motor(int, int, int, int, int, int)
   *  @brief Default constructor for motor with an encoder
   *  @author Frederick Wachter
   *  @date Created: 2018-03-06
   */

  /* Public Functions */
  int setupMotor(int, int, int pin_a = 0, int pin_b = 0);
  /** @fn void setupMotor(int, int, int, int)
   *  @brief Sets up the pins of the motor class
   *  @author Frederick Wachter
   *  @date Created: 2018-03-06
   */

  void run(int);
  /** @fn void run(int)
   *  @brief Run the motor at the desired speed and direction or position
   *  @author Frederick Wachter
   *  @date Created: 2018-03-06
   */

  long readPosition(void);
  /** @fn long readPosition(void)
   *  @brief Returns the position of the motor
   *  @author Frederick Wachter
   *  @date Created: 2018-03-06
   */

private:
  
  /* Private Variables */
  MotorState _state;
  MotorPins  _pins;

  Encoder _encoder;
  Servo   _servo;
  // Sensor  _current_sensor;

  bool ENCODER_SETUP        = false;
  bool CURRENT_SENSOR_SETUP = false;

  /* Private Functions */
  void _setupDCMotor(int, int);
  /** @fn void _setupDCMotor(int, int)
   *  @brief Sets up pins for DC motor
   *  @author Frederick Wachter
   *  @date Created: 2018-03-07
   */


  void _setupBLDCMotor(void);
  /** @fn void _setupBLDCMotor(void)
   *  @brief Arms ECS for BLDC motor
   *  @author Frederick Wachter
   *  @date Created: 2018-03-16
   */

  void _runDCMotor(int);
  /** @fn void _runDCMotor(int)
   *  @brief Runs DC motor at a desired speed and direction
   *  @author Frederick Wachter
   *  @date Created: 2018-03-07
   */

  void _runBLDCMotor(int);
  /** @fn void _runBLDCMotor(int)
   *  @brief Runs BLDC motor at desired speed and direction
   *  @author Frederick Wachter
   *  @date Created: 2018-03-07
   */

  void _runServoMotor(int);
  /** @fn void _runServoMotor(int)
   *  @brief Sets the position of the servo motor
   *  @author Frederick Wachter
   *  @date Created: 2018-03-07
   */

  String _getMotorTypeName(void);
  /** @fn String _getMotorTypeName(void)
   *  @brief Returns the name of the type of motor
   *  @author Frederick Wachter
   *  @date Created: 2018-03-07
   */

};

#endif /* MOTOR_H_ */


