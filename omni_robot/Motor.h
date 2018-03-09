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

// #include "Sensor.h"  // Sensor class

#define MOTOR_TYPE_DC    1
#define MOTOR_TYPE_BLDC  2
#define MOTOR_TYPE_SERVO 3

#define MOTOR_TYPE_MIN 1
#define MOTOR_TYPE_MAX 3

#define MOTOR_FORWARD  1
#define MOTOR_BRAKE    0
#define MOTOR_REVERSE -1

#define MOTOR_SPEED_MIN 0
#define MOTOR_SPEED_MAX 100

#define MOTOR_PWM_MIN 0
#define MOTOR_PWM_MAX 255

#define NO_ENCODER_PIN -1 // pin to supply encoder class if not using it

struct MotorState {
  int type = 0; // motor type
  int speed; // value between 0 and 100
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
  Motor(int, int, int, int);
  /** @fn Motor(int, int, int, int)
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

  long getPosition(void);
  /** @fn long getPosition(void)
   *  @brief Returns the position of the motor
   *  @author Frederick Wachter
   *  @date Created: 2018-03-06
   */

  double getCurrentUsage(void);

protected:
  
  /* Protected Variables */
  MotorState state;
  MotorPins  pins;

  bool ENCODER_SETUP        = false;
  bool CURRENT_SENSOR_SETUP = false;

private:

  /* Provate Variables */
  Servo   _servo;
  Encoder _encoder;
  // Sensor  _current_sensor;

  /* Private Functions */
  void _setupDCMotor(int, int);
  /** @fn void _setupDCMotor(int, int)
   *  @brief Sets up pins for DC motor
   *  @author Frederick Wachter
   *  @date Created: 2018-03-07
   */

  void _setupServo(int);
  /** @fn void _setupServo(int)
   *  @brief Sets up servo class for BLDC and servo motors
   *  @author Frederick Wachter
   *  @date Created: 2018-03-07
   */

  void _runDCMotor(void);
  /** @fn void _runDCMotor(void)
   *  @brief Runs DC motor at a desired speed and direction
   *  @author Frederick Wachter
   *  @date Created: 2018-03-07
   */

  void _runBLDCMotor(void);
  /** @fn void _runBLDCMotor(void)
   *  @brief Runs BLDC motor at desired speed and direction
   *  @author Frederick Wachter
   *  @date Created: 2018-03-07
   */

  void _runServoMotor(void);
  /** @fn void _runServoMotor(void)
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


