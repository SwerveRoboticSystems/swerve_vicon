/** 
 * @file rc_motor_control.ino
 * @brief Example of how to actuate a servo linear actuator
 * @author Frederick Wachter - wachterfreddy@gmail.com
 * @date Created: 2017-02-14
 */
 
#include <Servo.h>

#define LIN_ACT_MIN 45
#define LIN_ACT_MAX 140

Servo linear_actuator;

void setup() {
  linear_actuator.attach(3);
}

void loop() {
  linear_actuator.write(LIN_ACT_MIN);
  delay(1000);
  linear_actuator.write(LIN_ACT_MAX);
  delay(1000);
}
