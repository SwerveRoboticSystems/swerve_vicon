/**
 * @file Error.h
 * @breif Header file for error codes
 * @author Frederick Wachter - wachterfreddy@gmail.com
 * @date Created: 2018-03-06
 */

#ifndef ERROR_H_
#define ERROR_H_

#include <Arduino.h>

#define SUCCESS              0
#define MOTOR_TYPE_ERROR    -1 // motor type not recognized
#define ENCODER_SETUP_ERROR -2 // encoder has not been setup yet
#define SENSOR_TYPE_ERROR   -3 // sensor type not recognized
#define SENSOR_INT_ERROR    -4 // interrupt function not provided to interruptable sensor

namespace error {

	int displayError(int);
  /** int displayError(int)
   *  @brief Display error to serial monitor
   *  @author Frederick Wachter
   *  @date Created: 2018-03-07
   */

};

#endif /* ERROR_H_ */


