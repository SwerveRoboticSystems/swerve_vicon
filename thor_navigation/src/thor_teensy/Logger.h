/**
 * @file Logger.h
 * @breif Header file for  logger levels
 * @author Frederick Wachter - wachterfreddy@gmail.com
 * @date Created: 2018-03-06
 */

#ifndef LOGGER_H_
#define LOGGER_H_

#include <Arduino.h>

#define DEBUG_LOGGER_LEVEL 0 // 0 don't display debug messages, 1 to display them

namespace logger {

  int displayError(String);
  /** int displayError(String)
   *  @brief Display error message to serial monitor
   *  @author Frederick Wachter
   *  @date Created: 2018-03-07
   */

  void displayInfo(String);
  /** int void displayInfo(String)
   *  @brief Display info message to serial monitor
   *  @author Frederick Wachter
   *  @date Created: 2018-03-07
   */	

  void displayDebug(String);
  /** void displayDebug(String)
   *  @brief Display debug message to serial monitor
   *  @author Frederick Wachter
   *  @date Created: 2018-03-08
   */

};

#endif /* LOGGER_H_ */




