/**
* @file Logger.cpp
* @breif Implementation file for logger levels
* @author Frederick Wachter - wachterfreddy@gmail.com
* @date Created: 2018-03-07
*/

#include "Logger.h"

int logger::displayError(String message) {

  Serial.print("[ERROR] ");
  Serial.println(message);

}

void logger::displayInfo(String message) {

  Serial.print("[INFO] ");
  Serial.println(message);

}

void logger::displayDebug(String message) {

  if (DEBUG_LOGGER_LEVEL) {
    Serial.print("[DEBUG] ");
    Serial.println(message);
  }

}





