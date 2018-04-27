/**
* @file LED.cpp
* @breif Implementation file for LED class
* @author Frederick Wachter - wachterfreddy@gmail.com
* @date Created: 2018-03-06
*/

#include <Arduino.h>

#include "LED.h"
#include "Logger.h"

/* CONSTRUCTOR FUNCTIONS */
LED::LED(int pin_left_red, int pin_left_green, int pin_left_blue, 
    int pin_right_red, int pin_right_green, int pin_right_blue) {

  Serial.begin(115200);

  setup(pin_left_red, pin_left_green, pin_left_blue, 
    pin_right_red, pin_right_green, pin_right_blue);

}

/* PUBLIC FUNCTIONS */
void LED::setup(int pin_left_red, int pin_left_green, int pin_left_blue, 
    int pin_right_red, int pin_right_green, int pin_right_blue) {

  int min_val = MIN_COLOR_VALUE;

  _pins.PIN_LEFT_RED    = pin_left_red;
  _pins.PIN_LEFT_GREEN  = pin_left_green;
  _pins.PIN_LEFT_BLUE   = pin_left_blue;
  _pins.PIN_RIGHT_RED   = pin_right_red;
  _pins.PIN_RIGHT_GREEN = pin_right_green;
  _pins.PIN_RIGHT_BLUE  = pin_right_blue;

  pinMode(_pins.PIN_LEFT_RED,    OUTPUT);
  pinMode(_pins.PIN_LEFT_GREEN,  OUTPUT);
  pinMode(_pins.PIN_LEFT_BLUE,   OUTPUT);
  pinMode(_pins.PIN_RIGHT_RED,   OUTPUT);
  pinMode(_pins.PIN_RIGHT_GREEN, OUTPUT);
  pinMode(_pins.PIN_RIGHT_BLUE,  OUTPUT);

#ifdef COMMON_ANODE
  min_val = MAX_COLOR_VALUE;
#endif

  _setColor(min_val, min_val, min_val, SIDE_BOTH);
  _state.led = -1;

  logger::displayInfo("LED has been setup");

}

void LED::setState(int des_state, bool blink, int side) {

  if ((des_state < MIN_STATE_VAL) || (des_state > MAX_STATE_VAL)) {
    logger::displayError("Provided LED state is outside of available inputs:" + String(des_state));
    return;
  }
  _state.is_blink = blink;
  _state.side     = side;

  if (_state.led == des_state) {
    if (blink) {
      _updateBlink();
    }
  } else {
    _state.led = des_state;
    _updateState();
  }

}

/* PRIVATE FUNCTIONS */
void LED::_updateState(void) {

  int red_value   = COLOR_NONE;
  int green_value = COLOR_NONE;
  int blue_value  = COLOR_NONE;

  int des_state = _state.led;

  if ((_state.is_blink == true) && (_state.blink == BLINK_OFF)) {
    des_state = COLOR_NONE;
  }

  switch (des_state) {
    case COLOR_NONE:
      red_value   = MIN_COLOR_VALUE;
      green_value = MIN_COLOR_VALUE;
      blue_value  = MIN_COLOR_VALUE;
      logger::displayDebug("LED color set to: Off");
      break;
    case COLOR_RED:
      red_value   = MAX_COLOR_VALUE;
      green_value = MIN_COLOR_VALUE;
      blue_value  = MIN_COLOR_VALUE;
      logger::displayDebug("LED color set to: Red");
      break;
    case COLOR_YELLOW:
      red_value   = MAX_COLOR_VALUE;
      green_value = int(MAX_COLOR_VALUE/6.0);
      blue_value  = MIN_COLOR_VALUE;
      logger::displayDebug("LED color set to: Yellow");
      break;
    case COLOR_GREEN:
      red_value   = MIN_COLOR_VALUE;
      green_value = MAX_COLOR_VALUE;
      blue_value  = MIN_COLOR_VALUE;
      logger::displayDebug("LED color set to: Green");
      break;
  }

#ifdef COMMON_ANODE
  red_value   = MAX_COLOR_VALUE - red_value;
  green_value = MAX_COLOR_VALUE - green_value;
  blue_value  = MAX_COLOR_VALUE - blue_value;
#endif

  _setColor(red_value, green_value, blue_value, _state.side);

  _state.red   = red_value;
  _state.green = green_value;
  _state.blue  = blue_value;

  _state.blink_start = millis();

}

void LED::_updateBlink(void) {

  if ((millis() - _state.blink_start) > BLINK_SPEED) {
    _state.blink = !_state.blink;
    _updateState();
  }

}

void LED::_setColor(int red_value, int green_value, int blue_value, int side) {

  int min_val = MIN_COLOR_VALUE;

#ifdef COMMON_ANODE
  min_val = MAX_COLOR_VALUE;
#endif

  cli();
  if (side == SIDE_BOTH) {
    analogWrite(_pins.PIN_LEFT_RED,    red_value);
    analogWrite(_pins.PIN_LEFT_GREEN,  green_value);
    analogWrite(_pins.PIN_LEFT_BLUE,   blue_value);
    analogWrite(_pins.PIN_RIGHT_RED,   red_value);
    analogWrite(_pins.PIN_RIGHT_GREEN, green_value);
    analogWrite(_pins.PIN_RIGHT_BLUE,  blue_value);
    logger::displayDebug("LED displayed on both sides");
  } else if (side == SIDE_LEFT) {
    analogWrite(_pins.PIN_LEFT_RED,    red_value);
    analogWrite(_pins.PIN_LEFT_GREEN,  green_value);
    analogWrite(_pins.PIN_LEFT_BLUE,   blue_value);
    analogWrite(_pins.PIN_RIGHT_RED,   min_val);
    analogWrite(_pins.PIN_RIGHT_GREEN, min_val);
    analogWrite(_pins.PIN_RIGHT_BLUE,  min_val);
    logger::displayDebug("LED displayed on left side");
  } else if (side == SIDE_RIGHT) {
    analogWrite(_pins.PIN_LEFT_RED,    min_val);
    analogWrite(_pins.PIN_LEFT_GREEN,  min_val);
    analogWrite(_pins.PIN_LEFT_BLUE,   min_val);
    analogWrite(_pins.PIN_RIGHT_RED,   red_value);
    analogWrite(_pins.PIN_RIGHT_GREEN, green_value);
    analogWrite(_pins.PIN_RIGHT_BLUE,  blue_value);
    logger::displayDebug("LED displayed on right side");
  }
  sei();

}


