/**
* @file RCController6CH.cpp
* @breif Implementation file for RCController6CH class
* @author Frederick Wachter - wachterfreddy@gmail.com
* @date Created: 2018-03-06
*/

#include <Arduino.h>

#include "Logger.h"
#include "RCController6CH.h"

/* CONSTRUCTOR FUNCTIONS */
RCController6CH::RCController6CH(int pin_ch_1, void (*interrupt_func)(void)) {

  delay(500);
  Serial.begin(115200);

  for (int pin = 0; pin < MAX_CHANNELS; pin++) {
    *_pins.PINS[pin] = pin_ch_1 + pin;
  }

  setupController(interrupt_func);

}

RCController6CH::RCController6CH(int pin_ch_1, int pin_ch_2, int pin_ch_3,
  int pin_ch_4, int pin_ch_5, int pin_ch_6, void (*interrupt_func)(void)) {

  _pins.PIN_CH_1 = pin_ch_1;
  _pins.PIN_CH_2 = pin_ch_2;
  _pins.PIN_CH_3 = pin_ch_3;
  _pins.PIN_CH_4 = pin_ch_4;
  _pins.PIN_CH_5 = pin_ch_5;
  _pins.PIN_CH_6 = pin_ch_6;

  setupController(interrupt_func);

}

/* PUBLIC FUNCTIONS */
void RCController6CH::setupController(void (*interrupt_func)(void)) {

  for (int pin = 0; pin < MAX_CHANNELS; pin++) {
    pinMode(*_pins.PINS[pin], INPUT);
  }

  _interval_timer.begin(*interrupt_func, CHANNEL_UPDATE_RATE);

  logger::displayInfo("RC controller has been setup");

}

void RCController6CH::updateChannels(void) {

  int channel_input;

  cli();
  for (int pin = 0; pin < MAX_CHANNELS; pin++) {
    channel_input = pulseIn(*_pins.PINS[pin], HIGH, PULSE_IN_TIMEOUT);
    if (channel_input == 0) { // if the controller is off, set input to zero and move to next input
      *_state.values[pin] = 0;
      continue;
    }

    channel_input = map(channel_input, CH_VALUE_MIN, CH_VALUE_MAX, CH_VALUE_MAP_MIN, CH_VALUE_MAP_MAX);
    channel_input = constrain(channel_input, CH_VALUE_MAP_MIN, CH_VALUE_MAP_MAX);
    *_state.values[pin] = channel_input;
  }
  sei();

}

void RCController6CH::displayChannels(void) {

  String channel_info = "Channel Values: ";

  cli();
  for (int pin = 0; pin < MAX_CHANNELS; pin++) {
    channel_info += String(*_state.values[pin]) + " (CH " + String(pin+1) + ") | ";
  }
  sei();

  channel_info = channel_info.substring(0, channel_info.length()-3);
  logger::displayInfo(channel_info);

}

int RCController6CH::getChannelValue(int channel) {

  return *_state.values[channel-1];

}


