// -----
// RotaryEncoder.h - Library for using rotary encoders.
// This class is implemented for use with the Arduino environment.
// Copyright (c) by Matthias Hertel, http://www.mathertel.de
// This work is licensed under a BSD style license. See http://www.mathertel.de/License.aspx
// More information on: http://www.mathertel.de/Arduino
// -----
// 18.01.2014 created by Matthias Hertel
// 16.06.2019 pin initialization using INPUT_PULLUP
// 08.08.2020 hugo3132: changed to header-only, 
//                      changed code-style to personal preferences.
//                      added switch handlers
// -----
#pragma once

#include <Arduino.h>

#define LATCHSTATE 3

class RotaryEncoder {
public:
  enum class Direction { NOROTATION = 0, CLOCKWISE = 1, COUNTERCLOCKWISE = -1 };

  // The array holds the values �1 for the entries where a position was decremented,
  // a 1 for the entries where the position was incremented
  // and 0 in all the other (no change or not valid) cases.

  const int8_t KNOBDIR[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

  // positions: [3] 1 0 2 [3] 1 0 2 [3]
  // [3] is the positions where my rotary switch detends
  // ==> right, count up
  // <== left,  count down

protected:
  /**
   * @brief The pin of the microcontroller to which the CLK pin of the encoder
   * is connected.
   */
  const uint8_t pinClk;

protected:
  /**
   * @brief The pin of the microcontroller to which the DT pin of the encoder
   * is connected.
   */
  const uint8_t pinDt;

protected:
  /**
   * @brief The pin of the microcontroller to which the Switch pin of the
   * encoder is connected.
   */
  const uint8_t pinSwitch;

protected:
  /**
   * @brief
   */
  int8_t oldState;

protected:
  /**
   * @brief Internal position (4 times _positionExt)
   */
  long position;

protected:
  /**
   * @brief External position
   */
  long positionExt;

protected:
  /**
   * @brief External position (used only for direction checking)
   */
  long positionExtPrev;

protected:
  /**
   * @brief The time the last position change was detected.
   */
  unsigned long positionExtTime;

protected:
  /**
   * @brief The time the previous position change was detected.
   */
  unsigned long positionExtTimePrev;

protected:
  /**
   * @brief The current switch state
   */
  uint8_t currentSwitchState = 0;

protected:
  /**
   * @brief The time when the currentSwitchState was seen last
   */
  unsigned long timeCurrentSwitchState;

public:
  /**
   * @brief Creates an encoder instance without a switch
   *
   * @param pinClk The pin of the microcontroller to which the CLK pin of the
   * encoder is connected.
   * @param pinDt The pin of the microcontroller to which the DT pin of the
   * encoder is connected.
   */
  RotaryEncoder(const uint8_t& pinClk, const uint8_t& pinDt)
    : pinClk(pinClk)
    , pinDt(pinDt)
    , pinSwitch(0)
    , oldState(3) // when not started in motion, the current state of the encoder should be 3
    , position(0) // start with position 0
    , positionExt(0)
    , positionExtPrev(0) {
    // Setup the input pins and turn on pullup resistor
    pinMode(pinClk, INPUT_PULLUP);
    pinMode(pinDt, INPUT_PULLUP);
  }

public:
  /**
   * @brief Creates an encoder instance with a switch
   *
   * @param pinClk The pin of the microcontroller to which the CLK pin of the
   * encoder is connected.
   * @param pinDt The pin of the microcontroller to which the DT pin of the
   * encoder is connected.
   * @param pinSwitch The pin of the microcontroller to which the Switch pin of
   * the encoder is connected.
   */
  RotaryEncoder(const uint8_t& pinClk, const uint8_t& pinDt, const uint8_t& pinSwitch)
    : pinClk(pinClk)
    , pinDt(pinDt)
    , pinSwitch(pinSwitch)
    , oldState(3) // when not started in motion, the current state of the encoder should be 3
    , position(0) // start with position 0
    , positionExt(0)
    , positionExtPrev(0) {
    // Setup the input pins and turn on pullup resistor
    pinMode(pinClk, INPUT_PULLUP);
    pinMode(pinDt, INPUT_PULLUP);
    pinMode(pinSwitch, INPUT_PULLUP);
  }

public:
  /**
   * @brief retrieve the current position
   */
  long getPosition() const {
    return positionExt;
  }

public:
  /**
   * @brief simple retrieve of the direction the knob was rotated at.
   */
  Direction getDirection() {
    RotaryEncoder::Direction ret = Direction::NOROTATION;

    if (positionExtPrev > positionExt) {
      ret = Direction::COUNTERCLOCKWISE;
    }
    else if (positionExtPrev < positionExt) {
      ret = Direction::CLOCKWISE;
    }
    positionExtPrev = positionExt;

    return ret;
  }

public:
  /**
   * @brief Adjust the current position
   */
  void setPosition(const long& newPosition) {
    // only adjust the external part of the position.
    position = ((newPosition << 2) | (position & 0x03L));
    positionExt = newPosition;
    positionExtPrev = newPosition;
  }

public:
  /**
   * @brief Returns the time in milliseconds between the current observed
   */
  unsigned long getMillisBetweenRotations() const {
    return positionExtTime - positionExtTimePrev;
  }

public:
  /**
   * @brief call this function every some milliseconds or by using an interrupt
   * for handling state changes of the rotary encoder.
   */
  void tick() {
    int8_t thisState = digitalRead(pinDt) | (digitalRead(pinClk) << 1);
    if (pinSwitch != 0) {
      uint8_t newState = !digitalRead(pinSwitch);
      if (newState == currentSwitchState % 2) {
        // no change
        timeCurrentSwitchState = millis();
      }
      else if (millis() - timeCurrentSwitchState > 100) {
        // debounced state
        currentSwitchState = newState;
        timeCurrentSwitchState = millis();
      }
    }

    if (oldState != thisState) {
      position += KNOBDIR[thisState | (oldState << 2)];

      if (thisState == LATCHSTATE) {
        positionExt = position >> 2;
        positionExtTimePrev = positionExtTime;
        positionExtTime = millis();
      }

      oldState = thisState;
    }
  }

public:
  /**
   * @brief Get the state of the switch
   */
  bool getSwitchState() {
    return currentSwitchState % 2 == 1;
  }

public:
  /**
   * @brief Returns true once if the switch was clicked
   */
  bool getNewClick() {
    if (currentSwitchState == 1) {
      currentSwitchState |= B10;
      return true;
    }
    return false;
  }
};
