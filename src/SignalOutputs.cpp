#include "SignalOutputs.hpp"

#ifdef ARDUINO
#include <Arduino.h>
#else
#include <iostream>
#endif

SignalOutputs::SignalOutputs()
    : current_pins_{true, false, false, true, false, false} {}

void SignalOutputs::setupPins() {
#ifdef ARDUINO
  pinMode(BoardConfig::PIN_NS_RED, OUTPUT);
  pinMode(BoardConfig::PIN_NS_YELLOW, OUTPUT);
  pinMode(BoardConfig::PIN_NS_GREEN, OUTPUT);
  pinMode(BoardConfig::PIN_EW_RED, OUTPUT);
  pinMode(BoardConfig::PIN_EW_YELLOW, OUTPUT);
  pinMode(BoardConfig::PIN_EW_GREEN, OUTPUT);

  digitalWrite(BoardConfig::PIN_NS_RED, HIGH);
  digitalWrite(BoardConfig::PIN_EW_RED, HIGH);
  digitalWrite(BoardConfig::PIN_NS_YELLOW, LOW);
  digitalWrite(BoardConfig::PIN_EW_YELLOW, LOW);
  digitalWrite(BoardConfig::PIN_NS_GREEN, LOW);
  digitalWrite(BoardConfig::PIN_EW_GREEN, LOW);
#endif
  current_pins_ = {true, false, false, true, false, false};
}

bool SignalOutputs::verifyHardwareInterlock(bool ns_green, bool ns_yellow,
                                           bool ew_green, bool ew_yellow) {
  bool ns_active = ns_green || ns_yellow;
  bool ew_active = ew_green || ew_yellow;
  return !(ns_active && ew_active);
}

void SignalOutputs::applyState(SignalState state, unsigned long now_ms) {
  bool ns_r = false, ns_y = false, ns_g = false;
  bool ew_r = false, ew_y = false, ew_g = false;

  switch (state) {
  case STATE_NS_GREEN:
    ns_g = true;
    ew_r = true;
    break;

  case STATE_NS_YELLOW:
    ns_y = true;
    ew_r = true;
    break;

  case STATE_ALL_RED_STARTUP:
  case STATE_ALL_RED_TO_EW:
  case STATE_ALL_RED_TO_NS:
    ns_r = true;
    ew_r = true;
    break;

  case STATE_EW_GREEN:
    ew_g = true;
    ns_r = true;
    break;

  case STATE_EW_YELLOW:
    ew_y = true;
    ns_r = true;
    break;

  case STATE_FAILSAFE_FLASH:
  default: {
    bool flash_on = ((now_ms / 500) % 2 == 0);
    ns_r = flash_on;
    ew_r = flash_on;
    ns_y = false;
    ew_y = false;
    ns_g = false;
    ew_g = false;
    break;
  }
  }

  // Independent physical interlock check
  if (!verifyHardwareInterlock(ns_g, ns_y, ew_g, ew_y)) {
    current_pins_ = {true, false, false, true, false, false};
    writeHardwarePins();
#ifdef ARDUINO
    Serial.println("[CRITICAL] Hardware Interlock Tripped! Conflicting active "
                   "signals prohibited.");
#else
    std::cerr << "[CRITICAL] Hardware Interlock Tripped! Conflicting active "
                 "signals prohibited."
              << std::endl;
#endif
    return;
  }

  current_pins_ = {ns_r, ns_y, ns_g, ew_r, ew_y, ew_g};
  writeHardwarePins();
}

void SignalOutputs::writeHardwarePins() {
#ifdef ARDUINO
  digitalWrite(BoardConfig::PIN_NS_RED, current_pins_.ns_red ? HIGH : LOW);
  digitalWrite(BoardConfig::PIN_NS_YELLOW, current_pins_.ns_yellow ? HIGH : LOW);
  digitalWrite(BoardConfig::PIN_NS_GREEN, current_pins_.ns_green ? HIGH : LOW);

  digitalWrite(BoardConfig::PIN_EW_RED, current_pins_.ew_red ? HIGH : LOW);
  digitalWrite(BoardConfig::PIN_EW_YELLOW, current_pins_.ew_yellow ? HIGH : LOW);
  digitalWrite(BoardConfig::PIN_EW_GREEN, current_pins_.ew_green ? HIGH : LOW);
#endif
}
