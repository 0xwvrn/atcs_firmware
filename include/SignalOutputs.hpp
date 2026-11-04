#pragma once

#include <stdint.h>
#include "BoardConfig.hpp"
#include "TrafficController.hpp"

#ifdef ARDUINO
#include <Arduino.h>
#endif

struct SignalPinState {
  bool ns_red;
  bool ns_yellow;
  bool ns_green;
  bool ew_red;
  bool ew_yellow;
  bool ew_green;
};

class SignalOutputs {
public:
  SignalOutputs();

  // Configures GPIO pin modes to OUTPUT and sets initial startup state (All-Red hold)
  void setupPins();

  // Applies physical signals according to current FSM state and timestamp
  void applyState(SignalState state, unsigned long now_ms);

#ifdef ARDUINO
  // Arduino convenience overload taking current time from millis()
  void applyState(SignalState state) {
    applyState(state, millis());
  }
#endif

  // Accessor for the active physical pin state
  const SignalPinState& getPinState() const { return current_pins_; }

  // Independent physical interlock verifying conflicting corridors are never simultaneously active
  static bool verifyHardwareInterlock(bool ns_green, bool ns_yellow, bool ew_green, bool ew_yellow);

private:
  SignalPinState current_pins_;
  void writeHardwarePins();
};
