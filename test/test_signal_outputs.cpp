#include <iostream>
#include <cassert>
#include "../include/BoardConfig.hpp"
#include "../include/SignalOutputs.hpp"

// Native unit tests for SignalOutputs abstraction and truth table compliance

void testBoardConfigPinAssignments() {
  std::cout << "\n--- SignalOutputs Test 1: BoardConfig Pin Assignments ---" << std::endl;
  assert(BoardConfig::PIN_NS_RED == 4);
  assert(BoardConfig::PIN_NS_YELLOW == 5);
  assert(BoardConfig::PIN_NS_GREEN == 6);
  assert(BoardConfig::PIN_EW_RED == 7);
  assert(BoardConfig::PIN_EW_YELLOW == 8);
  assert(BoardConfig::PIN_EW_GREEN == 9);
  assert(BoardConfig::PIN_UART0_TX == 43);
  assert(BoardConfig::PIN_UART0_RX == 44);
  assert(BoardConfig::UART_BAUD_RATE == 115200);
  assert(BoardConfig::SERIAL_BUF_SIZE == 512);
  assert(BoardConfig::MAX_BYTES_PER_LOOP == 256);
  std::cout << "  [PASS] Hardware pin assignments and constants verified." << std::endl;
}

void testStartupAndClearanceStates() {
  std::cout << "\n--- SignalOutputs Test 2: Startup and Clearance States ---" << std::endl;
  SignalOutputs outputs;
  outputs.setupPins();
  SignalPinState pins = outputs.getPinState();
  assert(pins.ns_red == true);
  assert(pins.ew_red == true);
  assert(pins.ns_yellow == false);
  assert(pins.ns_green == false);
  assert(pins.ew_yellow == false);
  assert(pins.ew_green == false);
  std::cout << "  [PASS] Startup pins initialized to All-Red hold." << std::endl;

  outputs.applyState(STATE_ALL_RED_STARTUP, 1000);
  pins = outputs.getPinState();
  assert(pins.ns_red && pins.ew_red && !pins.ns_green && !pins.ew_green);

  outputs.applyState(STATE_ALL_RED_TO_EW, 2000);
  pins = outputs.getPinState();
  assert(pins.ns_red && pins.ew_red && !pins.ns_green && !pins.ew_green);

  outputs.applyState(STATE_ALL_RED_TO_NS, 3000);
  pins = outputs.getPinState();
  assert(pins.ns_red && pins.ew_red && !pins.ns_green && !pins.ew_green);
  std::cout << "  [PASS] All clearance states enforce All-Red outputs." << std::endl;
}

void testPhaseTruthTable() {
  std::cout << "\n--- SignalOutputs Test 3: Green and Yellow State Truth Table ---" << std::endl;
  SignalOutputs outputs;

  // NS Green
  outputs.applyState(STATE_NS_GREEN, 5000);
  SignalPinState pins = outputs.getPinState();
  assert(pins.ns_green == true && pins.ns_yellow == false && pins.ns_red == false);
  assert(pins.ew_red == true && pins.ew_yellow == false && pins.ew_green == false);

  // NS Yellow
  outputs.applyState(STATE_NS_YELLOW, 6000);
  pins = outputs.getPinState();
  assert(pins.ns_yellow == true && pins.ns_green == false && pins.ns_red == false);
  assert(pins.ew_red == true && pins.ew_yellow == false && pins.ew_green == false);

  // EW Green
  outputs.applyState(STATE_EW_GREEN, 7000);
  pins = outputs.getPinState();
  assert(pins.ew_green == true && pins.ew_yellow == false && pins.ew_red == false);
  assert(pins.ns_red == true && pins.ns_yellow == false && pins.ns_green == false);

  // EW Yellow
  outputs.applyState(STATE_EW_YELLOW, 8000);
  pins = outputs.getPinState();
  assert(pins.ew_yellow == true && pins.ew_green == false && pins.ew_red == false);
  assert(pins.ns_red == true && pins.ns_yellow == false && pins.ns_green == false);

  std::cout << "  [PASS] NS and EW green/yellow truth tables verified." << std::endl;
}

void testFailsafeFlashingCadence() {
  std::cout << "\n--- SignalOutputs Test 4: Failsafe Flashing Cadence ---" << std::endl;
  SignalOutputs outputs;

  // 1 Hz cadence: 500ms ON (0-499ms), 500ms OFF (500-999ms)
  outputs.applyState(STATE_FAILSAFE_FLASH, 0);
  SignalPinState pins = outputs.getPinState();
  assert(pins.ns_red == true && pins.ew_red == true);
  assert(!pins.ns_yellow && !pins.ns_green && !pins.ew_yellow && !pins.ew_green);

  outputs.applyState(STATE_FAILSAFE_FLASH, 250);
  pins = outputs.getPinState();
  assert(pins.ns_red == true && pins.ew_red == true);

  outputs.applyState(STATE_FAILSAFE_FLASH, 500);
  pins = outputs.getPinState();
  assert(pins.ns_red == false && pins.ew_red == false);
  assert(!pins.ns_yellow && !pins.ns_green && !pins.ew_yellow && !pins.ew_green);

  outputs.applyState(STATE_FAILSAFE_FLASH, 750);
  pins = outputs.getPinState();
  assert(pins.ns_red == false && pins.ew_red == false);

  outputs.applyState(STATE_FAILSAFE_FLASH, 1000);
  pins = outputs.getPinState();
  assert(pins.ns_red == true && pins.ew_red == true);

  std::cout << "  [PASS] Failsafe non-blocking 500ms cadence verified." << std::endl;
}

void testHardwareInterlock() {
  std::cout << "\n--- SignalOutputs Test 5: Hardware Interlock Verification ---" << std::endl;
  // Non-conflicting combinations
  assert(SignalOutputs::verifyHardwareInterlock(false, false, false, false) == true);
  assert(SignalOutputs::verifyHardwareInterlock(true, false, false, false) == true);
  assert(SignalOutputs::verifyHardwareInterlock(false, true, false, false) == true);
  assert(SignalOutputs::verifyHardwareInterlock(false, false, true, false) == true);
  assert(SignalOutputs::verifyHardwareInterlock(false, false, false, true) == true);

  // Conflicting combinations
  assert(SignalOutputs::verifyHardwareInterlock(true, false, true, false) == false);
  assert(SignalOutputs::verifyHardwareInterlock(true, false, false, true) == false);
  assert(SignalOutputs::verifyHardwareInterlock(false, true, true, false) == false);
  assert(SignalOutputs::verifyHardwareInterlock(false, true, false, true) == false);

  std::cout << "  [PASS] Hardware interlock logic strictly blocks dual active corridors." << std::endl;
}

int main() {
  std::cout << "=================================================================" << std::endl;
  std::cout << "  SignalOutputs C++ Native Verification Test Suite" << std::endl;
  std::cout << "=================================================================" << std::endl;

  testBoardConfigPinAssignments();
  testStartupAndClearanceStates();
  testPhaseTruthTable();
  testFailsafeFlashingCadence();
  testHardwareInterlock();

  std::cout << "\n=================================================================" << std::endl;
  std::cout << "  ALL SIGNALOUTPUTS VERIFICATION TESTS PASSED (0 ERRORS)." << std::endl;
  std::cout << "=================================================================" << std::endl;
  return 0;
}
