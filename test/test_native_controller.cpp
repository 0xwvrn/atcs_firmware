#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>
#include "../include/TrafficController.hpp"

// Native C++ Academic Verification Test Harness for TrafficController

void testInputValidation() {
  std::cout << "\n--- C++ Test 1: Telemetry Input Validation & Edge Cases ---" << std::endl;
  TrafficController ctrl;

  // Case A: Valid 0.0 PCU reading (must stay 0.0, NOT fallback to count q)
  bool res = ctrl.updateApproach('N', 0, 0.0f, 0.0f, true);
  assert(res == true);
  ApproachState st = ctrl.getApproachState('N');
  assert(st.p_pcu == 0.0f);
  assert(st.is_valid == true);
  std::cout << "  [PASS] Valid 0.0 PCU correctly preserved without artificial fallback." << std::endl;

  // Case B: Negative count rejected
  res = ctrl.updateApproach('N', -1, 10.0f, 1.0f, true);
  assert(res == false);
  assert(ctrl.getApproachState('N').is_valid == false);
  std::cout << "  [PASS] Negative vehicle count rejected." << std::endl;

  // Case C: Non-finite values (NaN / Inf) rejected
  float nan_val = std::nanf("");
  res = ctrl.updateApproach('N', 2, nan_val, 2.0f, true);
  assert(res == false);
  res = ctrl.updateApproach('N', 2, 10.0f, INFINITY, true);
  assert(res == false);
  std::cout << "  [PASS] NaN and Inf telemetry values rejected." << std::endl;

  // Case D: Negative distance rejected
  res = ctrl.updateApproach('N', 2, -5.0f, 2.0f, true);
  assert(res == false);
  std::cout << "  [PASS] Negative distance values rejected." << std::endl;
}

void testPartialMeasurementGapOutSuppression() {
  std::cout << "\n--- C++ Test 2: Partial Measurement & Gap-Out Suppression ---" << std::endl;
  TrafficController ctrl;

  // Setup NS Green with 16.0 PCU demand (allocates 19.5s green)
  ctrl.setMockTimeMs(0);
  ctrl.updateApproach('N', 8, 35.0f, 8.0f, true);
  ctrl.updateApproach('S', 8, 35.0f, 8.0f, true);
  ctrl.tick();

  // Advance past 2.0s startup hold to enter NS Green
  ctrl.setMockTimeMs(2100);
  ctrl.tick();
  assert(ctrl.getState() == STATE_NS_GREEN);
  assert(ctrl.getAllocatedGreen() == 19.5f);

  // At t = 13.0s (elapsed = 10.9s, past min-green 10s):
  // North and South are fully measured and empty (0.0 PCU)
  ctrl.setMockTimeMs(13000);
  ctrl.updateApproach('N', 0, 0.0f, 0.0f, true);
  ctrl.updateApproach('S', 0, 0.0f, 0.0f, true);

  // East camera is FRESH with 0.0 PCU. West camera is STALE/DISCONNECTED (>5s)
  // East = 0.0 (measured), West = 2.0 (fallback) -> Total EW = 2.0 PCU, status = PHASE_PARTIALLY_MEASURED
  ctrl.updateApproach('E', 0, 0.0f, 0.0f, true);
  ctrl.tick();

  // Conservative policy test: Gap-out must NOT occur because EW is partially measured (fallback cannot trigger gap-out)
  assert(ctrl.getState() == STATE_NS_GREEN);
  std::cout << "  [PASS] NS Green: Gap-out strictly suppressed when opposing EW has a stale sensor." << std::endl;

  // Now confirm West camera comes online with measured 4.0 PCU (now EW is PHASE_FULLY_MEASURED)
  ctrl.setMockTimeMs(13100);
  ctrl.updateApproach('W', 4, 20.0f, 4.0f, true);
  ctrl.tick();

  // Now with BOTH corridors confirmed fully measured, gap-out triggers cleanly!
  assert(ctrl.getState() == STATE_NS_YELLOW);
  std::cout << "  [PASS] Gap-out executed when opposing EW corridor became fully measured." << std::endl;
}

void testMirroredEwGapOutSuppression() {
  std::cout << "\n--- C++ Test 3: Mirrored EW Green Gap-Out Suppression ---" << std::endl;
  TrafficController ctrl;

  // Advance to EW Green with 16.0 PCU demand
  ctrl.setMockTimeMs(0);
  ctrl.updateApproach('N', 1, 5.0f, 1.0f, true);
  ctrl.updateApproach('S', 1, 5.0f, 1.0f, true);
  ctrl.updateApproach('E', 8, 35.0f, 8.0f, true);
  ctrl.updateApproach('W', 8, 35.0f, 8.0f, true);

  // Clear startup, NS green (10s), NS yellow (3s), All-red (1.5s) to enter EW green
  // Stream fresh 1 Hz telemetry updates for EW while NS is discharging
  unsigned long t = 0;
  while (ctrl.getState() != STATE_EW_GREEN) {
    t += 100;
    ctrl.setMockTimeMs(t);
    if (t % 1000 == 0) {
      ctrl.updateApproach('E', 8, 35.0f, 8.0f, true);
      ctrl.updateApproach('W', 8, 35.0f, 8.0f, true);
    }
    ctrl.tick();
  }
  assert(ctrl.getState() == STATE_EW_GREEN);
  assert(ctrl.getAllocatedGreen() == 19.5f);

  // At elapsed = 11.0s into EW green, EW queue clears to 0.0 PCU
  unsigned long ew_start = t;
  while (t - ew_start < 11000) {
    t += 100;
    ctrl.setMockTimeMs(t);
    ctrl.tick();
  }

  // EW is fully measured at 0.0 PCU. North is fresh at 0.0 PCU, but South is STALE (>5s)
  ctrl.updateApproach('E', 0, 0.0f, 0.0f, true);
  ctrl.updateApproach('W', 0, 0.0f, 0.0f, true);
  ctrl.updateApproach('N', 0, 0.0f, 0.0f, true);
  ctrl.tick();

  // Symmetrically: Gap-out must NOT occur because NS is partially measured
  assert(ctrl.getState() == STATE_EW_GREEN);
  std::cout << "  [PASS] EW Green: Gap-out strictly suppressed when opposing NS has a stale sensor." << std::endl;
}

void testContinuousWaitAccumulationAcrossClearance() {
  std::cout << "\n--- C++ Test 4: Continuous Wait Accumulation Across Clearance Phases ---" << std::endl;
  TrafficController ctrl;

  // Start in ALL_RED_STARTUP
  ctrl.setMockTimeMs(0);
  ctrl.updateApproach('N', 4, 20.0f, 4.0f, true);
  ctrl.updateApproach('S', 4, 20.0f, 4.0f, true);
  ctrl.updateApproach('E', 5, 25.0f, 5.0f, true);
  ctrl.updateApproach('W', 5, 25.0f, 5.0f, true);
  ctrl.tick();

  // Advance through startup (2.0s) into NS Green
  unsigned long t = 0;
  while (ctrl.getState() != STATE_NS_GREEN) {
    t += 100;
    ctrl.setMockTimeMs(t);
    ctrl.tick();
  }

  // Continuous demand on EW (10.0 PCU). Step clock through NS green (11.5s) and NS yellow (3.0s)
  while (ctrl.getState() == STATE_NS_GREEN) {
    t += 100;
    ctrl.setMockTimeMs(t);
    // Maintain fresh EW demand
    ctrl.updateApproach('E', 5, 25.0f, 5.0f, true);
    ctrl.updateApproach('W', 5, 25.0f, 5.0f, true);
    ctrl.tick();
  }

  float wait_at_green_exit = ctrl.getEwWaitTime();
  assert(wait_at_green_exit > 10.0f);

  // Now step through NS Yellow (3.0s)
  while (ctrl.getState() == STATE_NS_YELLOW) {
    t += 100;
    ctrl.setMockTimeMs(t);
    ctrl.updateApproach('E', 5, 25.0f, 5.0f, true);
    ctrl.updateApproach('W', 5, 25.0f, 5.0f, true);
    ctrl.tick();
  }

  float wait_at_yellow_exit = ctrl.getEwWaitTime();
  // Wait time must CONTINUE to accumulate during yellow clearance!
  assert(wait_at_yellow_exit > wait_at_green_exit);
  std::cout << "  [PASS] EW wait timer accumulated continuously through Yellow clearance ("
            << wait_at_green_exit << "s -> " << wait_at_yellow_exit << "s)." << std::endl;

  // Step through All-Red (1.5s) to EW Green
  while (ctrl.getState() != STATE_EW_GREEN) {
    t += 100;
    ctrl.setMockTimeMs(t);
    ctrl.updateApproach('E', 5, 25.0f, 5.0f, true);
    ctrl.updateApproach('W', 5, 25.0f, 5.0f, true);
    ctrl.tick();
  }

  // Once EW receives green service, its wait timer must reset to 0.0s!
  assert(ctrl.getEwWaitTime() == 0.0f);
  std::cout << "  [PASS] EW wait timer reset to 0.0s upon receiving green service." << std::endl;
}

void testFailsafeAndIllegalTransitions() {
  std::cout << "\n--- C++ Test 5: Strict Transition Graph & Failsafe Latching ---" << std::endl;
  TrafficController ctrl;

  // Initial state: ALL_RED_STARTUP
  assert(ctrl.getState() == STATE_ALL_RED_STARTUP);

  // Attempt illegal skip directly to EW_GREEN
  ctrl.switchState(STATE_EW_GREEN);
  assert(ctrl.getState() == STATE_FAILSAFE_FLASH);
  std::cout << "  [PASS] Illegal skip to EW_GREEN intercepted; forced into STATE_FAILSAFE_FLASH." << std::endl;

  // Once in FAILSAFE_FLASH, all transitions must be rejected (latched failsafe)
  ctrl.switchState(STATE_NS_GREEN);
  assert(ctrl.getState() == STATE_FAILSAFE_FLASH);
  ctrl.switchState(STATE_ALL_RED_TO_NS);
  assert(ctrl.getState() == STATE_FAILSAFE_FLASH);
  std::cout << "  [PASS] Failsafe mode strictly latched against further autonomous state changes." << std::endl;
}

void testCompleteSensorBlackoutFixedFallback() {
  std::cout << "\n--- C++ Test 6: Complete Sensor Blackout & Fixed-Time Fallback Mode ---" << std::endl;
  TrafficController ctrl;

  // Zero sensor observations ever delivered -> All sensors unobserved
  ctrl.setMockTimeMs(0);
  ctrl.tick();
  assert(ctrl.getOperatingMode() == MODE_FIXED_FALLBACK);

  // Advance past 2.0s startup hold to enter NS Green
  ctrl.setMockTimeMs(2100);
  ctrl.tick();
  assert(ctrl.getState() == STATE_NS_GREEN);
  // In MODE_FIXED_FALLBACK, allocated green MUST be fixed fallback green (25.0s), NOT 4.0 PCU dynamic formula (7.5s)
  assert(ctrl.getAllocatedGreen() == 25.0f);
  std::cout << "  [PASS] Sensor blackout correctly detected: Operating mode set to MODE_FIXED_FALLBACK." << std::endl;
  std::cout << "  [PASS] Fixed-time allocation of exactly 25.0s applied (dynamic green suppressed)." << std::endl;

  // Advance to t = 18.0s (elapsed = 15.9s, well past min-green 10.0s)
  ctrl.setMockTimeMs(18000);
  ctrl.tick();
  // In fixed fallback mode, gap-out must NEVER trigger
  assert(ctrl.getState() == STATE_NS_GREEN);
  std::cout << "  [PASS] Gap-out strictly suppressed during fixed-time fallback operation." << std::endl;

  // Advance to t = 27.100s (elapsed = 25.0s) - green expires normally
  ctrl.setMockTimeMs(27150);
  ctrl.tick();
  assert(ctrl.getState() == STATE_NS_YELLOW);

  // Advance through Yellow (3.0s) and All-red (1.5s) to enter EW Green
  ctrl.setMockTimeMs(30200);
  ctrl.tick();
  assert(ctrl.getState() == STATE_ALL_RED_TO_EW);

  ctrl.setMockTimeMs(31750);
  ctrl.tick();
  assert(ctrl.getState() == STATE_EW_GREEN);
  // EW Green also receives exactly 25.0s fixed allocation
  assert(ctrl.getAllocatedGreen() == 25.0f);
  std::cout << "  [PASS] Symmetrical fixed-time 25.0s green allocated to EW corridor." << std::endl;
}

int main() {
  std::cout << "=================================================================" << std::endl;
  std::cout << "  TrafficController C++ Native Verification Test Suite (v3.3)" << std::endl;
  std::cout << "=================================================================" << std::endl;

  testInputValidation();
  testPartialMeasurementGapOutSuppression();
  testMirroredEwGapOutSuppression();
  testContinuousWaitAccumulationAcrossClearance();
  testFailsafeAndIllegalTransitions();
  testCompleteSensorBlackoutFixedFallback();

  std::cout << "\n=================================================================" << std::endl;
  std::cout << "  ALL 6 C++ EMBEDDED CONTROLLER VERIFICATION TESTS PASSED (0 ERRORS)." << std::endl;
  std::cout << "=================================================================" << std::endl;
  return 0;
}
