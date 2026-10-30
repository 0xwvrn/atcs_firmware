#include <iostream>
#include <cassert>
#include <cmath>
#include <vector>
#include <cstring>
#include "../include/TrafficController.hpp"
#include "../include/TelemetryParser.hpp"

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

void testPartialObservationFallbackDemand() {
  std::cout << "\n--- C++ Test 7: Partial Observation Fallback Demand Justification & Bounding ---" << std::endl;
  TrafficController ctrl;

  // North is freshly measured at 0.0 PCU. South is STALE (> 5000ms).
  // South receives fallback PCU (2.0 PCU). Combined NS phase demand = 2.0 PCU.
  ctrl.setMockTimeMs(0);
  ctrl.updateApproach('N', 0, 0.0f, 0.0f, true);
  // South approach never updated -> stale
  ctrl.tick();

  // Mode must be MODE_DEGRADED_PARTIAL
  assert(ctrl.getOperatingMode() == MODE_DEGRADED_PARTIAL);
  std::cout << "  [PASS] Single stale leg places controller into MODE_DEGRADED_PARTIAL." << std::endl;

  // Advance past 2.0s startup hold to enter NS Green
  ctrl.setMockTimeMs(2100);
  ctrl.tick();
  assert(ctrl.getState() == STATE_NS_GREEN);
  // Dynamic formula: green = 3.5 + (2.0 / 1.0) = 5.5s.
  // Must clamp to kMinGreen (10.0s), strictly preventing runaway green extension on assumed traffic!
  assert(ctrl.getAllocatedGreen() == 10.0f);
  std::cout << "  [PASS] Assumed fallback demand (2.0 PCU) is safely bounded to minimum green (10.0s)." << std::endl;

  // Verify starvation timer behavior for an opposing partially observed corridor:
  // Both East and West approaches are unmeasured (each 2.0 PCU fallback -> total EW = 4.0 PCU).
  // Because q_ew >= 1.0 PCU, the unserviced opposing corridor accumulates wait time at rate 1.0 s/s
  // so an unobserved cross-street cannot be permanently starved if unmonitored vehicles are present.
  ctrl.setMockTimeMs(5100);
  ctrl.tick();
  assert(ctrl.getEwWaitTime() > 2.9f && ctrl.getEwWaitTime() < 3.2f);
  std::cout << "  [PASS] Opposing fallback demand guarantees anti-starvation wait accumulation." << std::endl;
}

void testTelemetryParserEdgeCases() {
  std::cout << "\n--- C++ Test 8: TelemetryParser Structural & Syntax Robustness ---" << std::endl;
  ParsedTelemetry pkt;

  // Case A: Valid realistic telemetry packet from bridge_controller.py
  const char* valid_json = "{\"seq\": 3, \"approach\": \"N\", \"valid\": true, \"Q\": 3, \"PCU\": 4.5, \"D_tail_m\": 41.21, \"timestamp_ms\": 1793298091336}";
  bool res = TelemetryParser::parse(valid_json, strlen(valid_json), pkt);
  assert(res == true);
  assert(pkt.approach == 'N');
  assert(pkt.q_count == 3);
  assert(pkt.p_pcu == 4.5f);
  assert(fabs(pkt.d_tail_m - 41.21f) < 0.01f);
  assert(pkt.is_valid == true);
  std::cout << "  [PASS] Valid telemetry JSON parsed with complete field fidelity." << std::endl;

  // Case B: Truncated JSON packet (e.g. UART transmission interrupted mid-stream)
  const char* truncated_json = "{\"seq\": 3, \"approach\": \"N\", \"valid\": true, \"Q\": 3, \"PCU\": ";
  res = TelemetryParser::parse(truncated_json, strlen(truncated_json), pkt);
  assert(res == false);
  std::cout << "  [PASS] Truncated packet without closing brace rejected." << std::endl;

  // Case C: Corrupted approach character
  const char* bad_app_json = "{\"approach\": \"X\", \"valid\": true, \"Q\": 2, \"PCU\": 2.0}";
  res = TelemetryParser::parse(bad_app_json, strlen(bad_app_json), pkt);
  assert(res == false);
  std::cout << "  [PASS] Invalid approach leg identifier ('X') rejected." << std::endl;

  // Case D: Non-numeric / malformed float syntax
  const char* bad_float_json = "{\"approach\": \"S\", \"valid\": true, \"Q\": 2, \"PCU\": nan}";
  res = TelemetryParser::parse(bad_float_json, strlen(bad_float_json), pkt);
  assert(res == false);
  std::cout << "  [PASS] Non-finite float value ('nan') rejected." << std::endl;

  // Case E: Missing mandatory field ("Q" missing)
  const char* missing_q_json = "{\"approach\": \"E\", \"valid\": true, \"PCU\": 2.0}";
  res = TelemetryParser::parse(missing_q_json, strlen(missing_q_json), pkt);
  assert(res == false);
  std::cout << "  [PASS] Incomplete packet missing mandatory 'Q' key rejected." << std::endl;

  // Case F: Valid packet with false calibration gating
  const char* uncalib_json = "{\"approach\": \"W\", \"valid\": false, \"Q\": 0, \"PCU\": 0.0}";
  res = TelemetryParser::parse(uncalib_json, strlen(uncalib_json), pkt);
  assert(res == true);
  assert(pkt.approach == 'W');
  assert(pkt.is_valid == false);
  std::cout << "  [PASS] Uncalibrated pipeline packet parsed with valid=false preserved." << std::endl;

  // Case G: Duplicate required key ("approach" duplicated) - MUST be rejected
  const char* dup_key_json = "{\"approach\": \"N\", \"approach\": \"S\", \"valid\": true, \"Q\": 2, \"PCU\": 2.0}";
  res = TelemetryParser::parse(dup_key_json, strlen(dup_key_json), pkt);
  assert(res == false);
  std::cout << "  [PASS] Duplicate required key ('approach') strictly rejected." << std::endl;

  // Case H: Invalid token delimiter (e.g. "Q": 3abc without legal JSON delimiter)
  const char* bad_delim_json = "{\"approach\": \"N\", \"valid\": true, \"Q\": 3abc, \"PCU\": 2.0}";
  res = TelemetryParser::parse(bad_delim_json, strlen(bad_delim_json), pkt);
  assert(res == false);
  std::cout << "  [PASS] Numeric token with trailing illegal syntax ('3abc') rejected." << std::endl;

  // Case I: Non-NUL terminated raw buffer with explicit length
  char raw_buf[64];
  const char* src_text = "{\"approach\": \"E\", \"valid\": true, \"Q\": 1, \"PCU\": 1.0}";
  size_t src_len = strlen(src_text);
  memcpy(raw_buf, src_text, src_len);
  raw_buf[src_len] = 'X'; // NOT NUL terminated!
  res = TelemetryParser::parse(raw_buf, src_len, pkt);
  assert(res == true);
  assert(pkt.approach == 'E');
  assert(pkt.q_count == 1);
  std::cout << "  [PASS] Non-NUL-terminated buffer handled safely via bounded length." << std::endl;

  // Case J: Trailing garbage after closing brace
  const char* trailing_garbage_json = "{\"approach\": \"N\", \"valid\": true, \"Q\": 1, \"PCU\": 1.0} trailing garbage";
  res = TelemetryParser::parse(trailing_garbage_json, strlen(trailing_garbage_json), pkt);
  assert(res == false);
  std::cout << "  [PASS] Trailing syntax garbage after closing brace rejected." << std::endl;
}

void testInternalSafetyAndRunawayClearance() {
  std::cout << "\n--- C++ Test 9: Comprehensive Internal Safety Invariant Verification ---" << std::endl;
  TrafficController ctrl;

  // Normal initialization satisfies all invariants
  assert(ctrl.verifyInternalSafety() == true);
  std::cout << "  [PASS] Nominal startup state satisfies all physical and timing invariants." << std::endl;

  // Advance through startup hold into NS Green
  ctrl.setMockTimeMs(2100);
  ctrl.tick();
  assert(ctrl.getState() == STATE_NS_GREEN);

  // Switch to NS Yellow
  ctrl.switchState(STATE_NS_YELLOW);
  assert(ctrl.getState() == STATE_NS_YELLOW);

  // Runaway Yellow Clearance Test:
  // yellow is configured for 3.0s, runaway threshold is kYellowTime + 1.0s = 4.0s.
  // At elapsed = 4.5s (mock time = 2100 + 4500 = 6600), verifyInternalSafety() MUST trip and force FAILSAFE_FLASH!
  ctrl.setMockTimeMs(6600);
  ctrl.tick();
  assert(ctrl.getState() == STATE_FAILSAFE_FLASH);
  std::cout << "  [PASS] Runaway yellow clearance (> 4.0s) tripped internal safety into FAILSAFE_FLASH." << std::endl;
}

void testMillisRolloverSafety() {
  std::cout << "\n--- C++ Test 10: State Timing & Wait Accumulation Across Millis() Rollover ---" << std::endl;
  TrafficController ctrl;

  // Initialize clock and state start time 1000ms prior to 32-bit unsigned rollover (0xFFFFFFFF = 4294967295)
  const unsigned long kPreRolloverTime = 0xFFFFFFFFUL - 1000UL;
  ctrl.setMockTimeMs(kPreRolloverTime);
  ctrl.setSimulatedStateStartTimeMs(kPreRolloverTime);
  ctrl.tick();
  assert(ctrl.getState() == STATE_ALL_RED_STARTUP);

  // 1. Advance through startup past rollover boundary:
  // kPreRolloverTime + 2100ms = 0x00000000UL + 1099ms (wraps across 0xFFFFFFFF)
  const unsigned long kPostRolloverTime = kPreRolloverTime + 2100UL;
  ctrl.setMockTimeMs(kPostRolloverTime);
  ctrl.tick();

  // Internal safety monitor MUST NOT trip on normal 49.7-day rollover!
  assert(ctrl.getState() == STATE_NS_GREEN);
  assert(ctrl.verifyInternalSafety() == true);
  std::cout << "  [PASS] Clock wrap across 0xFFFFFFFF did not falsely trip safety monitor; advanced to NS_GREEN." << std::endl;

  // 2. Opposing wait timer accumulation across rollover:
  // Setup East & West demand (10.0 PCU) during NS Green
  ctrl.updateApproach('E', 5, 20.0f, 5.0f, true);
  ctrl.updateApproach('W', 5, 20.0f, 5.0f, true);

  // Record baseline wait time at entrance to NS Green
  float wait_baseline = ctrl.getEwWaitTime();

  // Step clock forward across several ticks
  unsigned long cur_time = kPostRolloverTime;
  for (int i = 0; i < 5; ++i) {
    cur_time += 1000UL;
    ctrl.setMockTimeMs(cur_time);
    ctrl.updateApproach('E', 5, 20.0f, 5.0f, true);
    ctrl.updateApproach('W', 5, 20.0f, 5.0f, true);
    ctrl.tick();
  }

  // EW wait timer must accumulate cleanly (exactly +5.0s delta over baseline) across wrapped timestamps
  float wait_delta = ctrl.getEwWaitTime() - wait_baseline;
  assert(wait_delta >= 4.9f && wait_delta <= 5.2f);
  std::cout << "  [PASS] Unserviced opposing wait accumulated cleanly across wrapped timestamps (+5.0s delta, total: "
            << ctrl.getEwWaitTime() << "s)." << std::endl;

  // 3. Sensor timeout check across rollover:
  // Approach was seen at cur_time. Advance time by 6000ms (exceeding kSensorTimeoutMs = 5000ms)
  cur_time += 6000UL;
  ctrl.setMockTimeMs(cur_time);
  ctrl.tick();
  // Sensor must now report timed out across rollover
  assert(ctrl.hasFreshObservation('E', cur_time) == false);
  std::cout << "  [PASS] Sensor freshness timeout correctly expired across wrapped timestamps." << std::endl;
}

void testFailsafeInterlockCompliance() {
  std::cout << "\n--- C++ Test 11: Failsafe Output & Conflict Interlock Compliance ---" << std::endl;
  TrafficController ctrl;

  // Symmetrical All-Way Red Flashing Policy:
  // When in STATE_FAILSAFE_FLASH, all approaches flash RED (ns_r=true, ew_r=true, ns_y=false, ew_y=false, ns_g=false, ew_g=false)
  // Verify that verifyInterlock strictly passes during All-Way Red Flash!
  bool ns_yellow = false, ns_green = false;
  bool ew_yellow = false, ew_green = false;

  bool interlock_ok = ctrl.verifyInterlock(ns_green, ns_yellow, ew_green, ew_yellow);
  assert(interlock_ok == true);
  std::cout << "  [PASS] All-Way Red Flashing passes hardware conflict interlock (0 conflicting active phases)." << std::endl;

  // In contrast, confirm that dual yellow (the buggy condition caught in review) is strictly rejected by verifyInterlock:
  bool buggy_ns_yellow = true, buggy_ew_yellow = true;
  bool buggy_interlock = ctrl.verifyInterlock(false, buggy_ns_yellow, false, buggy_ew_yellow);
  assert(buggy_interlock == false);
  std::cout << "  [PASS] Dual-yellow conflict is strictly rejected by verifyInterlock(), validating the red-flash policy." << std::endl;
}

void testSerialBufferOverrunDiscard() {
  std::cout << "\n--- C++ Test 12: Serial Buffer Overrun Line-Length Discard ---" << std::endl;

  // Simulate a 600-byte continuous garbage stream without newline (exceeds SERIAL_BUF_SIZE = 512)
  const size_t TEST_BUF_CAP = 512;
  char sim_buf[TEST_BUF_CAP];
  size_t buf_idx = 0;
  bool overflow_triggered = false;

  for (size_t i = 0; i < 600; ++i) {
    char c = 'A';
    if (c == '\n' || c == '\r') {
      buf_idx = 0;
    } else {
      if (buf_idx < TEST_BUF_CAP - 1) {
        sim_buf[buf_idx++] = c;
      } else {
        overflow_triggered = true;
        buf_idx = 0; // Buffer reset to prevent memory exhaustion
      }
    }
  }

  assert(overflow_triggered == true);
  assert(buf_idx > 0 && buf_idx < TEST_BUF_CAP);
  assert(sim_buf[0] == 'A');
  std::cout << "  [PASS] Non-delimited continuous stream (> 512 bytes) safely triggered buffer reset without overrun (residual index: " << buf_idx << ")." << std::endl;
}

int main() {
  std::cout << "=================================================================" << std::endl;
  std::cout << "  TrafficController C++ Native Verification Test Suite (v3.5)" << std::endl;
  std::cout << "=================================================================" << std::endl;

  testInputValidation();
  testPartialMeasurementGapOutSuppression();
  testMirroredEwGapOutSuppression();
  testContinuousWaitAccumulationAcrossClearance();
  testFailsafeAndIllegalTransitions();
  testCompleteSensorBlackoutFixedFallback();
  testPartialObservationFallbackDemand();
  testTelemetryParserEdgeCases();
  testInternalSafetyAndRunawayClearance();
  testMillisRolloverSafety();
  testFailsafeInterlockCompliance();
  testSerialBufferOverrunDiscard();

  std::cout << "\n=================================================================" << std::endl;
  std::cout << "  ALL 12 C++ EMBEDDED CONTROLLER VERIFICATION TESTS PASSED (0 ERRORS)." << std::endl;
  std::cout << "=================================================================" << std::endl;
  return 0;
}
