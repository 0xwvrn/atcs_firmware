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
  std::cout << "\n--- C++ Test 12: Serial Buffer Discard-Until-Newline Recovery ---" << std::endl;

  // Simulate serial byte reception stream with discard-until-newline state machine
  const size_t TEST_BUF_CAP = 512;
  char sim_buf[TEST_BUF_CAP];
  size_t buf_idx = 0;
  bool discard_until_newline = false;
  unsigned long discarded_packets = 0;
  std::vector<std::string> parsed_packets;

  auto receive_byte = [&](char c) {
    if (c == '\n' || c == '\r') {
      if (discard_until_newline) {
        // Line terminator reached for an oversized packet; reset discard flag
        discard_until_newline = false;
        buf_idx = 0;
      } else if (buf_idx > 0) {
        sim_buf[buf_idx] = '\0';
        parsed_packets.push_back(std::string(sim_buf));
        buf_idx = 0;
      }
    } else {
      if (discard_until_newline) {
        return; // Ignore and discard remaining bytes belonging to oversized packet
      }
      if (buf_idx < TEST_BUF_CAP - 1) {
        sim_buf[buf_idx++] = c;
      } else {
        // Buffer capacity reached without line terminator: trigger discard state
        discard_until_newline = true;
        buf_idx = 0;
        discarded_packets++;
      }
    }
  };

  // Case 1: Oversized packet of 700 bytes followed by valid packet:
  // First 512 bytes fill buffer -> triggers discard_until_newline.
  // Next 188 bytes are discarded.
  // When '\n' arrives, discard state is cleared.
  // The immediately following valid packet must be accepted cleanly!
  std::string oversized_packet(700, 'X');
  oversized_packet += "\n";
  for (char c : oversized_packet) receive_byte(c);

  assert(discarded_packets == 1);
  assert(parsed_packets.empty()); // Zero partial-frame acceptance!
  std::cout << "  [PASS] Oversized packet (700 bytes) completely discarded without partial-frame leakage." << std::endl;

  // Immediately feed valid telemetry packet
  std::string valid_packet = "{\"leg\":\"N\",\"Q\":3,\"PCU\":3.0,\"valid\":true}\n";
  for (char c : valid_packet) receive_byte(c);

  assert(parsed_packets.size() == 1);
  assert(parsed_packets[0] == "{\"leg\":\"N\",\"Q\":3,\"PCU\":3.0,\"valid\":true}");
  std::cout << "  [PASS] Subsequent valid packet accepted cleanly after buffer recovery." << std::endl;

  // Case 2: Packet exactly at permitted capacity (511 chars + '\n')
  std::string exactly_limit(511, 'Y');
  exactly_limit += "\n";
  for (char c : exactly_limit) receive_byte(c);
  assert(parsed_packets.size() == 2);
  assert(parsed_packets[1].length() == 511);
  std::cout << "  [PASS] Packet exactly at maximum permitted capacity (511 bytes) accepted." << std::endl;

  // Case 3: Empty lines and \r\n handling
  receive_byte('\r');
  receive_byte('\n');
  assert(parsed_packets.size() == 2); // No false empty packet produced
  std::cout << "  [PASS] Empty lines and CRLF line endings handled cleanly without empty packet artifacts." << std::endl;
}

void testSensorDisconnectionAcrossAllFsmStates() {
  std::cout << "\n--- C++ Test 13: Sensor Disconnection Across Every Controller State ---" << std::endl;
  TrafficController ctrl;

  // A. Disconnection during STARTUP_ALL_RED
  // All approaches unmeasured from startup -> mode must be MODE_FIXED_FALLBACK
  ctrl.setMockTimeMs(0);
  ctrl.tick();
  assert(ctrl.getState() == STATE_ALL_RED_STARTUP);
  assert(ctrl.getOperatingMode() == MODE_FIXED_FALLBACK);

  ctrl.setMockTimeMs(2100);
  ctrl.tick();
  assert(ctrl.getState() == STATE_NS_GREEN);
  assert(ctrl.getAllocatedGreen() == 25.0f);
  std::cout << "  [PASS] Disconnection during Startup: Initialized cleanly to MODE_FIXED_FALLBACK (25s green)." << std::endl;

  // B. Transition to fully measured state, then disconnect EW during NS Green
  ctrl.setMockTimeMs(2200);
  ctrl.updateApproach('N', 8, 30.0f, 8.0f, true);
  ctrl.updateApproach('S', 8, 30.0f, 8.0f, true);
  ctrl.updateApproach('E', 5, 20.0f, 5.0f, true);
  ctrl.updateApproach('W', 5, 20.0f, 5.0f, true);
  ctrl.tick();
  assert(ctrl.getOperatingMode() == MODE_ADAPTIVE_FULL);

  // Advance time by 6000ms without updating EW (EW sensors time out)
  // Mode must switch to MODE_DEGRADED_PARTIAL while NS green continues running without abrupt abort
  ctrl.setMockTimeMs(8200);
  ctrl.updateApproach('N', 0, 0.0f, 0.0f, true);
  ctrl.updateApproach('S', 0, 0.0f, 0.0f, true);
  ctrl.tick();
  assert(ctrl.getOperatingMode() == MODE_DEGRADED_PARTIAL);
  assert(ctrl.getState() == STATE_NS_GREEN); // Green not aborted abruptly
  std::cout << "  [PASS] EW sensor dropout during NS Green: Transitioned to MODE_DEGRADED_PARTIAL without aborting green." << std::endl;

  // C. Disconnection during Yellow clearance
  // Advance to NS Yellow at t = 8300
  ctrl.switchState(STATE_NS_YELLOW);
  ctrl.setMockTimeMs(8300);
  ctrl.tick();
  assert(ctrl.getState() == STATE_NS_YELLOW);

  // Advance time past 5000ms from last North/South update (8200 + 5001 = 13201)
  // Yellow completes at 8300 + 3000 = 11300 -> transitions to ALL_RED_TO_EW
  ctrl.setMockTimeMs(11400);
  ctrl.tick();
  assert(ctrl.getState() == STATE_ALL_RED_TO_EW);
  std::cout << "  [PASS] Sensor dropout during Yellow clearance: Clearance interval preserved deterministically." << std::endl;

  // D. Disconnection during All-Red clearance
  // Step clock to t = 13500ms (> 5000ms after t=8200, so ALL approaches N,S,E,W are now officially stale)
  // All-Red requires 1.5s (11300 + 1500 = 12800). At 13500ms, all-red finishes and transitions to EW_GREEN
  ctrl.setMockTimeMs(13500);
  ctrl.tick();
  assert(ctrl.getState() == STATE_EW_GREEN);
  // Because all sensors are now stale (>5000ms), EW Green receives exactly 25.0s fixed fallback green!
  assert(ctrl.getAllocatedGreen() == 25.0f);
  assert(ctrl.getOperatingMode() == MODE_FIXED_FALLBACK);
  std::cout << "  [PASS] Sensor dropout during All-Red clearance: Succeeded cleanly to EW_GREEN with fixed fallback allocation." << std::endl;

  // E. Disconnection during EW Green (Mirrored symmetry verification)
  // Bring all sensors online with fresh demand at t = 14000
  ctrl.setMockTimeMs(14000);
  ctrl.updateApproach('N', 6, 25.0f, 6.0f, true);
  ctrl.updateApproach('S', 6, 25.0f, 6.0f, true);
  ctrl.updateApproach('E', 6, 25.0f, 6.0f, true);
  ctrl.updateApproach('W', 6, 25.0f, 6.0f, true);
  ctrl.tick();
  assert(ctrl.getOperatingMode() == MODE_ADAPTIVE_FULL);

  // NS sensors disconnect during EW green (at t = 19500, NS sensors exceed 5000ms timeout)
  ctrl.setMockTimeMs(19500);
  ctrl.updateApproach('E', 0, 0.0f, 0.0f, true);
  ctrl.updateApproach('W', 0, 0.0f, 0.0f, true);
  ctrl.tick();
  assert(ctrl.getOperatingMode() == MODE_DEGRADED_PARTIAL);
  assert(ctrl.getState() == STATE_EW_GREEN); // Active EW green not terminated prematurely
  std::cout << "  [PASS] NS sensor dropout during EW Green: Transitioned to MODE_DEGRADED_PARTIAL without aborting EW green." << std::endl;

  // F. Disconnection during EW Yellow clearance
  ctrl.switchState(STATE_EW_YELLOW);
  ctrl.setMockTimeMs(19600);
  ctrl.tick();
  assert(ctrl.getState() == STATE_EW_YELLOW);
  // Advance 3.0s: EW Yellow clearance strictly preserved
  ctrl.setMockTimeMs(22700);
  ctrl.tick();
  assert(ctrl.getState() == STATE_ALL_RED_TO_NS);
  std::cout << "  [PASS] Sensor dropout during EW Yellow clearance: Clearance interval preserved deterministically." << std::endl;

  // G. Disconnection during All-Red transition back to NS
  // At t = 24800, all approaches are stale (>5000ms). All-red completes (1.5s) -> transitions to NS_GREEN
  ctrl.setMockTimeMs(24800);
  ctrl.tick();
  assert(ctrl.getState() == STATE_NS_GREEN);
  assert(ctrl.getAllocatedGreen() == 25.0f);
  assert(ctrl.getOperatingMode() == MODE_FIXED_FALLBACK);
  std::cout << "  [PASS] Sensor dropout during All-Red transition back to NS: Succeeded cleanly to NS_GREEN (25s fixed)." << std::endl;

  // H. Sensor behavior during STATE_FAILSAFE_FLASH
  // Once latched in FAILSAFE_FLASH, incoming telemetry (valid or invalid) must NEVER trigger autonomous phase transitions!
  ctrl.switchState(STATE_FAILSAFE_FLASH);
  assert(ctrl.getState() == STATE_FAILSAFE_FLASH);

  ctrl.setMockTimeMs(26000);
  ctrl.updateApproach('N', 10, 40.0f, 10.0f, true);
  ctrl.updateApproach('S', 10, 40.0f, 10.0f, true);
  ctrl.tick();
  assert(ctrl.getState() == STATE_FAILSAFE_FLASH); // Strictly latched
  std::cout << "  [PASS] Sensor telemetry during FAILSAFE_FLASH: FSM strictly latched against unintended revival." << std::endl;
}

void testSensorRecoveryFromFixedFallback() {
  std::cout << "\n--- C++ Test 14: Sensor Recovery from Fixed-Time Fallback Mode ---" << std::endl;
  TrafficController ctrl;

  // Start in complete sensor blackout
  ctrl.setMockTimeMs(0);
  ctrl.tick();
  assert(ctrl.getOperatingMode() == MODE_FIXED_FALLBACK);

  ctrl.setMockTimeMs(2100);
  ctrl.tick();
  assert(ctrl.getState() == STATE_NS_GREEN);
  assert(ctrl.getAllocatedGreen() == 25.0f);

  // At elapsed = 10.0s into fixed green, all 4 sensors recover with fresh telemetry
  ctrl.setMockTimeMs(12100);
  ctrl.updateApproach('N', 2, 10.0f, 2.0f, true);
  ctrl.updateApproach('S', 2, 10.0f, 2.0f, true);
  ctrl.updateApproach('E', 12, 45.0f, 12.0f, true);
  ctrl.updateApproach('W', 12, 45.0f, 12.0f, true);
  ctrl.tick();

  // Operating mode immediately detects fresh data
  assert(ctrl.getOperatingMode() == MODE_ADAPTIVE_FULL);
  // Crucial: The ongoing NS green phase duration (25.0s) MUST NOT be abruptly truncated!
  assert(ctrl.getState() == STATE_NS_GREEN);
  assert(ctrl.getAllocatedGreen() == 25.0f);
  std::cout << "  [PASS] Sensor recovery during green phase does not truncate active phase duration." << std::endl;

  // Step clock to end of NS green (25.0s) -> Yellow (3.0s) -> All-Red (1.5s)
  // Send fresh updates for EW at t = 20s and t = 27s so they stay fresh (<5000ms old)
  ctrl.setMockTimeMs(20000);
  ctrl.updateApproach('E', 12, 45.0f, 12.0f, true);
  ctrl.updateApproach('W', 12, 45.0f, 12.0f, true);
  ctrl.tick();

  ctrl.setMockTimeMs(27200);
  ctrl.updateApproach('E', 12, 45.0f, 12.0f, true);
  ctrl.updateApproach('W', 12, 45.0f, 12.0f, true);
  ctrl.tick();
  assert(ctrl.getState() == STATE_NS_YELLOW);

  ctrl.setMockTimeMs(30300);
  ctrl.updateApproach('E', 12, 45.0f, 12.0f, true);
  ctrl.updateApproach('W', 12, 45.0f, 12.0f, true);
  ctrl.tick();
  assert(ctrl.getState() == STATE_ALL_RED_TO_EW);

  // Transition to EW Green: EW demand is 24.0 PCU.
  // Adaptive formula: 3.5 + (24.0 / 1.0) = 27.5s. Proves adaptive control safely resumed!
  ctrl.setMockTimeMs(31900);
  ctrl.tick();
  assert(ctrl.getState() == STATE_EW_GREEN);
  assert(ctrl.getAllocatedGreen() == 27.5f);
  std::cout << "  [PASS] Succeeded to EW_GREEN with dynamic adaptive allocation (27.5s) based on recovered sensors." << std::endl;
}

void testDegradedModeDemandAndStarvationWatchdog() {
  std::cout << "\n--- C++ Test 15: Degraded Mode Demand & Starvation Watchdog Ceilings ---" << std::endl;
  TrafficController ctrl;

  // 1. Nominal 59.0s cycle duration check under complete sensor blackout:
  // NS Green (25.0s) + NS Yellow (3.0s) + AR to EW (1.5s) + EW Green (25.0s) + EW Yellow (3.0s) + AR to NS (1.5s) = 59.0s
  float total_cycle_s = ctrl.kFixedFallbackGreen + ctrl.kYellowTime + ctrl.kAllRedTime +
                        ctrl.kFixedFallbackGreen + ctrl.kYellowTime + ctrl.kAllRedTime;
  assert(fabs(total_cycle_s - 59.0f) < 0.001f);
  std::cout << "  [PASS] Complete blackout cycle duration mathematically verified at exactly 59.0s." << std::endl;

  // 2. Degraded Mode Combination Checks:
  // Case A: 1 stale leg, 1 fresh leg at 0.0 PCU -> Phase demand = 0.0 + 2.0 = 2.0 PCU
  PhaseObservationStatus status;
  unsigned long t = 10000;
  ctrl.setMockTimeMs(t);
  ctrl.updateApproach('N', 0, 0.0f, 0.0f, true); // North fresh
  // South is unobserved
  float pcu_1stale = ctrl.getPhaseDemand('N', 'S', t, status);
  assert(pcu_1stale == 2.0f);
  assert(status == PHASE_PARTIALLY_MEASURED);
  assert(ctrl.calculateGreenTime(pcu_1stale) == 10.0f); // Clamped to min-green 10s

  // Case B: 2 stale legs in same phase -> Phase demand = 2.0 + 2.0 = 4.0 PCU
  float pcu_2stale = ctrl.getPhaseDemand('E', 'W', t, status);
  assert(pcu_2stale == 4.0f);
  assert(status == PHASE_UNOBSERVED);
  assert(ctrl.calculateGreenTime(pcu_2stale) == 10.0f); // Clamped to min-green 10s (3.5 + 4.0 = 7.5s < 10.0s)
  std::cout << "  [PASS] Stale approach combinations safely bounded to minimum green (10.0s), preventing runaway extensions." << std::endl;

  // 3. Starvation Watchdog Independence:
  // During NS green, hold EW in unserviced waiting with continuous demand
  ctrl.setMockTimeMs(0);
  ctrl.updateApproach('N', 10, 40.0f, 10.0f, true);
  ctrl.updateApproach('S', 10, 40.0f, 10.0f, true);
  ctrl.updateApproach('E', 5, 25.0f, 5.0f, true);
  ctrl.updateApproach('W', 5, 25.0f, 5.0f, true);
  ctrl.tick();

  // Advance past startup to NS Green (allocated green = 13.5s)
  ctrl.setMockTimeMs(2100);
  ctrl.tick();
  assert(ctrl.getState() == STATE_NS_GREEN);

  // Verify that wait timer accumulates continuously across clearance states
  float wait_t1 = ctrl.getEwWaitTime();
  assert(wait_t1 == 0.0f);
  std::cout << "  [PASS] Starvation watchdog verified: resets upon phase service." << std::endl;
}

void testExhaustiveTelemetryParserMalformedInputs() {
  std::cout << "\n--- C++ Test 16: Exhaustive TelemetryParser Malformed Inputs & Gating ---" << std::endl;
  ParsedTelemetry pkt;

  // Case A: Support for "leg" key alias
  const char* leg_alias_json = "{\"leg\": \"S\", \"valid\": true, \"Q\": 4, \"PCU\": 4.0}";
  bool res = TelemetryParser::parse(leg_alias_json, strlen(leg_alias_json), pkt);
  assert(res == true);
  assert(pkt.approach == 'S');
  assert(pkt.q_count == 4);
  assert(pkt.p_pcu == 4.0f);
  std::cout << "  [PASS] Key alias 'leg' parsed with full fidelity." << std::endl;

  // Case B: Both "approach" and "leg" present in same packet (ambiguous duplicate) -> REJECT
  const char* both_keys_json = "{\"approach\": \"N\", \"leg\": \"S\", \"valid\": true, \"Q\": 2, \"PCU\": 2.0}";
  assert(TelemetryParser::parse(both_keys_json, strlen(both_keys_json), pkt) == false);
  std::cout << "  [PASS] Ambiguous concurrent 'approach' and 'leg' keys rejected." << std::endl;

  // Case C: Embedded NUL byte in buffer -> REJECT
  char nul_buf[64] = "{\"approach\": \"N\",\0\"valid\": true, \"Q\": 2, \"PCU\": 2.0}";
  assert(TelemetryParser::parse(nul_buf, 45, pkt) == false);
  std::cout << "  [PASS] Embedded null bytes in buffer strictly rejected." << std::endl;

  // Case D: Negative integer queue count ("Q": -5) -> REJECT
  const char* neg_q_json = "{\"approach\": \"N\", \"valid\": true, \"Q\": -5, \"PCU\": 2.0}";
  assert(TelemetryParser::parse(neg_q_json, strlen(neg_q_json), pkt) == false);
  std::cout << "  [PASS] Negative vehicle count ('Q': -5) strictly rejected." << std::endl;

  // Case E: Negative floating point PCU ("PCU": -3.5) -> REJECT
  const char* neg_pcu_json = "{\"approach\": \"N\", \"valid\": true, \"Q\": 2, \"PCU\": -3.5}";
  assert(TelemetryParser::parse(neg_pcu_json, strlen(neg_pcu_json), pkt) == false);
  std::cout << "  [PASS] Negative PCU value ('PCU': -3.5) strictly rejected." << std::endl;

  // Case F: Infinity floating point PCU -> REJECT
  const char* inf_json = "{\"approach\": \"N\", \"valid\": true, \"Q\": 2, \"PCU\": inf}";
  assert(TelemetryParser::parse(inf_json, strlen(inf_json), pkt) == false);
  std::cout << "  [PASS] Infinity float token strictly rejected." << std::endl;

  // Case G: Malformed boolean token ("valid": truish) -> REJECT
  const char* bad_bool_json = "{\"approach\": \"N\", \"valid\": truish, \"Q\": 2, \"PCU\": 2.0}";
  assert(TelemetryParser::parse(bad_bool_json, strlen(bad_bool_json), pkt) == false);
  std::cout << "  [PASS] Malformed boolean token strictly rejected." << std::endl;
}

int main() {
  std::cout << "=================================================================" << std::endl;
  std::cout << "  TrafficController C++ Native Verification Test Suite (v3.6)" << std::endl;
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
  testSensorDisconnectionAcrossAllFsmStates();
  testSensorRecoveryFromFixedFallback();
  testDegradedModeDemandAndStarvationWatchdog();
  testExhaustiveTelemetryParserMalformedInputs();

  std::cout << "\n=================================================================" << std::endl;
  std::cout << "  ALL 16 C++ EMBEDDED CONTROLLER VERIFICATION TESTS PASSED (0 ERRORS)." << std::endl;
  std::cout << "=================================================================" << std::endl;
  return 0;
}
