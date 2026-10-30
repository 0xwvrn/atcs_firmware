#pragma once

#ifdef ARDUINO
  #include <Arduino.h>
#else
  #include <stdint.h>
  #include <math.h>
  #include <algorithm>
  using std::min;
  using std::max;
  using std::isfinite;
#endif

// NEMA TS2 Traffic Signal Phase States
enum SignalState {
  STATE_ALL_RED_STARTUP,
  STATE_NS_GREEN,
  STATE_NS_YELLOW,
  STATE_ALL_RED_TO_EW,
  STATE_EW_GREEN,
  STATE_EW_YELLOW,
  STATE_ALL_RED_TO_NS,
  STATE_FAILSAFE_FLASH
};

// Approach observation fidelity
enum ObservationStatus {
  OBS_FRESH_MEASURED,
  OBS_TIMEOUT_FALLBACK
};

// Phase observation classification
enum PhaseObservationStatus {
  PHASE_FULLY_MEASURED,
  PHASE_PARTIALLY_MEASURED,
  PHASE_UNOBSERVED
};

// Overall controller operating mode
enum ControllerMode {
  MODE_ADAPTIVE_FULL,        // Both phases fully measured: dynamic green + gap-out
  MODE_DEGRADED_PARTIAL,     // Partially observed: conservative green, gap-out strictly disabled on unobserved
  MODE_FIXED_FALLBACK        // Complete sensor blackout: predefined fixed-time cycle, zero adaptive decisions
};

// Physical state variables for a single approach leg
struct ApproachState {
  int q_count;             // Count of stopped vehicles in queue
  float d_tail_m;          // Distance to queue tail in meters
  float p_pcu;             // Total Passenger Car Unit demand
  unsigned long last_seen; // Timestamp (ms) of last valid observation
  bool is_valid;           // True if sensor is reporting valid, calibrated data
};

// Verified Adaptive Traffic Control Engine
class TrafficController {
 public:
  // Timing parameters (in seconds)
  const float kStartupAllRedHold = 2.0f;     // Physical initial all-red clearance before any green
  const float kStartupLostTime = 3.5f;       // Saturation flow startup inertia in green allocation formula
  const float kMinGreen = 10.0f;             // Configured minimum vehicle green interval
  const float kMaxGreen = 45.0f;             // Maximum green phase ceiling
  const float kYellowTime = 3.0f;            // Yellow clearance buffer
  const float kAllRedTime = 1.5f;            // Intersection all-red clearance buffer
  const float kDischargeRatePerLane = 0.5f;   // Saturation flow per lane (0.5 PCU/sec ~= 1800 PCU/hr)
  const float kPhaseDischargeRate = 1.0f;     // Concurrent dual-approach phase rate (2 * 0.5 = 1.0 PCU/sec)
  const float kMaxWaitStarvation = 50.0f;    // Operational unserviced waiting ceiling before forced switch
  const float kFixedFallbackGreen = 25.0f;   // Predefined fixed-time allocation during complete sensor blackout
  const unsigned long kSensorTimeoutMs = 5000;// Sensor freshness timeout threshold
  const float kSingleLegFallbackPCU = 2.0f;  // Fallback demand per unmeasured approach

  TrafficController() {
    time_mock_ms_ = 0;
    use_mock_time_ = false;
    last_tick_time_ms_ = 0;

    state_ = STATE_ALL_RED_STARTUP;
    state_start_time_ms_ = getCurrentTimeMs();
    allocated_green_s_ = kMinGreen;

    ns_wait_time_s_ = 0.0f;
    ew_wait_time_s_ = 0.0f;

    initApproach(north_);
    initApproach(south_);
    initApproach(east_);
    initApproach(west_);
  }

  // Set explicit simulated clock for native unit test harnesses
  void setMockTimeMs(unsigned long mock_time_ms) {
    use_mock_time_ = true;
    time_mock_ms_ = mock_time_ms;
  }

  // Set explicit start time for testing specific clock regimes (e.g. rollover)
  void setSimulatedStateStartTimeMs(unsigned long start_time_ms) {
    state_start_time_ms_ = start_time_ms;
    last_tick_time_ms_ = start_time_ms;
  }

  unsigned long getCurrentTimeMs() const {
#ifdef ARDUINO
    if (use_mock_time_) return time_mock_ms_;
    return millis();
#else
    return time_mock_ms_;
#endif
  }

  // Update observation for a specific monitored approach with strict input validation
  bool updateApproach(char leg, int q, float d_tail, float pcu, bool valid = true) {
    ApproachState *app = getApproachPtr(leg);
    if (!app) return false;

    // Strict input validation: reject negative, non-finite (NaN/Inf), or malformed telemetry
    if (!valid || q < 0 || !isfinite(d_tail) || d_tail < 0.0f || !isfinite(pcu) || pcu < 0.0f) {
      app->is_valid = false;
      return false;
    }

    app->q_count = q;
    app->d_tail_m = d_tail;
    app->p_pcu = pcu; // Strictly preserve exact 0.0 PCU when valid (no false conversion to count)
    app->last_seen = getCurrentTimeMs();
    app->is_valid = true;
    return true;
  }

  // Checks whether an approach currently has an active, fresh measurement
  bool hasFreshObservation(char leg, unsigned long now) const {
    const ApproachState *app = getApproachPtrConst(leg);
    if (!app || !app->is_valid) return false;
    unsigned long age_ms = now - app->last_seen;
    return (age_ms <= kSensorTimeoutMs);
  }

  // Evaluates phase observation status and combined demand (N+S or E+W)
  float getPhaseDemand(char leg1, char leg2, unsigned long now, PhaseObservationStatus &status) const {
    bool leg1_fresh = hasFreshObservation(leg1, now);
    bool leg2_fresh = hasFreshObservation(leg2, now);

    const ApproachState *app1 = getApproachPtrConst(leg1);
    const ApproachState *app2 = getApproachPtrConst(leg2);

    float pcu1 = leg1_fresh ? app1->p_pcu : kSingleLegFallbackPCU;
    float pcu2 = leg2_fresh ? app2->p_pcu : kSingleLegFallbackPCU;

    if (leg1_fresh && leg2_fresh) {
      status = PHASE_FULLY_MEASURED;
    } else if (leg1_fresh || leg2_fresh) {
      status = PHASE_PARTIALLY_MEASURED;
    } else {
      status = PHASE_UNOBSERVED;
    }

    return pcu1 + pcu2;
  }

  // Evaluates current overall controller operating mode
  ControllerMode getOperatingMode() const {
    unsigned long now = getCurrentTimeMs();
    PhaseObservationStatus ns_status, ew_status;
    getPhaseDemand('N', 'S', now, ns_status);
    getPhaseDemand('E', 'W', now, ew_status);

    if (ns_status == PHASE_FULLY_MEASURED && ew_status == PHASE_FULLY_MEASURED) {
      return MODE_ADAPTIVE_FULL;
    } else if (ns_status == PHASE_UNOBSERVED && ew_status == PHASE_UNOBSERVED) {
      return MODE_FIXED_FALLBACK;
    } else {
      return MODE_DEGRADED_PARTIAL;
    }
  }

  // Calculate dynamic green time based on total concurrent phase demand (PCU)
  float calculateGreenTime(float phase_pcu) const {
    float green = kStartupLostTime + (phase_pcu / kPhaseDischargeRate);
    if (green < kMinGreen) green = kMinGreen;
    if (green > kMaxGreen) green = kMaxGreen;
    return green;
  }

  // Queue-Weighted Phase Score Heuristic (Max-Pressure Inspired for isolated intersections)
  float computeQueueWeightedDemand(char leg1, char leg2) const {
    unsigned long now = getCurrentTimeMs();
    PhaseObservationStatus status;
    float total_pcu = getPhaseDemand(leg1, leg2, now, status);
    return kPhaseDischargeRate * total_pcu;
  }

  // Enforces NEMA TS2 legal transition sequencing
  bool isValidTransition(SignalState from, SignalState to) const {
    if (to == STATE_FAILSAFE_FLASH) return true; // Emergency failsafe always permitted
    switch (from) {
      case STATE_ALL_RED_STARTUP: return (to == STATE_NS_GREEN);
      case STATE_NS_GREEN:        return (to == STATE_NS_YELLOW);
      case STATE_NS_YELLOW:       return (to == STATE_ALL_RED_TO_EW);
      case STATE_ALL_RED_TO_EW:   return (to == STATE_EW_GREEN);
      case STATE_EW_GREEN:        return (to == STATE_EW_YELLOW);
      case STATE_EW_YELLOW:       return (to == STATE_ALL_RED_TO_NS);
      case STATE_ALL_RED_TO_NS:   return (to == STATE_NS_GREEN);
      case STATE_FAILSAFE_FLASH:  return false; // Terminal requires hardware reset
      default:                    return false;
    }
  }

  // Execute State Machine cycle
  void tick() {
    unsigned long now = getCurrentTimeMs();
    unsigned long state_elapsed_ms = now - state_start_time_ms_;
    float elapsed_s = state_elapsed_ms / 1000.0f;

    // Safety Interlock Assertions
    if (!verifyInternalSafety()) {
      switchState(STATE_FAILSAFE_FLASH);
      return;
    }

    // Evaluate phase observation status and combined demand
    PhaseObservationStatus ns_status, ew_status;
    float q_ns = getPhaseDemand('N', 'S', now, ns_status);
    float q_ew = getPhaseDemand('E', 'W', now, ew_status);

    ControllerMode mode = MODE_DEGRADED_PARTIAL;
    if (ns_status == PHASE_FULLY_MEASURED && ew_status == PHASE_FULLY_MEASURED) {
      mode = MODE_ADAPTIVE_FULL;
    } else if (ns_status == PHASE_UNOBSERVED && ew_status == PHASE_UNOBSERVED) {
      mode = MODE_FIXED_FALLBACK;
    }

    // Track unserviced waiting times for anti-starvation protection across wrap-safe dt
    float dt_s = 0.0f;
    if (last_tick_time_ms_ > 0) {
      unsigned long dt_ms = now - last_tick_time_ms_;
      // Guard against anomalous gaps while remaining wrap-safe
      if (dt_ms <= 10000) {
        dt_s = dt_ms / 1000.0f;
      }
    }
    last_tick_time_ms_ = now;

    // Continuous unserviced wait accumulation across all conflicting states (green, yellow, and clearance)
    if (state_ != STATE_EW_GREEN) {
      if (q_ew >= 1.0f) {
        ew_wait_time_s_ += dt_s;
      } else {
        ew_wait_time_s_ = 0.0f;
      }
    } else {
      ew_wait_time_s_ = 0.0f; // Reset EW wait timer upon receiving green service
    }

    if (state_ != STATE_NS_GREEN) {
      if (q_ns >= 1.0f) {
        ns_wait_time_s_ += dt_s;
      } else {
        ns_wait_time_s_ = 0.0f;
      }
    } else {
      ns_wait_time_s_ = 0.0f; // Reset NS wait timer upon receiving green service
    }

    switch (state_) {
      case STATE_ALL_RED_STARTUP:
        if (elapsed_s >= kStartupAllRedHold) {
          allocated_green_s_ = (mode == MODE_FIXED_FALLBACK) ? kFixedFallbackGreen : calculateGreenTime(q_ns);
          ns_wait_time_s_ = 0.0f;
          switchState(STATE_NS_GREEN);
        }
        break;

      case STATE_NS_GREEN: {
        // Gap-Out Policy: Strictly requires MODE_ADAPTIVE_FULL (both corridors FULLY MEASURED)
        // Partially measured or fallback demand is strictly barred from truncating active green
        bool gap_out = (mode == MODE_ADAPTIVE_FULL) &&
                       (elapsed_s >= kMinGreen) &&
                       (q_ns < 0.5f) &&
                       (q_ew >= 1.0f);

        // Anti-Starvation Protection (EW continuous unserviced wait ceiling)
        bool starvation = (ew_wait_time_s_ >= kMaxWaitStarvation) && (q_ew >= 1.0f);
        bool max_green_expired = (elapsed_s >= allocated_green_s_);

        if (gap_out || starvation || max_green_expired) {
          switchState(STATE_NS_YELLOW);
        }
        break;
      }

      case STATE_NS_YELLOW:
        if (elapsed_s >= kYellowTime) {
          switchState(STATE_ALL_RED_TO_EW);
        }
        break;

      case STATE_ALL_RED_TO_EW:
        if (elapsed_s >= kAllRedTime) {
          allocated_green_s_ = (mode == MODE_FIXED_FALLBACK) ? kFixedFallbackGreen : calculateGreenTime(q_ew);
          ew_wait_time_s_ = 0.0f;
          switchState(STATE_EW_GREEN);
        }
        break;

      case STATE_EW_GREEN: {
        // Gap-Out Policy: Symmetrically requires MODE_ADAPTIVE_FULL
        bool gap_out = (mode == MODE_ADAPTIVE_FULL) &&
                       (elapsed_s >= kMinGreen) &&
                       (q_ew < 0.5f) &&
                       (q_ns >= 1.0f);

        bool starvation = (ns_wait_time_s_ >= kMaxWaitStarvation) && (q_ns >= 1.0f);
        bool max_green_expired = (elapsed_s >= allocated_green_s_);

        if (gap_out || starvation || max_green_expired) {
          switchState(STATE_EW_YELLOW);
        }
        break;
      }

      case STATE_EW_YELLOW:
        if (elapsed_s >= kYellowTime) {
          switchState(STATE_ALL_RED_TO_NS);
        }
        break;

      case STATE_ALL_RED_TO_NS:
        if (elapsed_s >= kAllRedTime) {
          allocated_green_s_ = (mode == MODE_FIXED_FALLBACK) ? kFixedFallbackGreen : calculateGreenTime(q_ns);
          ns_wait_time_s_ = 0.0f; // Reset NS wait timer upon granting green
          switchState(STATE_NS_GREEN);
        }
        break;

      case STATE_FAILSAFE_FLASH:
      default:
        // Continuous failsafe mode
        break;
    }
  }

  // Comprehensive Internal Safety & State-Transition Invariant Monitor
  bool verifyInternalSafety() const {
    // 1. Assert valid state enumeration
    if (state_ < STATE_ALL_RED_STARTUP || state_ > STATE_FAILSAFE_FLASH) {
      return false;
    }

    // 2. Invariant: Allocated green must be finite and within operational bounds
    if (!isfinite(allocated_green_s_) || allocated_green_s_ < kMinGreen ||
        allocated_green_s_ > max(kMaxGreen, kFixedFallbackGreen)) {
      return false;
    }

    // 3. Invariant: Wrap-safe clock difference and finite state elapsed duration
    // In unsigned 32-bit modulo arithmetic, now - state_start_time_ms_ yields true elapsed duration.
    // An interval > 0x7FFFFFFFUL (~24.8 days) represents an apparent backwards clock jump.
    unsigned long now = getCurrentTimeMs();
    unsigned long state_elapsed_ms = now - state_start_time_ms_;
    if (state_elapsed_ms > 0x7FFFFFFFUL) {
      return false; // Backwards clock jump / corrupted timestamp
    }
    float elapsed_s = state_elapsed_ms / 1000.0f;
    if (!isfinite(elapsed_s) || elapsed_s < 0.0f) {
      return false;
    }

    // 4. Runaway Clearance Guard: Clearance phases must terminate deterministically
    // If yellow exceeds kYellowTime + 1.0s or all-red exceeds kAllRedTime + 1.0s, FSM has stalled
    if (state_ == STATE_NS_YELLOW || state_ == STATE_EW_YELLOW) {
      if (elapsed_s > (kYellowTime + 1.0f)) return false;
    }
    if (state_ == STATE_ALL_RED_TO_EW || state_ == STATE_ALL_RED_TO_NS) {
      if (elapsed_s > (kAllRedTime + 1.0f)) return false;
    }
    if (state_ == STATE_ALL_RED_STARTUP) {
      if (elapsed_s > (kStartupAllRedHold + 1.0f)) return false;
    }

    // 5. Output Conflict Invariant: State must never imply concurrent conflicting active phases
    bool ns_active = (state_ == STATE_NS_GREEN || state_ == STATE_NS_YELLOW);
    bool ew_active = (state_ == STATE_EW_GREEN || state_ == STATE_EW_YELLOW);
    if (ns_active && ew_active) {
      return false; // Asserts physical non-conflict invariant on internal state
    }

    return true;
  }

  // Safety Invariant: Conflicting corridors can NEVER both be green/yellow
  bool verifyInterlock(bool ns_green, bool ns_yellow, bool ew_green, bool ew_yellow) const {
    bool ns_active = ns_green || ns_yellow;
    bool ew_active = ew_green || ew_yellow;
    return !(ns_active && ew_active);
  }

  SignalState getState() const { return state_; }
  float getAllocatedGreen() const { return allocated_green_s_; }
  float getNsWaitTime() const { return ns_wait_time_s_; }
  float getEwWaitTime() const { return ew_wait_time_s_; }

  float getRemainingTime() const {
    unsigned long elapsed_ms = getCurrentTimeMs() - state_start_time_ms_;
    float elapsed = elapsed_ms / 1000.0f;
    float total = 0.0f;
    switch (state_) {
      case STATE_ALL_RED_STARTUP: total = kStartupAllRedHold; break;
      case STATE_NS_GREEN:
      case STATE_EW_GREEN:        total = allocated_green_s_; break;
      case STATE_NS_YELLOW:
      case STATE_EW_YELLOW:       total = kYellowTime; break;
      case STATE_ALL_RED_TO_EW:
      case STATE_ALL_RED_TO_NS:   total = kAllRedTime; break;
      case STATE_FAILSAFE_FLASH:
      default:                    return 0.0f;
    }
    float rem = total - elapsed;
    return (rem > 0.0f) ? rem : 0.0f;
  }

  const char* getStateName() const {
    switch (state_) {
      case STATE_ALL_RED_STARTUP: return "STARTUP_ALL_RED";
      case STATE_NS_GREEN:        return "NS_GREEN";
      case STATE_NS_YELLOW:       return "NS_YELLOW";
      case STATE_ALL_RED_TO_EW:   return "ALL_RED_TO_EW";
      case STATE_EW_GREEN:        return "EW_GREEN";
      case STATE_EW_YELLOW:       return "EW_YELLOW";
      case STATE_ALL_RED_TO_NS:   return "ALL_RED_TO_NS";
      case STATE_FAILSAFE_FLASH:  return "FAILSAFE_FLASH";
      default:                    return "UNKNOWN";
    }
  }

  ApproachState getApproachState(char leg) const {
    const ApproachState *app = getApproachPtrConst(leg);
    return app ? *app : north_;
  }

  void switchState(SignalState new_state) {
    if (!isValidTransition(state_, new_state)) {
      state_ = STATE_FAILSAFE_FLASH;
      state_start_time_ms_ = getCurrentTimeMs();
      return;
    }
    state_ = new_state;
    state_start_time_ms_ = getCurrentTimeMs();
  }

 private:
  SignalState state_;
  unsigned long state_start_time_ms_;
  unsigned long last_tick_time_ms_;
  float allocated_green_s_;

  float ns_wait_time_s_;
  float ew_wait_time_s_;

  bool use_mock_time_;
  unsigned long time_mock_ms_;

  ApproachState north_;
  ApproachState south_;
  ApproachState east_;
  ApproachState west_;

  void initApproach(ApproachState &app) {
    app.q_count = 0;
    app.d_tail_m = 0.0f;
    app.p_pcu = 0.0f;
    app.last_seen = 0;
    app.is_valid = false;
  }

  ApproachState* getApproachPtr(char leg) {
    if (leg == 'N') return &north_;
    if (leg == 'S') return &south_;
    if (leg == 'E') return &east_;
    if (leg == 'W') return &west_;
    return nullptr;
  }

  const ApproachState* getApproachPtrConst(char leg) const {
    if (leg == 'N') return &north_;
    if (leg == 'S') return &south_;
    if (leg == 'E') return &east_;
    if (leg == 'W') return &west_;
    return nullptr;
  }
};
