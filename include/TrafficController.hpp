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

// MODERATO congestion control regimes (Hayashi & Sugimoto, IEEE 1999)
enum ModeratoRegime {
  REGIME_DELAY_MINIMUM,      // Under-saturated (L_total < 0.50): queue-only short cycle
  REGIME_CONGESTION_CONTROL, // Near-saturated (0.50 <= L_total <= 1.00): queue + inflow + step-down cap
  REGIME_BALANCED_CONTROL    // Over-saturated (L_total > 1.00): balanced saturation & waiting time split
};

// IRC:93-1985 / Indo-HCM (2017) selectable timing profiles
enum IRC93TimingProfile {
  PROFILE_BASELINE_45S,      // 10-45s green, 3.0s yellow, 1.5s all-red, 50s starvation, 25s fallback
  PROFILE_URBAN_ARTERIAL,    // 10-60s green, 3.0s yellow, 1.5s all-red, 65s starvation, 30s fallback
  PROFILE_HEAVY_METRO        // 15-75s green, 4.0s yellow, 2.0s all-red, 82s starvation, 35s fallback
};

// 2-Phase & Multi-Stage intersection topologies without unsafe phase skipping
enum IntersectionTopology {
  TOPOLOGY_STANDARD_4WAY,          // Phase 0: (N, S), Phase 1: (E, W)
  TOPOLOGY_T_JUNCTION_3WAY_NS_E,   // Phase 0: (N, S), Phase 1: (E only)
  TOPOLOGY_T_JUNCTION_3WAY_EW_N,   // Phase 0: (N only), Phase 1: (E, W)
  TOPOLOGY_DOUBLE_CYCLE_NS_E_NS_W  // Double-Cycling Ring: Stage 0 (N,S) -> Stage 1 (E) -> Stage 2 (N,S) -> Stage 3 (W)
};

// Physical state variables for a single approach leg
struct ApproachState {
  int q_count;             // Count of stopped vehicles in queue
  float d_tail_m;          // Distance to queue tail in meters
  float p_pcu;             // Total Passenger Car Unit demand
  float inflow_pcu;        // Approaching upstream/moving inflow PCU
  unsigned long last_seen; // Timestamp (ms) of last valid observation
  bool is_valid;           // True if sensor is reporting valid, calibrated data
};

// Verified Adaptive Traffic Control Engine
class TrafficController {
 public:
  // Timing parameters (in seconds)
  static constexpr float kStartupAllRedHold = 2.0f;     // Physical initial all-red clearance before any green
  static constexpr float kStartupLostTime = 3.5f;       // Saturation flow startup inertia in green allocation formula
  static constexpr float kMinGreen = 10.0f;             // Configured minimum vehicle green interval
  static constexpr float kMaxGreen = 45.0f;             // Maximum green phase ceiling
  static constexpr float kYellowTime = 3.0f;            // Yellow clearance buffer
  static constexpr float kAllRedTime = 1.5f;            // Intersection all-red clearance buffer
  static constexpr float kDischargeRatePerLane = 0.5f;  // Saturation flow per lane (0.5 PCU/sec ~= 1800 PCU/hr)
  static constexpr float kPhaseDischargeRate = 1.0f;    // Concurrent dual-approach phase rate (2 * 0.5 = 1.0 PCU/sec)
  static constexpr float kMaxWaitStarvation = 50.0f;    // Operational unserviced waiting ceiling before forced switch
  static constexpr float kFixedFallbackGreen = 25.0f;   // Predefined fixed-time allocation during complete sensor blackout
  static constexpr unsigned long kSensorTimeoutMs = 5000; // Sensor freshness timeout threshold
  static constexpr float kSingleLegFallbackPCU = 2.0f;  // Fallback demand per unmeasured approach

  // Operational and safety threshold constants
  static constexpr unsigned long kMaxTickGapMs = 10000UL;          // Maximum tick interval before suppressing wait accumulation
  static constexpr float kMinOpposingDemandPCU = 1.0f;             // Opposing PCU threshold for wait accumulation, gap-out, and starvation
  static constexpr float kGapOutActiveThresholdPCU = 0.5f;         // Active-phase PCU dissipation threshold for early gap-out
  static constexpr float kClearanceRunawayMarginS = 1.0f;          // Runaway clearance tolerance margin in seconds
  static constexpr unsigned long kMaxValidElapsedMs = 0x7FFFFFFFUL;// Unsigned 32-bit half-range clock-jump boundary

  TrafficController() {
    time_mock_ms_ = 0;
    use_mock_time_ = false;
    last_tick_time_ms_ = 0;

    applyTimingProfile(PROFILE_BASELINE_45S);
    topology_ = TOPOLOGY_STANDARD_4WAY;
    stage_idx_ = 0;
    max_stage_skips_ = 1;
    ew_substage_skips_[0] = 0;
    ew_substage_skips_[1] = 0;
    enable_moderato_ = false;
    moderato_max_step_down_s_ = 10.0f;
    active_regime_ = REGIME_DELAY_MINIMUM;
    prev_phase_green_s_[0] = -1.0f;
    prev_phase_green_s_[1] = -1.0f;

    state_ = STATE_ALL_RED_STARTUP;
    state_start_time_ms_ = getCurrentTimeMs();
    allocated_green_s_ = min_green_s_;

    ns_wait_time_s_ = 0.0f;
    ew_wait_time_s_ = 0.0f;

    initApproach(north_);
    initApproach(south_);
    initApproach(east_);
    initApproach(west_);
  }

  void applyTimingProfile(IRC93TimingProfile profile) {
    active_profile_ = profile;
    if (profile == PROFILE_URBAN_ARTERIAL) {
      min_green_s_ = 10.0f;
      max_green_s_ = 60.0f;
      yellow_time_s_ = 3.0f;
      all_red_time_s_ = 1.5f;
      max_wait_starvation_s_ = 65.0f;
      fixed_fallback_green_s_ = 30.0f;
    } else if (profile == PROFILE_HEAVY_METRO) {
      min_green_s_ = 15.0f;
      max_green_s_ = 75.0f;
      yellow_time_s_ = 4.0f;
      all_red_time_s_ = 2.0f;
      max_wait_starvation_s_ = 82.0f;
      fixed_fallback_green_s_ = 35.0f;
    } else {
      min_green_s_ = kMinGreen;
      max_green_s_ = kMaxGreen;
      yellow_time_s_ = kYellowTime;
      all_red_time_s_ = kAllRedTime;
      max_wait_starvation_s_ = kMaxWaitStarvation;
      fixed_fallback_green_s_ = kFixedFallbackGreen;
    }
  }

  void setTopology(IntersectionTopology topology, int max_stage_skips = 1) {
    topology_ = topology;
    max_stage_skips_ = (max_stage_skips >= 0) ? max_stage_skips : 0;
    stage_idx_ = 0;
    ew_substage_skips_[0] = 0;
    ew_substage_skips_[1] = 0;
  }
  IntersectionTopology getTopology() const { return topology_; }
  int getActiveStageIndex() const { return stage_idx_; }
  IRC93TimingProfile getTimingProfile() const { return active_profile_; }

  void setModeratoEnabled(bool enabled, float max_step_down_s = 10.0f) {
    enable_moderato_ = enabled;
    moderato_max_step_down_s_ = max_step_down_s;
  }
  ModeratoRegime getActiveRegime() const { return active_regime_; }

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
  bool updateApproach(char leg, int q, float d_tail, float pcu, bool valid = true, float inflow_pcu = 0.0f) {
    ApproachState *app = getApproachPtr(leg);
    if (!app) return false;

    // Strict input validation: reject negative, non-finite (NaN/Inf), or malformed telemetry
    if (!valid || q < 0 || !isfinite(d_tail) || d_tail < 0.0f ||
        !isfinite(pcu) || pcu < 0.0f || !isfinite(inflow_pcu) || inflow_pcu < 0.0f) {
      app->is_valid = false;
      return false;
    }

    app->q_count = q;
    app->d_tail_m = d_tail;
    app->p_pcu = pcu; // Strictly preserve exact 0.0 PCU when valid (no false conversion to count)
    app->inflow_pcu = inflow_pcu;
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

  float getApproachInflow(char leg, unsigned long now) const {
    if (!hasFreshObservation(leg, now)) return 0.0f;
    const ApproachState *app = getApproachPtrConst(leg);
    return app ? app->inflow_pcu : 0.0f;
  }

  // Evaluates phase observation status and combined demand (N+S or E+W, respecting 3-way topology)
  float getPhaseDemand(char leg1, char leg2, unsigned long now, PhaseObservationStatus &status) const {
    bool is_ns = (leg1 == 'N' || leg2 == 'N');
    if (is_ns && topology_ == TOPOLOGY_T_JUNCTION_3WAY_EW_N) {
      bool fresh = hasFreshObservation('N', now);
      status = fresh ? PHASE_FULLY_MEASURED : PHASE_UNOBSERVED;
      return fresh ? north_.p_pcu : kSingleLegFallbackPCU;
    }
    if (!is_ns && topology_ == TOPOLOGY_T_JUNCTION_3WAY_NS_E) {
      bool fresh = hasFreshObservation('E', now);
      status = fresh ? PHASE_FULLY_MEASURED : PHASE_UNOBSERVED;
      return fresh ? east_.p_pcu : kSingleLegFallbackPCU;
    }
    if (!is_ns && topology_ == TOPOLOGY_DOUBLE_CYCLE_NS_E_NS_W) {
      char active_ew_leg = (stage_idx_ == 3) ? 'W' : 'E';
      const ApproachState &app = (active_ew_leg == 'W') ? west_ : east_;
      bool fresh = hasFreshObservation(active_ew_leg, now);
      status = fresh ? PHASE_FULLY_MEASURED : PHASE_UNOBSERVED;
      return fresh ? app.p_pcu : kSingleLegFallbackPCU;
    }

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

  float getPhaseInflow(int phase_idx, unsigned long now) const {
    if (phase_idx == 0) {
      if (topology_ == TOPOLOGY_T_JUNCTION_3WAY_EW_N) return getApproachInflow('N', now);
      return getApproachInflow('N', now) + getApproachInflow('S', now);
    } else {
      if (topology_ == TOPOLOGY_T_JUNCTION_3WAY_NS_E) return getApproachInflow('E', now);
      return getApproachInflow('E', now) + getApproachInflow('W', now);
    }
  }

  // Evaluates current overall controller operating mode
  ControllerMode getOperatingMode() const {
    unsigned long now = getCurrentTimeMs();
    PhaseObservationStatus ns_status, ew_status;
    getPhaseDemand('N', 'S', now, ns_status);
    getPhaseDemand('E', 'W', now, ew_status);
    return classifyMode(ns_status, ew_status);
  }

  // Calculate dynamic green time based on total concurrent phase demand (PCU)
  float calculateGreenTime(float phase_pcu) const {
    float green = kStartupLostTime + (phase_pcu / kPhaseDischargeRate);
    if (green < min_green_s_) green = min_green_s_;
    if (green > max_green_s_) green = max_green_s_;
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
    if (!verifyInternalSafety(state_elapsed_ms, elapsed_s)) {
      switchState(STATE_FAILSAFE_FLASH);
      return;
    }

    // Evaluate phase observation status and combined demand
    PhaseObservationStatus ns_status, ew_status;
    float q_ns = getPhaseDemand('N', 'S', now, ns_status);
    float q_ew = getPhaseDemand('E', 'W', now, ew_status);

    ControllerMode mode = classifyMode(ns_status, ew_status);

    // Track unserviced waiting times for anti-starvation protection across wrap-safe dt
    float dt_s = 0.0f;
    if (last_tick_time_ms_ > 0) {
      unsigned long dt_ms = now - last_tick_time_ms_;
      // Guard against anomalous gaps while remaining wrap-safe
      if (dt_ms <= kMaxTickGapMs) {
        dt_s = dt_ms / 1000.0f;
      }
    }
    last_tick_time_ms_ = now;

    // Continuous unserviced wait accumulation across all conflicting states (green, yellow, and clearance)
    updateCorridorWaitTime(state_ == STATE_EW_GREEN, q_ew, dt_s, ew_wait_time_s_);
    updateCorridorWaitTime(state_ == STATE_NS_GREEN, q_ns, dt_s, ns_wait_time_s_);

    switch (state_) {
      case STATE_ALL_RED_STARTUP:
        if (elapsed_s >= kStartupAllRedHold) {
          enterGreenPhase(STATE_NS_GREEN, mode, q_ns, q_ew, 0, now, ns_wait_time_s_);
        }
        break;

      case STATE_NS_GREEN:
        if (shouldTerminateGreen(mode, elapsed_s, q_ns, q_ew, ew_wait_time_s_)) {
          switchState(STATE_NS_YELLOW);
        }
        break;

      case STATE_NS_YELLOW:
        if (elapsed_s >= yellow_time_s_) {
          switchState(STATE_ALL_RED_TO_EW);
        }
        break;

      case STATE_ALL_RED_TO_EW:
        if (elapsed_s >= all_red_time_s_) {
          enterGreenPhase(STATE_EW_GREEN, mode, q_ns, q_ew, 1, now, ew_wait_time_s_);
        }
        break;

      case STATE_EW_GREEN:
        if (shouldTerminateGreen(mode, elapsed_s, q_ew, q_ns, ns_wait_time_s_)) {
          switchState(STATE_EW_YELLOW);
        }
        break;

      case STATE_EW_YELLOW:
        if (elapsed_s >= yellow_time_s_) {
          switchState(STATE_ALL_RED_TO_NS);
        }
        break;

      case STATE_ALL_RED_TO_NS:
        if (elapsed_s >= all_red_time_s_) {
          enterGreenPhase(STATE_NS_GREEN, mode, q_ns, q_ew, 0, now, ns_wait_time_s_);
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
    unsigned long now = getCurrentTimeMs();
    unsigned long state_elapsed_ms = now - state_start_time_ms_;
    float elapsed_s = state_elapsed_ms / 1000.0f;
    return verifyInternalSafety(state_elapsed_ms, elapsed_s);
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
      case STATE_EW_YELLOW:       total = yellow_time_s_; break;
      case STATE_ALL_RED_TO_EW:
      case STATE_ALL_RED_TO_NS:   total = all_red_time_s_; break;
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

  IRC93TimingProfile active_profile_;
  IntersectionTopology topology_;
  int stage_idx_;
  int max_stage_skips_;
  int ew_substage_skips_[2];
  float min_green_s_;
  float max_green_s_;
  float yellow_time_s_;
  float all_red_time_s_;
  float max_wait_starvation_s_;
  float fixed_fallback_green_s_;

  bool enable_moderato_;
  float moderato_max_step_down_s_;
  ModeratoRegime active_regime_;
  float prev_phase_green_s_[2];

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
    app.inflow_pcu = 0.0f;
    app.last_seen = 0;
    app.is_valid = false;
  }

  static ControllerMode classifyMode(PhaseObservationStatus ns_status,
                                     PhaseObservationStatus ew_status) {
    if (ns_status == PHASE_FULLY_MEASURED && ew_status == PHASE_FULLY_MEASURED) {
      return MODE_ADAPTIVE_FULL;
    } else if (ns_status == PHASE_UNOBSERVED && ew_status == PHASE_UNOBSERVED) {
      return MODE_FIXED_FALLBACK;
    } else {
      return MODE_DEGRADED_PARTIAL;
    }
  }

  static void updateCorridorWaitTime(bool is_serviced_green, float corridor_pcu,
                                     float dt_s, float &wait_time_s) {
    if (!is_serviced_green) {
      if (corridor_pcu >= kMinOpposingDemandPCU) {
        wait_time_s += dt_s;
      } else {
        wait_time_s = 0.0f;
      }
    } else {
      wait_time_s = 0.0f;
    }
  }

  bool shouldTerminateGreen(ControllerMode mode, float elapsed_s, float active_pcu,
                            float opposing_pcu, float opposing_wait_s) const {
    // Gap-Out Policy: Strictly requires MODE_ADAPTIVE_FULL (both corridors FULLY MEASURED)
    // Partially measured or fallback demand is strictly barred from truncating active green
    bool gap_out = (mode == MODE_ADAPTIVE_FULL) &&
                   (elapsed_s >= min_green_s_) &&
                   (active_pcu < kGapOutActiveThresholdPCU) &&
                   (opposing_pcu >= kMinOpposingDemandPCU);

    // Anti-Starvation Protection (continuous unserviced wait ceiling)
    bool starvation = (opposing_wait_s >= max_wait_starvation_s_) && (opposing_pcu >= kMinOpposingDemandPCU);
    bool max_green_expired = (elapsed_s >= allocated_green_s_);

    return gap_out || starvation || max_green_expired;
  }

  float allocatePhaseGreen(int phase_idx, float q_ns, float q_ew, unsigned long now) {
    float phase_pcu = (phase_idx == 0) ? q_ns : q_ew;
    if (!enable_moderato_) {
      float alloc = calculateGreenTime(phase_pcu);
      prev_phase_green_s_[phase_idx] = alloc;
      return alloc;
    }

    float in0 = getPhaseInflow(0, now);
    float in1 = getPhaseInflow(1, now);
    float cap = max(1.0f, (max_green_s_ - kStartupLostTime) * kPhaseDischargeRate);
    float l0 = (q_ns + in0) / cap;
    float l1 = (q_ew + in1) / cap;
    float l_total = l0 + l1;

    if (l_total < 0.50f) {
      active_regime_ = REGIME_DELAY_MINIMUM;
    } else if (l_total <= 1.00f) {
      active_regime_ = REGIME_CONGESTION_CONTROL;
    } else {
      active_regime_ = REGIME_BALANCED_CONTROL;
    }

    float alloc = min_green_s_;
    float phase_inflow = (phase_idx == 0) ? in0 : in1;

    if (active_regime_ == REGIME_DELAY_MINIMUM) {
      alloc = calculateGreenTime(phase_pcu);
    } else if (active_regime_ == REGIME_CONGESTION_CONTROL) {
      float raw_alloc = calculateGreenTime(phase_pcu + phase_inflow);
      float prev = prev_phase_green_s_[phase_idx];
      if (prev > 0.0f && raw_alloc < prev - moderato_max_step_down_s_) {
        alloc = max(min_green_s_, prev - moderato_max_step_down_s_);
      } else {
        alloc = raw_alloc;
      }
    } else {
      float score_0 = max(0.05f, l0 + (ns_wait_time_s_ / max(1.0f, max_wait_starvation_s_)));
      float score_1 = max(0.05f, l1 + (ew_wait_time_s_ / max(1.0f, max_wait_starvation_s_)));
      float raw_g0 = kStartupLostTime + (q_ns + in0) / kPhaseDischargeRate;
      float raw_g1 = kStartupLostTime + (q_ew + in1) / kPhaseDischargeRate;
      float budget = min(2.0f * max_green_s_, max(2.0f * min_green_s_, raw_g0 + raw_g1));
      float share = ((phase_idx == 0) ? score_0 : score_1) / (score_0 + score_1);
      alloc = min(max(budget * share, min_green_s_), max_green_s_);
    }

    prev_phase_green_s_[phase_idx] = alloc;
    return alloc;
  }

  void enterGreenPhase(SignalState green_state, ControllerMode mode,
                       float q_ns, float q_ew, int phase_idx, unsigned long now, float &wait_time_s) {
    if (topology_ == TOPOLOGY_DOUBLE_CYCLE_NS_E_NS_W) {
      if (phase_idx == 0) {
        stage_idx_ = (stage_idx_ == 1) ? 2 : 0;
      } else {
        int cand_stage = (stage_idx_ == 0) ? 1 : 3;
        char cand_leg = (cand_stage == 1) ? 'E' : 'W';
        int skip_slot = (cand_stage == 1) ? 0 : 1;
        const ApproachState &cand_app = (cand_leg == 'E') ? east_ : west_;
        if (max_stage_skips_ > 0 && hasFreshObservation(cand_leg, now) &&
            cand_app.p_pcu < kGapOutActiveThresholdPCU && ew_substage_skips_[skip_slot] < max_stage_skips_) {
          ew_substage_skips_[skip_slot]++;
          cand_stage = (cand_stage == 1) ? 3 : 1;
          skip_slot = (cand_stage == 1) ? 0 : 1;
        }
        ew_substage_skips_[skip_slot] = 0;
        stage_idx_ = cand_stage;
        PhaseObservationStatus ew_st;
        q_ew = getPhaseDemand('E', 'W', now, ew_st);
      }
    } else {
      stage_idx_ = phase_idx;
    }

    allocated_green_s_ = (mode == MODE_FIXED_FALLBACK)
                             ? fixed_fallback_green_s_
                             : allocatePhaseGreen(phase_idx, q_ns, q_ew, now);
    wait_time_s = 0.0f;
    switchState(green_state);
  }

  bool verifyInternalSafety(unsigned long state_elapsed_ms, float elapsed_s) const {
    // 1. Assert valid state enumeration
    if (state_ < STATE_ALL_RED_STARTUP || state_ > STATE_FAILSAFE_FLASH) {
      return false;
    }

    // 2. Invariant: Allocated green must be finite and within operational bounds
    if (!isfinite(allocated_green_s_) || allocated_green_s_ < min_green_s_ ||
        allocated_green_s_ > max(max_green_s_, fixed_fallback_green_s_)) {
      return false;
    }

    // 3. Invariant: Wrap-safe clock difference and finite state elapsed duration
    // In unsigned 32-bit modulo arithmetic, now - state_start_time_ms_ yields true elapsed duration.
    // An interval > 0x7FFFFFFFUL (~24.8 days) represents an apparent backwards clock jump.
    if (state_elapsed_ms > kMaxValidElapsedMs) {
      return false; // Backwards clock jump / corrupted timestamp
    }
    if (!isfinite(elapsed_s) || elapsed_s < 0.0f) {
      return false;
    }

    // 4. Runaway Clearance Guard: Clearance phases must terminate deterministically
    // If yellow exceeds yellow_time_s_ + 1.0s or all-red exceeds all_red_time_s_ + 1.0s, FSM has stalled
    if (state_ == STATE_NS_YELLOW || state_ == STATE_EW_YELLOW) {
      if (elapsed_s > (yellow_time_s_ + kClearanceRunawayMarginS)) return false;
    }
    if (state_ == STATE_ALL_RED_TO_EW || state_ == STATE_ALL_RED_TO_NS) {
      if (elapsed_s > (all_red_time_s_ + kClearanceRunawayMarginS)) return false;
    }
    if (state_ == STATE_ALL_RED_STARTUP) {
      if (elapsed_s > (kStartupAllRedHold + kClearanceRunawayMarginS)) return false;
    }

    // 5. Output Conflict Invariant: State must never imply concurrent conflicting active phases
    bool ns_active = (state_ == STATE_NS_GREEN || state_ == STATE_NS_YELLOW);
    bool ew_active = (state_ == STATE_EW_GREEN || state_ == STATE_EW_YELLOW);
    if (ns_active && ew_active) {
      return false; // Asserts physical non-conflict invariant on internal state
    }

    return true;
  }

  template <typename Self>
  static auto resolveApproachPtr(Self &self, char leg) -> decltype(&self.north_) {
    if (leg == 'N') return &self.north_;
    if (leg == 'S') return &self.south_;
    if (leg == 'E') return &self.east_;
    if (leg == 'W') return &self.west_;
    return nullptr;
  }

  ApproachState* getApproachPtr(char leg) {
    return resolveApproachPtr(*this, leg);
  }

  const ApproachState* getApproachPtrConst(char leg) const {
    return resolveApproachPtr(*this, leg);
  }
};
