#pragma once

#include <cstddef>
#include <stdint.h>

#include "TelemetryReceiver.hpp"

// Lightweight microsecond timing statistics accumulator
struct StatAccumulator {
  int64_t min_val = INT64_MAX;
  int64_t max_val = 0;
  int64_t sum_val = 0;
  uint32_t count = 0;

  void record(int64_t val) {
    if (val < min_val) min_val = val;
    if (val > max_val) max_val = val;
    sum_val += val;
    count++;
  }

  void reset() {
    min_val = INT64_MAX;
    max_val = 0;
    sum_val = 0;
    count = 0;
  }

  double avg() const {
    return count > 0 ? static_cast<double>(sum_val) / count : 0.0;
  }
};

class RuntimeProfiler {
 public:
  // Fixed compile-time window size: exactly 500 iterations (~10s at 50Hz)
  static constexpr uint32_t kWindowSize = 500;

  RuntimeProfiler();

  // 1. Loop start: records interval (now - prev) and jitter (|interval - 20000|) using timestamp from main.cpp
  void recordLoopStart(int64_t loop_start_us);

  // 2. Explicitly resets the loop start reference after long operations (e.g. printReport)
  void resetLoopReference(int64_t timestamp_us);

  // 3. Subsystem duration recording (durations in microseconds, computed in main.cpp or TelemetryReceiver)
  void recordControllerTick(int64_t duration_us);
  void recordGpioUpdate(int64_t duration_us);
  void recordParserExec(int64_t duration_us);
  void recordDiagTx(int64_t duration_us);
  void recordLoopExec(int64_t duration_us);

  // 4. Increments iteration count and returns true when window completes (window_iterations >= 500)
  bool stepWindow();

  // 5. Emits the 8-part profile report and resets all 7 accumulators + window_iterations
  void printReport(const TelemetryCounters& rx_counters);

  // 6. Manual reset of all accumulators and iteration count
  void reset();

  // Read-only inspection accessors for native unit test verification
  uint32_t getWindowIterations() const { return window_iterations_; }
  int64_t getPrevLoopStartUs() const { return prev_loop_start_us_; }
  const StatAccumulator& getLoopExecStats() const { return loop_exec_us_; }
  const StatAccumulator& getLoopIntervalStats() const { return loop_interval_us_; }
  const StatAccumulator& getLoopJitterStats() const { return loop_jitter_us_; }
  const StatAccumulator& getControllerTickStats() const { return controller_tick_us_; }
  const StatAccumulator& getGpioUpdateStats() const { return gpio_update_us_; }
  const StatAccumulator& getParserExecStats() const { return parser_exec_us_; }
  const StatAccumulator& getDiagTxStats() const { return diag_tx_us_; }

 private:
  StatAccumulator loop_exec_us_;
  StatAccumulator loop_interval_us_;
  StatAccumulator loop_jitter_us_;
  StatAccumulator controller_tick_us_;
  StatAccumulator parser_exec_us_;
  StatAccumulator gpio_update_us_;
  StatAccumulator diag_tx_us_;

  uint32_t window_iterations_;
  int64_t prev_loop_start_us_;
};
