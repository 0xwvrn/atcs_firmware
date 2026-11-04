#include <cassert>
#include <cmath>
#include <iostream>

#include "../include/RuntimeProfiler.hpp"
#include "../include/TelemetryReceiver.hpp"

// Native C++ Unit Test Suite for RuntimeProfiler

void testStatAccumulatorBehavior() {
  std::cout << "\n--- RuntimeProfiler Test 1: StatAccumulator Basics ---" << std::endl;
  StatAccumulator acc;

  // Initial state
  assert(acc.min_val == INT64_MAX);
  assert(acc.max_val == 0);
  assert(acc.sum_val == 0);
  assert(acc.count == 0);
  assert(acc.avg() == 0.0);

  // Single sample
  acc.record(150);
  assert(acc.min_val == 150);
  assert(acc.max_val == 150);
  assert(acc.sum_val == 150);
  assert(acc.count == 1);
  assert(acc.avg() == 150.0);

  // Additional samples
  acc.record(50);
  acc.record(400);
  assert(acc.min_val == 50);
  assert(acc.max_val == 400);
  assert(acc.sum_val == 600);
  assert(acc.count == 3);
  assert(std::fabs(acc.avg() - 200.0) < 1e-6);

  // Zero duration sample
  acc.record(0);
  assert(acc.min_val == 0);
  assert(acc.max_val == 400);
  assert(acc.sum_val == 600);
  assert(acc.count == 4);
  assert(std::fabs(acc.avg() - 150.0) < 1e-6);

  // Reset behavior
  acc.reset();
  assert(acc.min_val == INT64_MAX);
  assert(acc.max_val == 0);
  assert(acc.sum_val == 0);
  assert(acc.count == 0);
  assert(acc.avg() == 0.0);

  std::cout << "  [PASS] StatAccumulator min/max/sum/avg and reset verified." << std::endl;
}

void testLoopIntervalAndJitterCalculation() {
  std::cout << "\n--- RuntimeProfiler Test 2: Loop Interval and Jitter Calculation ---" << std::endl;
  RuntimeProfiler profiler;

  // Initial call sets previous timestamp without recording interval
  profiler.recordLoopStart(1000000);
  assert(profiler.getPrevLoopStartUs() == 1000000);
  assert(profiler.getLoopIntervalStats().count == 0);
  assert(profiler.getLoopJitterStats().count == 0);

  // Nominal 20ms (20,000 us) interval: jitter = 0
  profiler.recordLoopStart(1020000);
  assert(profiler.getLoopIntervalStats().count == 1);
  assert(profiler.getLoopIntervalStats().min_val == 20000);
  assert(profiler.getLoopIntervalStats().max_val == 20000);
  assert(profiler.getLoopJitterStats().count == 1);
  assert(profiler.getLoopJitterStats().min_val == 0);
  assert(profiler.getLoopJitterStats().max_val == 0);

  // Interval longer than nominal: 22,500 us (+2,500 us jitter)
  profiler.recordLoopStart(1042500);
  assert(profiler.getLoopIntervalStats().count == 2);
  assert(profiler.getLoopIntervalStats().max_val == 22500);
  assert(profiler.getLoopJitterStats().count == 2);
  assert(profiler.getLoopJitterStats().max_val == 2500);

  // Interval shorter than nominal: 18,000 us (+2,000 us jitter)
  profiler.recordLoopStart(1060500);
  assert(profiler.getLoopIntervalStats().count == 3);
  assert(profiler.getLoopIntervalStats().min_val == 18000);
  assert(profiler.getLoopJitterStats().count == 3);
  assert(profiler.getLoopJitterStats().min_val == 0);
  assert(profiler.getLoopJitterStats().max_val == 2500);

  std::cout << "  [PASS] Interval and jitter calculated accurately from supplied timestamps." << std::endl;
}

void testExplicitLoopReferenceReset() {
  std::cout << "\n--- RuntimeProfiler Test 3: Loop Reference Reset After Report ---" << std::endl;
  RuntimeProfiler profiler;

  profiler.recordLoopStart(1000000);
  profiler.recordLoopStart(1020000);
  assert(profiler.getLoopIntervalStats().count == 1);

  // Simulate long report print taking 50,000 us (50 ms)
  int64_t post_report_timestamp = 1070000;
  profiler.resetLoopReference(post_report_timestamp);
  assert(profiler.getPrevLoopStartUs() == post_report_timestamp);

  // Next normal loop tick 20,000 us after reference reset
  profiler.recordLoopStart(post_report_timestamp + 20000);
  assert(profiler.getLoopIntervalStats().count == 2);
  assert(profiler.getLoopIntervalStats().max_val == 20000); // Excluded the 50ms report delay!

  std::cout << "  [PASS] resetLoopReference() correctly isolates reporting delays." << std::endl;
}

void testSubsystemMetricRecording() {
  std::cout << "\n--- RuntimeProfiler Test 4: Subsystem Duration Recording ---" << std::endl;
  RuntimeProfiler profiler;

  profiler.recordControllerTick(75);
  profiler.recordGpioUpdate(15);
  profiler.recordParserExec(1200);
  profiler.recordDiagTx(340);
  profiler.recordLoopExec(2500);

  assert(profiler.getControllerTickStats().count == 1);
  assert(profiler.getControllerTickStats().min_val == 75);
  assert(profiler.getGpioUpdateStats().count == 1);
  assert(profiler.getGpioUpdateStats().min_val == 15);
  assert(profiler.getParserExecStats().count == 1);
  assert(profiler.getParserExecStats().min_val == 1200);
  assert(profiler.getDiagTxStats().count == 1);
  assert(profiler.getDiagTxStats().min_val == 340);
  assert(profiler.getLoopExecStats().count == 1);
  assert(profiler.getLoopExecStats().min_val == 2500);

  std::cout << "  [PASS] All discrete subsystem metrics recorded independently." << std::endl;
}

void testWindowSteppingAndReset() {
  std::cout << "\n--- RuntimeProfiler Test 5: Window Stepping and Reset Semantics ---" << std::endl;
  RuntimeProfiler profiler;

  assert(RuntimeProfiler::kWindowSize == 500);
  assert(profiler.getWindowIterations() == 0);

  // Step 499 times: must return false
  for (uint32_t i = 1; i < 500; ++i) {
    bool ready = profiler.stepWindow();
    assert(ready == false);
    assert(profiler.getWindowIterations() == i);
  }

  // 500th step: must return true
  bool ready = profiler.stepWindow();
  assert(ready == true);
  assert(profiler.getWindowIterations() == 500);

  // Record samples in accumulators
  profiler.recordControllerTick(80);
  profiler.recordLoopExec(2100);

  // Invoke printReport and verify reset
  TelemetryCounters mock_counters{1024, 25, 2, 0, 0};
  profiler.printReport(mock_counters);

  assert(profiler.getWindowIterations() == 0);
  assert(profiler.getControllerTickStats().count == 0);
  assert(profiler.getLoopExecStats().count == 0);

  std::cout << "  [PASS] Fixed 500-step window and report reset verified." << std::endl;
}

int main() {
  std::cout << "=================================================================" << std::endl;
  std::cout << "  RuntimeProfiler C++ Native Verification Test Suite" << std::endl;
  std::cout << "=================================================================" << std::endl;

  testStatAccumulatorBehavior();
  testLoopIntervalAndJitterCalculation();
  testExplicitLoopReferenceReset();
  testSubsystemMetricRecording();
  testWindowSteppingAndReset();

  std::cout << "\n=================================================================" << std::endl;
  std::cout << "  ALL RUNTIMEPROFILER VERIFICATION TESTS PASSED (0 ERRORS)." << std::endl;
  std::cout << "=================================================================" << std::endl;
  return 0;
}
