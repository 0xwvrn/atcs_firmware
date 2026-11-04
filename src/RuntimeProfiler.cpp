#include "RuntimeProfiler.hpp"

#ifdef ARDUINO
#include <Arduino.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#else
#include <cstdio>
#include <iostream>
#endif

RuntimeProfiler::RuntimeProfiler() {
  reset();
}

void RuntimeProfiler::reset() {
  loop_exec_us_.reset();
  loop_interval_us_.reset();
  loop_jitter_us_.reset();
  controller_tick_us_.reset();
  parser_exec_us_.reset();
  gpio_update_us_.reset();
  diag_tx_us_.reset();
  window_iterations_ = 0;
  prev_loop_start_us_ = 0;
}

void RuntimeProfiler::recordLoopStart(int64_t loop_start_us) {
  if (prev_loop_start_us_ > 0) {
    int64_t interval_us = loop_start_us - prev_loop_start_us_;
    loop_interval_us_.record(interval_us);
    int64_t jitter_us =
        interval_us > 20000 ? (interval_us - 20000) : (20000 - interval_us);
    loop_jitter_us_.record(jitter_us);
  }
  prev_loop_start_us_ = loop_start_us;
}

void RuntimeProfiler::resetLoopReference(int64_t timestamp_us) {
  prev_loop_start_us_ = timestamp_us;
}

void RuntimeProfiler::recordControllerTick(int64_t duration_us) {
  controller_tick_us_.record(duration_us);
}

void RuntimeProfiler::recordGpioUpdate(int64_t duration_us) {
  gpio_update_us_.record(duration_us);
}

void RuntimeProfiler::recordParserExec(int64_t duration_us) {
  parser_exec_us_.record(duration_us);
}

void RuntimeProfiler::recordDiagTx(int64_t duration_us) {
  diag_tx_us_.record(duration_us);
}

void RuntimeProfiler::recordLoopExec(int64_t duration_us) {
  loop_exec_us_.record(duration_us);
}

bool RuntimeProfiler::stepWindow() {
  window_iterations_++;
  return (window_iterations_ >= kWindowSize);
}

void RuntimeProfiler::printReport(const TelemetryCounters& rx_counters) {
#ifdef ARDUINO
  uint32_t internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  uint32_t internal_min = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
  uint32_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  UBaseType_t stack_high_water_bytes = uxTaskGetStackHighWaterMark(NULL);

  Serial.printf(
      "  ATCS RUNTIME PROFILE REPORT (Sample Window: %u iterations)\n",
      window_iterations_);
  Serial.printf("1. Loop Execution Time (CPU work):\n");
  Serial.printf("   - Min: %lld us | Avg: %.2f us | Max: %lld us\n",
                loop_exec_us_.min_val == INT64_MAX ? 0 : loop_exec_us_.min_val,
                loop_exec_us_.avg(), loop_exec_us_.max_val);

  Serial.printf("2. Loop Interval (Target: 20000 us / 50.0 Hz):\n");
  Serial.printf("   - Min: %lld us | Avg: %.2f us | Max: %lld us\n",
                loop_interval_us_.min_val == INT64_MAX ? 0 : loop_interval_us_.min_val,
                loop_interval_us_.avg(), loop_interval_us_.max_val);

  Serial.printf("3. Loop Period Jitter (|interval - 20000 us|):\n");
  Serial.printf("   - Min: %lld us | Avg: %.2f us | Max: %lld us\n",
                loop_jitter_us_.min_val == INT64_MAX ? 0 : loop_jitter_us_.min_val,
                loop_jitter_us_.avg(), loop_jitter_us_.max_val);

  Serial.printf("4. Controller Tick Execution Time (FSM):\n");
  Serial.printf("   - Min: %lld us | Avg: %.2f us | Max: %lld us\n",
                controller_tick_us_.min_val == INT64_MAX ? 0 : controller_tick_us_.min_val,
                controller_tick_us_.avg(), controller_tick_us_.max_val);

  Serial.printf("5. GPIO Update Execution Time (Actuation):\n");
  Serial.printf("   - Min: %lld us | Avg: %.2f us | Max: %lld us\n",
                gpio_update_us_.min_val == INT64_MAX ? 0 : gpio_update_us_.min_val,
                gpio_update_us_.avg(), gpio_update_us_.max_val);

  Serial.printf("6. Telemetry Ingestion (Total Bytes: %u | Valid: %u | "
                "Rejected: %u | Overflows: %u):\n",
                rx_counters.rx_bytes_total, rx_counters.valid_packets,
                rx_counters.rejected_packets, rx_counters.overflow_events);
  if (parser_exec_us_.count > 0) {
    Serial.printf("   - Parser Duration (Packets: %u): Min: %lld us | Avg: "
                  "%.2f us | Max: %lld us\n",
                  parser_exec_us_.count,
                  parser_exec_us_.min_val == INT64_MAX ? 0 : parser_exec_us_.min_val,
                  parser_exec_us_.avg(), parser_exec_us_.max_val);
  } else {
    Serial.println("   - Parser: (No packets received in window)");
  }

  if (diag_tx_us_.count > 0) {
    Serial.printf("7. Periodic Serial Diagnostics Time (Tx count: %u):\n",
                  diag_tx_us_.count);
    Serial.printf("   - Min: %lld us | Avg: %.2f us | Max: %lld us\n",
                  diag_tx_us_.min_val == INT64_MAX ? 0 : diag_tx_us_.min_val,
                  diag_tx_us_.avg(), diag_tx_us_.max_val);
  }

  Serial.printf("8. Memory Breakdown:\n");
  Serial.printf("   - Internal SRAM Free:   %u bytes (Watermark: %u bytes)\n",
                internal_free, internal_min);
  Serial.printf("   - External PSRAM Free:   %u bytes\n", psram_free);
  Serial.printf("   - Stack High-Water Mark: %u bytes remaining\n",
                (uint32_t)stack_high_water_bytes);
#else
  std::printf("  ATCS RUNTIME PROFILE REPORT (Sample Window: %u iterations)\n",
              window_iterations_);
  std::printf("1. Loop Execution Time (CPU work):\n");
  std::printf("   - Min: %lld us | Avg: %.2f us | Max: %lld us\n",
              loop_exec_us_.min_val == INT64_MAX ? 0 : loop_exec_us_.min_val,
              loop_exec_us_.avg(), loop_exec_us_.max_val);
  std::printf("2. Loop Interval (Target: 20000 us / 50.0 Hz):\n");
  std::printf("   - Min: %lld us | Avg: %.2f us | Max: %lld us\n",
              loop_interval_us_.min_val == INT64_MAX ? 0 : loop_interval_us_.min_val,
              loop_interval_us_.avg(), loop_interval_us_.max_val);
  std::printf("3. Loop Period Jitter (|interval - 20000 us|):\n");
  std::printf("   - Min: %lld us | Avg: %.2f us | Max: %lld us\n",
              loop_jitter_us_.min_val == INT64_MAX ? 0 : loop_jitter_us_.min_val,
              loop_jitter_us_.avg(), loop_jitter_us_.max_val);
  std::printf("4. Controller Tick Execution Time (FSM):\n");
  std::printf("   - Min: %lld us | Avg: %.2f us | Max: %lld us\n",
              controller_tick_us_.min_val == INT64_MAX ? 0 : controller_tick_us_.min_val,
              controller_tick_us_.avg(), controller_tick_us_.max_val);
  std::printf("5. GPIO Update Execution Time (Actuation):\n");
  std::printf("   - Min: %lld us | Avg: %.2f us | Max: %lld us\n",
              gpio_update_us_.min_val == INT64_MAX ? 0 : gpio_update_us_.min_val,
              gpio_update_us_.avg(), gpio_update_us_.max_val);
  std::printf("6. Telemetry Ingestion (Total Bytes: %u | Valid: %u | "
              "Rejected: %u | Overflows: %u):\n",
              rx_counters.rx_bytes_total, rx_counters.valid_packets,
              rx_counters.rejected_packets, rx_counters.overflow_events);
  if (parser_exec_us_.count > 0) {
    std::printf("   - Parser Duration (Packets: %u): Min: %lld us | Avg: "
                "%.2f us | Max: %lld us\n",
                parser_exec_us_.count,
                parser_exec_us_.min_val == INT64_MAX ? 0 : parser_exec_us_.min_val,
                parser_exec_us_.avg(), parser_exec_us_.max_val);
  } else {
    std::printf("   - Parser: (No packets received in window)\n");
  }
  if (diag_tx_us_.count > 0) {
    std::printf("7. Periodic Serial Diagnostics Time (Tx count: %u):\n",
                diag_tx_us_.count);
    std::printf("   - Min: %lld us | Avg: %.2f us | Max: %lld us\n",
                diag_tx_us_.min_val == INT64_MAX ? 0 : diag_tx_us_.min_val,
                diag_tx_us_.avg(), diag_tx_us_.max_val);
  }
  std::printf("8. Memory Breakdown (Native Host Simulation)\n");
#endif

  loop_exec_us_.reset();
  loop_interval_us_.reset();
  loop_jitter_us_.reset();
  controller_tick_us_.reset();
  parser_exec_us_.reset();
  gpio_update_us_.reset();
  diag_tx_us_.reset();
  window_iterations_ = 0;
}
