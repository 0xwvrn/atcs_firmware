#include <Arduino.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "TelemetryParser.hpp"
#include "TrafficController.hpp"

// Physical GPIO Pin Definitions for Traffic Signal LEDs
// North-South Axis
const int PIN_NS_RED = 4;
const int PIN_NS_YELLOW = 5;
const int PIN_NS_GREEN = 6;

// East-West Axis
const int PIN_EW_RED = 7;
const int PIN_EW_YELLOW = 8;
const int PIN_EW_GREEN = 9;

// Global Controller Instance
TrafficController controller;

// Fixed-size, zero-allocation serial ingestion buffer with overrun protection
const size_t SERIAL_BUF_SIZE = 512;
char serial_buffer[SERIAL_BUF_SIZE];
size_t serial_buf_idx = 0;
bool discard_until_newline = false;
unsigned long discarded_packet_count = 0;

// Non-blocking telemetry ingestion and error counters (Phase 1 & 2)
uint32_t telemetry_rx_bytes_total = 0;
uint32_t telemetry_valid_packets = 0;
uint32_t telemetry_rejected_packets = 0;
uint32_t telemetry_overflow_events = 0;

unsigned long last_telemetry_tx_ms = 0;
unsigned long last_queue_rx_ms = 0;

struct StatAccumulator {
  int64_t min_val = INT64_MAX;
  int64_t max_val = 0;
  int64_t sum_val = 0;
  uint32_t count = 0;

  void record(int64_t val) {
    if (val < min_val)
      min_val = val;
    if (val > max_val)
      max_val = val;
    sum_val += val;
    count++;
  }

  void reset() {
    min_val = INT64_MAX;
    max_val = 0;
    sum_val = 0;
    count = 0;
  }

  double avg() const { return count > 0 ? (double)sum_val / count : 0.0; }
};

struct RuntimeProfileWindow {
  StatAccumulator loop_exec_us; // CPU time per loop iteration before delay()
  StatAccumulator loop_interval_us; // Wall-clock interval between loop() starts
  StatAccumulator loop_jitter_us;   // Deviation from nominal 20,000 us (20ms)
  StatAccumulator controller_tick_us; // Duration of controller.tick()
  StatAccumulator parser_exec_us;     // Duration of TelemetryParser::parse()
  StatAccumulator gpio_update_us;     // Duration of updatePhysicalSignals()
  StatAccumulator
      diag_tx_us; // Duration of periodic Serial.printf() diagnostics

  uint32_t window_iterations = 0;
  const uint32_t kWindowSize = 500; // 500 iterations (~10s at 50Hz)
} profiler;

int64_t prev_loop_start_us = 0;

void updatePhysicalSignals(SignalState state) {
  bool ns_r = false, ns_y = false, ns_g = false;
  bool ew_r = false, ew_y = false, ew_g = false;

  switch (state) {
  case STATE_NS_GREEN:
    ns_g = true;
    ew_r = true;
    break;

  case STATE_NS_YELLOW:
    ns_y = true;
    ew_r = true;
    break;

  case STATE_ALL_RED_STARTUP:
  case STATE_ALL_RED_TO_EW:
  case STATE_ALL_RED_TO_NS:
    ns_r = true;
    ew_r = true;
    break;

  case STATE_EW_GREEN:
    ew_g = true;
    ns_r = true;
    break;

  case STATE_EW_YELLOW:
    ew_y = true;
    ns_r = true;
    break;

  case STATE_FAILSAFE_FLASH:
  default: {
    bool flash_on = ((millis() / 500) % 2 == 0);
    ns_r = flash_on;
    ew_r = flash_on;
    ns_y = false;
    ew_y = false;
    ns_g = false;
    ew_g = false;
    break;
  }
  }

  if (!controller.verifyInterlock(ns_g, ns_y, ew_g, ew_y)) {
    digitalWrite(PIN_NS_RED, HIGH);
    digitalWrite(PIN_EW_RED, HIGH);
    digitalWrite(PIN_NS_YELLOW, LOW);
    digitalWrite(PIN_EW_YELLOW, LOW);
    digitalWrite(PIN_NS_GREEN, LOW);
    digitalWrite(PIN_EW_GREEN, LOW);
    Serial.println("[CRITICAL] Hardware Interlock Tripped! Conflicting active "
                   "signals prohibited.");
    return;
  }

  digitalWrite(PIN_NS_RED, ns_r ? HIGH : LOW);
  digitalWrite(PIN_NS_YELLOW, ns_y ? HIGH : LOW);
  digitalWrite(PIN_NS_GREEN, ns_g ? HIGH : LOW);

  digitalWrite(PIN_EW_RED, ew_r ? HIGH : LOW);
  digitalWrite(PIN_EW_YELLOW, ew_y ? HIGH : LOW);
  digitalWrite(PIN_EW_GREEN, ew_g ? HIGH : LOW);
}

// Parses and strictly validates telemetry packets (Non-blocking, zero
// per-packet synchronous UART prints)
void parseTelemetryPacket(const char *json, size_t len) {
  int64_t t_parse_start = esp_timer_get_time();
  ParsedTelemetry pkt;
  bool parse_success = TelemetryParser::parse(json, len, pkt);
  int64_t t_parse_dur = esp_timer_get_time() - t_parse_start;
  profiler.parser_exec_us.record(t_parse_dur);

  if (!parse_success) {
    // Non-blocking: increment counter without blocking UART TX FIFO
    telemetry_rejected_packets++;
    return;
  }

  telemetry_valid_packets++;
  controller.updateApproach(pkt.approach, pkt.q_count, pkt.d_tail_m, pkt.p_pcu,
                            pkt.is_valid);
  last_queue_rx_ms = millis();
}

void printProfileReport() {
  uint32_t internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
  uint32_t internal_min = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
  uint32_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  UBaseType_t stack_high_water_bytes = uxTaskGetStackHighWaterMark(
      NULL); // In ESP-IDF FreeRTOS, watermark is returned in bytes

  Serial.printf(
      "  ATCS RUNTIME PROFILE REPORT (Sample Window: %u iterations)\n",
      profiler.window_iterations);
  Serial.printf("1. Loop Execution Time (CPU work):\n");
  Serial.printf("   - Min: %lld us | Avg: %.2f us | Max: %lld us\n",
                profiler.loop_exec_us.min_val == INT64_MAX
                    ? 0
                    : profiler.loop_exec_us.min_val,
                profiler.loop_exec_us.avg(), profiler.loop_exec_us.max_val);

  Serial.printf("2. Loop Interval (Target: 20000 us / 50.0 Hz):\n");
  Serial.printf("   - Min: %lld us | Avg: %.2f us | Max: %lld us\n",
                profiler.loop_interval_us.min_val == INT64_MAX
                    ? 0
                    : profiler.loop_interval_us.min_val,
                profiler.loop_interval_us.avg(),
                profiler.loop_interval_us.max_val);

  Serial.printf("3. Loop Period Jitter (|interval - 20000 us|):\n");
  Serial.printf("   - Min: %lld us | Avg: %.2f us | Max: %lld us\n",
                profiler.loop_jitter_us.min_val == INT64_MAX
                    ? 0
                    : profiler.loop_jitter_us.min_val,
                profiler.loop_jitter_us.avg(), profiler.loop_jitter_us.max_val);

  Serial.printf("4. Controller Tick Execution Time (FSM):\n");
  Serial.printf("   - Min: %lld us | Avg: %.2f us | Max: %lld us\n",
                profiler.controller_tick_us.min_val == INT64_MAX
                    ? 0
                    : profiler.controller_tick_us.min_val,
                profiler.controller_tick_us.avg(),
                profiler.controller_tick_us.max_val);

  Serial.printf("5. GPIO Update Execution Time (Actuation):\n");
  Serial.printf("   - Min: %lld us | Avg: %.2f us | Max: %lld us\n",
                profiler.gpio_update_us.min_val == INT64_MAX
                    ? 0
                    : profiler.gpio_update_us.min_val,
                profiler.gpio_update_us.avg(), profiler.gpio_update_us.max_val);

  Serial.printf("6. Telemetry Ingestion (Total Bytes: %u | Valid: %u | "
                "Rejected: %u | Overflows: %u):\n",
                telemetry_rx_bytes_total, telemetry_valid_packets,
                telemetry_rejected_packets, telemetry_overflow_events);
  if (profiler.parser_exec_us.count > 0) {
    Serial.printf("   - Parser Duration (Packets: %u): Min: %lld us | Avg: "
                  "%.2f us | Max: %lld us\n",
                  profiler.parser_exec_us.count,
                  profiler.parser_exec_us.min_val == INT64_MAX
                      ? 0
                      : profiler.parser_exec_us.min_val,
                  profiler.parser_exec_us.avg(),
                  profiler.parser_exec_us.max_val);
  } else {
    Serial.println("   - Parser: (No packets received in window)");
  }

  if (profiler.diag_tx_us.count > 0) {
    Serial.printf("7. Periodic Serial Diagnostics Time (Tx count: %u):\n",
                  profiler.diag_tx_us.count);
    Serial.printf("   - Min: %lld us | Avg: %.2f us | Max: %lld us\n",
                  profiler.diag_tx_us.min_val == INT64_MAX
                      ? 0
                      : profiler.diag_tx_us.min_val,
                  profiler.diag_tx_us.avg(), profiler.diag_tx_us.max_val);
  }

  Serial.printf("8. Memory Breakdown:\n");
  Serial.printf("   - Internal SRAM Free:   %u bytes (Watermark: %u bytes)\n",
                internal_free, internal_min);
  Serial.printf("   - External PSRAM Free:   %u bytes\n", psram_free);
  Serial.printf("   - Stack High-Water Mark: %u bytes remaining\n",
                (uint32_t)stack_high_water_bytes);

  profiler.loop_exec_us.reset();
  profiler.loop_interval_us.reset();
  profiler.loop_jitter_us.reset();
  profiler.controller_tick_us.reset();
  profiler.parser_exec_us.reset();
  profiler.gpio_update_us.reset();
  profiler.diag_tx_us.reset();
  profiler.window_iterations = 0;
}

void setup() {
  Serial.begin(115200);
  delay(1500);

  pinMode(PIN_NS_RED, OUTPUT);
  pinMode(PIN_NS_YELLOW, OUTPUT);
  pinMode(PIN_NS_GREEN, OUTPUT);
  pinMode(PIN_EW_RED, OUTPUT);
  pinMode(PIN_EW_YELLOW, OUTPUT);
  pinMode(PIN_EW_GREEN, OUTPUT);

  digitalWrite(PIN_NS_RED, HIGH);
  digitalWrite(PIN_EW_RED, HIGH);
}

void loop() {
  int64_t loop_start_us = esp_timer_get_time();

  if (prev_loop_start_us > 0) {
    int64_t interval_us = loop_start_us - prev_loop_start_us;
    profiler.loop_interval_us.record(interval_us);
    int64_t jitter_us =
        interval_us > 20000 ? (interval_us - 20000) : (20000 - interval_us);
    profiler.loop_jitter_us.record(jitter_us);
  }
  prev_loop_start_us = loop_start_us;

  // 1. Ingest Serial Telemetry with Bounded Budget (Max 256 bytes per loop) and
  // non-blocking handling
  const size_t MAX_BYTES_PER_LOOP = 256;
  size_t bytes_ingested_this_loop = 0;

  while (Serial.available() && bytes_ingested_this_loop < MAX_BYTES_PER_LOOP) {
    bytes_ingested_this_loop++;
    telemetry_rx_bytes_total++;
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (discard_until_newline) {
        // Line terminator reached for an oversized/malformed packet; exit
        // discard state
        discard_until_newline = false;
        serial_buf_idx = 0;
      } else if (serial_buf_idx > 0) {
        serial_buffer[serial_buf_idx] = '\0';
        parseTelemetryPacket(serial_buffer, serial_buf_idx);
        serial_buf_idx = 0;
      }
    } else {
      if (discard_until_newline) {
        // Continue discarding characters belonging to the oversized packet
        continue;
      }
      if (serial_buf_idx < SERIAL_BUF_SIZE - 1) {
        serial_buffer[serial_buf_idx++] = c;
      } else {
        // Buffer capacity reached without line terminator: trigger discard
        // state Non-blocking: increment counter without blocking UART TX FIFO
        discard_until_newline = true;
        serial_buf_idx = 0;
        discarded_packet_count++;
        telemetry_overflow_events++;
      }
    }
  }

  // 2. Step Controller State Machine (Guaranteed to execute every loop tick
  // regardless of serial backlog)
  int64_t t_tick_start = esp_timer_get_time();
  controller.tick();
  int64_t t_tick_dur = esp_timer_get_time() - t_tick_start;
  profiler.controller_tick_us.record(t_tick_dur);

  // 3. Update Physical Signal Actuators (instrumented)
  int64_t t_gpio_start = esp_timer_get_time();
  updatePhysicalSignals(controller.getState());
  int64_t t_gpio_dur = esp_timer_get_time() - t_gpio_start;
  profiler.gpio_update_us.record(t_gpio_dur);

  // 4. Broadcast Controller Feedback Telemetry every 500ms (instrumented)
  if (millis() - last_telemetry_tx_ms >= 500) {
    last_telemetry_tx_ms = millis();
    int64_t t_diag_start = esp_timer_get_time();

    ApproachState n = controller.getApproachState('N');
    ApproachState s = controller.getApproachState('S');
    ApproachState e = controller.getApproachState('E');
    ApproachState w = controller.getApproachState('W');

    Serial.printf("{\"state\":\"%s\",\"rem\":%.1f,\"alloc\":%.1f,\"Q\":[%d,%d,%"
                  "d,%d],\"PCU\":[%.1f,%.1f,%.1f,%.1f]}\n",
                  controller.getStateName(), controller.getRemainingTime(),
                  controller.getAllocatedGreen(), n.q_count, s.q_count,
                  e.q_count, w.q_count, n.p_pcu, s.p_pcu, e.p_pcu, w.p_pcu);

    int64_t t_diag_dur = esp_timer_get_time() - t_diag_start;
    profiler.diag_tx_us.record(t_diag_dur);
  }

  // 5. Measure CPU Execution Time (pure execution overhead before nominal
  // delay)
  int64_t loop_exec_dur = esp_timer_get_time() - loop_start_us;
  profiler.loop_exec_us.record(loop_exec_dur);

  // 6. Check Profile Window
  profiler.window_iterations++;
  if (profiler.window_iterations >= profiler.kWindowSize) {
    printProfileReport();
    // Exclude the long diagnostic printing duration from the next normal loop
    // interval
    prev_loop_start_us = esp_timer_get_time();
  }

  delay(20);
}
