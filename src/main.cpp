#include <Arduino.h>
#include <esp_system.h>
#include <esp_timer.h>

#include "BoardConfig.hpp"
#include "RuntimeProfiler.hpp"
#include "SignalOutputs.hpp"
#include "TelemetryParser.hpp"
#include "TelemetryReceiver.hpp"
#include "TrafficController.hpp"

// Global Controller, Physical Signal Output, Telemetry Receiver, and Profiler Instances
TrafficController controller;
SignalOutputs signal_outputs;
TelemetryReceiver telemetry_receiver;
RuntimeProfiler profiler;

unsigned long last_telemetry_tx_ms = 0;
unsigned long last_queue_rx_ms = 0;

void setup() {
  Serial.begin(BoardConfig::UART_BAUD_RATE);
  delay(1500);

  signal_outputs.setupPins();
}

void loop() {
  int64_t loop_start_us = esp_timer_get_time();
  profiler.recordLoopStart(loop_start_us);

  // 1. Ingest Serial Telemetry with Bounded Budget (Max 256 bytes per loop) and
  // non-blocking handling
  telemetry_receiver.process(
      Serial,
      [](const ParsedTelemetry &pkt) {
        controller.updateApproach(pkt.approach, pkt.q_count, pkt.d_tail_m,
                                  pkt.p_pcu, pkt.is_valid);
        last_queue_rx_ms = millis();
      },
      [](int64_t parse_us) {
        profiler.recordParserExec(parse_us);
      });

  // 2. Step Controller State Machine (Guaranteed to execute every loop tick
  // regardless of serial backlog)
  int64_t t_tick_start = esp_timer_get_time();
  controller.tick();
  profiler.recordControllerTick(esp_timer_get_time() - t_tick_start);

  // 3. Update Physical Signal Actuators (instrumented)
  int64_t t_gpio_start = esp_timer_get_time();
  signal_outputs.applyState(controller.getState());
  profiler.recordGpioUpdate(esp_timer_get_time() - t_gpio_start);

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

    profiler.recordDiagTx(esp_timer_get_time() - t_diag_start);
  }

  // 5. Measure CPU Execution Time (pure execution overhead before nominal
  // delay)
  profiler.recordLoopExec(esp_timer_get_time() - loop_start_us);

  // 6. Check Profile Window
  if (profiler.stepWindow()) {
    profiler.printReport(telemetry_receiver.getCounters());
    // Exclude the long diagnostic printing duration from the next normal loop
    // interval
    profiler.resetLoopReference(esp_timer_get_time());
  }

  delay(20);
}
