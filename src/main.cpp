#include <Arduino.h>
#include "TrafficController.hpp"
#include "TelemetryParser.hpp"

// Physical GPIO Pin Definitions for Traffic Signal LEDs
// North-South Axis
const int PIN_NS_RED    = 4;
const int PIN_NS_YELLOW = 5;
const int PIN_NS_GREEN  = 6;

// East-West Axis
const int PIN_EW_RED    = 7;
const int PIN_EW_YELLOW = 8;
const int PIN_EW_GREEN  = 9;

// Global Controller Instance
TrafficController controller;

// Fixed-size, zero-allocation serial ingestion buffer with overrun protection
const size_t SERIAL_BUF_SIZE = 512;
char serial_buffer[SERIAL_BUF_SIZE];
size_t serial_buf_idx = 0;

unsigned long last_telemetry_tx_ms = 0;
unsigned long last_queue_rx_ms = 0;

// Updates physical GPIO pins with hardcoded conflict interlock verification
// Standards Policy (MUTCD 4D): In STATE_FAILSAFE_FLASH, all approaches flash RED (1 Hz, 500ms ON / 500ms OFF).
// Flashing Red commands an All-Way Stop, eliminating opposing yellow/green right-of-way conflicts.
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
      // Symmetrical 4-Way Intersection Failsafe: All-Way Red Flashing (1 Hz: 500ms ON / 500ms OFF)
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

  // Hardware Safety Interlock: Physical impossibility of simultaneous active conflicting corridors
  if (!controller.verifyInterlock(ns_g, ns_y, ew_g, ew_y)) {
    digitalWrite(PIN_NS_RED, HIGH);
    digitalWrite(PIN_EW_RED, HIGH);
    digitalWrite(PIN_NS_YELLOW, LOW);
    digitalWrite(PIN_EW_YELLOW, LOW);
    digitalWrite(PIN_NS_GREEN, LOW);
    digitalWrite(PIN_EW_GREEN, LOW);
    Serial.println("[CRITICAL] Hardware Interlock Tripped! Conflicting active signals prohibited.");
    return;
  }

  // Actuate physical pins
  digitalWrite(PIN_NS_RED, ns_r ? HIGH : LOW);
  digitalWrite(PIN_NS_YELLOW, ns_y ? HIGH : LOW);
  digitalWrite(PIN_NS_GREEN, ns_g ? HIGH : LOW);

  digitalWrite(PIN_EW_RED, ew_r ? HIGH : LOW);
  digitalWrite(PIN_EW_YELLOW, ew_y ? HIGH : LOW);
  digitalWrite(PIN_EW_GREEN, ew_g ? HIGH : LOW);
}

// Parses and strictly validates telemetry packets using TelemetryParser
void parseTelemetryPacket(const char* json, size_t len) {
  ParsedTelemetry pkt;
  if (!TelemetryParser::parse(json, len, pkt)) {
    Serial.println("[TELEMETRY RX REJECTED] Malformed, incomplete, or corrupted packet discarded.");
    return;
  }

  controller.updateApproach(pkt.approach, pkt.q_count, pkt.d_tail_m, pkt.p_pcu, pkt.is_valid);
  last_queue_rx_ms = millis();
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

  // Initial startup: all-red
  digitalWrite(PIN_NS_RED, HIGH);
  digitalWrite(PIN_EW_RED, HIGH);

  Serial.println("\n========================================================");
  Serial.println("  ATCS Hardware Traffic Signal Controller Online");
  Serial.println("  Awaiting Ingested Telemetry over Serial");
  Serial.println("========================================================");
}

void loop() {
  // 1. Ingest Serial Telemetry from Vision Edge Node with fixed-size buffer & overrun protection
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serial_buf_idx > 0) {
        serial_buffer[serial_buf_idx] = '\0';
        parseTelemetryPacket(serial_buffer, serial_buf_idx);
        serial_buf_idx = 0;
      }
    } else {
      if (serial_buf_idx < SERIAL_BUF_SIZE - 1) {
        serial_buffer[serial_buf_idx++] = c;
      } else {
        // Line exceeded buffer capacity without newline: reject and reset to prevent memory exhaustion
        serial_buf_idx = 0;
        Serial.println("[WARN] Serial line buffer overrun; discarded non-delimited packet.");
      }
    }
  }

  // 2. Deterministic Sensor Timeout Policy handled inside controller.tick()
  // If no vision data is received for > 5000 ms, approaches expire to is_valid=false
  // and controller falls back to nominal deterministic cycles without fabricating values.

  // 3. Step Controller State Machine
  controller.tick();

  // 4. Update Physical Signal Actuators (with interlock check)
  updatePhysicalSignals(controller.getState());

  // 5. Broadcast Controller Feedback Telemetry every 500ms
  if (millis() - last_telemetry_tx_ms >= 500) {
    last_telemetry_tx_ms = millis();
    ApproachState n = controller.getApproachState('N');
    ApproachState s = controller.getApproachState('S');
    ApproachState e = controller.getApproachState('E');
    ApproachState w = controller.getApproachState('W');

    Serial.printf("{\"state\":\"%s\",\"rem\":%.1f,\"alloc\":%.1f,\"Q\":[%d,%d,%d,%d],\"PCU\":[%.1f,%.1f,%.1f,%.1f]}\n",
                  controller.getStateName(),
                  controller.getRemainingTime(),
                  controller.getAllocatedGreen(),
                  n.q_count, s.q_count, e.q_count, w.q_count,
                  n.p_pcu, s.p_pcu, e.p_pcu, w.p_pcu);
  }

  delay(20);
}
