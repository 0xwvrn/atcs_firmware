#include <Arduino.h>
#include "TrafficController.hpp"

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

// Serial Ingestion Buffers
String serial_buffer = "";
unsigned long last_telemetry_tx_ms = 0;
unsigned long last_queue_rx_ms = 0;

// Updates physical GPIO pins with hardcoded conflict interlock verification
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
    default:
      // Flashing yellow fail-safe
      ns_y = (millis() / 500) % 2;
      ew_y = (millis() / 500) % 2;
      break;
  }

  // Hardware Safety Interlock: Physical impossibility of dual-green
  if (!controller.verifyInterlock(ns_g, ns_y, ew_g, ew_y)) {
    digitalWrite(PIN_NS_RED, HIGH);
    digitalWrite(PIN_EW_RED, HIGH);
    digitalWrite(PIN_NS_YELLOW, LOW);
    digitalWrite(PIN_EW_YELLOW, LOW);
    digitalWrite(PIN_NS_GREEN, LOW);
    digitalWrite(PIN_EW_GREEN, LOW);
    Serial.println("[CRITICAL] Hardware Interlock Tripped! Dual green prohibited.");
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

// Parses validated telemetry packets from Vision Edge Node:
// {"seq": 1, "approach": "N", "valid": true, "Q": 3, "D_tail_m": 18.4, "PCU": 4.5, "timestamp_ms": 1730234567}
void parseTelemetryPacket(const String &json) {
  int idx_app = json.indexOf("\"approach\":");
  int idx_q   = json.indexOf("\"Q\":");
  int idx_d   = json.indexOf("\"D_tail_m\":");
  int idx_pcu = json.indexOf("\"PCU\":");

  if (idx_app == -1 || idx_q == -1) return;

  // Extract approach char ('N', 'S', 'E', 'W')
  int quote1 = json.indexOf("\"", idx_app + 11);
  char leg = 'N';
  if (quote1 != -1 && quote1 + 1 < (int)json.length()) {
    leg = json.charAt(quote1 + 1);
  }

  int q = json.substring(idx_q + 4).toInt();

  float d_tail = 0.0f;
  if (idx_d != -1) {
    d_tail = json.substring(idx_d + 11).toFloat();
  }

  float pcu = (float)q;
  if (idx_pcu != -1) {
    pcu = json.substring(idx_pcu + 6).toFloat();
  }

  // Parse validity and calibration gating
  bool is_valid = true;
  int idx_val = json.indexOf("\"valid\":");
  if (idx_val != -1) {
    String val_str = json.substring(idx_val + 8, idx_val + 13);
    is_valid = val_str.startsWith("true");
  }

  controller.updateApproach(leg, q, d_tail, pcu, is_valid);
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
  // 1. Ingest Serial Telemetry from Vision Edge Node
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serial_buffer.length() > 0) {
        parseTelemetryPacket(serial_buffer);
        serial_buffer = "";
      }
    } else {
      serial_buffer += c;
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
