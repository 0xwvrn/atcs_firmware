#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#ifndef ARDUINO
#include <cmath>
using std::isfinite;
#endif

// Structured telemetry packet container
struct ParsedTelemetry {
  char approach;     // 'N', 'S', 'E', 'W'
  int q_count;       // Valid vehicle queue count (>= 0)
  float d_tail_m;    // Distance to queue tail in meters (>= 0.0)
  float p_pcu;       // Passenger Car Unit demand (>= 0.0)
  bool is_valid;     // Strict pipeline calibration and sanity gate
};

// Zero-allocation, deterministic JSON telemetry parser for ATCS serial packets
class TelemetryParser {
 public:
  // Parses and strictly validates a telemetry packet string.
  // Returns true if and only if the packet has valid structural delimiters,
  // contains all required keys ("approach", "valid", "Q", "PCU"), and contains
  // fully valid, finite numeric values without trailing malformed syntax.
  static bool parse(const char* buffer, size_t length, ParsedTelemetry &out) {
    if (!buffer || length < 7 || length > 1024) return false;

    // 1. Boundary Verification: Must start with '{' and end with '}'
    size_t start = 0;
    while (start < length && (buffer[start] == ' ' || buffer[start] == '\r' || buffer[start] == '\n' || buffer[start] == '\t')) {
      start++;
    }
    if (start >= length || buffer[start] != '{') return false;

    size_t end = length - 1;
    while (end > start && (buffer[end] == ' ' || buffer[end] == '\r' || buffer[end] == '\n' || buffer[end] == '\t')) {
      end--;
    }
    if (buffer[end] != '}') return false;

    // 2. Extract "approach": "X"
    const char* p_app = strstr(buffer, "\"approach\"");
    if (!p_app) return false;
    const char* colon = strchr(p_app, ':');
    if (!colon || colon > buffer + end) return false;
    const char* q1 = strchr(colon, '\"');
    if (!q1 || q1 >= buffer + end) return false;
    char leg = *(q1 + 1);
    if (leg != 'N' && leg != 'S' && leg != 'E' && leg != 'W') return false;
    if (*(q1 + 2) != '\"') return false;

    // 3. Extract "valid": true / false
    bool valid = false;
    const char* p_val = strstr(buffer, "\"valid\"");
    if (!p_val) return false;
    colon = strchr(p_val, ':');
    if (!colon || colon > buffer + end) return false;
    colon++;
    while (*colon == ' ' || *colon == '\t') colon++;
    if (strncmp(colon, "true", 4) == 0) {
      valid = true;
    } else if (strncmp(colon, "false", 5) == 0) {
      valid = false;
    } else {
      return false; // Malformed boolean token
    }

    // 4. Extract "Q": <int> (strictly matches "\"Q\":", preventing match on "Q_stopline")
    const char* p_q = nullptr;
    const char* cur = buffer;
    while ((cur = strstr(cur, "\"Q\"")) != nullptr) {
      const char* after_key = cur + 3;
      while (*after_key == ' ' || *after_key == '\t') after_key++;
      if (*after_key == ':') {
        p_q = cur;
        break;
      }
      cur += 3;
    }
    if (!p_q) return false;
    colon = strchr(p_q, ':');
    if (!colon || colon > buffer + end) return false;
    colon++;
    while (*colon == ' ' || *colon == '\t') colon++;
    char* endptr = nullptr;
    long q_val = strtol(colon, &endptr, 10);
    if (endptr == colon || q_val < 0 || q_val > 500) return false;

    // 5. Extract "D_tail_m": <float> (optional, defaults to 0.0)
    float d_tail = 0.0f;
    const char* p_d = strstr(buffer, "\"D_tail_m\"");
    if (p_d) {
      colon = strchr(p_d, ':');
      if (colon && colon < buffer + end) {
        colon++;
        while (*colon == ' ' || *colon == '\t') colon++;
        d_tail = strtof(colon, &endptr);
        if (endptr == colon || !isfinite(d_tail) || d_tail < 0.0f || d_tail > 2000.0f) {
          return false;
        }
      }
    }

    // 6. Extract "PCU": <float> (strictly matches "\"PCU\":", preventing match on "PCU_stopline")
    const char* p_pcu = nullptr;
    cur = buffer;
    while ((cur = strstr(cur, "\"PCU\"")) != nullptr) {
      const char* after_key = cur + 5;
      while (*after_key == ' ' || *after_key == '\t') after_key++;
      if (*after_key == ':') {
        p_pcu = cur;
        break;
      }
      cur += 5;
    }
    if (!p_pcu) return false;
    colon = strchr(p_pcu, ':');
    if (!colon || colon > buffer + end) return false;
    colon++;
    while (*colon == ' ' || *colon == '\t') colon++;
    float pcu_val = strtof(colon, &endptr);
    if (endptr == colon || !isfinite(pcu_val) || pcu_val < 0.0f || pcu_val > 2000.0f) {
      return false;
    }

    out.approach = leg;
    out.q_count = (int)q_val;
    out.d_tail_m = d_tail;
    out.p_pcu = pcu_val;
    out.is_valid = valid;
    return true;
  }
};
