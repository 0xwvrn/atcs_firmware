#pragma once

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
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

// Zero-allocation, strict length-bounded JSON telemetry validator & parser for ATCS
class TelemetryParser {
 public:
  static const size_t kMaxTelemetryLength = 1024;

  // Parses and strictly validates a telemetry packet.
  // Enforces:
  // 1. Length bounds (7 to 1024 bytes) without assuming NUL-termination.
  // 2. Strict outer JSON object boundaries '{' and '}'.
  // 3. Exact key matching with legal token delimiters (',', '}', whitespace).
  // 4. Duplicate key rejection (ensures each required key appears exactly once).
  // 5. Strict numeric and boolean syntax (rejects NaN, Inf, and trailing syntax garbage).
  // 6. Value domain validity (approach in {N,S,E,W}, non-negative queue, PCU, distance).
  static bool parse(const char* buffer, size_t length, ParsedTelemetry &out) {
    if (!buffer || length < 7 || length > kMaxTelemetryLength) return false;

    // Copy to stack working buffer to guarantee safe NUL termination without heap allocation
    char buf[kMaxTelemetryLength + 1];
    memcpy(buf, buffer, length);
    buf[length] = '\0';

    // 1. Validate Outer Boundaries
    size_t start = 0;
    while (start < length && isspace((unsigned char)buf[start])) start++;
    if (start >= length || buf[start] != '{') return false;

    size_t end = length - 1;
    while (end > start && isspace((unsigned char)buf[end])) end--;
    if (buf[end] != '}') return false;

    // Check balanced braces: exactly one root object '{' and '}'
    size_t brace_depth = 0;
    for (size_t i = start; i <= end; ++i) {
      if (buf[i] == '{') brace_depth++;
      else if (buf[i] == '}') {
        if (brace_depth == 0) return false;
        brace_depth--;
        if (brace_depth == 0 && i < end) return false; // Trailing garbage after root object
      }
    }
    if (brace_depth != 0) return false;

    // 2. Locate and enforce uniqueness for each required key
    const char* k_app = findKey(buf, start, end, "approach");
    const char* k_val = findKey(buf, start, end, "valid");
    const char* k_q   = findKey(buf, start, end, "Q");
    const char* k_pcu = findKey(buf, start, end, "PCU");

    if (!k_app || !k_val || !k_q || !k_pcu) return false;

    // Check for duplicate keys
    if (findNextKey(k_app + 10, end - (k_app + 10 - buf), "approach") != nullptr) return false;
    if (findNextKey(k_val + 7, end - (k_val + 7 - buf), "valid") != nullptr) return false;
    if (findNextKey(k_q + 3, end - (k_q + 3 - buf), "Q") != nullptr) return false;
    if (findNextKey(k_pcu + 5, end - (k_pcu + 5 - buf), "PCU") != nullptr) return false;

    // 3. Parse "approach": "X"
    char leg = '\0';
    if (!parseApproach(k_app, end, buf, leg)) return false;

    // 4. Parse "valid": true / false
    bool valid = false;
    if (!parseBool(k_val, end, buf, valid)) return false;

    // 5. Parse "Q": <int>
    int q_val = 0;
    if (!parseInt(k_q, end, buf, q_val)) return false;

    // 6. Parse "D_tail_m": <float> (optional, defaults to 0.0)
    float d_tail = 0.0f;
    const char* k_d = findKey(buf, start, end, "D_tail_m");
    if (k_d) {
      if (findNextKey(k_d + 10, end - (k_d + 10 - buf), "D_tail_m") != nullptr) return false;
      if (!parseFloat(k_d, end, buf, d_tail)) return false;
    }

    // 7. Parse "PCU": <float>
    float pcu_val = 0.0f;
    if (!parseFloat(k_pcu, end, buf, pcu_val)) return false;

    out.approach = leg;
    out.q_count = q_val;
    out.d_tail_m = d_tail;
    out.p_pcu = pcu_val;
    out.is_valid = valid;
    return true;
  }

 private:
  // Strict key search: must be enclosed in quotes and immediately preceded by '{' or ',' (ignoring whitespace)
  static const char* findKey(const char* buf, size_t start, size_t end, const char* key_name) {
    return findNextKey(buf + start, end - start + 1, key_name);
  }

  static const char* findNextKey(const char* start_ptr, size_t len, const char* key_name) {
    if (!start_ptr || len == 0) return nullptr;
    size_t key_len = strlen(key_name);
    const char* cur = start_ptr;
    const char* limit = start_ptr + len;

    while (cur + key_len + 2 <= limit) {
      if (*cur == '\"' && strncmp(cur + 1, key_name, key_len) == 0 && *(cur + 1 + key_len) == '\"') {
        // Verify key delimiter: after quote, must have ':' with optional whitespace
        const char* after = cur + 1 + key_len + 1;
        while (after < limit && isspace((unsigned char)*after)) after++;
        if (after < limit && *after == ':') {
          return cur;
        }
      }
      cur++;
    }
    return nullptr;
  }

  static bool isLegalValueDelimiter(char c) {
    return (c == ',' || c == '}' || isspace((unsigned char)c) || c == '\0');
  }

  static bool parseApproach(const char* key_ptr, size_t end, const char* buf, char &leg_out) {
    const char* colon = strchr(key_ptr, ':');
    if (!colon || colon > buf + end) return false;
    colon++;
    while (colon <= buf + end && isspace((unsigned char)*colon)) colon++;
    if (colon > buf + end || *colon != '\"') return false;
    colon++;
    if (colon >= buf + end) return false;
    char leg = *colon;
    if (leg != 'N' && leg != 'S' && leg != 'E' && leg != 'W') return false;
    colon++;
    if (colon > buf + end || *colon != '\"') return false;
    colon++;
    // Check legal delimiter after closing quote
    while (colon <= buf + end && isspace((unsigned char)*colon)) colon++;
    if (colon <= buf + end && !isLegalValueDelimiter(*colon)) return false;
    leg_out = leg;
    return true;
  }

  static bool parseBool(const char* key_ptr, size_t end, const char* buf, bool &bool_out) {
    const char* colon = strchr(key_ptr, ':');
    if (!colon || colon > buf + end) return false;
    colon++;
    while (colon <= buf + end && isspace((unsigned char)*colon)) colon++;
    if (colon > buf + end) return false;

    if (strncmp(colon, "true", 4) == 0) {
      char next = *(colon + 4);
      if (!isLegalValueDelimiter(next)) return false;
      bool_out = true;
      return true;
    } else if (strncmp(colon, "false", 5) == 0) {
      char next = *(colon + 5);
      if (!isLegalValueDelimiter(next)) return false;
      bool_out = false;
      return true;
    }
    return false;
  }

  static bool parseInt(const char* key_ptr, size_t end, const char* buf, int &int_out) {
    const char* colon = strchr(key_ptr, ':');
    if (!colon || colon > buf + end) return false;
    colon++;
    while (colon <= buf + end && isspace((unsigned char)*colon)) colon++;
    if (colon > buf + end) return false;
    if (!isdigit((unsigned char)*colon) && *colon != '-') return false;

    char* endptr = nullptr;
    long val = strtol(colon, &endptr, 10);
    if (!endptr || endptr == colon) return false;
    if (!isLegalValueDelimiter(*endptr)) return false;
    if (val < 0 || val > 500) return false;
    int_out = (int)val;
    return true;
  }

  static bool parseFloat(const char* key_ptr, size_t end, const char* buf, float &float_out) {
    const char* colon = strchr(key_ptr, ':');
    if (!colon || colon > buf + end) return false;
    colon++;
    while (colon <= buf + end && isspace((unsigned char)*colon)) colon++;
    if (colon > buf + end) return false;
    if (!isdigit((unsigned char)*colon) && *colon != '-' && *colon != '+') return false;

    char* endptr = nullptr;
    float val = strtof(colon, &endptr);
    if (!endptr || endptr == colon) return false;
    if (!isLegalValueDelimiter(*endptr)) return false;
    if (!isfinite(val) || val < 0.0f || val > 2000.0f) return false;
    float_out = val;
    return true;
  }
};
