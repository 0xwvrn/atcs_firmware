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

// Structured telemetry packet container (Canonical Schema v1)
struct ParsedTelemetry {
  int protocol_version;     // Schema version ("v"), must be 1 (defaults to 1 if omitted)
  char approach;            // 'N', 'S', 'E', 'W'
  int q_count;              // Valid vehicle queue count (>= 0)
  float d_tail_m;           // Validated metric distance to queue tail in meters (>= 0.0)
  float p_pcu;              // Stopline-connected queue Passenger Car Unit demand (>= 0.0)
  float pcu_upstream;       // Stationary upstream cluster PCU (>= 0.0)
  float inflow_pcu;         // Approaching physical upstream demand PCU (>= 0.0)
  float arrival_rate_pcu_s; // Rolling-window unique vehicle arrival rate in PCU/s (>= 0.0)
  bool is_calibrated;       // True only when homography calibration status is VALID
  bool is_fresh;            // True only when camera stream is fresh and not frozen
  bool is_valid;            // Strict pipeline observation validity gate (valid && fresh)
  long seq;                 // Packet sequence counter (>= 0)
  int8_t profile_cmd;       // -1: unchanged, 0: BASELINE_45S, 1: URBAN_ARTERIAL, 2: HEAVY_METRO
  int8_t topology_cmd;      // -1: unchanged, 0: STANDARD_4WAY, 1: T_3WAY_NS_E, 2: T_3WAY_EW_N, 3: DOUBLE_CYCLE
  int8_t moderato_cmd;      // -1: unchanged, 0: disable, 1: enable
};

// Zero-allocation, strict length-bounded JSON telemetry validator & parser for ATCS
class TelemetryParser {
 public:
  // Matches BoardConfig::SERIAL_BUF_SIZE - 1 (511 JSON payload bytes + 1 newline/NUL byte = 512 bytes)
  static const size_t kMaxTelemetryLength = 511;
  static const int kSupportedProtocolVersion = 1;

  // Parses and strictly validates a telemetry packet.
  // Enforces:
  // 1. Length bounds (7 to 511 bytes) without assuming NUL-termination.
  // 2. Strict outer JSON object boundaries '{' and '}'.
  // 3. Exact key matching with legal token delimiters (',', '}', whitespace).
  // 4. Duplicate key rejection (ensures each key appears at most once).
  // 5. Strict numeric and boolean syntax (rejects NaN, Inf, negative values, and trailing syntax garbage).
  // 6. Value domain validity (approach in {N,S,E,W}, protocol version == 1, non-negative queue, PCU, distance).
  static bool parse(const char* buffer, size_t length, ParsedTelemetry &out) {
    if (!buffer || length < 7 || length > kMaxTelemetryLength) return false;

    // Reject embedded null bytes in input
    for (size_t i = 0; i < length; ++i) {
      if (buffer[i] == '\0') return false;
    }

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

    // Check balanced braces and strict top-level JSON key-value syntax with universal duplicate-key rejection
    if (!validateJsonObjectAndUniqueKeys(buf, start, end)) return false;

    // 1b. Optional protocol version ("v"): defaults to 1, rejects unknown versions
    int proto_ver = kSupportedProtocolVersion;
    const char* k_ver = findKey(buf, start, end, "v");
    if (k_ver) {
      if (findNextKey(k_ver + 3, end - (k_ver + 3 - buf), "v") != nullptr) return false;
      if (!parseInt(k_ver, end, buf, proto_ver) || proto_ver != kSupportedProtocolVersion) return false;
    }

    // 2. Locate and enforce uniqueness for approach identifier ("approach" or "leg")
    const char* k_app = findKey(buf, start, end, "approach");
    const char* k_leg = findKey(buf, start, end, "leg");
    if (!k_app && !k_leg) return false; // Missing approach/leg key
    if (k_app && k_leg) return false;   // Both "approach" and "leg" present is an ambiguous duplicate

    const char* app_key_ptr = nullptr;
    if (k_app) {
      if (findNextKey(k_app + 10, end - (k_app + 10 - buf), "approach") != nullptr) return false;
      app_key_ptr = k_app;
    } else {
      if (findNextKey(k_leg + 5, end - (k_leg + 5 - buf), "leg") != nullptr) return false;
      app_key_ptr = k_leg;
    }

    // Locate remaining required keys: "valid", "Q", "PCU"
    const char* k_val = findKey(buf, start, end, "valid");
    const char* k_q   = findKey(buf, start, end, "Q");
    const char* k_pcu = findKey(buf, start, end, "PCU");

    if (!k_val || !k_q || !k_pcu) return false;

    // Check for duplicate keys
    if (findNextKey(k_val + 7, end - (k_val + 7 - buf), "valid") != nullptr) return false;
    if (findNextKey(k_q + 3, end - (k_q + 3 - buf), "Q") != nullptr) return false;
    if (findNextKey(k_pcu + 5, end - (k_pcu + 5 - buf), "PCU") != nullptr) return false;

    // 3. Parse approach: "X"
    char leg = '\0';
    if (!parseApproach(app_key_ptr, end, buf, leg)) return false;

    // 4. Parse "valid": true / false
    bool valid = false;
    if (!parseBool(k_val, end, buf, valid)) return false;

    // 5. Parse "Q": <int>
    int q_val = 0;
    if (!parseInt(k_q, end, buf, q_val)) return false;

    // 6. Parse "D_tail_m" or "d_tail": <float> (optional, defaults to 0.0)
    float d_tail = 0.0f;
    const char* k_d = findKey(buf, start, end, "D_tail_m");
    const char* k_d_alias = findKey(buf, start, end, "d_tail");
    if (k_d && k_d_alias) return false; // Ambiguous concurrent duplicate

    const char* d_key_ptr = k_d ? k_d : k_d_alias;
    if (d_key_ptr) {
      const char* key_name = k_d ? "D_tail_m" : "d_tail";
      size_t key_len = strlen(key_name);
      if (findNextKey(d_key_ptr + key_len + 2, end - (d_key_ptr + key_len + 2 - buf), key_name) != nullptr) return false;
      if (!parseFloat(d_key_ptr, end, buf, d_tail)) return false;
    }

    // 7. Parse "PCU": <float>
    float pcu_val = 0.0f;
    if (!parseFloat(k_pcu, end, buf, pcu_val)) return false;

    // 8. Parse "pcu_upstream": <float> (optional, defaults to 0.0)
    float pcu_up_val = 0.0f;
    const char* k_up = findKey(buf, start, end, "pcu_upstream");
    if (k_up) {
      if (findNextKey(k_up + 14, end - (k_up + 14 - buf), "pcu_upstream") != nullptr) return false;
      if (!parseFloat(k_up, end, buf, pcu_up_val)) return false;
    }

    // 8b. Parse "inflow_pcu": <float> (optional, defaults to 0.0)
    float inflow_val = 0.0f;
    const char* k_in = findKey(buf, start, end, "inflow_pcu");
    if (k_in) {
      if (findNextKey(k_in + 12, end - (k_in + 12 - buf), "inflow_pcu") != nullptr) return false;
      if (!parseFloat(k_in, end, buf, inflow_val)) return false;
    }

    // 8c. Parse "arrival_rate_pcu_s": <float> (optional, defaults to 0.0, max 50.0 PCU/s)
    float arr_rate_val = 0.0f;
    const char* k_arr = findKey(buf, start, end, "arrival_rate_pcu_s");
    if (k_arr) {
      if (findNextKey(k_arr + 20, end - (k_arr + 20 - buf), "arrival_rate_pcu_s") != nullptr) return false;
      if (!parseFloat(k_arr, end, buf, arr_rate_val) || arr_rate_val > 50.0f) return false;
    }

    // 8d. Parse optional "calibrated" and "fresh" booleans and "seq" counter
    bool calibrated = valid;
    const char* k_cal = findKey(buf, start, end, "calibrated");
    if (k_cal) {
      if (findNextKey(k_cal + 12, end - (k_cal + 12 - buf), "calibrated") != nullptr) return false;
      if (!parseBool(k_cal, end, buf, calibrated)) return false;
    }

    bool fresh = valid;
    const char* k_fr = findKey(buf, start, end, "fresh");
    if (k_fr) {
      if (findNextKey(k_fr + 7, end - (k_fr + 7 - buf), "fresh") != nullptr) return false;
      if (!parseBool(k_fr, end, buf, fresh)) return false;
    }

    long seq_val = 0;
    const char* k_seq = findKey(buf, start, end, "seq");
    if (k_seq) {
      if (findNextKey(k_seq + 5, end - (k_seq + 5 - buf), "seq") != nullptr) return false;
      int parsed_seq = 0;
      if (!parseInt(k_seq, end, buf, parsed_seq, 2147483647L)) return false;
      seq_val = parsed_seq;
    }

    // 9. Optional runtime configuration commands ("profile", "topology", "moderato")
    int8_t profile_cmd = -1;
    const char* k_prof = findKey(buf, start, end, "profile");
    if (k_prof && !parseProfileCmd(k_prof, end, buf, profile_cmd)) return false;

    int8_t topology_cmd = -1;
    const char* k_topo = findKey(buf, start, end, "topology");
    if (k_topo && !parseTopologyCmd(k_topo, end, buf, topology_cmd)) return false;

    int8_t moderato_cmd = -1;
    const char* k_mod = findKey(buf, start, end, "moderato");
    if (k_mod) {
      bool mod_val = false;
      if (!parseBool(k_mod, end, buf, mod_val)) return false;
      moderato_cmd = mod_val ? 1 : 0;
    }

    bool effective_valid = valid && fresh;
    out.protocol_version = proto_ver;
    out.approach = leg;
    out.q_count = effective_valid ? q_val : 0;
    out.d_tail_m = (effective_valid && calibrated) ? d_tail : 0.0f;
    out.p_pcu = effective_valid ? pcu_val : 0.0f;
    out.pcu_upstream = (effective_valid && calibrated) ? pcu_up_val : 0.0f;
    out.inflow_pcu = (effective_valid && calibrated) ? inflow_val : 0.0f;
    out.arrival_rate_pcu_s = (effective_valid && calibrated) ? arr_rate_val : 0.0f;
    out.is_calibrated = calibrated;
    out.is_fresh = fresh;
    out.is_valid = effective_valid;
    out.seq = seq_val;
    out.profile_cmd = profile_cmd;
    out.topology_cmd = topology_cmd;
    out.moderato_cmd = moderato_cmd;
    return true;
  }

 private:
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

  static bool validateJsonObjectAndUniqueKeys(const char* buf, size_t start, size_t end) {
    if (buf[start] != '{' || buf[end] != '}') return false;

    const char* key_ptrs[32];
    size_t key_lens[32];
    size_t key_count = 0;

    size_t pos = start + 1;
    while (pos < end && isspace((unsigned char)buf[pos])) pos++;
    if (pos == end) return false; // Empty object "{}"

    while (pos < end) {
      // 1. Parse quoted key
      if (buf[pos] != '\"') return false;
      size_t k_start = pos + 1;
      size_t k_end = k_start;
      while (k_end < end && buf[k_end] != '\"') {
        if (buf[k_end] == '\\') return false;
        k_end++;
      }
      if (k_end >= end || k_end == k_start) return false;
      size_t k_len = k_end - k_start;

      // Reject duplicate keys across the entire JSON object
      for (size_t i = 0; i < key_count; ++i) {
        if (key_lens[i] == k_len && strncmp(key_ptrs[i], buf + k_start, k_len) == 0) {
          return false;
        }
      }
      if (key_count >= 32) return false;
      key_ptrs[key_count] = buf + k_start;
      key_lens[key_count] = k_len;
      key_count++;

      pos = k_end + 1;
      while (pos < end && isspace((unsigned char)buf[pos])) pos++;
      if (pos >= end || buf[pos] != ':') return false;
      pos++;
      while (pos < end && isspace((unsigned char)buf[pos])) pos++;
      if (pos >= end) return false;

      // 2. Validate JSON value token (string, number, true, false, null)
      if (buf[pos] == '\"') {
        pos++;
        while (pos < end && buf[pos] != '\"') {
          if (buf[pos] == '\\') return false;
          pos++;
        }
        if (pos >= end) return false;
        pos++;
      } else if (strncmp(buf + pos, "true", 4) == 0) {
        pos += 4;
      } else if (strncmp(buf + pos, "false", 5) == 0) {
        pos += 5;
      } else if (strncmp(buf + pos, "null", 4) == 0) {
        pos += 4;
      } else if (isdigit((unsigned char)buf[pos]) || buf[pos] == '-') {
        if (buf[pos] == '-') {
          pos++;
          if (pos >= end || !isdigit((unsigned char)buf[pos])) return false;
        }
        if (buf[pos] == '0' && pos + 1 < end && (buf[pos + 1] == 'x' || buf[pos + 1] == 'X' || isdigit((unsigned char)buf[pos + 1]))) {
          return false; // Reject hex literals and leading zeros
        }
        while (pos < end && isdigit((unsigned char)buf[pos])) pos++;
        if (pos < end && buf[pos] == '.') {
          pos++;
          if (pos >= end || !isdigit((unsigned char)buf[pos])) return false;
          while (pos < end && isdigit((unsigned char)buf[pos])) pos++;
        }
        if (pos < end && (buf[pos] == 'e' || buf[pos] == 'E')) {
          pos++;
          if (pos < end && (buf[pos] == '+' || buf[pos] == '-')) pos++;
          if (pos >= end || !isdigit((unsigned char)buf[pos])) return false;
          while (pos < end && isdigit((unsigned char)buf[pos])) pos++;
        }
      } else {
        return false; // Reject nested objects/arrays or unquoted syntax garbage
      }

      while (pos < end && isspace((unsigned char)buf[pos])) pos++;
      if (pos == end) break;
      if (buf[pos] != ',') return false;
      pos++;
      while (pos < end && isspace((unsigned char)buf[pos])) pos++;
      if (pos == end) return false; // Reject trailing comma before '}'
    }
    return true;
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

  static bool parseInt(const char* key_ptr, size_t end, const char* buf, int &int_out, long max_val = 500L) {
    const char* colon = strchr(key_ptr, ':');
    if (!colon || colon > buf + end) return false;
    colon++;
    while (colon <= buf + end && isspace((unsigned char)*colon)) colon++;
    if (colon > buf + end) return false;
    // Strictly reject negative signs for vehicle counts
    if (!isdigit((unsigned char)*colon)) return false;

    char* endptr = nullptr;
    long val = strtol(colon, &endptr, 10);
    if (!endptr || endptr == colon) return false;
    if (!isLegalValueDelimiter(*endptr)) return false;
    if (val < 0 || val > max_val) return false;
    int_out = (int)val;
    return true;
  }

  static bool parseFloat(const char* key_ptr, size_t end, const char* buf, float &float_out) {
    const char* colon = strchr(key_ptr, ':');
    if (!colon || colon > buf + end) return false;
    colon++;
    while (colon <= buf + end && isspace((unsigned char)*colon)) colon++;
    if (colon > buf + end) return false;
    // Strictly reject negative numbers, leading '+', hex literals, NaN, and Inf
    if (!isdigit((unsigned char)*colon)) return false;
    if (*colon == '0' && (colon[1] == 'x' || colon[1] == 'X')) return false;

    char* endptr = nullptr;
    float val = strtof(colon, &endptr);
    if (!endptr || endptr == colon) return false;
    if (!isLegalValueDelimiter(*endptr)) return false;
    if (!isfinite(val) || val < 0.0f || val > 2000.0f) return false;
    float_out = val;
    return true;
  }

  static bool parseStringToken(const char* key_ptr, size_t end, const char* buf, char* out_tok, size_t max_len) {
    const char* colon = strchr(key_ptr, ':');
    if (!colon || colon > buf + end) return false;
    colon++;
    while (colon <= buf + end && isspace((unsigned char)*colon)) colon++;
    if (colon > buf + end || *colon != '\"') return false;
    colon++;
    const char* close_q = strchr(colon, '\"');
    if (!close_q || close_q > buf + end) return false;
    size_t tok_len = (size_t)(close_q - colon);
    if (tok_len == 0 || tok_len >= max_len) return false;
    memcpy(out_tok, colon, tok_len);
    out_tok[tok_len] = '\0';
    const char* after = close_q + 1;
    while (after <= buf + end && isspace((unsigned char)*after)) after++;
    return (after > buf + end || isLegalValueDelimiter(*after));
  }

  static bool parseProfileCmd(const char* key_ptr, size_t end, const char* buf, int8_t &cmd_out) {
    char tok[32];
    if (!parseStringToken(key_ptr, end, buf, tok, sizeof(tok))) return false;
    if (strcmp(tok, "BASELINE_45S") == 0 || strcmp(tok, "BASELINE") == 0) { cmd_out = 0; return true; }
    if (strcmp(tok, "URBAN_ARTERIAL") == 0) { cmd_out = 1; return true; }
    if (strcmp(tok, "HEAVY_METRO") == 0) { cmd_out = 2; return true; }
    return false;
  }

  static bool parseTopologyCmd(const char* key_ptr, size_t end, const char* buf, int8_t &cmd_out) {
    char tok[32];
    if (!parseStringToken(key_ptr, end, buf, tok, sizeof(tok))) return false;
    if (strcmp(tok, "STANDARD_4WAY") == 0) { cmd_out = 0; return true; }
    if (strcmp(tok, "T_JUNCTION_3WAY_NS_E") == 0) { cmd_out = 1; return true; }
    if (strcmp(tok, "T_JUNCTION_3WAY_EW_N") == 0) { cmd_out = 2; return true; }
    if (strcmp(tok, "DOUBLE_CYCLE_NS_E_NS_W") == 0) { cmd_out = 3; return true; }
    return false;
  }
};
