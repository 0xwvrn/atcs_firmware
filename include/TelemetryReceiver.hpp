#pragma once

#include <stdint.h>
#include <cstddef>

#include "BoardConfig.hpp"
#include "TelemetryParser.hpp"

#ifdef ARDUINO
#include <Arduino.h>
#include <esp_timer.h>
#else
#include <chrono>
#endif

// Structured snapshot of telemetry reception counters
struct TelemetryCounters {
  uint32_t rx_bytes_total;
  uint32_t valid_packets;
  uint32_t rejected_packets;
  uint32_t overflow_events;
  unsigned long discarded_packet_count;
};

// Function pointer callbacks for zero-allocation packet and profiling delivery
typedef void (*TelemetryPacketHandler)(const ParsedTelemetry& pkt);
typedef void (*TelemetryProfileHandler)(int64_t parse_duration_us);

class TelemetryReceiver {
 public:
  TelemetryReceiver();

  // Resets reception buffer, discard flags, and counters
  void reset();

  // Ingests up to max_bytes from an input byte stream (e.g. Arduino Serial or native mock stream)
  template <typename TStream>
  size_t process(TStream& stream,
                 TelemetryPacketHandler on_packet,
                 TelemetryProfileHandler on_profile = nullptr,
                 size_t max_bytes = BoardConfig::MAX_BYTES_PER_LOOP) {
    size_t bytes_ingested_this_loop = 0;

    while (stream.available() && bytes_ingested_this_loop < max_bytes) {
      bytes_ingested_this_loop++;
      rx_bytes_total_++;
      char c = static_cast<char>(stream.read());

      if (c == '\n' || c == '\r') {
        if (discard_until_newline_) {
          // Line terminator reached for an oversized packet; exit discard state
          discard_until_newline_ = false;
          rx_buf_idx_ = 0;
        } else if (rx_buf_idx_ > 0) {
          rx_buffer_[rx_buf_idx_] = '\0';
          handleCompleteFrame(on_packet, on_profile);
          rx_buf_idx_ = 0;
        }
      } else {
        if (discard_until_newline_) {
          // Continue discarding characters belonging to the oversized packet
          continue;
        }
        if (rx_buf_idx_ < BoardConfig::SERIAL_BUF_SIZE - 1) {
          rx_buffer_[rx_buf_idx_++] = c;
        } else {
          // Buffer capacity reached without line terminator: trigger discard state
          discard_until_newline_ = true;
          rx_buf_idx_ = 0;
          discarded_packet_count_++;
          overflow_events_++;
        }
      }
    }

    return bytes_ingested_this_loop;
  }

  // Diagnostics and profiling counters
  TelemetryCounters getCounters() const;
  uint32_t getRxBytesTotal() const { return rx_bytes_total_; }
  uint32_t getValidPackets() const { return valid_packets_; }
  uint32_t getRejectedPackets() const { return rejected_packets_; }
  uint32_t getOverflowEvents() const { return overflow_events_; }
  unsigned long getDiscardedPacketCount() const { return discarded_packet_count_; }

  // Buffer state inspection for verification
  size_t getBufferIndex() const { return rx_buf_idx_; }
  bool isDiscarding() const { return discard_until_newline_; }

 private:
  char rx_buffer_[BoardConfig::SERIAL_BUF_SIZE];
  size_t rx_buf_idx_;
  bool discard_until_newline_;

  uint32_t rx_bytes_total_;
  uint32_t valid_packets_;
  uint32_t rejected_packets_;
  uint32_t overflow_events_;
  unsigned long discarded_packet_count_;

  void handleCompleteFrame(TelemetryPacketHandler on_packet,
                           TelemetryProfileHandler on_profile);
};
