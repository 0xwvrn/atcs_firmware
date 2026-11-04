#include "TelemetryReceiver.hpp"

TelemetryReceiver::TelemetryReceiver() {
  reset();
}

void TelemetryReceiver::reset() {
  rx_buf_idx_ = 0;
  discard_until_newline_ = false;
  rx_bytes_total_ = 0;
  valid_packets_ = 0;
  rejected_packets_ = 0;
  overflow_events_ = 0;
  discarded_packet_count_ = 0;
  rx_buffer_[0] = '\0';
}

void TelemetryReceiver::handleCompleteFrame(TelemetryPacketHandler on_packet,
                                            TelemetryProfileHandler on_profile) {
#ifdef ARDUINO
  int64_t t_parse_start = esp_timer_get_time();
#else
  auto t_parse_start = std::chrono::steady_clock::now();
#endif

  ParsedTelemetry pkt;
  bool parse_success = TelemetryParser::parse(rx_buffer_, rx_buf_idx_, pkt);

#ifdef ARDUINO
  int64_t t_parse_dur = esp_timer_get_time() - t_parse_start;
#else
  int64_t t_parse_dur = std::chrono::duration_cast<std::chrono::microseconds>(
                            std::chrono::steady_clock::now() - t_parse_start)
                            .count();
#endif

  if (on_profile) {
    on_profile(t_parse_dur);
  }

  if (!parse_success) {
    rejected_packets_++;
    return;
  }

  valid_packets_++;
  if (on_packet) {
    on_packet(pkt);
  }
}

TelemetryCounters TelemetryReceiver::getCounters() const {
  return TelemetryCounters{
      rx_bytes_total_,
      valid_packets_,
      rejected_packets_,
      overflow_events_,
      discarded_packet_count_};
}
