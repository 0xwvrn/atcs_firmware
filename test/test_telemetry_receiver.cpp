#include <cassert>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "../include/BoardConfig.hpp"
#include "../include/TelemetryParser.hpp"
#include "../include/TelemetryReceiver.hpp"

// Lightweight in-memory byte stream conforming to Arduino Stream interface
class MockByteStream {
 public:
  MockByteStream() = default;
  explicit MockByteStream(const std::string& data) : buffer_(data) {}

  void append(const std::string& data) { buffer_ += data; }

  int available() const {
    return static_cast<int>(buffer_.size() - read_pos_);
  }

  int read() {
    if (read_pos_ < buffer_.size()) {
      return static_cast<unsigned char>(buffer_[read_pos_++]);
    }
    return -1;
  }

 private:
  std::string buffer_;
  size_t read_pos_ = 0;
};

// Global test fixture containers for callback capture
static std::vector<ParsedTelemetry> g_received_packets;
static int64_t g_last_profile_us = -1;
static uint32_t g_profile_invocations = 0;

static void testPacketCallback(const ParsedTelemetry& pkt) {
  g_received_packets.push_back(pkt);
}

static void testProfileCallback(int64_t duration_us) {
  g_last_profile_us = duration_us;
  g_profile_invocations++;
}

static void resetTestFixtures() {
  g_received_packets.clear();
  g_last_profile_us = -1;
  g_profile_invocations = 0;
}

void testValidPacketReception() {
  std::cout << "\n--- TelemetryReceiver Test 1: Single Valid Packet ---" << std::endl;
  resetTestFixtures();
  TelemetryReceiver rx;

  const std::string pkt_str = "{\"approach\":\"N\",\"valid\":true,\"Q\":4,\"D_tail_m\":15.0,\"PCU\":3.5}\n";
  MockByteStream stream(pkt_str);

  size_t read_bytes = rx.process(stream, testPacketCallback, testProfileCallback);
  assert(read_bytes == pkt_str.size());
  assert(rx.getRxBytesTotal() == pkt_str.size());
  assert(rx.getValidPackets() == 1);
  assert(rx.getRejectedPackets() == 0);
  assert(rx.getOverflowEvents() == 0);
  assert(g_received_packets.size() == 1);
  assert(g_received_packets[0].approach == 'N');
  assert(g_received_packets[0].q_count == 4);
  assert(g_received_packets[0].d_tail_m == 15.0f);
  assert(g_received_packets[0].p_pcu == 3.5f);
  assert(g_received_packets[0].is_valid == true);
  assert(g_profile_invocations == 1);
  assert(g_last_profile_us >= 0);
  std::cout << "  [PASS] Single valid packet parsed, delivered, and profiled correctly." << std::endl;
}

void testMultiplePacketsInSingleStream() {
  std::cout << "\n--- TelemetryReceiver Test 2: Multiple Packets in Single Loop ---" << std::endl;
  resetTestFixtures();
  TelemetryReceiver rx;

  const std::string multi_pkt =
      "{\"approach\":\"N\",\"valid\":true,\"Q\":2,\"PCU\":2.0}\n"
      "{\"approach\":\"S\",\"valid\":true,\"Q\":5,\"PCU\":4.5}\n";
  MockByteStream stream(multi_pkt);

  size_t read_bytes = rx.process(stream, testPacketCallback, testProfileCallback);
  assert(read_bytes == multi_pkt.size());
  assert(rx.getValidPackets() == 2);
  assert(g_received_packets.size() == 2);
  assert(g_received_packets[0].approach == 'N');
  assert(g_received_packets[0].q_count == 2);
  assert(g_received_packets[1].approach == 'S');
  assert(g_received_packets[1].q_count == 5);
  std::cout << "  [PASS] Sequential packets within budget parsed and dispatched in order." << std::endl;
}

void testPartialPacketsSpanningMultipleLoops() {
  std::cout << "\n--- TelemetryReceiver Test 3: Partial Packet Spanning Iterations ---" << std::endl;
  resetTestFixtures();
  TelemetryReceiver rx;

  // First fragment arrives
  MockByteStream fragment1("{\"approach\":\"E\",");
  size_t b1 = rx.process(fragment1, testPacketCallback, testProfileCallback);
  assert(b1 == 16);
  assert(rx.getValidPackets() == 0);
  assert(g_received_packets.empty());
  assert(rx.getBufferIndex() == 16);

  // Second fragment arrives and finishes the line
  const std::string frag2 = "\"valid\":true,\"Q\":3,\"PCU\":3.0}\n";
  MockByteStream fragment2(frag2);
  size_t b2 = rx.process(fragment2, testPacketCallback, testProfileCallback);
  assert(b2 == frag2.size());
  assert(rx.getValidPackets() == 1);
  assert(g_received_packets.size() == 1);
  assert(g_received_packets[0].approach == 'E');
  assert(g_received_packets[0].q_count == 3);
  assert(rx.getBufferIndex() == 0);
  std::cout << "  [PASS] Fragmented telemetry cleanly assembled across iterations." << std::endl;
}

void testEmptyLinesAndCRLFHandling() {
  std::cout << "\n--- TelemetryReceiver Test 4: Empty Lines and CRLF Delimiters ---" << std::endl;
  resetTestFixtures();
  TelemetryReceiver rx;

  const std::string stream_data =
      "\r\n\r\n\n"
      "{\"approach\":\"W\",\"valid\":true,\"Q\":1,\"PCU\":1.0}\r\n"
      "\r\n\n";
  MockByteStream stream(stream_data);

  rx.process(stream, testPacketCallback, testProfileCallback);
  assert(rx.getValidPackets() == 1);
  assert(rx.getRejectedPackets() == 0);
  assert(g_received_packets.size() == 1);
  assert(g_received_packets[0].approach == 'W');
  std::cout << "  [PASS] Blank lines, CR, and CRLF handled without false rejections." << std::endl;
}

void testMalformedPackets() {
  std::cout << "\n--- TelemetryReceiver Test 5: Malformed and Rejected Packets ---" << std::endl;
  resetTestFixtures();
  TelemetryReceiver rx;

  const std::string bad_stream =
      "{\"approach\":\"Z\",\"valid\":true,\"Q\":1,\"PCU\":1.0}\n"
      "{bad json syntax}\n";
  MockByteStream stream(bad_stream);

  rx.process(stream, testPacketCallback, testProfileCallback);
  assert(rx.getValidPackets() == 0);
  assert(rx.getRejectedPackets() == 2);
  assert(g_received_packets.empty());
  assert(g_profile_invocations == 2);
  std::cout << "  [PASS] Malformed packets rejected and recorded in profiling without crash." << std::endl;
}

void testOversizedPacketAndRecovery() {
  std::cout << "\n--- TelemetryReceiver Test 6: Oversized Packet and Discard Recovery ---" << std::endl;
  resetTestFixtures();
  TelemetryReceiver rx;

  // Generate 700 bytes of continuous garbage without a delimiter
  std::string oversized_garbage(700, 'X');
  oversized_garbage += "\n";
  oversized_garbage += "{\"approach\":\"N\",\"valid\":true,\"Q\":0,\"PCU\":0.0}\n";

  MockByteStream stream(oversized_garbage);

  // Ingest stream in chunks of 256 bytes
  while (stream.available() > 0) {
    rx.process(stream, testPacketCallback, testProfileCallback, 256);
  }

  assert(rx.getOverflowEvents() == 1);
  assert(rx.getDiscardedPacketCount() == 1);
  assert(rx.getValidPackets() == 1);
  assert(g_received_packets.size() == 1);
  assert(g_received_packets[0].approach == 'N');
  assert(g_received_packets[0].q_count == 0);
  std::cout << "  [PASS] Oversized packet triggered overflow discard; recovered immediately on next line." << std::endl;
}

void testExactBufferBoundaries() {
  std::cout << "\n--- TelemetryReceiver Test 7: Exact Buffer Boundaries ---" << std::endl;
  resetTestFixtures();
  TelemetryReceiver rx;

  // Maximum valid capacity: exactly 511 chars + '\n' (total 512 bytes)
  std::string padding(450, ' ');
  std::string border_pkt = "{\"approach\":\"S\",\"valid\":true,\"Q\":10,\"PCU\":10.0," + padding + "\"d_tail\":5.0}\n";
  assert(border_pkt.size() < BoardConfig::SERIAL_BUF_SIZE);

  MockByteStream stream(border_pkt);
  rx.process(stream, testPacketCallback, testProfileCallback, 512);
  assert(rx.getValidPackets() == 1);
  assert(rx.getOverflowEvents() == 0);

  // Now test boundary overflow: exactly 512 non-delimiter characters triggers overflow
  rx.reset();
  resetTestFixtures();
  std::string exact_overflow(BoardConfig::SERIAL_BUF_SIZE, 'A');
  MockByteStream stream_ovf(exact_overflow);
  rx.process(stream_ovf, testPacketCallback, testProfileCallback, 512);
  assert(rx.getOverflowEvents() == 1);
  assert(rx.isDiscarding() == true);
  std::cout << "  [PASS] Exact 511-byte capacity and 512-byte overflow boundary verified." << std::endl;
}

void testByteBudgetEnforcement() {
  std::cout << "\n--- TelemetryReceiver Test 8: Per-Loop Ingestion Budget Enforcement ---" << std::endl;
  resetTestFixtures();
  TelemetryReceiver rx;

  // Provide 300 bytes of stream data
  std::string stream_data(300, ' ');
  MockByteStream stream(stream_data);

  // Default max_bytes is BoardConfig::MAX_BYTES_PER_LOOP (256)
  size_t bytes_read = rx.process(stream, testPacketCallback, testProfileCallback);
  assert(bytes_read == BoardConfig::MAX_BYTES_PER_LOOP);
  assert(stream.available() == 300 - 256);
  assert(rx.getRxBytesTotal() == 256);
  std::cout << "  [PASS] Processing budget strictly limited to 256 bytes per iteration." << std::endl;
}

int main() {
  std::cout << "=================================================================" << std::endl;
  std::cout << "  TelemetryReceiver C++ Native Verification Test Suite" << std::endl;
  std::cout << "=================================================================" << std::endl;

  testValidPacketReception();
  testMultiplePacketsInSingleStream();
  testPartialPacketsSpanningMultipleLoops();
  testEmptyLinesAndCRLFHandling();
  testMalformedPackets();
  testOversizedPacketAndRecovery();
  testExactBufferBoundaries();
  testByteBudgetEnforcement();

  std::cout << "\n=================================================================" << std::endl;
  std::cout << "  ALL TELEMETRYRECEIVER VERIFICATION TESTS PASSED (0 ERRORS)." << std::endl;
  std::cout << "=================================================================" << std::endl;
  return 0;
}
