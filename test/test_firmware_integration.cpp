#include <cassert>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "../include/BoardConfig.hpp"
#include "../include/RuntimeProfiler.hpp"
#include "../include/SignalOutputs.hpp"
#include "../include/TelemetryParser.hpp"
#include "../include/TelemetryReceiver.hpp"
#include "../include/TrafficController.hpp"

// Mock Byte Stream simulating hardware UART0 FIFO in memory
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

  void clear() {
    buffer_.clear();
    read_pos_ = 0;
  }

 private:
  std::string buffer_;
  size_t read_pos_ = 0;
};

// Test harness simulating the main.cpp orchestration sequence
struct SimulatedFirmwareContext;
static SimulatedFirmwareContext* g_ctx = nullptr;

static void onTelemetryPacket(const ParsedTelemetry& pkt);
static void onTelemetryProfile(int64_t parse_us);

struct SimulatedFirmwareContext {
  TrafficController controller;
  SignalOutputs signal_outputs;
  TelemetryReceiver telemetry_receiver;
  RuntimeProfiler profiler;
  MockByteStream serial_stream;

  unsigned long mock_time_ms = 0;
  int64_t mock_time_us = 0;
  unsigned long last_telemetry_tx_ms = 0;
  unsigned long last_queue_rx_ms = 0;
  uint32_t profile_report_count = 0;

  void setup() {
    signal_outputs.setupPins();
  }

  // Simulates one complete iteration of main.cpp loop()
  void executeLoopIteration(int64_t loop_start_us) {
    g_ctx = this;
    mock_time_us = loop_start_us;
    mock_time_ms = static_cast<unsigned long>(loop_start_us / 1000);
    controller.setMockTimeMs(mock_time_ms);

    // 0. Loop start & jitter tracking
    profiler.recordLoopStart(loop_start_us);

    // 1. Ingest Serial Telemetry (budget 256 bytes per loop)
    telemetry_receiver.process(serial_stream, onTelemetryPacket, onTelemetryProfile);

    // 2. Controller FSM Step
    int64_t t_tick_start = mock_time_us;
    controller.tick();
    profiler.recordControllerTick(mock_time_us - t_tick_start + 12); // synthetic 12 us

    // 3. Apply Physical Signal Actuators
    int64_t t_gpio_start = mock_time_us;
    signal_outputs.applyState(controller.getState(), mock_time_ms);
    profiler.recordGpioUpdate(mock_time_us - t_gpio_start + 6); // synthetic 6 us

    // 4. Periodic Diagnostics (every 500 ms)
    if (mock_time_ms - last_telemetry_tx_ms >= 500) {
      last_telemetry_tx_ms = mock_time_ms;
      int64_t t_diag_start = mock_time_us;
      // Simulated Serial.printf output
      profiler.recordDiagTx(mock_time_us - t_diag_start + 30); // synthetic 30 us
    }

    // 5. Measure CPU Execution Overhead
    profiler.recordLoopExec(150); // synthetic 150 us

    // 6. Check Profile Window
    if (profiler.stepWindow()) {
      profiler.printReport(telemetry_receiver.getCounters());
      profile_report_count++;
      // Exclude report latency from next interval (synthetic 5ms print delay)
      profiler.resetLoopReference(mock_time_us + 5000);
    }
  }
};

static void onTelemetryPacket(const ParsedTelemetry& pkt) {
  if (g_ctx) {
    g_ctx->controller.updateApproach(pkt.approach, pkt.q_count, pkt.d_tail_m,
                                     pkt.p_pcu, pkt.is_valid);
    g_ctx->last_queue_rx_ms = g_ctx->mock_time_ms;
  }
}

static void onTelemetryProfile(int64_t parse_us) {
  if (g_ctx) {
    g_ctx->profiler.recordParserExec(parse_us);
  }
}

// Scenario A: Telemetry-to-controller flow
void testScenarioA_TelemetryToController() {
  std::cout << "\n--- Integration Scenario A: Telemetry-to-Controller Flow ---" << std::endl;
  SimulatedFirmwareContext ctx;
  ctx.setup();

  const std::string pkt_str = "{\"approach\":\"N\",\"valid\":true,\"Q\":7,\"D_tail_m\":28.0,\"PCU\":6.5}\n";
  ctx.serial_stream.append(pkt_str);

  ctx.executeLoopIteration(1000000); // t = 1.0s

  assert(ctx.telemetry_receiver.getValidPackets() == 1);
  assert(ctx.telemetry_receiver.getRejectedPackets() == 0);

  ApproachState n = ctx.controller.getApproachState('N');
  assert(n.q_count == 7);
  assert(n.d_tail_m == 28.0f);
  assert(n.p_pcu == 6.5f);
  assert(n.is_valid == true);
  assert(ctx.last_queue_rx_ms == 1000);

  std::cout << "  [PASS] Valid telemetry parsed and applied to controller approach state." << std::endl;
}

// Scenario B: Partial and malformed telemetry
void testScenarioB_PartialAndMalformedTelemetry() {
  std::cout << "\n--- Integration Scenario B: Partial and Malformed Telemetry ---" << std::endl;
  SimulatedFirmwareContext ctx;
  ctx.setup();

  // 1. Partial packet fragment across iterations
  ctx.serial_stream.append("{\"approach\":\"S\",");
  ctx.executeLoopIteration(1000000);

  assert(ctx.telemetry_receiver.getValidPackets() == 0);
  assert(ctx.controller.getApproachState('S').q_count == 0); // Not dispatched yet

  ctx.serial_stream.append("\"valid\":true,\"Q\":5,\"PCU\":4.0}\n");
  ctx.executeLoopIteration(1020000);

  assert(ctx.telemetry_receiver.getValidPackets() == 1);
  assert(ctx.controller.getApproachState('S').q_count == 5);
  assert(ctx.controller.getApproachState('S').p_pcu == 4.0f);

  // 2. Malformed packet rejected without contaminating controller
  ctx.serial_stream.append("{\"approach\":\"X\",\"valid\":true,\"Q\":99,\"PCU\":99.0}\n");
  ctx.executeLoopIteration(1040000);

  assert(ctx.telemetry_receiver.getRejectedPackets() == 1);
  assert(ctx.controller.getApproachState('S').q_count == 5); // Controller state unchanged

  std::cout << "  [PASS] Fragmented telemetry assembled cleanly; malformed packets rejected." << std::endl;
}

// Scenario C: Controller-to-signal flow
void testScenarioC_ControllerToSignalFlow() {
  std::cout << "\n--- Integration Scenario C: Controller-to-Signal Flow ---" << std::endl;
  SimulatedFirmwareContext ctx;
  ctx.setup();

  // Startup all-red hold at t = 0
  ctx.executeLoopIteration(0);
  assert(ctx.controller.getState() == STATE_ALL_RED_STARTUP);
  SignalPinState pins = ctx.signal_outputs.getPinState();
  assert(pins.ns_red == true && pins.ew_red == true);
  assert(!pins.ns_green && !pins.ew_green && !pins.ns_yellow && !pins.ew_yellow);

  // Advance past 2.0s startup hold into NS Green
  ctx.executeLoopIteration(2100000); // t = 2.1s
  assert(ctx.controller.getState() == STATE_NS_GREEN);
  pins = ctx.signal_outputs.getPinState();
  assert(pins.ns_green == true && pins.ew_red == true);
  assert(!pins.ns_red && !pins.ew_green);

  // Transition through phases: NS Green -> NS Yellow -> All-Red -> EW Green
  // Advance through 10.0s min-green
  ctx.executeLoopIteration(12500000); // t = 12.5s
  // Feed EW demand to trigger transition after min green
  ctx.serial_stream.append("{\"approach\":\"E\",\"valid\":true,\"Q\":8,\"PCU\":8.0}\n");
  ctx.executeLoopIteration(12520000);
  ctx.executeLoopIteration(12600000);

  // Force yellow transition by stepping past active green
  int64_t t_sim = 12600000;
  while (ctx.controller.getState() == STATE_NS_GREEN && t_sim < 60000000) {
    t_sim += 20000;
    ctx.executeLoopIteration(t_sim);
  }
  assert(ctx.controller.getState() == STATE_NS_YELLOW);
  pins = ctx.signal_outputs.getPinState();
  assert(pins.ns_yellow == true && pins.ew_red == true);
  assert(!pins.ns_green && !pins.ew_green);

  // Step through 3.0s yellow clearance to All-Red
  t_sim += 3100000;
  ctx.executeLoopIteration(t_sim);
  assert(ctx.controller.getState() == STATE_ALL_RED_TO_EW);
  pins = ctx.signal_outputs.getPinState();
  assert(pins.ns_red == true && pins.ew_red == true);

  // Step through 1.5s all-red clearance to EW Green
  t_sim += 1600000;
  ctx.executeLoopIteration(t_sim);
  assert(ctx.controller.getState() == STATE_EW_GREEN);
  pins = ctx.signal_outputs.getPinState();
  assert(pins.ew_green == true && pins.ns_red == true);
  assert(!pins.ew_red && !pins.ns_green);

  std::cout << "  [PASS] Controller phase progression reliably drives physical signal outputs." << std::endl;
}

// Scenario D: Profiler integration
void testScenarioD_ProfilerIntegration() {
  std::cout << "\n--- Integration Scenario D: Profiler Integration ---" << std::endl;
  SimulatedFirmwareContext ctx;
  ctx.setup();

  // Simulate 10 loop ticks with nominal 20,000 us spacing
  int64_t t_loop = 1000000;
  for (int i = 0; i < 10; ++i) {
    ctx.executeLoopIteration(t_loop);
    t_loop += 20000;
  }

  assert(ctx.profiler.getLoopIntervalStats().count == 9);
  assert(ctx.profiler.getLoopIntervalStats().min_val == 20000);
  assert(ctx.profiler.getLoopIntervalStats().max_val == 20000);
  assert(ctx.profiler.getLoopJitterStats().max_val == 0);

  // Inject a single jittered tick (+3,000 us deviation)
  t_loop += 3000; // Adds 3,000 us on top of the 20,000 us already advanced
  ctx.executeLoopIteration(t_loop);
  assert(ctx.profiler.getLoopJitterStats().max_val == 3000);

  // Complete 500-step reporting window
  for (uint32_t i = ctx.profiler.getWindowIterations(); i < 500; ++i) {
    t_loop += 20000;
    ctx.executeLoopIteration(t_loop);
  }
  assert(ctx.profile_report_count == 1);
  assert(ctx.profiler.getWindowIterations() == 0); // Window reset verified

  // Verify that resetLoopReference excluded the synthetic 5ms print delay
  // In main.cpp, delay(20ms) executes after the report finishes, so the next loop
  // starts 20,000 us after the post-report reference timestamp (t_loop + 5000)
  t_loop = (t_loop + 5000) + 20000;
  ctx.executeLoopIteration(t_loop);
  assert(ctx.profiler.getLoopIntervalStats().count == 1);
  // Interval must be 20,000 us relative to reference reset, not 25,000 us
  assert(ctx.profiler.getLoopIntervalStats().max_val == 20000);

  std::cout << "  [PASS] Profiler records intervals, jitter, window steps, and reference resets correctly." << std::endl;
}

// Scenario E: Combined end-to-end execution
void testScenarioE_CombinedExecution() {
  std::cout << "\n--- Integration Scenario E: Combined End-to-End Execution ---" << std::endl;
  SimulatedFirmwareContext ctx;
  ctx.setup();

  // Run 100 consecutive 20ms ticks (2.0 seconds of intersection time)
  int64_t t_loop = 1000000;
  for (int tick = 0; tick < 100; ++tick) {
    // Stream telemetry every 10 ticks (200 ms)
    if (tick % 10 == 0) {
      char leg = (tick % 20 == 0) ? 'N' : 'E';
      std::string pkt = "{\"approach\":\"" + std::string(1, leg) + "\",\"valid\":true,\"Q\":3,\"PCU\":3.0}\n";
      ctx.serial_stream.append(pkt);
    }
    ctx.executeLoopIteration(t_loop);
    t_loop += 20000;

    // Invariant check on every tick: conflicting active corridors prohibited
    SignalPinState pins = ctx.signal_outputs.getPinState();
    bool ns_active = pins.ns_green || pins.ns_yellow;
    bool ew_active = pins.ew_green || pins.ew_yellow;
    assert(!(ns_active && ew_active));
  }

  assert(ctx.telemetry_receiver.getValidPackets() == 10);
  assert(ctx.profiler.getControllerTickStats().count == 100);
  assert(ctx.profiler.getGpioUpdateStats().count == 100);

  std::cout << "  [PASS] 100 continuous orchestration ticks completed with 0 invariant violations." << std::endl;
}

int main() {
  std::cout << "=================================================================" << std::endl;
  std::cout << "  ESP32-S3 ATCS Firmware Native Integration Test Suite" << std::endl;
  std::cout << "=================================================================" << std::endl;

  testScenarioA_TelemetryToController();
  testScenarioB_PartialAndMalformedTelemetry();
  testScenarioC_ControllerToSignalFlow();
  testScenarioD_ProfilerIntegration();
  testScenarioE_CombinedExecution();

  std::cout << "\n=================================================================" << std::endl;
  std::cout << "  ALL 5 INTEGRATION SCENARIOS PASSED WITH ZERO DEFECTS (0 ERRORS)." << std::endl;
  std::cout << "=================================================================" << std::endl;
  return 0;
}
