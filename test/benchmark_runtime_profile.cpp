#include <algorithm>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <string>
#include <vector>

#include "../include/TelemetryParser.hpp"
#include "../include/TrafficController.hpp"

struct TimingStats {
  double min_ns = 1e18;
  double max_ns = 0.0;
  double total_ns = 0.0;
  uint64_t count = 0;

  void record(double ns) {
    if (ns < min_ns)
      min_ns = ns;
    if (ns > max_ns)
      max_ns = ns;
    total_ns += ns;
    count++;
  }

  double avg_us() const {
    return count > 0 ? (total_ns / count) / 1000.0 : 0.0;
  }
  double min_us() const { return min_ns / 1000.0; }
  double max_us() const { return max_ns / 1000.0; }
};

int main() {
  std::cout
      << "  ATCS Subsystem Execution Time Benchmarking (High-Res Clock)   "
      << std::endl;

  // 1. Benchmark TelemetryParser::parse() on Valid Packets
  TimingStats valid_parse_stats;
  std::string valid_json = "{\"approach\": \"N\", \"Q\": 12, \"d_tail\": 45.2, "
                           "\"PCU\": 12.0, \"valid\": true}\n";
  ParsedTelemetry pkt;

  const int PARSE_ITERATIONS = 100000;
  for (int i = 0; i < PARSE_ITERATIONS; ++i) {
    auto t0 = std::chrono::high_resolution_clock::now();
    bool ok =
        TelemetryParser::parse(valid_json.c_str(), valid_json.length(), pkt);
    auto t1 = std::chrono::high_resolution_clock::now();
    (void)ok;
    double ns = std::chrono::duration<double, std::nano>(t1 - t0).count();
    valid_parse_stats.record(ns);
  }

  // 2. Benchmark TelemetryParser::parse() on Fuzzed/Malformed Packets
  TimingStats fuzzed_parse_stats;
  std::string fuzzed_json =
      "{\"approach\": \"E\", \"Q\": -5, \"d_tail\": -1.0, \"PCU\": nan}\n";
  for (int i = 0; i < PARSE_ITERATIONS; ++i) {
    auto t0 = std::chrono::high_resolution_clock::now();
    bool ok =
        TelemetryParser::parse(fuzzed_json.c_str(), fuzzed_json.length(), pkt);
    auto t1 = std::chrono::high_resolution_clock::now();
    (void)ok;
    double ns = std::chrono::duration<double, std::nano>(t1 - t0).count();
    fuzzed_parse_stats.record(ns);
  }

  // 3. Benchmark TrafficController::tick() (Normal nominal tick)
  TimingStats tick_stats;
  TrafficController ctrl;
  ctrl.setMockTimeMs(0);
  ctrl.updateApproach('N', 5, 20.0f, 5.0f, true);
  ctrl.updateApproach('S', 5, 20.0f, 5.0f, true);
  ctrl.updateApproach('E', 5, 20.0f, 5.0f, true);
  ctrl.updateApproach('W', 5, 20.0f, 5.0f, true);

  const int TICK_ITERATIONS = 100000;
  unsigned long mock_time = 0;
  for (int i = 0; i < TICK_ITERATIONS; ++i) {
    mock_time += 20; // 20ms steps
    ctrl.setMockTimeMs(mock_time);
    auto t0 = std::chrono::high_resolution_clock::now();
    ctrl.tick();
    auto t1 = std::chrono::high_resolution_clock::now();
    double ns = std::chrono::duration<double, std::nano>(t1 - t0).count();
    tick_stats.record(ns);
  }

  // 4. Benchmark TrafficController::updateApproach()
  TimingStats update_stats;
  for (int i = 0; i < PARSE_ITERATIONS; ++i) {
    auto t0 = std::chrono::high_resolution_clock::now();
    ctrl.updateApproach('N', 8, 32.5f, 8.0f, true);
    auto t1 = std::chrono::high_resolution_clock::now();
    double ns = std::chrono::duration<double, std::nano>(t1 - t0).count();
    update_stats.record(ns);
  }

  std::cout << std::fixed << std::setprecision(3);
  std::cout << "\n1. TelemetryParser::parse() [Valid JSON, " << PARSE_ITERATIONS
            << " runs]:" << std::endl;
  std::cout << "   - Average: " << valid_parse_stats.avg_us() << " us"
            << std::endl;
  std::cout << "   - Min:     " << valid_parse_stats.min_us() << " us"
            << std::endl;
  std::cout << "   - Max:     " << valid_parse_stats.max_us() << " us"
            << std::endl;

  std::cout << "\n2. TelemetryParser::parse() [Fuzzed/Rejected JSON, "
            << PARSE_ITERATIONS << " runs]:" << std::endl;
  std::cout << "   - Average: " << fuzzed_parse_stats.avg_us() << " us"
            << std::endl;
  std::cout << "   - Min:     " << fuzzed_parse_stats.min_us() << " us"
            << std::endl;
  std::cout << "   - Max:     " << fuzzed_parse_stats.max_us() << " us"
            << std::endl;

  std::cout << "\n3. TrafficController::tick() [FSM Step, " << TICK_ITERATIONS
            << " runs]:" << std::endl;
  std::cout << "   - Average: " << tick_stats.avg_us() << " us" << std::endl;
  std::cout << "   - Min:     " << tick_stats.min_us() << " us" << std::endl;
  std::cout << "   - Max:     " << tick_stats.max_us() << " us" << std::endl;

  std::cout << "\n4. TrafficController::updateApproach() [Approach Cache, "
            << PARSE_ITERATIONS << " runs]:" << std::endl;
  std::cout << "   - Average: " << update_stats.avg_us() << " us" << std::endl;
  std::cout << "   - Min:     " << update_stats.min_us() << " us" << std::endl;
  std::cout << "   - Max:     " << update_stats.max_us() << " us" << std::endl;

  std::cout << "Subsystem Latency Budget Analysis (vs 20,000 us Nominal Cycle):"
            << std::endl;
  double total_active_cycle_us = (valid_parse_stats.avg_us() * 4) +
                                 update_stats.avg_us() + tick_stats.avg_us();
  std::cout << "  Combined active processing per iteration (4 parses + tick): ~"
            << total_active_cycle_us << " us" << std::endl;
  std::cout << "  CPU Utilization in 20,000 us loop: "
            << (total_active_cycle_us / 20000.0 * 100.0) << "%" << std::endl;

  return 0;
}
