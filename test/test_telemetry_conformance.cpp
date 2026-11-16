#include <cassert>
#include <cmath>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "../include/BoardConfig.hpp"
#include "../include/TelemetryParser.hpp"
#include "../include/TelemetryReceiver.hpp"
#include "../include/TrafficController.hpp"

class MockStream {
 public:
  explicit MockStream(const std::string& data = "") : buf_(data) {}
  void append(const std::string& data) { buf_ += data; }
  int available() const { return static_cast<int>(buf_.size() - pos_); }
  int read() { return (pos_ < buf_.size()) ? static_cast<unsigned char>(buf_[pos_++]) : -1; }
 private:
  std::string buf_;
  size_t pos_ = 0;
};

static std::vector<ParsedTelemetry> g_rx_pkts;
static void onPkt(const ParsedTelemetry& p) { g_rx_pkts.push_back(p); }

void testCanonicalSchemaV1Fixtures() {
  std::cout << "# C++ Canonical Telemetry Schema (v1) Conformance & Phase 1F Verification Suite" << std::endl;
  ParsedTelemetry pkt;

  // 1. Valid complete observation
  {
    const char* raw = "{\"v\":1,\"approach\":\"N\",\"Q\":6,\"PCU\":7.5,\"D_tail_m\":28.4,\"pcu_upstream\":2.5,\"inflow_pcu\":2.5,\"arrival_rate_pcu_s\":0.35,\"calibrated\":true,\"fresh\":true,\"valid\":true,\"seq\":42}";
    assert(TelemetryParser::parse(raw, strlen(raw), pkt) == true);
    assert(pkt.protocol_version == 1);
    assert(pkt.approach == 'N');
    assert(pkt.q_count == 6);
    assert(std::fabs(pkt.p_pcu - 7.5f) < 0.01f);
    assert(std::fabs(pkt.d_tail_m - 28.4f) < 0.01f);
    assert(std::fabs(pkt.pcu_upstream - 2.5f) < 0.01f);
    assert(std::fabs(pkt.inflow_pcu - 2.5f) < 0.01f);
    assert(std::fabs(pkt.arrival_rate_pcu_s - 0.35f) < 0.005f);
    assert(pkt.is_calibrated == true);
    assert(pkt.is_fresh == true);
    assert(pkt.is_valid == true);
    assert(pkt.seq == 42);
  }

  // 2. All 8 combinations of (valid, calibrated, fresh)
  for (int v_i = 0; v_i < 2; ++v_i) {
    for (int c_i = 0; c_i < 2; ++c_i) {
      for (int f_i = 0; f_i < 2; ++f_i) {
        bool v_flag = (v_i == 1);
        bool c_flag = (c_i == 1);
        bool f_flag = (f_i == 1);
        std::string raw = std::string("{\"v\":1,\"approach\":\"N\",\"Q\":7,\"PCU\":8.5,\"D_tail_m\":31.2,") +
                          "\"pcu_upstream\":3.0,\"inflow_pcu\":3.0,\"arrival_rate_pcu_s\":0.45," +
                          "\"calibrated\":" + (c_flag ? "true" : "false") + "," +
                          "\"fresh\":" + (f_flag ? "true" : "false") + "," +
                          "\"valid\":" + (v_flag ? "true" : "false") + ",\"seq\":10}";
        assert(TelemetryParser::parse(raw.c_str(), raw.size(), pkt) == true);
        bool eff_valid = v_flag && f_flag;
        assert(pkt.is_valid == eff_valid);
        assert(pkt.is_calibrated == c_flag);
        assert(pkt.is_fresh == f_flag);
        if (!eff_valid) {
          assert(pkt.q_count == 0);
          assert(pkt.p_pcu == 0.0f);
          assert(pkt.d_tail_m == 0.0f);
          assert(pkt.pcu_upstream == 0.0f);
          assert(pkt.inflow_pcu == 0.0f);
          assert(pkt.arrival_rate_pcu_s == 0.0f);
        } else if (!c_flag) {
          assert(pkt.q_count == 7);
          assert(std::fabs(pkt.p_pcu - 8.5f) < 0.01f);
          assert(pkt.d_tail_m == 0.0f);
          assert(pkt.pcu_upstream == 0.0f);
          assert(pkt.inflow_pcu == 0.0f);
          assert(pkt.arrival_rate_pcu_s == 0.0f);
        } else {
          assert(pkt.q_count == 7);
          assert(std::fabs(pkt.p_pcu - 8.5f) < 0.01f);
          assert(std::fabs(pkt.d_tail_m - 31.2f) < 0.01f);
          assert(std::fabs(pkt.inflow_pcu - 3.0f) < 0.01f);
        }
      }
    }
  }

  // 3. Strict JSON syntax & duplicate key rejection (including optional/unknown keys, hex floats, leading '+', trailing commas)
  const char* rejected_cases[] = {
      "{\"v\":1,\"approach\":\"N\",\"Q\":3,\"valid\":true}",
      "{\"v\":1,\"approach\":\"NE\",\"Q\":3,\"PCU\":3.0,\"valid\":true}",
      "{\"v\":1,\"approach\":\"Z\",\"Q\":3,\"PCU\":3.0,\"valid\":true}",
      "{\"v\":1,\"approach\":\"N\",\"Q\":3,\"PCU\":-1.5,\"valid\":true}",
      "{\"v\":1,\"approach\":\"N\",\"Q\":3,\"PCU\":NaN,\"valid\":true}",
      "{\"v\":1,\"approach\":\"N\",\"Q\":3,\"PCU\":3.0,",
      "{\"v\":2,\"approach\":\"N\",\"Q\":3,\"PCU\":3.0,\"valid\":true}",
      "{\"approach\":\"N\",\"Q\":3,\"PCU\":3.0,\"valid\":true,\"moderato\":true,\"moderato\":false}",
      "{\"approach\":\"N\",\"Q\":3,\"PCU\":3.0,\"valid\":true,\"extra\":1,\"extra\":2}",
      "{\"approach\":\"N\",\"Q\":3,\"PCU\":0x10,\"valid\":true}",
      "{\"approach\":\"N\",\"Q\":3,\"PCU\":+3.0,\"valid\":true}",
      "{\"approach\":\"N\",\"Q\":3,\"PCU\":3.0,\"valid\":true,}",
      "{\"approach\":\"N\",\"Q\":3,\"PCU\":3.0,\"valid\":true,\"extra\":unquoted}",
  };
  for (const char* bad : rejected_cases) {
    assert(TelemetryParser::parse(bad, strlen(bad), pkt) == false);
  }

  // 4. Fragmented 511-byte maximum packet, missing newline hold, and overflow recovery via TelemetryReceiver
  {
    g_rx_pkts.clear();
    TelemetryReceiver rx;
    std::string prefix = "{\"v\":1,\"approach\":\"N\",\"Q\":6,\"PCU\":7.5,\"valid\":true,";
    std::string suffix = "\"D_tail_m\":20.0}";
    size_t pad = TelemetryParser::kMaxTelemetryLength - prefix.size() - suffix.size();
    std::string max_pkt = prefix + std::string(pad, ' ') + suffix;
    assert(max_pkt.size() == 511);

    // Feed first 300 bytes without newline -> held in buffer, 0 packets dispatched
    MockStream s1(max_pkt.substr(0, 300));
    rx.process(s1, onPkt, nullptr, 256);
    rx.process(s1, onPkt, nullptr, 256);
    assert(g_rx_pkts.empty());
    assert(rx.getBufferIndex() == 300);

    // Feed remaining 211 bytes + '\n' -> assembled and dispatched cleanly
    MockStream s2(max_pkt.substr(300) + "\n");
    rx.process(s2, onPkt, nullptr, 256);
    assert(g_rx_pkts.size() == 1);
    assert(g_rx_pkts[0].q_count == 6);
  }

  // 5. Pre-Phase-1 vs Post-Phase-1 C++ TrafficController scheduling equivalence over 3000 ticks (300s)
  {
    TrafficController ctrl_legacy;
    TrafficController ctrl_phase1;
    ctrl_legacy.applyTimingProfile(PROFILE_URBAN_ARTERIAL);
    ctrl_phase1.applyTimingProfile(PROFILE_URBAN_ARTERIAL);
    ctrl_legacy.setModeratoEnabled(true, 10.0f);
    ctrl_phase1.setModeratoEnabled(true, 10.0f);

    for (unsigned long step = 0; step < 3000; ++step) {
      unsigned long t_ms = step * 100UL;
      ctrl_legacy.setMockTimeMs(t_ms);
      ctrl_phase1.setMockTimeMs(t_ms);

      if (step % 10 == 0) {
        int ns_q = (step < 1000) ? 12 : ((step < 1500) ? 0 : 22);
        int ew_q = (step < 1000) ? 4 : ((step < 2200) ? 15 : 25);
        float inf_n = (step >= 1500) ? 3.0f : 0.0f;
        const char legs[4] = {'N', 'S', 'E', 'W'};
        for (char leg : legs) {
          int q = (leg == 'N' || leg == 'S') ? ns_q : ew_q;
          float inf = (leg == 'N' || leg == 'S') ? inf_n : 0.0f;
          std::string p1_str = std::string("{\"v\":1,\"approach\":\"") + leg + "\",\"Q\":" + std::to_string(q) +
                               ",\"PCU\":" + std::to_string(q) + ".0,\"D_tail_m\":20.0,\"pcu_upstream\":" +
                               std::to_string((int)inf) + ".0,\"inflow_pcu\":" + std::to_string((int)inf) +
                               ".0,\"arrival_rate_pcu_s\":0.45,\"calibrated\":true,\"fresh\":true,\"valid\":true,\"seq\":1}";
          ParsedTelemetry parsed;
          assert(TelemetryParser::parse(p1_str.c_str(), p1_str.size(), parsed) == true);
          ctrl_legacy.updateApproach(leg, q, 20.0f, static_cast<float>(q), true, inf);
          ctrl_phase1.updateApproach(parsed.approach, parsed.q_count, parsed.d_tail_m, parsed.p_pcu, parsed.is_valid, parsed.inflow_pcu);
        }
      }

      ctrl_legacy.tick();
      ctrl_phase1.tick();
      assert(ctrl_legacy.getState() == ctrl_phase1.getState());
      assert(std::fabs(ctrl_legacy.getAllocatedGreen() - ctrl_phase1.getAllocatedGreen()) < 1e-4f);
      assert(std::fabs(ctrl_legacy.getNsWaitTime() - ctrl_phase1.getNsWaitTime()) < 1e-4f);
      assert(std::fabs(ctrl_legacy.getEwWaitTime() - ctrl_phase1.getEwWaitTime()) < 1e-4f);
    }
  }

  std::cout << "  [PASS] All C++ Canonical Schema v1 & Phase 1F equivalence checks passed." << std::endl;
}

int main() {
  testCanonicalSchemaV1Fixtures();
  return 0;
}
