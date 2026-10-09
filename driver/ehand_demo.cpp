// g++ -std=c++17 -O2 ehand_demo.cpp -o ehand_demo
// ./ehand_demo can0 right          -> print state once
// ./ehand_demo can0 left move      -> also send a gentle position command, then print state
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>
#include <iostream>
#include <string>
#include <vector>
#include <yaml-cpp/yaml.h>

#include "ehand_can.hpp"

static void printState(const ehand::HandState& s) {
  static const char* names[] = {"thumb-H", "thumb-V", "index", "middle", "ring", "little"};
  std::printf("type=%u mask=0x%02X system: %s / fault=%s\n",
              static_cast<unsigned>(s.msg_type), s.motor_mask,
              ehand::stateName(s.sys_state), ehand::faultName(s.sys_fault));
  for (size_t i = 0; i < ehand::kNumMotors; ++i) {
    const auto& j = s.joint[i];
    std::printf("  %-8s pos=%3u (%5.1f%%) speed=%3u (%5.1f%%) state=%s fault=%s\n",
                names[i], j.position, j.position_pct(), j.speed, j.speed_pct(),
                ehand::stateName(j.state), ehand::faultName(j.fault));
  }
}

// 全てのジョイントのj.stateがStandby(1)になるまで待つ
static void waitForStandby(ehand::EHandCan& hand) {
  ehand::HandState st;
  while (true) {
    if (!hand.queryState(st, 200)) {
      std::puts("no state response (timeout)");
      break;
    }
    bool all_standby = true;
    for (const auto& j : st.joint) {
      if (j.state != static_cast<uint8_t>(ehand::JointStateCode::Standby)) {
	all_standby = false;
	break;
      }
    }
    if (all_standby) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}
// どれか一つ以上のジョイントがStandby(1)でなくなるまで待つ
static void waitForNotStandby(ehand::EHandCan& hand) {
  ehand::HandState st;
  while (true) {
    if (!hand.queryState(st, 200)) {
      std::puts("no state response (timeout)");
      break;
    }
    bool all_standby = true;
    for (const auto& j : st.joint) {
      if (j.state != static_cast<uint8_t>(ehand::JointStateCode::Standby)) {
	all_standby = false;
	break;
      }
    }
    if (!all_standby) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}


void moveSequence(ehand::EHandCan& hand,
		  const std::vector<ehand::JointCmds>& seq,
		  const std::vector<std::string>& names,
		  int delay_ms) {
  for (size_t i = 0; i < seq.size(); ++i) {
    const auto& cmd = seq[i];
    std::printf("moving to %s\n", names[i].c_str());
    hand.movePosition(cmd);
    waitForNotStandby(hand);
    waitForStandby(hand);
    std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
  }
}

int main(int argc, char* argv[]) {
  char* yamlfile = nullptr;
  const char* ifname = argc > 1 ? argv[1] : "can0";
  const bool left    = argc > 2 && std::strcmp(argv[2], "left") == 0;
  const bool move    = argc > 3 && std::strcmp(argv[3], "move") == 0;
  if (argc > 3 && ! move) {
    yamlfile = argv[3];
  }
  const bool zero    = argc > 4 && std::strcmp(argv[4], "reset") == 0;
  
  try {
    ehand::EHandCan hand(ifname, left ? ehand::kLeftHandId : ehand::kRightHandId);

    ehand::HandState st;
    if (hand.queryState(st, 200)) printState(st);
    else std::puts("no state response (timeout)");
    if (zero) {
      hand.zeroReset();
      waitForStandby(hand);
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
      if (hand.queryState(st, 200)) printState(st);
      else std::puts("no state response (timeout)");
    }
    if (move) {
      // low speed (20%), low torque (30%), all joints to 50% stroke
      ehand::JointCmds cmd;
      cmd.fill(ehand::JointCmd::fromPercent(50, 20, 80));
      for (int i = 0; i < 20; ++i) {             // resend periodically (comm-timeout safety)
        hand.movePosition(cmd);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }
      if (hand.queryState(st, 200)) printState(st);
    } else if (yamlfile) {
      std::vector<ehand::JointCmds> seq;
      std::vector<std::string> names;
      YAML::Node root = YAML::LoadFile(yamlfile);
      for (const auto& node : root) {
	ehand::JointCmds cmd;
	for (const auto& finger : node["fingers"]) {
	  int idx = finger.first.as<int>();
	  const auto& f = finger.second;
	  cmd.at(idx-1) = ehand::JointCmd::fromRaw(f["position"].as<uint8_t>(),
						   f["speed"].as<uint8_t>(),
						   f["torque"].as<uint8_t>());
	}
	names.push_back(node["name"].as<std::string>());
	seq.push_back(cmd);
      }
      moveSequence(hand, seq, names, 100);
    }
  } catch (const std::exception& e) {
    std::fprintf(stderr, "error: %s\n", e.what());
    return 1;
  }
}
