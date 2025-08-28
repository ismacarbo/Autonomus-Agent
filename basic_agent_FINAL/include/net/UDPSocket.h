#pragma once
#include <cstdint>

#include "../model/Msgs.h"

extern "C" {
#include "server_lib.h"
}

class UDPSocket {
 public:
  UDPSocket() = default;

  void open(const char* host, int port);
  void close();

  bool receiveScenario(ScenarioMsg& out);

  bool sendManoeuvre(const ManoeuvreMsg& m);

 private:
  uint32_t server_run_{1};
  scenario_msg_t scenario_msg_{};
  manoeuvre_msg_t manoeuvre_msg_{};
  uint32_t message_id_{0};
};
