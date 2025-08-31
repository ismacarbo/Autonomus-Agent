#include "../../include/net/UDPSocket.h"

#include <cstring>

#ifndef _WIN32
#include <signal.h>
static void intHandler(int) {}
#endif

static inline TLState castState(int x) {
  Þ switch (x) {
    case 1:
      return TLState::GREEN;
    case 2:
      return TLState::YELLOW;
    case 3:
      return TLState::RED;
    default:
      return TLState::UNKNOWN;
  }
}

static inline const char* tlStateToStr(int st) {
  switch (st) {
    case 1:
      return "green";
    case 2:
      return "yellow";
    case 3:
      return "red";
    default:
      return "unknown";
  }
}

void UDPSocket::open(const char* host, int port) {
#ifndef _WIN32
  struct sigaction act{};
  act.sa_handler = intHandler;
  sigaction(SIGINT, &act, nullptr);
#endif
  server_agent_init(host, port);
}

void UDPSocket::close() { server_agent_close(); }

bool UDPSocket::receiveScenario(ScenarioMsg& out) {
  std::memset(scenario_msg_.data_buffer, 0, sizeof(scenario_msg_.data_buffer));

  if (server_receive_from_client(&server_run_, &message_id_, &scenario_msg_.data_struct) != 0) return false;

  out.vehicleState.v = scenario_msg_.data_struct.VLgtFild;
  out.vehicleState.a = scenario_msg_.data_struct.ALgtFild;
  out.vehicleState.t = scenario_msg_.data_struct.ECUupTime;
  out.vehicleState.laneWidth = scenario_msg_.data_struct.LaneWidth;
  out.vehicleState.laneHeading = scenario_msg_.data_struct.LaneHeading;
  out.vehicleState.latOfsLineLeft = scenario_msg_.data_struct.LatOffsLineL;
  out.vehicleState.requestedCruising = scenario_msg_.data_struct.RequestedCruisingSpeed;

  out.trafficLight.currentState = castState(scenario_msg_.data_struct.TrfLightCurrState);
  out.trafficLight.next1 = castState(scenario_msg_.data_struct.TrfLightFirstNextState);
  out.trafficLight.next2 = castState(scenario_msg_.data_struct.TrfLightSecondNextState);
  out.trafficLight.t1 = scenario_msg_.data_struct.TrfLightFirstTimeToChange;
  out.trafficLight.t2 = scenario_msg_.data_struct.TrfLightSecondTimeToChange;
  out.trafficLight.t3 = scenario_msg_.data_struct.TrfLightThirdTimeToChange;
  out.trafficLight.nr = scenario_msg_.data_struct.NrTrfLights;

  out.cycleNumber = scenario_msg_.data_struct.CycleNumber;
  out.status = scenario_msg_.data_struct.Status;

  return true;
}

bool UDPSocket::sendManoeuvre(const ManoeuvreMsg& m) {
  manoeuvre_msg_.data_struct.CycleNumber = scenario_msg_.data_struct.CycleNumber;
  manoeuvre_msg_.data_struct.Status = scenario_msg_.data_struct.Status;

  manoeuvre_msg_.data_struct.RequestedAcc = m.accelCmd;

  manoeuvre_msg_.data_struct.RequestedSteerWhlAg = m.steerCmd;  // sending steer also

  return server_send_to_client(server_run_, message_id_, &manoeuvre_msg_.data_struct) != -1;
}
