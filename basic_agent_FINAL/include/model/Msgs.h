#pragma once
#include <cstdint>
#include <string>

enum class TLState { GREEN, YELLOW, RED, UNKNOWN };

struct TrafficLightInfo {
  TLState currentState{TLState::UNKNOWN};
  TLState next1{TLState::UNKNOWN};
  TLState next2{TLState::UNKNOWN};
  double dist{0};
  double t1{0}, t2{0}, t3{0};
  int nr{0};

}

struct VehicleState {
  double x{0}, y{0};
  double psi{0};    // heading [rad]
  double v{0};      // long. speed [m/s]
  double a{0};      // long. accel [m/s^2]
  double delta{0};  // steering [rad] (se vuoi usarlo)
  double t{0};      // ECUupTime [s]
  double laneWidth{0};
  double laneHeading{0};
  double latOfsLineLeft{0};
  double requestedCruising{0};
};
struct ScenarioMsg {
  // real world data
  VehicleState vehicleState;
  TrafficLightInfo trafficLight;
  uint32_t cycleNumber{0};
  uint32_t status{0};
};

struct Manoeuvre {
  // final costraints to build the quintic polynomial
  double tf{0};  // time to arrive
  double sf{0};  // final space
  double vf{0};  // desired velocity
  double af{0};  // desired acceleration
  // with this conditions we can build the quintic polynomial and get jerk/accel
  double coeff[6]{0, 0, 0, 0, 0, 0};
};

struct ManoeuvreMsg {
  double accelCmd{0};  // longitudinal controller (pid)
  double steerCmd{0};  // lateral controller (simple pure pursuit?)
};