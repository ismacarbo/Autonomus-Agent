#pragma once

#include <vector>

#include "vehicle_dynamics.h"

namespace thesis_sim {

enum class TrackingControllerMode {
    PlannerCommand = 0,
    MpcPathFollower = 1,
};

const char* tracking_controller_mode_name(TrackingControllerMode mode);

struct ReferenceWaypoint {
    Vec2 position;
    double yaw = 0.0;
    double curvature = 0.0;
    double speed = 0.0;
};

struct MpcFollowerConfig {
    int horizon_steps = 12;
    double horizon_dt = 0.10;
    double preview_distance = 1.4;
    int preview_min_index = 1;
    double min_accel = -1.8;
    double max_accel = 1.3;
    double max_steer_rate = 1.8;
    double w_cross_track = 24.0;
    double w_heading = 10.0;
    double w_speed = 0.8;
    double w_steer = 0.35;
    double w_accel = 0.10;
    double w_steer_rate = 0.04;
    double w_terminal = 8.0;
};

struct MpcCommand {
    double accel_cmd = 0.0;
    double steer_rate_cmd = 0.0;
    double target_speed = 0.0;
    double target_steer_angle = 0.0;
    double cost = 0.0;
    double cross_track_error = 0.0;
    double heading_error = 0.0;
    int anchor_index = 0;
    bool valid = false;
};

class KinematicBicycleMpcFollower {
  public:
    explicit KinematicBicycleMpcFollower(MpcFollowerConfig config = {});

    const MpcFollowerConfig& config() const { return config_; }

    MpcCommand solve(const VehicleGeometry& geometry,
                     const VehicleModelState& vehicle_state,
                     const std::vector<ReferenceWaypoint>& reference,
                     double desired_speed,
                     int anchor_hint_index = 0) const;

  private:
    MpcFollowerConfig config_;
};

}  // namespace thesis_sim
