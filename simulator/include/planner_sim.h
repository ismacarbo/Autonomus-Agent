#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include "action_selection.h"
#include "sim_world.h"
#include "vehicle_dynamics.h"

namespace thesis_sim {

struct WheelPose {
    Vec2 center;
    double yaw = 0.0;
    double speed = 0.0;
    bool steering = false;
};

struct VehicleSnapshot {
    Vec2 position;
    double yaw = 0.0;
    double speed = 0.0;
    double accel = 0.0;
    double curvature = 0.0;
    double steer_angle = 0.0;
    double yaw_rate = 0.0;
    double sideslip = 0.0;
    double left_wheel_speed = 0.0;
    double right_wheel_speed = 0.0;
    double target_speed = 0.0;
    double target_yaw_rate = 0.0;
    std::int32_t left_encoder_ticks = 0;
    std::int32_t right_encoder_ticks = 0;
    std::int32_t left_encoder_delta = 0;
    std::int32_t right_encoder_delta = 0;
    int left_pwm = 0;
    int right_pwm = 0;
    double encoder_dt_ms = 0.0;
    std::string model_name;
    std::array<Vec2, 4> body_corners{};
    std::array<WheelPose, 4> wheels{};
};

struct TelemetrySample {
    double time = 0.0;
    double speed = 0.0;
    double accel = 0.0;
    double jerk = 0.0;
    double curvature = 0.0;
    double command_r = 0.0;
    double steer_angle = 0.0;
    double sideslip = 0.0;
    double left_wheel_speed = 0.0;
    double right_wheel_speed = 0.0;
    double target_speed = 0.0;
    double target_yaw_rate = 0.0;
    double left_encoder_ticks = 0.0;
    double right_encoder_ticks = 0.0;
    double left_encoder_delta = 0.0;
    double right_encoder_delta = 0.0;
    double left_pwm = 0.0;
    double right_pwm = 0.0;
    double distance_to_goal = 0.0;
    double min_lidar = 0.0;
};

struct SimConfig {
    double dt = 0.05;
    int control_interval_steps = 2;
    int lidar_beams = 181;
    double lidar_fov_rad = 3.14159265358979323846;
    double lidar_range = 8.0;
    int max_history = 2400;
    double cruise_speed_limit = 0.75;
};

struct SimulationReport {
    bool goal_reached = false;
    bool collision = false;
    int steps = 0;
    double sim_time = 0.0;
    Vec2 final_position;
    double distance_to_goal = 0.0;
    int passed_gates = 0;
};

class PlannerDrivenVehicleSim {
  public:
    PlannerDrivenVehicleSim(WorldMap world, SimConfig config = {});

    void reset();
    void step();
    SimulationReport run_headless(int max_steps);

    const WorldMap& world() const { return world_; }
    const VehicleGeometry& geometry() const { return geometry_; }
    const VehicleSnapshot& vehicle() const { return vehicle_; }
    const std::vector<LidarHit>& lidar_hits() const { return lidar_hits_; }
    const std::vector<TelemetrySample>& history() const { return history_; }
    const std::vector<gate>& gates() const { return gates_; }
    const std::vector<int>& visible_gate_indices() const { return visible_gate_indices_; }
    const std::vector<Vec2>& trail() const { return trail_; }
    const VehicleDynamicsModel& dynamics_model() const { return *vehicle_model_; }

    bool goal_reached() const { return goal_reached_; }
    bool collision() const { return collision_; }
    int step_count() const { return step_count_; }
    double sim_time() const { return sim_time_; }
    double last_j() const { return last_j_; }
    double last_r() const { return last_r_; }
    double distance_to_goal() const { return distance_to_goal_; }
    double min_lidar_distance() const { return min_lidar_distance_; }
    int chosen_gate_index() const { return chosen_gate_index_; }

  private:
    void sync_planner_from_vehicle(bool reset_relative_state);
    void update_speed_limit();
    void update_lidar();
    void update_vehicle_snapshot();
    void update_telemetry();
    void plan_if_needed();
    std::vector<int> select_gate_candidates() const;
    void sync_gate_selection(const std::vector<int>& candidate_indices, const std::vector<gate>& local_gates, int chosen_local_index);
    double compute_min_lidar() const;
    int count_passed_gates() const;

    WorldMap world_;
    SimConfig config_;
    VehicleGeometry geometry_;
    std::unique_ptr<VehicleDynamicsModel> vehicle_model_;

    sim_info sim_{};
    states x0_{};
    global_states g_x0_{};
    clothoid_info cl_{};

    std::vector<gate> gates_;
    std::vector<LidarHit> lidar_hits_;
    std::vector<TelemetrySample> history_;
    std::vector<Vec2> trail_;
    std::vector<int> visible_gate_indices_;

    VehicleSnapshot vehicle_{};

    std::ofstream null_stream_;

    int step_count_ = 0;
    double sim_time_ = 0.0;
    double last_j_ = 0.0;
    double last_r_ = 0.0;
    double distance_to_goal_ = 0.0;
    double min_lidar_distance_ = 0.0;
    int chosen_gate_index_ = -1;
    bool goal_reached_ = false;
    bool collision_ = false;
};

}  // namespace thesis_sim
