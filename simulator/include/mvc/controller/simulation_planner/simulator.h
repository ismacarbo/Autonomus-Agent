#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <memory>
#include <limits>
#include <optional>
#include <random>
#include <string>
#include <unordered_set>
#include <vector>

#include "action_selection.h"
#include "mvc/model/navigation/mpc_path_follower.h"
#include "mvc/model/navigation/state_estimator_ekf.h"
#include "mvc/model/navigation/vehicle_dynamics.h"
#include "mvc/model/perception/perception_model.h"
#include "mvc/model/world/world.h"

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
    double target_steer_angle = 0.0;
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
    double x = 0.0;
    double y = 0.0;
    double speed = 0.0;
    double accel = 0.0;
    double yaw = 0.0;
    double jerk = 0.0;
    double curvature = 0.0;
    double yaw_rate = 0.0;
    double command_r = 0.0;
    double steer_angle = 0.0;
    double target_steer_angle = 0.0;
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
    double nav_xy_error = 0.0;
    double nav_yaw_error_deg = 0.0;
    double planner_speed_ref = 0.0;
    double tracker_accel_cmd = 0.0;
    double tracker_steer_rate_cmd = 0.0;
    double tracker_cross_track = 0.0;
    double tracker_heading_error_deg = 0.0;
    double planning_ms = 0.0;
    double tracking_ms = 0.0;
    double lidar_ms = 0.0;
    double estimator_ms = 0.0;
    double step_ms = 0.0;
    double visible_gates = 0.0;
    double lidar_samples = 0.0;
    double chosen_gate_index = -1.0;
    double chosen_gate_distance = -1.0;
    double passed_gates = 0.0;
    double candidate_gates = 0.0;
    double dynamic_gap_gates = 0.0;
    double planner_has_reference = 0.0;
    double mixed_mode = 0.0;
    double mixed_gate_score = 0.0;
    double mixed_structured_score = 0.0;
    double mixed_gate_confidence = 0.0;
    double mixed_switches = 0.0;
    double mixed_aborts = 0.0;
};

struct SimConfig {
    double dt = 0.05;
    int control_interval_steps = 2;
    int lidar_beams = 181;
    double lidar_fov_rad = 3.14159265358979323846;
    double lidar_range = 8.0;
    int max_history = 2400;
    double cruise_speed_limit = 0.75;
    bool imu_enabled = true;
    bool lidar_enabled = true;
    bool dynamic_lidar_gates = false;
    bool ideal_conditions = false;
    bool calibrated_conditions = false;
    RangeSensorProfile range_sensor_profile = RangeSensorProfile::RplidarA1;
    GateBehaviorMode gate_behavior = GateBehaviorMode::Static;
    std::uint32_t gate_seed = 7;
    VehicleModelKind vehicle_model = VehicleModelKind::CarLikeBicycle;
    TrackingControllerMode tracking_controller = TrackingControllerMode::MpcPathFollower;
    std::uint32_t simulation_seed = 20260711U;

    // Deterministic hardware-derived perturbations used by Sim-Calibrated.
    // Values are explicit in reports so a later identification campaign can
    // replace them without changing the simulation interface.
    int command_delay_steps = 1;
    double encoder_distance_noise_std_m = 0.00035;
    double encoder_left_scale = 0.995;
    double encoder_right_scale = 1.005;
    double imu_yaw_noise_std_rad = 0.004;
    double imu_yaw_rate_noise_std_rad_s = 0.012;
    double imu_yaw_bias_walk_std_rad = 0.00008;
    double lidar_range_noise_std_m = 0.005;
    double lidar_dropout_probability = 0.006;
    double actuator_left_scale = 0.985;
    double actuator_right_scale = 1.015;
    double actuator_time_constant_scale = 1.15;
};

struct VehicleTuningOverrides {
    std::optional<double> min_effective_pwm;
    std::optional<double> speed_estimate_per_pwm;
    std::optional<double> pwm_slew_rate;
    std::optional<double> motor_time_constant;
    std::optional<double> max_linear_speed;
    std::optional<double> max_curvature;
    std::optional<double> max_steer_angle;
    std::optional<double> max_steer_rate;
    std::optional<double> max_yaw_rate;
    std::optional<double> linear_feedback_gain;
    std::optional<double> yaw_feedback_gain;
    std::optional<double> left_pwm_scale;
    std::optional<double> right_pwm_scale;
    std::optional<double> yaw_response_scale;
    std::optional<double> cruise_speed_limit;
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
    const std::vector<Vec2>& estimated_trail() const { return estimated_trail_; }
    const std::vector<Vec2>& planned_trajectory() const { return planned_trajectory_; }
    const std::vector<Vec2>& slam_points() const { return slam_points_; }
    const VehicleDynamicsModel& dynamics_model() const { return *vehicle_model_; }
    const SimConfig& config() const { return config_; }
    EnvironmentMode environment_mode() const { return world_.environment_mode(); }

    bool goal_reached() const { return goal_reached_; }
    bool collision() const { return collision_; }
    int step_count() const { return step_count_; }
    double sim_time() const { return sim_time_; }
    double last_j() const { return last_j_; }
    double last_r() const { return last_r_; }
    double distance_to_goal() const { return distance_to_goal_; }
    double min_lidar_distance() const { return min_lidar_distance_; }
    int chosen_gate_index() const { return chosen_gate_index_; }
    int passed_gate_count() const { return count_passed_gates(); }
    bool imu_enabled() const { return config_.imu_enabled; }
    bool lidar_enabled() const { return config_.lidar_enabled; }
    RangeSensorProfile range_sensor_profile() const { return config_.range_sensor_profile; }
    GateBehaviorMode gate_behavior() const { return config_.gate_behavior; }
    std::uint32_t gate_seed() const { return config_.gate_seed; }
    VehicleModelKind vehicle_model_kind() const { return config_.vehicle_model; }
    TrackingControllerMode tracking_controller_mode() const { return config_.tracking_controller; }
    const Vec2& navigation_position() const { return navigation_position_; }
    double navigation_yaw() const { return navigation_yaw_; }
    double navigation_yaw_rate() const { return navigation_yaw_rate_; }
    double navigation_curvature() const { return navigation_curvature_; }
    double navigation_speed() const { return navigation_speed_; }
    double navigation_accel() const { return navigation_accel_; }
    double navigation_xy_error() const { return navigation_xy_error_; }
    double navigation_yaw_error_deg() const { return navigation_yaw_error_deg_; }
    double planner_speed_reference() const { return planner_speed_ref_; }
    double planner_yaw_rate_reference() const { return planner_yaw_rate_ref_; }
    double tracker_cross_track_error() const { return tracker_cross_track_error_; }
    double tracker_heading_error_deg() const { return tracker_heading_error_deg_; }
    const std::optional<MpcCommand>& last_mpc_command() const { return last_mpc_command_; }
    const char* heading_source_name() const { return config_.imu_enabled ? "IMU" : "Wheel odometry"; }
    const char* localization_mode_name() const;
    int active_lidar_beams() const;
    double active_lidar_fov_rad() const;
    double active_lidar_range() const;
    void load_world(WorldMap world);
    void set_sensor_suite(bool imu_enabled, bool lidar_enabled, RangeSensorProfile profile);
    void set_lidar_range(double range_m);
    void set_ideal_conditions(bool enabled);
    void set_calibrated_conditions(bool enabled);
    void set_dynamic_lidar_gates(bool enabled);
    void set_gate_behavior(GateBehaviorMode mode, std::uint32_t seed);
    void regenerate_gate_layout(std::uint32_t seed);
    void set_vehicle_stack(VehicleModelKind model, TrackingControllerMode controller);
    void set_tuning_overrides(const VehicleTuningOverrides& overrides);
    const VehicleTuningOverrides& tuning_overrides() const { return tuning_overrides_; }

  private:
    struct DynamicLidarGateCandidate {
        Vec2 position;
        double heading = 0.0;
        double score = 0.0;
        double width = 0.0;
    };

    void rebuild_vehicle_model();
    void sync_road_from_world();
    void sync_planner_from_vehicle(bool reset_relative_state);
    void sync_gate_specs_from_world(bool reset_flags);
    void update_navigation_state(double dt);
    void update_slam_map();
    void update_planner_references(double dt);
    void update_lidar();
    void update_vehicle_snapshot();
    void update_telemetry();
    void update_selected_trajectory();
    void plan_if_needed();
    void refresh_gate_diagnostics();
    bool use_dynamic_lidar_gates() const;
    bool mixed_mode_enabled() const;
    void update_mixed_arbitration();
    void leave_mixed_gate_mode(bool aborted);
    double compute_mixed_gate_score() const;
    double compute_mixed_structured_score() const;
    double compute_mixed_road_forward_clearance(double lookahead_m) const;
    void update_dynamic_lidar_gates();
    std::vector<DynamicLidarGateCandidate> extract_dynamic_lidar_gate_candidates() const;
    void update_unstructured_gate_progress();
    void update_gate_activation_window();
    std::vector<int> active_gate_indices() const;
    double compute_min_lidar() const;
    double score_lidar_pose_candidate(const Vec2& position, double yaw) const;
    int planning_interval_steps() const;
    int lidar_update_interval_steps() const;
    int count_passed_gates() const;

    WorldMap world_;
    SimConfig config_;
    VehicleGeometry geometry_;
    VehicleTuningOverrides tuning_overrides_;
    std::unique_ptr<VehicleDynamicsModel> vehicle_model_;

    sim_info sim_{};
    states x0_{};
    global_states g_x0_{};
    clothoid_info cl_{};
    KinematicBicycleEkf estimator_{};
    ModelPredictivePathFollower mpc_follower_{};
    std::unique_ptr<road_info> road_;

    std::vector<gate> gates_;
    std::vector<LidarHit> lidar_hits_;
    std::vector<TelemetrySample> history_;
    std::vector<Vec2> trail_;
    std::vector<Vec2> estimated_trail_;
    std::vector<Vec2> planned_trajectory_;
    std::vector<Vec2> slam_points_;
    std::unordered_set<std::uint64_t> slam_voxels_;
    std::vector<ReferenceWaypoint> reference_trajectory_;
    std::vector<int> visible_gate_indices_;

    VehicleSnapshot vehicle_{};

    long_lat_traj planner_traj_{};

    int step_count_ = 0;
    double sim_time_ = 0.0;
    double last_j_ = 0.0;
    double last_r_ = 0.0;
    double distance_to_goal_ = 0.0;
    double min_lidar_distance_ = 0.0;
    Vec2 navigation_position_{};
    double navigation_yaw_ = 0.0;
    double navigation_yaw_rate_ = 0.0;
    double navigation_curvature_ = 0.0;
    double navigation_speed_ = 0.0;
    double navigation_accel_ = 0.0;
    double navigation_xy_error_ = 0.0;
    double navigation_yaw_error_deg_ = 0.0;
    double planner_speed_ref_ = 0.0;
    double planner_accel_ref_ = 0.0;
    double planner_yaw_rate_ref_ = 0.0;
    double planner_yaw_accel_ref_ = 0.0;
    double planner_initial_yaw_rate_ = 0.0;
    int last_planner_command_step_ = 0;
    double tracker_cross_track_error_ = 0.0;
    double tracker_heading_error_deg_ = 0.0;
    double planning_compute_ms_ = 0.0;
    double tracking_compute_ms_ = 0.0;
    double lidar_compute_ms_ = 0.0;
    double estimator_compute_ms_ = 0.0;
    double step_compute_ms_ = 0.0;
    double structured_goal_progress_target_ = 0.0;
    double structured_progress_s_ = 0.0;
    double structured_last_s_ = std::numeric_limits<double>::quiet_NaN();
    double structured_route_last_s_ = std::numeric_limits<double>::quiet_NaN();
    Vec2 structured_goal_position_{};
    int chosen_gate_index_ = -1;
    int dynamic_lidar_passed_gates_ = 0;
    int dynamic_lidar_candidate_count_ = 0;
    int dynamic_lidar_gate_hold_steps_ = 0;
    int dynamic_lidar_stable_steps_ = 0;
    double dynamic_lidar_active_gate_width_ = 0.0;
    double dynamic_lidar_active_gate_score_ = 0.0;
    std::vector<Vec2> dynamic_lidar_passed_gate_positions_;
    bool mixed_gate_mode_active_ = false;
    int mixed_gate_rejoin_cooldown_steps_ = 0;
    int mixed_switch_count_ = 0;
    int mixed_abort_count_ = 0;
    double mixed_gate_score_ = 0.0;
    double mixed_structured_score_ = 1.0;
    double mixed_gate_confidence_ = 0.0;
    bool goal_reached_ = false;
    bool collision_ = false;
    bool lidar_scan_fresh_ = false;
    bool structured_goal_ready_ = false;
    std::optional<MpcCommand> last_mpc_command_;
    std::mt19937 simulation_rng_{};
    std::deque<VehicleControlInput> delayed_controls_;
    double simulated_imu_bias_rad_ = 0.0;
};

}  // namespace thesis_sim
