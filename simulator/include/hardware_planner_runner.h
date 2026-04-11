#pragma once

#include <cstdint>
#include <fstream>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "action_selection.h"
#include "mpc_path_follower.h"
#include "real_robot_bridge.h"
#include "sim_world.h"
#include "state_estimator_ekf.h"
#include "vehicle_dynamics.h"

namespace thesis_sim {

struct DifferentialDriveGeometry {
    double wheelbase = 0.30;
    double cg_to_front = 0.15;
    double cg_to_rear = 0.15;
    double track_width = 0.28;
    double body_length = 0.34;
    double body_width = 0.24;
    double wheel_radius = 0.04;
    std::int32_t encoder_ticks_per_revolution = 360;
    double max_steer_angle = 0.52;
    double max_steer_rate = 1.8;
    double max_linear_speed = 0.90;
    double max_yaw_rate = 2.40;
    double max_curvature = 1.80;
    double max_accel = 1.4;
    double max_decel = 1.8;
};

struct MotorPwmMapperConfig {
    int max_pwm = 255;
    int min_effective_pwm = 45;
    double wheel_speed_to_pwm_gain = 190.0;
    double wheel_speed_to_pwm_bias = 28.0;
    double left_scale = 1.00;
    double right_scale = 1.00;
    double wheel_speed_kp = 95.0;
    double wheel_speed_ki = 30.0;
    double wheel_speed_integral_limit = 0.35;
    double linear_feedback_gain = 75.0;
    double yaw_feedback_gain = 35.0;
    int start_motion_pwm = 110;
    double stall_speed_threshold_mps = 0.025;
    double stall_target_speed_threshold_mps = 0.08;
    int stall_boost_after_cycles = 3;
};

struct LidarLocalizationConfig {
    int min_scan_points = 80;
    int scan_downsample = 6;
    double max_range_m = 3.0;
    double min_valid_range_m = 0.12;
    double lidar_x_offset = 0.0;
    double lidar_y_offset = 0.0;
    double lidar_yaw_offset = 0.0;
    double xy_search_window_m = 0.20;
    double xy_search_step_m = 0.08;
    double yaw_search_window_rad = 0.08;
    double yaw_search_step_rad = 0.03;
    double front_sector_half_angle_rad = 0.35;
    double obstacle_stop_distance_m = 0.28;
};

struct GapExtractionConfig {
    bool enabled = true;
    double free_distance_threshold_m = 0.34;
    double min_gap_width_m = 0.38;
    double min_gap_angle_rad = 0.16;
    double planning_max_range_m = 1.75;
    double target_distance_scale = 0.72;
    double min_target_distance_m = 0.28;
    double max_target_distance_m = 0.95;
    int max_candidate_gates = 5;
    double map_point_resolution_m = 0.03;
    int max_persistent_points = 6000;
    int occupancy_confirm_hits = 2;
    int occupancy_decay_steps = 36;
    double target_clearance_radius_m = 0.14;
    double path_clearance_radius_m = 0.09;
};

struct PerceptionOccupancyCell {
    Vec2 center;
    int hit_count = 0;
    int last_seen_step = -1;
};

struct HardwarePlannerConfig {
    double nominal_dt = 0.10;
    int control_interval_steps = 1;
    int max_history = 2400;
    double cruise_speed_limit = 0.22;
    double goal_tolerance_m = 0.22;
    double goal_stop_speed_mps = 0.10;
    double goal_slowdown_radius_m = 0.90;
    bool auto_set_autonomous_mode = true;
    bool auto_gyro_zero = true;
    bool use_encoder_odometry = true;
    DifferentialDriveGeometry drive{};
    MotorPwmMapperConfig pwm{};
    LidarLocalizationConfig localization{};
    GapExtractionConfig gap_extraction{};
};

struct HardwarePlannerEstimate {
    Vec2 position;
    double yaw = 0.0;
    double speed = 0.0;
    double accel = 0.0;
    double yaw_rate = 0.0;
    double curvature = 0.0;
    double min_lidar_distance = 0.0;
    double front_lidar_distance = 0.0;
    bool localized = false;
};

struct HardwareControlCommand {
    int pwm_left = 0;
    int pwm_right = 0;
    double target_speed = 0.0;
    double target_yaw_rate = 0.0;
    double target_curvature = 0.0;
    bool safety_stop = false;
};

struct HardwareTelemetrySample {
    double time = 0.0;
    double position_x = 0.0;
    double position_y = 0.0;
    double yaw = 0.0;
    double speed = 0.0;
    double accel = 0.0;
    double yaw_rate = 0.0;
    double jerk = 0.0;
    double command_r = 0.0;
    double target_speed = 0.0;
    double target_yaw_rate = 0.0;
    double curvature = 0.0;
    double distance_to_goal = 0.0;
    double structured_track_s = 0.0;
    double structured_progress_s = 0.0;
    double min_lidar = 0.0;
    double front_lidar = 0.0;
    double planner_speed_ref = 0.0;
    double tracker_cross_track = 0.0;
    double tracker_heading_error_deg = 0.0;
    double planning_ms = 0.0;
    double tracking_ms = 0.0;
    double lidar_ms = 0.0;
    double estimator_ms = 0.0;
    double step_ms = 0.0;
    double visible_gates = 0.0;
    double lidar_samples = 0.0;
    double close_lidar_samples = 0.0;
    double front_close_lidar_samples = 0.0;
    double candidate_gates = 0.0;
    double chosen_gate_distance = 0.0;
    double accumulated_lidar_points = 0.0;
    double no_motion_cycles = 0.0;
    double chosen_gate_index = -1.0;
    double safety_stop_active = 0.0;
    double planner_has_reference = 0.0;
    double dynamic_gap_gates = 0.0;
    int pwm_left = 0;
    int pwm_right = 0;
    std::int16_t controller_pwm_left = 0;
    std::int16_t controller_pwm_right = 0;
    std::int16_t controller_target_pwm_left = 0;
    std::int16_t controller_target_pwm_right = 0;
    std::uint16_t controller_safety_flags = 0;
    std::uint16_t controller_motor_flags = 0;
    std::uint16_t controller_status_flags = 0;
    std::uint16_t controller_error_code = 0;
};

struct HardwarePlannerDiagnostics {
    bool dynamic_gap_gates = false;
    bool planner_has_reference = false;
    bool lidar_front_blocked = false;
    bool stall_boost_active = false;
    int valid_lidar_points = 0;
    int close_lidar_points = 0;
    int front_close_lidar_points = 0;
    int candidate_gates = 0;
    int accumulated_lidar_points = 0;
    int no_motion_command_cycles = 0;
    double chosen_gate_distance = std::numeric_limits<double>::infinity();
};

struct HardwarePlannerReport {
    bool goal_reached = false;
    bool telemetry_ready = false;
    bool safety_stop_active = false;
    bool controller_front_alert = false;
    bool lidar_front_blocked = false;
    bool have_lidar_scan = false;
    bool dynamic_gap_gates = false;
    bool planner_has_reference = false;
    bool stall_boost_active = false;
    int steps = 0;
    double runtime_s = 0.0;
    Vec2 final_position;
    double distance_to_goal = 0.0;
    double min_lidar_distance = 0.0;
    double front_lidar_distance = 0.0;
    double chosen_gate_distance = 0.0;
    int valid_lidar_points = 0;
    int close_lidar_points = 0;
    int front_close_lidar_points = 0;
    int candidate_gates = 0;
    int accumulated_lidar_points = 0;
    int no_motion_command_cycles = 0;
    int passed_gates = 0;
    std::uint16_t controller_safety_flags = 0;
    std::uint16_t controller_motor_flags = 0;
    std::uint16_t controller_status_flags = 0;
    std::uint16_t controller_error_code = 0;
    std::int16_t controller_pwm_left = 0;
    std::int16_t controller_pwm_right = 0;
    std::int16_t controller_target_pwm_left = 0;
    std::int16_t controller_target_pwm_right = 0;
    int planned_pwm_left = 0;
    int planned_pwm_right = 0;
};

class HardwarePlannerRunner {
  public:
    HardwarePlannerRunner(WorldMap world, RealRobotBridge::Options bridge_options, HardwarePlannerConfig config = {});

    void connect();
    void disconnect();

    void apply_world(WorldMap world);
    void reset();
    void reset_pose(const Vec2& position, double heading);
    void step();
    void step_with_observation(const RealRobotObservation& observation, double dt, bool send_pwm = false);
    HardwarePlannerReport current_report() const;
    HardwarePlannerReport run(int max_steps);

    const WorldMap& world() const { return world_; }
    const HardwarePlannerConfig& config() const { return config_; }
    const VehicleGeometry& geometry() const { return geometry_; }
    const HardwarePlannerEstimate& estimate() const { return estimate_; }
    const HardwareControlCommand& last_command() const { return last_command_; }
    const std::optional<MpcCommand>& last_mpc_command() const { return last_mpc_command_; }
    const std::vector<LidarHit>& lidar_hits() const { return lidar_hits_; }
    const std::vector<Vec2>& lidar_map_points() const { return lidar_map_points_; }
    const std::vector<HardwareTelemetrySample>& history() const { return history_; }
    const std::vector<Vec2>& trail() const { return trail_; }
    const std::vector<Vec2>& planned_trajectory() const { return planned_trajectory_; }
    const std::vector<ReferenceWaypoint>& reference_trajectory() const { return reference_trajectory_; }
    const std::vector<gate>& gates() const { return gates_; }
    const std::vector<GateSpec>& gate_specs() const { return gate_specs_; }
    const std::vector<int>& visible_gate_indices() const { return visible_gate_indices_; }
    const HardwarePlannerDiagnostics& diagnostics() const { return diagnostics_; }
    RealRobotBridge& bridge() { return bridge_; }
    const RealRobotBridge& bridge() const { return bridge_; }

    bool connected() const { return connected_; }
    bool telemetry_ready() const { return telemetry_ready_; }
    bool goal_reached() const { return goal_reached_; }
    bool safety_stop_active() const { return safety_stop_active_; }
    bool lidar_enabled_for_current_mode() const;
    int step_count() const { return step_count_; }
    double sim_time() const { return sim_time_; }
    double last_j() const { return last_j_; }
    double last_r() const { return last_r_; }
    double distance_to_goal() const { return distance_to_goal_; }
    int chosen_gate_index() const { return chosen_gate_index_; }
    double planner_speed_reference() const { return planner_speed_ref_; }
    double tracker_cross_track_error() const { return tracker_cross_track_error_; }
    double tracker_heading_error_deg() const { return tracker_heading_error_deg_; }

  private:
    void initialize_planner_state();
    void initialize_gates();
    void sync_road_from_world();
    void sync_gate_specs_from_world(bool reset_flags);
    void sync_planner_from_estimate(bool reset_relative_state);
    void sync_estimate_from_ekf_state();
    void update_speed_limit();
    void update_gate_activation_window();
    std::vector<int> active_gate_indices() const;
    void refresh_gate_diagnostics();
    void plan_if_needed();

    void update_estimate_from_observation(const RealRobotObservation& observation, double dt);
    void update_estimate_from_structured_motion_fallback(const ControllerTelemetry& telemetry,
                                                         double dt,
                                                         double measured_yaw,
                                                         double measured_yaw_rate);
    double stabilize_structured_track_s(double candidate_s,
                                        double max_forward_step,
                                        bool closed_loop,
                                        double* progress_delta) const;
    void correct_pose_with_lidar(const std::vector<RPLidarA1::ScanPoint>& scan);
    double score_candidate_pose(const Vec2& position, double yaw, const std::vector<RPLidarA1::ScanPoint>& scan) const;
    double score_candidate_pose_against_perception_map(const Vec2& position,
                                                       double yaw,
                                                       const std::vector<RPLidarA1::ScanPoint>& scan) const;
    void update_lidar_hits_world(const std::vector<RPLidarA1::ScanPoint>& scan);
    void rebuild_dynamic_gap_gates(const std::vector<RPLidarA1::ScanPoint>& scan);

    void update_planner_references(double dt);
    void update_selected_trajectory();
    void compute_control_command(double dt);
    void push_history();

    double compute_min_lidar_distance(const std::vector<RPLidarA1::ScanPoint>& scan) const;
    double compute_front_lidar_distance(const std::vector<RPLidarA1::ScanPoint>& scan) const;
    Vec2 scan_point_world_hit(const RPLidarA1::ScanPoint& point, const Vec2& position, double yaw) const;
    bool scan_point_is_self_hit(const RPLidarA1::ScanPoint& point, const Vec2& position, double yaw) const;
    bool scan_point_is_self_hit(const RPLidarA1::ScanPoint& point) const;
    int wheel_speed_to_pwm(double wheel_speed_mps, double scale) const;
    int planning_interval_steps() const;
    int count_passed_gates() const;
    bool dynamic_gap_mode_enabled() const;
    bool perception_map_ready() const;
    bool unstructured_perception_only_mode() const;
    bool scan_supports_target(const Vec2& target, const std::vector<RPLidarA1::ScanPoint>& scan) const;
    bool perception_map_supports_target(const Vec2& origin, const Vec2& target) const;

    WorldMap world_;
    HardwarePlannerConfig config_;
    VehicleGeometry geometry_{};
    RealRobotBridge bridge_;

    sim_info sim_{};
    states x0_{};
    global_states g_x0_{};
    clothoid_info cl_{};
    KinematicBicycleEkf estimator_{};
    KinematicBicycleMpcFollower mpc_follower_{};
    std::unique_ptr<road_info> road_;

    std::vector<gate> gates_;
    std::vector<GateSpec> gate_specs_;
    std::vector<LidarHit> lidar_hits_;
    std::vector<Vec2> lidar_map_points_;
    std::unordered_map<std::uint64_t, PerceptionOccupancyCell> lidar_occupancy_cells_;
    std::unordered_set<std::uint64_t> lidar_map_keys_;
    std::vector<HardwareTelemetrySample> history_;
    std::vector<Vec2> trail_;
    std::vector<Vec2> planned_trajectory_;
    std::vector<ReferenceWaypoint> reference_trajectory_;
    std::vector<int> visible_gate_indices_;

    HardwarePlannerEstimate estimate_{};
    HardwareControlCommand last_command_{};
    HardwarePlannerDiagnostics diagnostics_{};
    std::optional<MpcCommand> last_mpc_command_;

    std::ofstream null_stream_;

    int step_count_ = 0;
    double sim_time_ = 0.0;
    double last_j_ = 0.0;
    double last_r_ = 0.0;
    double planner_speed_ref_ = 0.0;
    double planner_accel_ref_ = 0.0;
    double tracker_cross_track_error_ = 0.0;
    double tracker_heading_error_deg_ = 0.0;
    double planning_compute_ms_ = 0.0;
    double tracking_compute_ms_ = 0.0;
    double lidar_compute_ms_ = 0.0;
    double estimator_compute_ms_ = 0.0;
    double step_compute_ms_ = 0.0;
    double yaw_offset_ = 0.0;
    double last_raw_imu_yaw_ = 0.0;
    double last_observation_time_ = 0.0;
    double distance_to_goal_ = 0.0;
    double commanded_speed_ = 0.0;
    double commanded_steer_angle_ = 0.0;
    double measured_left_wheel_speed_ = 0.0;
    double measured_right_wheel_speed_ = 0.0;
    double wheel_speed_error_integral_left_ = 0.0;
    double wheel_speed_error_integral_right_ = 0.0;
    std::int32_t last_left_encoder_ticks_ = 0;
    std::int32_t last_right_encoder_ticks_ = 0;
    int chosen_gate_index_ = -1;
    double structured_goal_progress_target_ = 0.0;
    double structured_progress_s_ = 0.0;
    double structured_last_s_ = std::numeric_limits<double>::quiet_NaN();
    Vec2 structured_goal_position_{};
    bool structured_goal_ready_ = false;
    bool yaw_offset_initialized_ = false;
    bool have_raw_imu_yaw_ = false;
    bool encoder_ticks_initialized_ = false;
    bool measured_wheel_speeds_valid_ = false;
    int no_motion_command_cycles_ = 0;
    bool use_dynamic_gap_gates_ = false;
    bool gap_recovery_turn_active_ = false;
    bool stall_boost_active_ = false;
    bool connected_ = false;
    bool telemetry_ready_ = false;
    bool goal_reached_ = false;
    bool safety_stop_active_ = false;
};

}  // namespace thesis_sim
