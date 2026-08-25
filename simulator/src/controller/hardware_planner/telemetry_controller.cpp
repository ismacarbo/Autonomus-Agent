#include "mvc/controller/hardware_planner/runner.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>

namespace thesis_sim {
namespace {

constexpr double kPi = 3.14159265358979323846;

double wrap_angle(double angle) {
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

}  // namespace

void HardwarePlannerRunner::push_history() {
    const RealRobotObservation& observation = bridge_.observation();
    const std::int16_t controller_pwm_left = observation.have_controller_telemetry
                                                 ? observation.controller.pwm_l
                                                 : static_cast<std::int16_t>(0);
    const std::int16_t controller_pwm_right = observation.have_controller_telemetry
                                                  ? observation.controller.pwm_r
                                                  : static_cast<std::int16_t>(0);
    const std::int16_t controller_target_pwm_left = observation.have_controller_telemetry
                                                        ? observation.controller.target_pwm_l
                                                        : static_cast<std::int16_t>(0);
    const std::int16_t controller_target_pwm_right = observation.have_controller_telemetry
                                                         ? observation.controller.target_pwm_r
                                                         : static_cast<std::int16_t>(0);
    const std::uint16_t controller_safety_flags = observation.have_controller_telemetry
                                                      ? observation.controller.safety_flags
                                                      : static_cast<std::uint16_t>(0);
    const std::uint16_t controller_motor_flags = observation.have_controller_telemetry
                                                     ? observation.controller.motor_flags
                                                     : static_cast<std::uint16_t>(0);
    const std::uint16_t controller_status_flags = observation.have_controller_telemetry
                                                      ? observation.controller.status_flags
                                                      : static_cast<std::uint16_t>(0);
    const std::uint16_t controller_error_code = observation.have_controller_telemetry
                                                    ? observation.controller.error_code
                                                    : static_cast<std::uint16_t>(0);
    const std::int32_t controller_left_encoder_ticks = observation.have_controller_telemetry
                                                           ? latest_controller_left_encoder_ticks_
                                                           : static_cast<std::int32_t>(0);
    const std::int32_t controller_right_encoder_ticks = observation.have_controller_telemetry
                                                            ? latest_controller_right_encoder_ticks_
                                                            : static_cast<std::int32_t>(0);
    const std::int32_t controller_left_encoder_delta = observation.have_controller_telemetry
                                                           ? latest_controller_left_encoder_delta_
                                                           : static_cast<std::int32_t>(0);
    const std::int32_t controller_right_encoder_delta = observation.have_controller_telemetry
                                                            ? latest_controller_right_encoder_delta_
                                                            : static_cast<std::int32_t>(0);
    const double controller_encoder_dt_ms = observation.have_controller_telemetry
                                                ? latest_controller_encoder_dt_ms_
                                                : 0.0;
    const double external_pose_age_s =
        external_pose_received_sim_time_s_ >= 0.0
            ? std::max(0.0, sim_time_ - external_pose_received_sim_time_s_)
            : -1.0;
    const bool external_pose_valid = external_pose_reference_fresh();
    const double external_position_error_m =
        external_pose_valid ? distance(estimate_.position, external_pose_reference_.position) : 0.0;
    const double external_yaw_error_deg =
        external_pose_valid
            ? std::abs(wrap_angle(estimate_.yaw - external_pose_reference_.yaw)) * 180.0 / kPi
            : 0.0;
    const double controller_age_s =
        observation.have_controller_telemetry &&
                observation.controller.rx_timestamp_s > 0.0
            ? std::max(0.0, observation.host_timestamp_s - observation.controller.rx_timestamp_s)
            : -1.0;
    const double imu_age_s =
        observation.have_controller_telemetry &&
                observation.controller.imu_host_timestamp_s > 0.0
            ? std::max(0.0, observation.host_timestamp_s - observation.controller.imu_host_timestamp_s)
            : -1.0;
    const double encoder_age_s =
        observation.have_controller_telemetry &&
                observation.controller.encoder_host_timestamp_s > 0.0
            ? std::max(0.0, observation.host_timestamp_s - observation.controller.encoder_host_timestamp_s)
            : -1.0;

    history_.push_back({
        sim_time_,
        estimate_.position.x,
        estimate_.position.y,
        estimate_.yaw,
        estimate_.speed,
        estimate_.accel,
        estimate_.yaw_rate,
        last_j_,
        last_r_,
        last_command_.target_speed,
        last_command_.target_yaw_rate,
        estimate_.curvature,
        distance_to_goal_,
        std::isfinite(x0_.x) ? x0_.x : 0.0,
        structured_progress_s_,
        estimate_.min_lidar_distance,
        estimate_.front_lidar_distance,
        planner_speed_ref_,
        tracker_cross_track_error_,
        tracker_heading_error_deg_,
        planning_compute_ms_,
        tracking_compute_ms_,
        lidar_compute_ms_,
        estimator_compute_ms_,
        step_compute_ms_,
        static_cast<double>(visible_gate_indices_.size()),
        static_cast<double>(diagnostics_.valid_lidar_points),
        static_cast<double>(diagnostics_.close_lidar_points),
        static_cast<double>(diagnostics_.front_close_lidar_points),
        static_cast<double>(diagnostics_.candidate_gates),
        std::isfinite(diagnostics_.chosen_gate_distance) ? diagnostics_.chosen_gate_distance : -1.0,
        static_cast<double>(count_passed_gates()),
        static_cast<double>(diagnostics_.accumulated_lidar_points),
        static_cast<double>(diagnostics_.no_motion_command_cycles),
        static_cast<double>(chosen_gate_index_),
        safety_stop_active_ ? 1.0 : 0.0,
        diagnostics_.planner_has_reference ? 1.0 : 0.0,
        diagnostics_.dynamic_gap_gates ? 1.0 : 0.0,
        last_command_.pwm_left,
        last_command_.pwm_right,
        controller_pwm_left,
        controller_pwm_right,
        controller_target_pwm_left,
        controller_target_pwm_right,
        controller_left_encoder_ticks,
        controller_right_encoder_ticks,
        controller_left_encoder_delta,
        controller_right_encoder_delta,
        controller_encoder_dt_ms,
        controller_safety_flags,
        controller_motor_flags,
        controller_status_flags,
        controller_error_code,
        external_pose_valid ? 1.0 : 0.0,
        external_pose_reference_.source_timestamp_s,
        external_pose_reference_.position.x,
        external_pose_reference_.position.y,
        external_pose_reference_.yaw,
        external_pose_reference_.quality,
        external_pose_age_s,
        external_position_error_m,
        external_yaw_error_deg,
        controller_age_s,
        observation.have_lidar_scan ? observation.lidar_scan_age_s : -1.0,
        observation.lidar_scan_duration_s,
        observation.lidar_scan_reused ? 1.0 : 0.0,
        observation.have_controller_telemetry
            ? observation.controller.mcu_to_host_offset_s
            : 0.0,
        observation.have_controller_telemetry
            ? observation.controller.mcu_clock_jitter_s
            : 0.0,
        imu_age_s,
        encoder_age_s,
        slam_pose_valid_ ? 1.0 : 0.0,
        last_slam_position_.x,
        last_slam_position_.y,
        last_slam_yaw_,
        last_slam_position_innovation_m_,
        last_slam_yaw_innovation_rad_ * 180.0 / kPi,
        last_slam_correction_accepted_ ? 1.0 : 0.0,
        stall_boost_active_ ? 1.0 : 0.0,
        static_cast<double>(left_wheel_stall_cycles_),
        static_cast<double>(right_wheel_stall_cycles_),
        encoder_slip_guard_active_ ? 1.0 : 0.0,
        static_cast<double>(static_cast<int>(exploration_map_source_)),
        static_cast<double>(static_cast<int>(exploration_state_)),
        static_cast<double>(static_cast<int>(control_source_)),
        selected_frontier_.has_value() ? 1.0 : 0.0,
        selected_frontier_.has_value() ? selected_frontier_->x : 0.0,
        selected_frontier_.has_value() ? selected_frontier_->y : 0.0,
        static_cast<double>(global_free_points_.size()),
        static_cast<double>(global_occupied_points_.size()),
        start_matching_status_.enabled ? 1.0 : 0.0,
        start_matching_status_.complete ? 1.0 : 0.0,
        start_matching_status_.accepted ? 1.0 : 0.0,
        start_matching_status_.confidence,
        config_.slam_toolbox_enabled ? 1.0 : 0.0,
        slam_toolbox_snapshot_.connected ? 1.0 : 0.0,
        static_cast<double>(slam_toolbox_snapshot_.map_updates),
        static_cast<double>(slam_toolbox_snapshot_.graph_nodes),
        static_cast<double>(slam_toolbox_snapshot_.loop_edges),
        diagnostics_.slam_map_age_s,
        last_command_.planner_target_yaw_rate,
        last_command_.yaw_rate_feedback_measurement,
        static_cast<double>(locked_gap_reference_failure_streak_),
        gate_reference_grace_active_ ? 1.0 : 0.0,
    });

    if (static_cast<int>(history_.size()) > config_.max_history) {
        history_.erase(history_.begin());
    }

    trail_.push_back(estimate_.position);
    if (static_cast<int>(trail_.size()) > config_.max_history) {
        trail_.erase(trail_.begin());
    }
}


HardwarePlannerReport HardwarePlannerRunner::current_report() const {
    const RealRobotObservation& observation = bridge_.observation();
    const bool lidar_enabled = lidar_enabled_for_current_mode();
    const bool controller_front_alert =
        observation.have_controller_telemetry &&
        ((observation.controller.safety_flags &
          static_cast<std::uint16_t>(SafetyFlag::FrontAlert)) != 0U);
    const bool lidar_front_blocked =
        estimate_.front_lidar_distance > 0.0 &&
        estimate_.front_lidar_distance < config_.localization.obstacle_stop_distance_m;

    const std::uint16_t controller_safety_flags = observation.have_controller_telemetry
                                                      ? observation.controller.safety_flags
                                                      : static_cast<std::uint16_t>(0);
    const std::uint16_t controller_motor_flags = observation.have_controller_telemetry
                                                     ? observation.controller.motor_flags
                                                     : static_cast<std::uint16_t>(0);
    const std::uint16_t controller_status_flags = observation.have_controller_telemetry
                                                      ? observation.controller.status_flags
                                                      : static_cast<std::uint16_t>(0);
    const std::uint16_t controller_error_code = observation.have_controller_telemetry
                                                    ? observation.controller.error_code
                                                    : static_cast<std::uint16_t>(0);
    const std::int16_t controller_pwm_left = observation.have_controller_telemetry
                                                 ? observation.controller.pwm_l
                                                 : static_cast<std::int16_t>(0);
    const std::int16_t controller_pwm_right = observation.have_controller_telemetry
                                                  ? observation.controller.pwm_r
                                                  : static_cast<std::int16_t>(0);
    const std::int16_t controller_target_pwm_left = observation.have_controller_telemetry
                                                        ? observation.controller.target_pwm_l
                                                        : static_cast<std::int16_t>(0);
    const std::int16_t controller_target_pwm_right = observation.have_controller_telemetry
                                                         ? observation.controller.target_pwm_r
                                                         : static_cast<std::int16_t>(0);

    return {
        goal_reached_,
        telemetry_ready_,
        safety_stop_active_,
        controller_front_alert,
        lidar_enabled && lidar_front_blocked,
        lidar_enabled && observation.have_lidar_scan,
        diagnostics_.dynamic_gap_gates,
        diagnostics_.planner_has_reference,
        diagnostics_.stall_boost_active,
        step_count_,
        sim_time_,
        estimate_.position,
        distance_to_goal_,
        estimate_.min_lidar_distance,
        estimate_.front_lidar_distance,
        std::isfinite(diagnostics_.chosen_gate_distance) ? diagnostics_.chosen_gate_distance : -1.0,
        diagnostics_.valid_lidar_points,
        diagnostics_.close_lidar_points,
        diagnostics_.front_close_lidar_points,
        diagnostics_.candidate_gates,
        diagnostics_.accumulated_lidar_points,
        diagnostics_.no_motion_command_cycles,
        count_passed_gates(),
        controller_safety_flags,
        controller_motor_flags,
        controller_status_flags,
        controller_error_code,
        controller_pwm_left,
        controller_pwm_right,
        controller_target_pwm_left,
        controller_target_pwm_right,
        last_command_.pwm_left,
        last_command_.pwm_right,
    };
}


int HardwarePlannerRunner::count_passed_gates() const {
    if (use_dynamic_gap_gates_ || world_.environment_mode() == EnvironmentMode::UnstructuredGates) {
        return passed_unstructured_gap_count_;
    }
    int total = 0;
    for (const gate& g : gates_) {
        if (g.passed) {
            ++total;
        }
    }
    return total;
}


}  // namespace thesis_sim
