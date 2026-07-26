#include "mvc/controller/simulation_planner/simulator.h"

#include <cmath>

namespace thesis_sim {

void PlannerDrivenVehicleSim::update_telemetry() {
    const double chosen_gate_distance =
        chosen_gate_index_ >= 0 && chosen_gate_index_ < static_cast<int>(gates_.size())
            ? std::hypot(
                  gates_[static_cast<size_t>(chosen_gate_index_)].x_pos - navigation_position_.x,
                  gates_[static_cast<size_t>(chosen_gate_index_)].y_pos - navigation_position_.y)
            : -1.0;
    history_.push_back({
        sim_time_,
        vehicle_.position.x,
        vehicle_.position.y,
        vehicle_.speed,
        vehicle_.accel,
        vehicle_.yaw,
        last_j_,
        vehicle_.curvature,
        vehicle_.yaw_rate,
        last_r_,
        vehicle_.steer_angle,
        vehicle_.target_steer_angle,
        vehicle_.sideslip,
        vehicle_.left_wheel_speed,
        vehicle_.right_wheel_speed,
        vehicle_.target_speed,
        vehicle_.target_yaw_rate,
        static_cast<double>(vehicle_.left_encoder_ticks),
        static_cast<double>(vehicle_.right_encoder_ticks),
        static_cast<double>(vehicle_.left_encoder_delta),
        static_cast<double>(vehicle_.right_encoder_delta),
        static_cast<double>(vehicle_.left_pwm),
        static_cast<double>(vehicle_.right_pwm),
        distance_to_goal_,
        min_lidar_distance_,
        navigation_xy_error_,
        navigation_yaw_error_deg_,
        planner_speed_ref_,
        last_mpc_command_.has_value() ? last_mpc_command_->accel_cmd : 0.0,
        last_mpc_command_.has_value() ? last_mpc_command_->steer_rate_cmd : 0.0,
        tracker_cross_track_error_,
        tracker_heading_error_deg_,
        planning_compute_ms_,
        tracking_compute_ms_,
        lidar_compute_ms_,
        estimator_compute_ms_,
        step_compute_ms_,
        static_cast<double>(visible_gate_indices_.size()),
        static_cast<double>(lidar_hits_.size()),
        static_cast<double>(chosen_gate_index_),
        chosen_gate_distance,
        static_cast<double>(count_passed_gates()),
        static_cast<double>(dynamic_lidar_candidate_count_),
        use_dynamic_lidar_gates() ? 1.0 : 0.0,
        !reference_trajectory_.empty() ? 1.0 : 0.0,
        mixed_gate_mode_active_ ? 1.0 : 0.0,
        mixed_gate_score_,
        mixed_structured_score_,
        mixed_gate_confidence_,
        static_cast<double>(mixed_switch_count_),
        static_cast<double>(mixed_abort_count_),
    });
    if (static_cast<int>(history_.size()) > config_.max_history) {
        history_.erase(history_.begin());
    }

    trail_.push_back(vehicle_.position);
    if (static_cast<int>(trail_.size()) > config_.max_history) {
        trail_.erase(trail_.begin());
    }

    estimated_trail_.push_back(navigation_position_);
    if (static_cast<int>(estimated_trail_.size()) > config_.max_history) {
        estimated_trail_.erase(estimated_trail_.begin());
    }
}


SimulationReport PlannerDrivenVehicleSim::run_headless(int max_steps) {
    const int limit = max_steps > 0 ? max_steps : 6000;
    while (!goal_reached_ && !collision_ && step_count_ < limit) {
        step();
    }

    return {
        goal_reached_,
        collision_,
        step_count_,
        sim_time_,
        vehicle_.position,
        distance_to_goal_,
        count_passed_gates(),
    };
}

int PlannerDrivenVehicleSim::count_passed_gates() const {
    if (use_dynamic_lidar_gates()) {
        return dynamic_lidar_passed_gates_;
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
