#include "support.inc"

namespace thesis_sim {

void HardwarePlannerRunner::plan_if_needed() {
    if (step_count_ % planning_interval_steps() != 0) {
        return;
    }

    update_speed_limit();
    const double planner_r_limit =
        config_.vehicle_model == VehicleModelKind::TrackedVehicle
            ? tracked_planner_yaw_accel_limit(geometry_)
            : 1.4;
    auto commit_planner_command = [&](double next_j, double next_r) {
        last_j_ = clamp_value(next_j, -3.5, 2.5);
        last_r_ = clamp_value(next_r, -planner_r_limit, planner_r_limit);
        planner_initial_yaw_rate_ = estimate_.yaw_rate;
        last_planner_command_step_ = step_count_;
    };
    const bool mixed_gate_active = world_is_mixed(world_) && locked_gap_goal_.has_value();
    if (world_has_structured_reference(world_) && road_ != nullptr && !mixed_gate_active) {
        std::vector<double> commands = sel_jr(
            false,
            step_count_,
            true,
            road_.get(),
            false,
            nullptr,
            sim_,
            x0_,
            g_x0_,
            cl_,
            planner_traj_);

        double next_j = commands.size() > 0 ? commands[0] : 0.0;
        double next_r = commands.size() > 1 ? commands[1] : 0.0;
        if (!is_finite_pair(next_j, next_r)) {
            next_j = 0.0;
            next_r = 0.0;
        }

        visible_gate_indices_.clear();
        chosen_gate_index_ = -1;
        commit_planner_command(next_j, next_r);
        update_selected_trajectory();
        return;
    }

    if (startup_scan_active()) {
        last_j_ = 0.0;
        last_r_ = 0.0;
        refresh_gate_diagnostics();
        if (chosen_gate_index_ < 0) {
            planned_trajectory_.clear();
            reference_trajectory_.clear();
        }
        return;
    }

    const std::vector<int> active_indices = active_gate_indices();
    if (gates_.empty() || active_indices.empty()) {
        last_j_ = 0.0;
        last_r_ = 0.0;
        chosen_gate_index_ = -1;
        planned_trajectory_.clear();
        reference_trajectory_.clear();
        return;
    }

    std::vector<gate> active_gates;
    active_gates.reserve(active_indices.size());
    for (int index : active_indices) {
        active_gates.push_back(gates_[static_cast<size_t>(index)]);
    }

    std::vector<double> commands = sel_jr(
        false,
        step_count_,
        false,
        nullptr,
        true,
        &active_gates,
        sim_,
        x0_,
        g_x0_,
        cl_,
        planner_traj_);

    double next_j = commands.size() > 0 ? commands[0] : 0.0;
    double next_r = commands.size() > 1 ? commands[1] : 0.0;
    int chosen_gate = commands.size() > 2 ? static_cast<int>(commands[2]) : -1;

    if (!std::isfinite(next_j) || !std::isfinite(next_r)) {
        next_j = 0.0;
        next_r = 0.0;
        chosen_gate = -1;
    }

    for (size_t i = 0; i < active_indices.size(); ++i) {
        gates_[static_cast<size_t>(active_indices[i])] = active_gates[i];
    }

    int chosen_gate_global = -1;
    if (chosen_gate >= 0 && chosen_gate < static_cast<int>(active_indices.size())) {
        chosen_gate_global = active_indices[static_cast<size_t>(chosen_gate)];
    }

    commit_planner_command(next_j, next_r);
    refresh_gate_diagnostics();
    if (chosen_gate_global >= 0 && chosen_gate_global < static_cast<int>(gates_.size())) {
        chosen_gate_index_ = chosen_gate_global;
    }
}

}  // namespace thesis_sim
