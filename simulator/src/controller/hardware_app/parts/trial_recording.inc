// Included inside hardware_app's anonymous namespace. Keep the field list in
// one visitor so the CSV header and values cannot silently drift apart.
template <class Emit>
void visit_trial_sample(const thesis_sim::HardwareTelemetrySample& sample, Emit emit) {
    emit("time", sample.time);
    emit("position_x", sample.position_x);
    emit("position_y", sample.position_y);
    emit("yaw", sample.yaw);
    emit("speed", sample.speed);
    emit("accel", sample.accel);
    emit("yaw_rate", sample.yaw_rate);
    emit("jerk", sample.jerk);
    emit("command_r", sample.command_r);
    emit("target_speed", sample.target_speed);
    emit("target_yaw_rate", sample.target_yaw_rate);
    emit("curvature", sample.curvature);
    emit("distance_to_goal", sample.distance_to_goal);
    emit("structured_track_s", sample.structured_track_s);
    emit("structured_progress_s", sample.structured_progress_s);
    emit("min_lidar", sample.min_lidar);
    emit("front_lidar", sample.front_lidar);
    emit("planner_speed_ref", sample.planner_speed_ref);
    emit("tracker_cross_track", sample.tracker_cross_track);
    emit("tracker_heading_error_deg", sample.tracker_heading_error_deg);
    emit("planning_ms", sample.planning_ms);
    emit("tracking_ms", sample.tracking_ms);
    emit("lidar_ms", sample.lidar_ms);
    emit("estimator_ms", sample.estimator_ms);
    emit("step_ms", sample.step_ms);
    emit("visible_gates", sample.visible_gates);
    emit("lidar_samples", sample.lidar_samples);
    emit("close_lidar_samples", sample.close_lidar_samples);
    emit("front_close_lidar_samples", sample.front_close_lidar_samples);
    emit("candidate_gates", sample.candidate_gates);
    emit("chosen_gate_distance", sample.chosen_gate_distance);
    emit("passed_gates", sample.passed_gates);
    emit("accumulated_lidar_points", sample.accumulated_lidar_points);
    emit("no_motion_cycles", sample.no_motion_cycles);
    emit("chosen_gate_index", sample.chosen_gate_index);
    emit("safety_stop_active", sample.safety_stop_active);
    emit("planner_has_reference", sample.planner_has_reference);
    emit("dynamic_gap_gates", sample.dynamic_gap_gates);
    emit("pwm_left", sample.pwm_left);
    emit("pwm_right", sample.pwm_right);
    emit("controller_pwm_left", sample.controller_pwm_left);
    emit("controller_pwm_right", sample.controller_pwm_right);
    emit("controller_target_pwm_left", sample.controller_target_pwm_left);
    emit("controller_target_pwm_right", sample.controller_target_pwm_right);
    emit("controller_left_encoder_ticks", sample.controller_left_encoder_ticks);
    emit("controller_right_encoder_ticks", sample.controller_right_encoder_ticks);
    emit("controller_left_encoder_delta", sample.controller_left_encoder_delta);
    emit("controller_right_encoder_delta", sample.controller_right_encoder_delta);
    emit("controller_encoder_dt_ms", sample.controller_encoder_dt_ms);
    emit("controller_safety_flags", sample.controller_safety_flags);
    emit("controller_motor_flags", sample.controller_motor_flags);
    emit("controller_status_flags", sample.controller_status_flags);
    emit("controller_error_code", sample.controller_error_code);
    emit("external_pose_valid", sample.external_pose_valid);
    emit("external_pose_source_timestamp_s", sample.external_pose_source_timestamp_s);
    emit("external_pose_x", sample.external_pose_x);
    emit("external_pose_y", sample.external_pose_y);
    emit("external_pose_yaw", sample.external_pose_yaw);
    emit("external_pose_quality", sample.external_pose_quality);
    emit("external_pose_age_s", sample.external_pose_age_s);
    emit("external_position_error_m", sample.external_position_error_m);
    emit("external_yaw_error_deg", sample.external_yaw_error_deg);
    emit("controller_age_s", sample.controller_age_s);
    emit("lidar_age_s", sample.lidar_age_s);
    emit("lidar_scan_duration_s", sample.lidar_scan_duration_s);
    emit("lidar_scan_reused", sample.lidar_scan_reused);
    emit("mcu_clock_offset_s", sample.mcu_clock_offset_s);
    emit("mcu_clock_jitter_s", sample.mcu_clock_jitter_s);
    emit("imu_age_s", sample.imu_age_s);
    emit("encoder_age_s", sample.encoder_age_s);
    emit("slam_pose_valid", sample.slam_pose_valid);
    emit("slam_pose_x", sample.slam_pose_x);
    emit("slam_pose_y", sample.slam_pose_y);
    emit("slam_pose_yaw", sample.slam_pose_yaw);
    emit("slam_position_innovation_m", sample.slam_position_innovation_m);
    emit("slam_yaw_innovation_deg", sample.slam_yaw_innovation_deg);
    emit("slam_correction_accepted", sample.slam_correction_accepted);
    emit("stall_boost_active", sample.stall_boost_active);
    emit("left_wheel_stall_cycles", sample.left_wheel_stall_cycles);
    emit("right_wheel_stall_cycles", sample.right_wheel_stall_cycles);
    emit("encoder_slip_guard_active", sample.encoder_slip_guard_active);
    emit("exploration_map_source", sample.exploration_map_source);
    emit("exploration_state", sample.exploration_state);
    emit("control_source", sample.control_source);
    emit("selected_frontier_valid", sample.selected_frontier_valid);
    emit("selected_frontier_x", sample.selected_frontier_x);
    emit("selected_frontier_y", sample.selected_frontier_y);
    emit("global_free_cells", sample.global_free_cells);
    emit("global_occupied_cells", sample.global_occupied_cells);
    emit("start_matching_enabled", sample.start_matching_enabled);
    emit("start_matching_complete", sample.start_matching_complete);
    emit("start_matching_accepted", sample.start_matching_accepted);
    emit("start_matching_confidence", sample.start_matching_confidence);
    emit("slam_toolbox_enabled", sample.slam_toolbox_enabled);
    emit("slam_toolbox_connected", sample.slam_toolbox_connected);
    emit("slam_map_updates", sample.slam_map_updates);
    emit("slam_graph_nodes", sample.slam_graph_nodes);
    emit("slam_loop_edges", sample.slam_loop_edges);
    emit("slam_map_age_s", sample.slam_map_age_s);
    emit("planner_target_yaw_rate", sample.planner_target_yaw_rate);
    emit("yaw_rate_feedback_measurement", sample.yaw_rate_feedback_measurement);
    emit("locked_gate_reference_failure_streak", sample.locked_gate_reference_failure_streak);
    emit("gate_reference_grace_active", sample.gate_reference_grace_active);
    emit("imu_yaw_rate_from_yaw", sample.imu_yaw_rate_from_yaw);
    emit("imu_yaw_rate_consistency_error", sample.imu_yaw_rate_consistency_error);
    emit("imu_yaw_rate_fallback_active", sample.imu_yaw_rate_fallback_active);
    emit("raw_gate_candidates", sample.raw_gate_candidates);
    emit("candidate_gate_heading_error_deg", sample.candidate_gate_heading_error_deg);
    emit("candidate_gate_track_score", sample.candidate_gate_track_score);
    emit("candidate_gate_track_hits", sample.candidate_gate_track_hits);
    emit("candidate_gate_track_misses", sample.candidate_gate_track_misses);
    emit("candidate_gate_width_m", sample.candidate_gate_width_m);
    emit("candidate_gate_status", sample.candidate_gate_status);
    emit("gate_acquisition_active", sample.gate_acquisition_active);
    emit("controller_imu_yaw", sample.controller_imu_yaw);
    emit("controller_imu_yaw_rate", sample.controller_imu_yaw_rate);
    emit("controller_imu_ms", sample.controller_imu_ms);
    emit("controller_encoder_ms", sample.controller_encoder_ms);
    emit("controller_encoder_packet_dt_ms", sample.controller_encoder_packet_dt_ms);
    emit("body_yaw_rate_measurement", sample.body_yaw_rate_measurement);
    emit("body_yaw_rate_source", sample.body_yaw_rate_source);
    emit("imu_sample_new", sample.imu_sample_new);
    emit("encoder_sample_new", sample.encoder_sample_new);
    emit("encoder_sample_valid", sample.encoder_sample_valid);
    emit("controller_firmware_major", sample.controller_firmware_major);
    emit("controller_firmware_minor", sample.controller_firmware_minor);
    emit("locked_gate_heading_rad", sample.locked_gate_heading_rad);
    emit("locked_gate_crossing_x", sample.locked_gate_crossing_x);
    emit("locked_gate_crossing_y", sample.locked_gate_crossing_y);
    emit("overspeed_braking_active", sample.overspeed_braking_active);
    emit("drivetrain_stall_stop_active", sample.drivetrain_stall_stop_active);
    emit("reference_invalidation_reason", sample.reference_invalidation_reason);
}

void write_trial_csv_cell(std::ostream& out, const std::string& value) {
    out << '"';
    for (char c : value) {
        if (c == '"') out << '"';
        out << c;
    }
    out << '"';
}

template <class T>
void write_trial_csv_cell(std::ostream& out, const T& value) {
    out << value;
}

class HardwareTrialRecorder {
  public:
    explicit HardwareTrialRecorder(const AppOptions& options)
        : trial_(options.validation_trial.empty() ? "dummy" : options.validation_trial) {
        if (options.telemetry_csv_path.empty()) return;
        const std::filesystem::path path(options.telemetry_csv_path);
        if (path.has_parent_path()) std::filesystem::create_directories(path.parent_path());
        // Atomic exclusive creation protects earlier acquisitions, even when two
        // runners accidentally choose the same filename.
        const int fd = ::open(path.c_str(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0644);
        if (fd < 0) throw std::runtime_error("cannot create telemetry CSV (must be new): " + path.string());
        file_.reset(::fdopen(fd, "w"));
        if (!file_) {
            ::close(fd);
            throw std::runtime_error("cannot open telemetry CSV stream: " + path.string());
        }
        std::ostringstream header;
        header << "schema_version,trial_id,row_index,configuration_epoch,control_step,host_unix_s,host_monotonic_s"
               << ",calibration_version,calibration_hash,slam_observe_only,slam_pose_feedback";
        visit_trial_sample(thesis_sim::HardwareTelemetrySample{}, [&](const char* name, const auto&) {
            header << ',' << name;
        });
        header << '\n';
        write(header.str());
    }

    void append(int epoch, int step, const HardwarePlannerConfig& config,
                const thesis_sim::HardwareTelemetrySample& sample) {
        if (!file_ || (epoch == last_epoch_ && step == last_step_)) return;
        const double unix_s = std::chrono::duration<double>(
            std::chrono::system_clock::now().time_since_epoch()).count();
        const double monotonic_s = std::chrono::duration<double>(
            std::chrono::steady_clock::now().time_since_epoch()).count();
        std::ostringstream row;
        row.imbue(std::locale::classic());
        row << std::setprecision(17) << "1,";
        write_trial_csv_cell(row, trial_);
        row << ',' << (rows_ + 1) << ',' << epoch << ',' << step << ',' << unix_s << ',' << monotonic_s << ',';
        write_trial_csv_cell(row, config.calibration_profile_version);
        row << ',';
        write_trial_csv_cell(row, config.calibration_profile_hash);
        row << ',' << config.slam_observe_only << ',' << config.slam_pose_feedback_enabled;
        visit_trial_sample(sample, [&](const char*, const auto& value) {
            row << ',';
            write_trial_csv_cell(row, value);
        });
        row << '\n';
        write(row.str());
        ++rows_;
        last_epoch_ = epoch;
        last_step_ = step;
    }

  private:
    struct CloseFile { void operator()(std::FILE* file) const { std::fclose(file); } };
    std::unique_ptr<std::FILE, CloseFile> file_;
    std::string trial_;
    std::uint64_t rows_ = 0;
    int last_epoch_ = -1;
    int last_step_ = -1;

    void write(const std::string& data) {
        if (std::fwrite(data.data(), 1, data.size(), file_.get()) != data.size() ||
            std::fflush(file_.get()) != 0) {
            // The entrypoint's exception handler disconnects/stops the robot.
            throw std::runtime_error("telemetry CSV write failed; stopping acquisition");
        }
    }
};
