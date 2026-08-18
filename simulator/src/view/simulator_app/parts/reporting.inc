std::string default_hardware_world_path(const UiState& ui_state) {
    namespace fs = std::filesystem;
    const fs::path report_dir = fs::path("reports");
    std::error_code ec;
    fs::create_directories(report_dir, ec);

    const auto now = std::chrono::system_clock::now();
    const std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm{};
#if defined(_WIN32)
    localtime_s(&local_tm, &now_c);
#else
    localtime_r(&now_c, &local_tm);
#endif
    const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;

    const EnvironmentMode environment = static_cast<EnvironmentMode>(ui_state.hardware_environment_mode);
    const std::string preset_name =
        environment == EnvironmentMode::MixedRoadGates
            ? "Mixed Hardware Road Gate"
            : (environment == EnvironmentMode::StructuredRoad
                   ? thesis_sim::structured_map_preset_name(static_cast<StructuredMapPreset>(ui_state.hardware_structured_preset))
                   : thesis_sim::unstructured_map_preset_name(static_cast<UnstructuredMapPreset>(ui_state.hardware_unstructured_preset)));

    std::ostringstream name;
    name << "thesis_hardware_world_"
         << environment_mode_slug(environment)
         << "_" << slugify(preset_name)
         << "_" << std::put_time(&local_tm, "%Y%m%d_%H%M%S")
         << "_" << std::setw(3) << std::setfill('0') << millis
         << ".thmap";
    return (report_dir / name.str()).string();
}

bool write_world_blob_file(const WorldMap& world, const std::string& file_path, std::string* error) {
    const std::vector<std::uint8_t> blob = thesis_sim::serialize_world_blob(world);
    std::ofstream out(file_path, std::ios::binary | std::ios::trunc);
    if (!out.is_open()) {
        if (error != nullptr) {
            *error = "Could not open output file.";
        }
        return false;
    }
    out.write(reinterpret_cast<const char*>(blob.data()), static_cast<std::streamsize>(blob.size()));
    if (!out.good()) {
        if (error != nullptr) {
            *error = "Could not write the world blob.";
        }
        return false;
    }
    if (error != nullptr) {
        error->clear();
    }
    return true;
}

void write_vec2_json(std::ostream& out, const Vec2& value) {
    out << "{\"x\":" << value.x << ",\"y\":" << value.y << "}";
}

void write_rect_json(std::ostream& out, const Rect& rect) {
    out << "{\"min_x\":" << rect.min_x
        << ",\"min_y\":" << rect.min_y
        << ",\"max_x\":" << rect.max_x
        << ",\"max_y\":" << rect.max_y
        << "}";
}

void write_world_json(std::ostream& out, const WorldMap& world) {
    out << "{\n";
    out << "      \"environment\": \"" << json_escape(thesis_sim::environment_mode_name(world.environment_mode())) << "\",\n";
    out << "      \"unstructured_preset\": \"" << json_escape(thesis_sim::unstructured_map_preset_name(world.unstructured_preset())) << "\",\n";
    out << "      \"structured_preset\": \"" << json_escape(thesis_sim::structured_map_preset_name(world.structured_preset())) << "\",\n";
    out << "      \"gate_behavior\": \"" << json_escape(thesis_sim::gate_behavior_mode_name(world.gate_behavior())) << "\",\n";
    out << "      \"gate_seed\": " << world.gate_seed() << ",\n";
    out << "      \"bounds\": ";
    write_rect_json(out, world.bounds());
    out << ",\n";
    out << "      \"start\": ";
    write_vec2_json(out, world.start());
    out << ",\n";
    out << "      \"goal\": ";
    write_vec2_json(out, world.goal());
    out << ",\n";
    out << "      \"start_heading_rad\": " << world.start_heading() << ",\n";
    out << "      \"obstacles\": [\n";
    for (size_t i = 0; i < world.obstacles().size(); ++i) {
        out << "        ";
        write_rect_json(out, world.obstacles()[i]);
        out << (i + 1 < world.obstacles().size() ? ",\n" : "\n");
    }
    out << "      ],\n";
    out << "      \"perception_obstacles\": [\n";
    for (size_t i = 0; i < world.perception_obstacles().size(); ++i) {
        out << "        ";
        write_rect_json(out, world.perception_obstacles()[i]);
        out << (i + 1 < world.perception_obstacles().size() ? ",\n" : "\n");
    }
    out << "      ],\n";
    out << "      \"gates\": [\n";
    for (size_t i = 0; i < world.gates().size(); ++i) {
        const GateSpec& gate = world.gates()[i];
        out << "        {\"name\":\"" << json_escape(gate.name) << "\",\"position\":";
        write_vec2_json(out, gate.position);
        out << ",\"anchor_position\":";
        write_vec2_json(out, gate.anchor_position);
        out << ",\"motion_amplitude\":";
        write_vec2_json(out, gate.motion_amplitude);
        out << ",\"motion_frequency_hz\":" << gate.motion_frequency_hz
            << ",\"motion_phase_rad\":" << gate.motion_phase_rad
            << ",\"heading_hint\":" << gate.heading_hint
            << ",\"final\":" << (gate.final ? "true" : "false") << "}";
        out << (i + 1 < world.gates().size() ? ",\n" : "\n");
    }
    out << "      ],\n";
    out << "      \"road_centerline\": [\n";
    for (size_t i = 0; i < world.road_centerline().size(); ++i) {
        out << "        ";
        write_vec2_json(out, world.road_centerline()[i]);
        out << (i + 1 < world.road_centerline().size() ? ",\n" : "\n");
    }
    out << "      ]\n";
    out << "    }";
}

void write_geometry_json(std::ostream& out, const thesis_sim::VehicleGeometry& geometry) {
    out << "{"
        << "\"wheelbase\":" << geometry.wheelbase
        << ",\"cg_to_front\":" << geometry.cg_to_front
        << ",\"cg_to_rear\":" << geometry.cg_to_rear
        << ",\"track\":" << geometry.track
        << ",\"body_length\":" << geometry.body_length
        << ",\"body_width\":" << geometry.body_width
        << ",\"wheel_length\":" << geometry.wheel_length
        << ",\"wheel_width\":" << geometry.wheel_width
        << ",\"wheel_radius\":" << geometry.wheel_radius
        << ",\"max_steer_angle\":" << geometry.max_steer_angle
        << ",\"max_steer_rate\":" << geometry.max_steer_rate
        << ",\"max_curvature\":" << geometry.max_curvature
        << ",\"max_linear_speed\":" << geometry.max_linear_speed
        << ",\"max_yaw_rate\":" << geometry.max_yaw_rate
        << ",\"max_accel\":" << geometry.max_accel
        << ",\"max_decel\":" << geometry.max_decel
        << ",\"max_pwm\":" << geometry.max_pwm
        << ",\"min_effective_pwm\":" << geometry.min_effective_pwm
        << ",\"wheel_speed_to_pwm_gain\":" << geometry.wheel_speed_to_pwm_gain
        << ",\"wheel_speed_to_pwm_bias\":" << geometry.wheel_speed_to_pwm_bias
        << ",\"speed_estimate_per_pwm\":" << geometry.speed_estimate_per_pwm
        << ",\"left_pwm_scale\":" << geometry.left_pwm_scale
        << ",\"right_pwm_scale\":" << geometry.right_pwm_scale
        << ",\"yaw_response_scale\":" << geometry.yaw_response_scale
        << ",\"linear_feedback_gain\":" << geometry.linear_feedback_gain
        << ",\"yaw_feedback_gain\":" << geometry.yaw_feedback_gain
        << ",\"pwm_slew_rate\":" << geometry.pwm_slew_rate
        << ",\"motor_time_constant\":" << geometry.motor_time_constant
        << ",\"encoder_ticks_per_revolution\":" << geometry.encoder_ticks_per_revolution
        << "}";
}

void write_live_vehicle_state_json(std::ostream& out, const LiveVehicleState& vehicle) {
    out << "{"
        << "\"position\":";
    write_vec2_json(out, vehicle.position);
    out << ",\"yaw\":" << vehicle.yaw
        << ",\"speed\":" << vehicle.speed
        << ",\"accel\":" << vehicle.accel
        << ",\"curvature\":" << vehicle.curvature
        << ",\"steer_angle\":" << vehicle.steer_angle
        << ",\"yaw_rate\":" << vehicle.yaw_rate
        << ",\"sideslip\":" << vehicle.sideslip
        << ",\"left_wheel_speed\":" << vehicle.left_wheel_speed
        << ",\"right_wheel_speed\":" << vehicle.right_wheel_speed
        << ",\"target_speed\":" << vehicle.target_speed
        << ",\"target_yaw_rate\":" << vehicle.target_yaw_rate
        << ",\"target_steer_angle\":" << vehicle.target_steer_angle
        << ",\"left_encoder_ticks\":" << vehicle.left_encoder_ticks
        << ",\"right_encoder_ticks\":" << vehicle.right_encoder_ticks
        << ",\"left_encoder_delta\":" << vehicle.left_encoder_delta
        << ",\"right_encoder_delta\":" << vehicle.right_encoder_delta
        << ",\"left_pwm\":" << vehicle.left_pwm
        << ",\"right_pwm\":" << vehicle.right_pwm
        << ",\"encoder_dt_ms\":" << vehicle.encoder_dt_ms
        << "}";
}

void write_live_gate_frame_json(std::ostream& out, const LiveGateFrame& gate) {
    out << "{"
        << "\"name\":\"" << json_escape(gate.spec.name) << "\",\"position\":";
    write_vec2_json(out, gate.spec.position);
    out << ",\"anchor_position\":";
    write_vec2_json(out, gate.spec.anchor_position);
    out << ",\"motion_amplitude\":";
    write_vec2_json(out, gate.spec.motion_amplitude);
    out << ",\"motion_frequency_hz\":" << gate.spec.motion_frequency_hz
        << ",\"motion_phase_rad\":" << gate.spec.motion_phase_rad
        << ",\"heading_hint\":" << gate.spec.heading_hint
        << ",\"final\":" << (gate.spec.final ? "true" : "false")
        << ",\"passed\":" << (gate.passed ? "true" : "false")
        << "}";
}

void write_lidar_hit_json(std::ostream& out, const LidarHit& hit) {
    out << "{"
        << "\"angle\":" << hit.angle
        << ",\"distance\":" << hit.distance
        << ",\"point\":";
    write_vec2_json(out, hit.point);
    out << ",\"hit\":" << (hit.hit ? "true" : "false")
        << "}";
}

void write_hardware_sample_json(std::ostream& out, const HardwareTelemetrySample& sample) {
    out << "{"
        << "\"time\":" << sample.time
        << ",\"position_x\":" << sample.position_x
        << ",\"position_y\":" << sample.position_y
        << ",\"yaw\":" << sample.yaw
        << ",\"speed\":" << sample.speed
        << ",\"accel\":" << sample.accel
        << ",\"yaw_rate\":" << sample.yaw_rate
        << ",\"jerk\":" << sample.jerk
        << ",\"command_r\":" << sample.command_r
        << ",\"target_speed\":" << sample.target_speed
        << ",\"target_yaw_rate\":" << sample.target_yaw_rate
        << ",\"curvature\":" << sample.curvature
        << ",\"distance_to_goal\":" << sample.distance_to_goal
        << ",\"structured_track_s\":" << sample.structured_track_s
        << ",\"structured_progress_s\":" << sample.structured_progress_s
        << ",\"min_lidar\":" << sample.min_lidar
        << ",\"front_lidar\":" << sample.front_lidar
        << ",\"planner_speed_ref\":" << sample.planner_speed_ref
        << ",\"tracker_cross_track\":" << sample.tracker_cross_track
        << ",\"tracker_heading_error_deg\":" << sample.tracker_heading_error_deg
        << ",\"planning_ms\":" << sample.planning_ms
        << ",\"tracking_ms\":" << sample.tracking_ms
        << ",\"lidar_ms\":" << sample.lidar_ms
        << ",\"estimator_ms\":" << sample.estimator_ms
        << ",\"step_ms\":" << sample.step_ms
        << ",\"visible_gates\":" << sample.visible_gates
        << ",\"lidar_samples\":" << sample.lidar_samples
        << ",\"close_lidar_samples\":" << sample.close_lidar_samples
        << ",\"front_close_lidar_samples\":" << sample.front_close_lidar_samples
        << ",\"candidate_gates\":" << sample.candidate_gates
        << ",\"chosen_gate_distance\":" << sample.chosen_gate_distance
        << ",\"passed_gates\":" << sample.passed_gates
        << ",\"accumulated_lidar_points\":" << sample.accumulated_lidar_points
        << ",\"no_motion_cycles\":" << sample.no_motion_cycles
        << ",\"chosen_gate_index\":" << sample.chosen_gate_index
        << ",\"safety_stop_active\":" << sample.safety_stop_active
        << ",\"planner_has_reference\":" << sample.planner_has_reference
        << ",\"dynamic_gap_gates\":" << sample.dynamic_gap_gates
        << ",\"pwm_left\":" << sample.pwm_left
        << ",\"pwm_right\":" << sample.pwm_right
        << ",\"controller_pwm_left\":" << sample.controller_pwm_left
        << ",\"controller_pwm_right\":" << sample.controller_pwm_right
        << ",\"controller_target_pwm_left\":" << sample.controller_target_pwm_left
        << ",\"controller_target_pwm_right\":" << sample.controller_target_pwm_right
        << ",\"controller_left_encoder_ticks\":" << sample.controller_left_encoder_ticks
        << ",\"controller_right_encoder_ticks\":" << sample.controller_right_encoder_ticks
        << ",\"controller_left_encoder_delta\":" << sample.controller_left_encoder_delta
        << ",\"controller_right_encoder_delta\":" << sample.controller_right_encoder_delta
        << ",\"controller_encoder_dt_ms\":" << sample.controller_encoder_dt_ms
        << ",\"controller_safety_flags\":" << sample.controller_safety_flags
        << ",\"controller_motor_flags\":" << sample.controller_motor_flags
        << ",\"controller_status_flags\":" << sample.controller_status_flags
        << ",\"controller_error_code\":" << sample.controller_error_code
        << ",\"external_pose_valid\":" << sample.external_pose_valid
        << ",\"external_pose_source_timestamp_s\":" << sample.external_pose_source_timestamp_s
        << ",\"external_pose_x\":" << sample.external_pose_x
        << ",\"external_pose_y\":" << sample.external_pose_y
        << ",\"external_pose_yaw\":" << sample.external_pose_yaw
        << ",\"external_pose_quality\":" << sample.external_pose_quality
        << ",\"external_pose_age_s\":" << sample.external_pose_age_s
        << ",\"external_position_error_m\":" << sample.external_position_error_m
        << ",\"external_yaw_error_deg\":" << sample.external_yaw_error_deg
        << ",\"controller_age_s\":" << sample.controller_age_s
        << ",\"lidar_age_s\":" << sample.lidar_age_s
        << ",\"lidar_scan_duration_s\":" << sample.lidar_scan_duration_s
        << ",\"lidar_scan_reused\":" << sample.lidar_scan_reused
        << ",\"mcu_clock_offset_s\":" << sample.mcu_clock_offset_s
        << ",\"mcu_clock_jitter_s\":" << sample.mcu_clock_jitter_s
        << ",\"imu_age_s\":" << sample.imu_age_s
        << ",\"encoder_age_s\":" << sample.encoder_age_s
        << ",\"slam_pose_valid\":" << sample.slam_pose_valid
        << ",\"slam_pose_x\":" << sample.slam_pose_x
        << ",\"slam_pose_y\":" << sample.slam_pose_y
        << ",\"slam_pose_yaw\":" << sample.slam_pose_yaw
        << ",\"slam_position_innovation_m\":" << sample.slam_position_innovation_m
        << ",\"slam_yaw_innovation_deg\":" << sample.slam_yaw_innovation_deg
        << ",\"slam_correction_accepted\":" << sample.slam_correction_accepted
        << ",\"stall_boost_active\":" << sample.stall_boost_active
        << ",\"left_wheel_stall_cycles\":" << sample.left_wheel_stall_cycles
        << ",\"right_wheel_stall_cycles\":" << sample.right_wheel_stall_cycles
        << ",\"encoder_slip_guard_active\":" << sample.encoder_slip_guard_active
        << ",\"exploration_map_source\":" << sample.exploration_map_source
        << ",\"exploration_state\":" << sample.exploration_state
        << ",\"control_source\":" << sample.control_source
        << ",\"selected_frontier_valid\":" << sample.selected_frontier_valid
        << ",\"selected_frontier_x\":" << sample.selected_frontier_x
        << ",\"selected_frontier_y\":" << sample.selected_frontier_y
        << ",\"global_free_cells\":" << sample.global_free_cells
        << ",\"global_occupied_cells\":" << sample.global_occupied_cells
        << ",\"start_matching_enabled\":" << sample.start_matching_enabled
        << ",\"start_matching_complete\":" << sample.start_matching_complete
        << ",\"start_matching_accepted\":" << sample.start_matching_accepted
        << ",\"start_matching_confidence\":" << sample.start_matching_confidence
        << ",\"slam_toolbox_enabled\":" << sample.slam_toolbox_enabled
        << ",\"slam_toolbox_connected\":" << sample.slam_toolbox_connected
        << ",\"slam_map_updates\":" << sample.slam_map_updates
        << ",\"slam_graph_nodes\":" << sample.slam_graph_nodes
        << ",\"slam_loop_edges\":" << sample.slam_loop_edges
        << ",\"slam_map_age_s\":" << sample.slam_map_age_s
        << "}";
}

bool write_hardware_slam_reference_png(const HardwareViewerState& hardware,
                                       const std::string& report_path,
                                       const SlamToolboxSnapshot* slam_snapshot = nullptr);

bool write_hardware_json_report(const HardwareViewerState& hardware,
                                const UiState& ui_state,
                                const LiveViewStreamServer& hardware_server,
                                const std::string& report_path,
                                const SlamToolboxSnapshot* slam_snapshot = nullptr) {
    std::ofstream out(report_path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        return false;
    }

    const std::string status =
        hardware.frame.goal_reached
            ? "goal_reached"
            : (hardware.frame.safety_stop_active ? "safety_stop"
                                                : (hardware_server.connected() || hardware.frame.connected
                                                       ? "live"
                                                       : (!hardware.history.empty() || hardware.frame.telemetry_ready || hardware.frame.step_count > 0
                                                              ? "disconnected"
                                                              : (hardware_server.listening() ? "listening" : "idle"))));

    const MetricSummary planning_summary = summarize_metric(hardware.history, &HardwareTelemetrySample::planning_ms);
    const MetricSummary tracking_summary = summarize_metric(hardware.history, &HardwareTelemetrySample::tracking_ms);
    const MetricSummary lidar_summary = summarize_metric(hardware.history, &HardwareTelemetrySample::lidar_ms);
    const MetricSummary estimator_summary = summarize_metric(hardware.history, &HardwareTelemetrySample::estimator_ms);
    const MetricSummary step_summary = summarize_metric(hardware.history, &HardwareTelemetrySample::step_ms);
    const HardwareThesisMetrics thesis_metrics = summarize_hardware_thesis_metrics(hardware);
    const HardwarePoseQuality pose_quality = estimate_hardware_pose_quality(hardware);

    out << std::fixed << std::setprecision(6);
    out << "{\n";
    out << "  \"schema\": \"thesis_hardware_report_v1\",\n";
    out << "  \"status\": \"" << json_escape(status) << "\",\n";
    out << "  \"workspace_source\": \"" << json_escape(workspace_source_label(ui_state.workspace_source)) << "\",\n";
    out << "  \"listener\": {\n";
    out << "    \"listening\": " << (hardware_server.listening() ? "true" : "false") << ",\n";
    out << "    \"connected\": " << (hardware_server.connected() ? "true" : "false") << ",\n";
    out << "    \"port\": " << hardware_server.port() << ",\n";
    out << "    \"remote_endpoint\": \"" << json_escape(hardware_server.remote_endpoint()) << "\",\n";
    out << "    \"last_error\": \"" << json_escape(hardware_server.last_error()) << "\"\n";
    out << "  },\n";
    out << "  \"scene\": {\n";
    out << "    \"available\": " << (hardware.has_scene ? "true" : "false");
    if (hardware.has_scene) {
        out << ",\n";
        out << "    \"stream_label\": \"" << json_escape(hardware.scene.stream_label) << "\",\n";
        out << "    \"stream_profile\": \"" << json_escape(hardware.scene.stream_profile) << "\",\n";
        out << "    \"imu_enabled\": " << (hardware.scene.imu_enabled ? "true" : "false") << ",\n";
        out << "    \"lidar_enabled\": " << (hardware.scene.lidar_enabled ? "true" : "false") << ",\n";
        out << "    \"localization_mode\": \"" << json_escape(hardware.scene.localization_mode) << "\",\n";
        out << "    \"heading_source\": \"" << json_escape(hardware.scene.heading_source) << "\",\n";
        out << "    \"range_sensor_name\": \"" << json_escape(hardware.scene.range_sensor_name) << "\",\n";
        out << "    \"vehicle_model_name\": \"" << json_escape(hardware.scene.vehicle_model_name) << "\",\n";
        out << "    \"tracking_controller_name\": \"" << json_escape(hardware.scene.tracking_controller_name) << "\",\n";
        out << "    \"calibration_profile_name\": \"" << json_escape(hardware.scene.calibration_profile_name) << "\",\n";
        out << "    \"calibration_profile_version\": \"" << json_escape(hardware.scene.calibration_profile_version) << "\",\n";
        out << "    \"calibration_profile_hash\": \"" << json_escape(hardware.scene.calibration_profile_hash) << "\",\n";
        out << "    \"wheel_radius_calibrated\": " << (hardware.scene.wheel_radius_calibrated ? "true" : "false") << ",\n";
        out << "    \"mcu_velocity_closed_loop\": " << (hardware.scene.mcu_velocity_closed_loop ? "true" : "false") << ",\n";
        out << "    \"active_lidar_beams\": " << hardware.scene.active_lidar_beams << ",\n";
        out << "    \"active_lidar_fov_rad\": " << hardware.scene.active_lidar_fov_rad << ",\n";
        out << "    \"active_lidar_range\": " << hardware.scene.active_lidar_range << ",\n";
        out << "    \"geometry\": ";
        write_geometry_json(out, hardware.scene.geometry);
        out << ",\n";
        out << "    \"world\": ";
        write_world_json(out, hardware.scene.world);
        out << '\n';
    } else {
        out << '\n';
    }
    out << "  },\n";
    out << "  \"frame\": {\n";
    out << "    \"sim_time\": " << hardware.frame.sim_time << ",\n";
    out << "    \"step_count\": " << hardware.frame.step_count << ",\n";
    out << "    \"connected\": " << (hardware.frame.connected ? "true" : "false") << ",\n";
    out << "    \"telemetry_ready\": " << (hardware.frame.telemetry_ready ? "true" : "false") << ",\n";
    out << "    \"goal_reached\": " << (hardware.frame.goal_reached ? "true" : "false") << ",\n";
    out << "    \"safety_stop_active\": " << (hardware.frame.safety_stop_active ? "true" : "false") << ",\n";
    out << "    \"dynamic_gap_gates\": " << (hardware.frame.dynamic_gap_gates ? "true" : "false") << ",\n";
    out << "    \"planner_has_reference\": " << (hardware.frame.planner_has_reference ? "true" : "false") << ",\n";
    out << "    \"stall_boost_active\": " << (hardware.frame.stall_boost_active ? "true" : "false") << ",\n";
    out << "    \"distance_to_goal\": " << hardware.frame.distance_to_goal << ",\n";
    out << "    \"min_lidar_distance\": " << hardware.frame.min_lidar_distance << ",\n";
    out << "    \"front_lidar_distance\": " << hardware.frame.front_lidar_distance << ",\n";
    out << "    \"chosen_gate_distance\": " << hardware.frame.chosen_gate_distance << ",\n";
    out << "    \"last_j\": " << hardware.frame.last_j << ",\n";
    out << "    \"last_r\": " << hardware.frame.last_r << ",\n";
    out << "    \"planner_speed_ref\": " << hardware.frame.planner_speed_ref << ",\n";
    out << "    \"tracker_cross_track_error\": " << hardware.frame.tracker_cross_track_error << ",\n";
    out << "    \"tracker_heading_error_deg\": " << hardware.frame.tracker_heading_error_deg << ",\n";
    out << "    \"valid_lidar_samples\": " << hardware.frame.valid_lidar_samples << ",\n";
    out << "    \"close_lidar_samples\": " << hardware.frame.close_lidar_samples << ",\n";
    out << "    \"front_close_lidar_samples\": " << hardware.frame.front_close_lidar_samples << ",\n";
    out << "    \"candidate_gates\": " << hardware.frame.candidate_gates << ",\n";
    out << "    \"accumulated_lidar_points\": " << hardware.frame.accumulated_lidar_points << ",\n";
    out << "    \"no_motion_command_cycles\": " << hardware.frame.no_motion_command_cycles << ",\n";
    out << "    \"chosen_gate_index\": " << hardware.frame.chosen_gate_index << ",\n";
    out << "    \"passed_gates\": " << hardware.frame.passed_gates << ",\n";
    out << "    \"occupancy_cell_size_m\": " << hardware.frame.occupancy_cell_size_m << ",\n";
    out << "    \"map_source\": \""
        << json_escape(thesis_sim::exploration_map_source_name(hardware.frame.map_source))
        << "\",\n";
    out << "    \"exploration_state\": \""
        << json_escape(thesis_sim::unstructured_exploration_state_name(
               hardware.frame.exploration_state))
        << "\",\n";
    out << "    \"control_source\": \""
        << json_escape(thesis_sim::hardware_control_source_name(
               hardware.frame.control_source))
        << "\",\n";
    out << "    \"selected_frontier_valid\": "
        << (hardware.frame.selected_frontier_valid ? "true" : "false") << ",\n";
    out << "    \"selected_frontier\": ";
    write_vec2_json(out, hardware.frame.selected_frontier);
    out << ",\n";
    out << "    \"start_matching_enabled\": "
        << (hardware.frame.start_matching_enabled ? "true" : "false") << ",\n";
    out << "    \"start_matching_complete\": "
        << (hardware.frame.start_matching_complete ? "true" : "false") << ",\n";
    out << "    \"start_matching_accepted\": "
        << (hardware.frame.start_matching_accepted ? "true" : "false") << ",\n";
    out << "    \"start_matching_confidence\": "
        << hardware.frame.start_matching_confidence << ",\n";
    out << "    \"start_matching_status\": \""
        << json_escape(hardware.frame.start_matching_status) << "\",\n";
    out << "    \"slam_enabled\": "
        << (hardware.frame.slam_enabled ? "true" : "false") << ",\n";
    out << "    \"slam_connected\": "
        << (hardware.frame.slam_connected ? "true" : "false") << ",\n";
    out << "    \"slam_pose_valid\": "
        << (hardware.frame.slam_pose_valid ? "true" : "false") << ",\n";
    out << "    \"slam_map_updates\": " << hardware.frame.slam_map_updates << ",\n";
    out << "    \"slam_graph_nodes\": " << hardware.frame.slam_graph_nodes << ",\n";
    out << "    \"slam_loop_edges\": " << hardware.frame.slam_loop_edges << ",\n";
    out << "    \"slam_map_age_s\": " << hardware.frame.slam_map_age_s << ",\n";
    out << "    \"slam_session_id\": \""
        << json_escape(hardware.frame.slam_session_id) << "\",\n";
    out << "    \"slam_reset_reason\": \""
        << json_escape(hardware.frame.slam_reset_reason) << "\",\n";
    out << "    \"slam_status\": \""
        << json_escape(hardware.frame.slam_status) << "\",\n";
    out << "    \"reference_invalidation_reason\": \""
        << json_escape(hardware.frame.reference_invalidation_reason) << "\",\n";
    out << "    \"vehicle\": ";
    write_live_vehicle_state_json(out, hardware.frame.vehicle);
    out << ",\n";
    out << "    \"navigation_position\": ";
    write_vec2_json(out, hardware.frame.navigation_position);
    out << ",\n";
    out << "    \"navigation_yaw\": " << hardware.frame.navigation_yaw << ",\n";
    out << "    \"navigation_yaw_rate\": " << hardware.frame.navigation_yaw_rate << ",\n";
    out << "    \"navigation_curvature\": " << hardware.frame.navigation_curvature << ",\n";
    out << "    \"navigation_speed\": " << hardware.frame.navigation_speed << ",\n";
    out << "    \"navigation_accel\": " << hardware.frame.navigation_accel << ",\n";
    out << "    \"has_last_mpc_command\": " << (hardware.frame.has_last_mpc_command ? "true" : "false") << ",\n";
    out << "    \"last_mpc_command\": ";
    if (hardware.frame.has_last_mpc_command) {
        out << "{\"accel_cmd\":" << hardware.frame.last_mpc_command.accel_cmd
            << ",\"steer_rate_cmd\":" << hardware.frame.last_mpc_command.steer_rate_cmd << "}";
    } else {
        out << "null";
    }
    out << ",\n";
    out << "    \"has_latest_sample\": " << (hardware.frame.has_latest_sample ? "true" : "false") << ",\n";
    out << "    \"latest_sample\": ";
    if (hardware.frame.has_latest_sample) {
        write_hardware_sample_json(out, hardware.frame.latest_sample);
    } else {
        out << "null";
    }
    out << ",\n";
    out << "    \"visible_gate_indices\": [";
    for (size_t i = 0; i < hardware.frame.visible_gate_indices.size(); ++i) {
        if (i > 0) {
            out << ',';
        }
        out << hardware.frame.visible_gate_indices[i];
    }
    out << "],\n";
    out << "    \"gates\": [\n";
    for (size_t i = 0; i < hardware.frame.gates.size(); ++i) {
        out << "      ";
        write_live_gate_frame_json(out, hardware.frame.gates[i]);
        out << (i + 1 < hardware.frame.gates.size() ? ",\n" : "\n");
    }
    out << "    ],\n";
    out << "    \"trail\": [\n";
    for (size_t i = 0; i < hardware.frame.trail.size(); ++i) {
        out << "      ";
        write_vec2_json(out, hardware.frame.trail[i]);
        out << (i + 1 < hardware.frame.trail.size() ? ",\n" : "\n");
    }
    out << "    ],\n";
    out << "    \"planned_trajectory\": [\n";
    for (size_t i = 0; i < hardware.frame.planned_trajectory.size(); ++i) {
        out << "      ";
        write_vec2_json(out, hardware.frame.planned_trajectory[i]);
        out << (i + 1 < hardware.frame.planned_trajectory.size() ? ",\n" : "\n");
    }
    out << "    ],\n";
    out << "    \"local_safety_occupied_points\": [\n";
    for (size_t i = 0;
         i < hardware.frame.local_safety_occupied_points.size(); ++i) {
        out << "      ";
        write_vec2_json(out, hardware.frame.local_safety_occupied_points[i]);
        out << (i + 1 < hardware.frame.local_safety_occupied_points.size()
                    ? ",\n"
                    : "\n");
    }
    out << "    ],\n";
    out << "    \"global_free_points\": [\n";
    for (size_t i = 0; i < hardware.frame.global_free_points.size(); ++i) {
        out << "      ";
        write_vec2_json(out, hardware.frame.global_free_points[i]);
        out << (i + 1 < hardware.frame.global_free_points.size() ? ",\n" : "\n");
    }
    out << "    ],\n";
    out << "    \"global_occupied_points\": [\n";
    for (size_t i = 0; i < hardware.frame.global_occupied_points.size(); ++i) {
        out << "      ";
        write_vec2_json(out, hardware.frame.global_occupied_points[i]);
        out << (i + 1 < hardware.frame.global_occupied_points.size() ? ",\n" : "\n");
    }
    out << "    ],\n";
    out << "    \"lidar_hits\": [\n";
    for (size_t i = 0; i < hardware.frame.lidar_hits.size(); ++i) {
        out << "      ";
        write_lidar_hit_json(out, hardware.frame.lidar_hits[i]);
        out << (i + 1 < hardware.frame.lidar_hits.size() ? ",\n" : "\n");
    }
    out << "    ]\n";
    out << "  },\n";
    const bool slam_toolbox_connected =
        slam_snapshot != nullptr && slam_snapshot->connected;
    const bool persistent_grid_active =
        slam_snapshot != nullptr &&
        !slam_snapshot->connected &&
        slam_snapshot->status == "persistent_local_grid" &&
        !slam_snapshot->occupied_points.empty();
    const bool occupancy_map_available =
        slam_toolbox_connected || persistent_grid_active;
    out << "  \"slam_backend\": {\n";
    out << "    \"backend\": \""
        << (slam_toolbox_connected
                ? "slam_toolbox"
                : (persistent_grid_active
                       ? "persistent_local_grid"
                       : "passive_lidar_reconstruction"))
        << "\",\n";
    out << "    \"connected\": " << (slam_toolbox_connected ? "true" : "false") << ",\n";
    out << "    \"pose_valid\": "
        << (slam_toolbox_connected && slam_snapshot->pose_valid ? "true" : "false") << ",\n";
    out << "    \"map_updates\": "
        << (slam_toolbox_connected ? slam_snapshot->map_updates : 0) << ",\n";
    out << "    \"graph_nodes\": "
        << (slam_toolbox_connected ? slam_snapshot->graph_nodes : 0) << ",\n";
    out << "    \"loop_edges\": "
        << (slam_toolbox_connected ? slam_snapshot->loop_edges : 0) << ",\n";
    out << "    \"map_resolution_m\": "
        << (occupancy_map_available ? slam_snapshot->map_resolution_m : 0.0) << ",\n";
    out << "    \"free_cells\": "
        << (occupancy_map_available ? slam_snapshot->free_points.size() : 0U) << ",\n";
    out << "    \"occupied_cells\": "
        << (occupancy_map_available ? slam_snapshot->occupied_points.size() : 0U) << ",\n";
    out << "    \"status\": \""
        << json_escape(
               slam_toolbox_connected
                   ? slam_snapshot->status
                   : (persistent_grid_active
                          ? "SLAM Toolbox unavailable; non-decaying local exploration grid"
                          : "No SLAM Toolbox response; passive endpoint accumulation only"))
        << "\"\n";
    out << "  },\n";
    out << "  \"performance\": {\n";
    out << "    \"history_samples\": " << hardware.history.size() << ",\n";
    out << "    \"lidar_reconstruction_points\": " << hardware.lidar_reconstruction.size() << ",\n";
    out << "    \"planning_ms\": {\"avg\": " << planning_summary.avg << ",\"max\": " << planning_summary.max << "},\n";
    out << "    \"tracking_ms\": {\"avg\": " << tracking_summary.avg << ",\"max\": " << tracking_summary.max << "},\n";
    out << "    \"lidar_ms\": {\"avg\": " << lidar_summary.avg << ",\"max\": " << lidar_summary.max << "},\n";
    out << "    \"estimator_ms\": {\"avg\": " << estimator_summary.avg << ",\"max\": " << estimator_summary.max << "},\n";
    out << "    \"step_ms\": {\"avg\": " << step_summary.avg << ",\"max\": " << step_summary.max << "}\n";
    out << "  },\n";
    out << "  \"thesis_metrics\": {\n";
    out << "    \"pose_source\": \"" << json_escape(pose_quality.source) << "\",\n";
    out << "    \"pose_confidence\": " << pose_quality.confidence << ",\n";
    out << "    \"pose_note\": \"" << json_escape(pose_quality.detail) << "\",\n";
    out << "    \"runtime_s\": " << thesis_metrics.runtime_s << ",\n";
    out << "    \"time_to_goal_s\": " << thesis_metrics.time_to_goal_s << ",\n";
    out << "    \"final_goal_distance_m\": " << thesis_metrics.final_goal_distance_m << ",\n";
    out << "    \"passed_gates\": " << thesis_metrics.passed_gates << ",\n";
    out << "    \"switch_events\": " << thesis_metrics.switch_events << ",\n";
    out << "    \"structured_to_gate_switches\": " << thesis_metrics.structured_to_gate_switches << ",\n";
    out << "    \"gate_to_structured_switches\": " << thesis_metrics.gate_to_structured_switches << ",\n";
    out << "    \"structured_time_s\": " << thesis_metrics.structured_time_s << ",\n";
    out << "    \"gate_time_s\": " << thesis_metrics.gate_time_s << ",\n";
    out << "    \"encoder_distance_m\": " << thesis_metrics.encoder_distance_m << ",\n";
    out << "    \"estimated_path_length_m\": " << thesis_metrics.estimated_path_length_m << ",\n";
    out << "    \"odom_pose_delta_m\": " << thesis_metrics.odom_pose_delta_m << ",\n";
    out << "    \"min_clearance_m\": " << thesis_metrics.min_clearance_m << ",\n";
    out << "    \"max_cross_track_m\": " << thesis_metrics.max_cross_track_m << ",\n";
    out << "    \"max_heading_error_deg\": " << thesis_metrics.max_heading_error_deg << ",\n";
    out << "    \"avg_speed_mps\": " << thesis_metrics.avg_speed_mps << ",\n";
    out << "    \"max_speed_mps\": " << thesis_metrics.max_speed_mps << "\n";
    out << "  },\n";
    out << "  \"history\": [\n";
    for (size_t i = 0; i < hardware.history.size(); ++i) {
        out << "    ";
        write_hardware_sample_json(out, hardware.history[i]);
        out << (i + 1 < hardware.history.size() ? ",\n" : "\n");
    }
    out << "  ]\n";
    out << "}\n";
    const bool json_ok = out.good();
    out.close();
    const std::string slam_path = append_stem_suffix(report_path, "_slam_reference", ".png");
    return json_ok && write_hardware_slam_reference_png(hardware, slam_path, slam_snapshot);
}

bool write_sim_csv_report(const PlannerDrivenVehicleSim& sim, const std::string& report_path) {
    std::ofstream out(report_path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        return false;
    }
    out << std::fixed << std::setprecision(6);
    out << "time,x,y,yaw,speed,target_speed,target_yaw_rate,jerk,command_r,distance_to_goal,min_lidar,"
           "tracker_cross_track,tracker_heading_error_deg,planning_ms,tracking_ms,lidar_ms,estimator_ms,step_ms,"
           "chosen_gate_index,chosen_gate_distance,candidate_gates,mixed_mode,mixed_gate_score,mixed_structured_score,"
           "mixed_switches,mixed_aborts\n";
    for (const TelemetrySample& sample : sim.history()) {
        out << sample.time << ','
            << sample.x << ','
            << sample.y << ','
            << sample.yaw << ','
            << sample.speed << ','
            << sample.target_speed << ','
            << sample.target_yaw_rate << ','
            << sample.jerk << ','
            << sample.command_r << ','
            << sample.distance_to_goal << ','
            << sample.min_lidar << ','
            << sample.tracker_cross_track << ','
            << sample.tracker_heading_error_deg << ','
            << sample.planning_ms << ','
            << sample.tracking_ms << ','
            << sample.lidar_ms << ','
            << sample.estimator_ms << ','
            << sample.step_ms << ','
            << sample.chosen_gate_index << ','
            << sample.chosen_gate_distance << ','
            << sample.candidate_gates << ','
            << sample.mixed_mode << ','
            << sample.mixed_gate_score << ','
            << sample.mixed_structured_score << ','
            << sample.mixed_switches << ','
            << sample.mixed_aborts << '\n';
    }
    return true;
}

bool write_hardware_csv_report(const HardwareViewerState& hardware, const std::string& report_path) {
    std::ofstream out(report_path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        return false;
    }
    out << std::fixed << std::setprecision(6);
    out << "time,x,y,yaw,speed,target_speed,target_yaw_rate,jerk,command_r,distance_to_goal,min_lidar,front_lidar,"
           "structured_track_s,structured_progress_s,passed_gates,"
           "tracker_cross_track,tracker_heading_error_deg,planning_ms,tracking_ms,lidar_ms,estimator_ms,step_ms,"
           "pwm_left,pwm_right,controller_pwm_left,controller_pwm_right,controller_target_pwm_left,controller_target_pwm_right,"
           "controller_left_encoder_ticks,controller_right_encoder_ticks,controller_left_encoder_delta,controller_right_encoder_delta,controller_encoder_dt_ms,"
           "chosen_gate_index,chosen_gate_distance,candidate_gates,safety_stop_active,planner_has_reference,dynamic_gap_gates,"
           "external_pose_valid,external_pose_source_timestamp_s,external_pose_x,external_pose_y,external_pose_yaw,"
           "external_pose_quality,external_pose_age_s,external_position_error_m,external_yaw_error_deg,"
           "controller_age_s,lidar_age_s,lidar_scan_duration_s,lidar_scan_reused,mcu_clock_offset_s,mcu_clock_jitter_s,"
           "imu_age_s,encoder_age_s,slam_pose_valid,slam_pose_x,slam_pose_y,slam_pose_yaw,"
           "slam_position_innovation_m,slam_yaw_innovation_deg,slam_correction_accepted,"
           "stall_boost_active,left_wheel_stall_cycles,right_wheel_stall_cycles,encoder_slip_guard_active,"
           "exploration_map_source,exploration_state,control_source,selected_frontier_valid,selected_frontier_x,selected_frontier_y,"
           "global_free_cells,global_occupied_cells,start_matching_enabled,start_matching_complete,start_matching_accepted,start_matching_confidence,"
           "slam_toolbox_enabled,slam_toolbox_connected,slam_map_updates,slam_graph_nodes,slam_loop_edges,slam_map_age_s\n";
    for (const HardwareTelemetrySample& sample : hardware.history) {
        out << sample.time << ','
            << sample.position_x << ','
            << sample.position_y << ','
            << sample.yaw << ','
            << sample.speed << ','
            << sample.target_speed << ','
            << sample.target_yaw_rate << ','
            << sample.jerk << ','
            << sample.command_r << ','
            << sample.distance_to_goal << ','
            << sample.min_lidar << ','
            << sample.front_lidar << ','
            << sample.structured_track_s << ','
            << sample.structured_progress_s << ','
            << sample.passed_gates << ','
            << sample.tracker_cross_track << ','
            << sample.tracker_heading_error_deg << ','
            << sample.planning_ms << ','
            << sample.tracking_ms << ','
            << sample.lidar_ms << ','
            << sample.estimator_ms << ','
            << sample.step_ms << ','
            << sample.pwm_left << ','
            << sample.pwm_right << ','
            << sample.controller_pwm_left << ','
            << sample.controller_pwm_right << ','
            << sample.controller_target_pwm_left << ','
            << sample.controller_target_pwm_right << ','
            << sample.controller_left_encoder_ticks << ','
            << sample.controller_right_encoder_ticks << ','
            << sample.controller_left_encoder_delta << ','
            << sample.controller_right_encoder_delta << ','
            << sample.controller_encoder_dt_ms << ','
            << sample.chosen_gate_index << ','
            << sample.chosen_gate_distance << ','
            << sample.candidate_gates << ','
            << sample.safety_stop_active << ','
            << sample.planner_has_reference << ','
            << sample.dynamic_gap_gates << ','
            << sample.external_pose_valid << ','
            << sample.external_pose_source_timestamp_s << ','
            << sample.external_pose_x << ','
            << sample.external_pose_y << ','
            << sample.external_pose_yaw << ','
            << sample.external_pose_quality << ','
            << sample.external_pose_age_s << ','
            << sample.external_position_error_m << ','
            << sample.external_yaw_error_deg << ','
            << sample.controller_age_s << ','
            << sample.lidar_age_s << ','
            << sample.lidar_scan_duration_s << ','
            << sample.lidar_scan_reused << ','
            << sample.mcu_clock_offset_s << ','
            << sample.mcu_clock_jitter_s << ','
            << sample.imu_age_s << ','
            << sample.encoder_age_s << ','
            << sample.slam_pose_valid << ','
            << sample.slam_pose_x << ','
            << sample.slam_pose_y << ','
            << sample.slam_pose_yaw << ','
            << sample.slam_position_innovation_m << ','
            << sample.slam_yaw_innovation_deg << ','
            << sample.slam_correction_accepted << ','
            << sample.stall_boost_active << ','
            << sample.left_wheel_stall_cycles << ','
            << sample.right_wheel_stall_cycles << ','
            << sample.encoder_slip_guard_active << ','
            << sample.exploration_map_source << ','
            << sample.exploration_state << ','
            << sample.control_source << ','
            << sample.selected_frontier_valid << ','
            << sample.selected_frontier_x << ','
            << sample.selected_frontier_y << ','
            << sample.global_free_cells << ','
            << sample.global_occupied_cells << ','
            << sample.start_matching_enabled << ','
            << sample.start_matching_complete << ','
            << sample.start_matching_accepted << ','
            << sample.start_matching_confidence << ','
            << sample.slam_toolbox_enabled << ','
            << sample.slam_toolbox_connected << ','
            << sample.slam_map_updates << ','
            << sample.slam_graph_nodes << ','
            << sample.slam_loop_edges << ','
            << sample.slam_map_age_s << '\n';
    }
    return true;
}

bool write_hardware_lidar_reconstruction_csv(const HardwareViewerState& hardware,
                                             const std::string& report_path) {
    std::ofstream out(report_path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        return false;
    }
    out << std::fixed << std::setprecision(6);
    out << "time,step,beam_index,pose_x,pose_y,pose_yaw,range_m,angle_world_rad,hit_x,hit_y,pose_source\n";
    for (const HardwareViewerState::LidarReconstructionPoint& point : hardware.lidar_reconstruction) {
        out << point.time << ','
            << point.step << ','
            << point.beam_index << ','
            << point.pose_x << ','
            << point.pose_y << ','
            << point.pose_yaw << ','
            << point.range_m << ','
            << point.angle_world_rad << ','
            << point.hit_x << ','
            << point.hit_y << ','
            << "estimated_onboard_pose\n";
    }
    return true;
}

bool write_hardware_lidar_reconstruction_ply(const HardwareViewerState& hardware,
                                             const std::string& report_path) {
    std::ofstream out(report_path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        return false;
    }
    out << std::fixed << std::setprecision(6);
    out << "ply\n";
    out << "format ascii 1.0\n";
    out << "comment Thesis hardware LiDAR reconstruction point cloud\n";
    out << "comment Coordinates are generated from LiDAR ranges transformed by estimated onboard pose, not external ground truth\n";
    out << "element vertex " << hardware.lidar_reconstruction.size() << "\n";
    out << "property float x\n";
    out << "property float y\n";
    out << "property float z\n";
    out << "end_header\n";
    for (const HardwareViewerState::LidarReconstructionPoint& point : hardware.lidar_reconstruction) {
        out << point.hit_x << ' ' << point.hit_y << " 0.000000\n";
    }
    return true;
}

bool write_hardware_lidar_reconstruction_png(const HardwareViewerState& hardware,
                                             const std::string& report_path) {
    thesis_sim::mvc::model::SlamReferenceArtifact artifact;
    artifact.bounds = hardware.has_scene
                          ? hardware.scene.world.bounds()
                          : Rect{-1.0, -1.0, 1.0, 1.0};
    artifact.draw_reference_geometry = false;
    artifact.draw_mission_markers = false;
    artifact.free_space_rays.reserve(hardware.lidar_reconstruction.size());
    artifact.occupied_points.reserve(hardware.lidar_reconstruction.size());
    for (const HardwareViewerState::LidarReconstructionPoint& point : hardware.lidar_reconstruction) {
        artifact.free_space_rays.push_back({
            {point.pose_x, point.pose_y},
            {point.hit_x, point.hit_y},
        });
        artifact.occupied_points.push_back({point.hit_x, point.hit_y});
    }
    artifact.estimated_trail.reserve(
        !hardware.history.empty() ? hardware.history.size() : hardware.frame.trail.size());
    if (!hardware.history.empty()) {
        for (const HardwareTelemetrySample& sample : hardware.history) {
            artifact.estimated_trail.push_back({sample.position_x, sample.position_y});
        }
    } else {
        artifact.estimated_trail = hardware.frame.trail;
    }
    return thesis_sim::mvc::model::write_slam_reference_png(artifact, report_path);
}

bool write_hardware_slam_reference_png(const HardwareViewerState& hardware,
                                       const std::string& report_path,
                                       const SlamToolboxSnapshot* slam_snapshot) {
    if (slam_snapshot != nullptr &&
        (slam_snapshot->connected ||
         slam_snapshot->status == "persistent_local_grid") &&
        !slam_snapshot->occupied_points.empty()) {
        thesis_sim::mvc::model::SlamReferenceArtifact artifact;
        artifact.bounds = hardware.has_scene
                              ? hardware.scene.world.bounds()
                              : Rect{-1.0, -1.0, 1.0, 1.0};
        artifact.draw_reference_geometry = false;
        artifact.draw_mission_markers = false;
        artifact.occupancy_resolution_m = slam_snapshot->map_resolution_m;
        artifact.free_points = slam_snapshot->free_points;
        artifact.occupied_points = slam_snapshot->occupied_points;
        artifact.estimated_trail.reserve(
            !hardware.history.empty() ? hardware.history.size() : hardware.frame.trail.size());
        if (!hardware.history.empty()) {
            for (const HardwareTelemetrySample& sample : hardware.history) {
                artifact.estimated_trail.push_back({sample.position_x, sample.position_y});
            }
        } else {
            artifact.estimated_trail = hardware.frame.trail;
        }
        return thesis_sim::mvc::model::write_slam_reference_png(artifact, report_path);
    }
    return write_hardware_lidar_reconstruction_png(hardware, report_path);
}


bool write_hardware_lidar_reconstruction_note(const HardwareViewerState& hardware,
                                              const std::string& report_path,
                                              const std::string& csv_path,
                                              const std::string& ply_path,
                                              const std::string& png_path) {
    std::ofstream out(report_path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        return false;
    }
    out << "# LiDAR Reconstruction Export\n\n";
    out << "- Points: `" << hardware.lidar_reconstruction.size() << "`\n";
    out << "- CSV: `" << csv_path << "`\n";
    out << "- PLY: `" << ply_path << "`\n";
    out << "- PNG occupancy map: `" << png_path << "`\n";
    out << "- Pose source: `estimated_onboard_pose`\n\n";
    out << "This export is intentionally passive: it records LiDAR hit points seen during the run without feeding them back into the gate arbiter, planner, or live UI state.\n\n";
    out << "The PNG rasterizes each estimated-pose-to-hit LiDAR ray as free space and each hit endpoint as occupied space, with the estimated robot trajectory overlaid in blue.\n\n";
    out << "The point coordinates are not external ground truth. They are LiDAR measurements transformed into the map frame using the onboard estimated pose at each frame. This makes the files useful for offline reconstruction, map consistency checks, and thesis figures, but a zenith camera or motion-capture system is still required for independent ground truth.\n";
    return true;
}

struct HardwareLidarReconstructionExport {
    std::string csv_path;
    std::string ply_path;
    std::string png_path;
    std::string note_path;
};

bool write_hardware_lidar_reconstruction_bundle(const HardwareViewerState& hardware,
                                                const std::string& json_report_path,
                                                HardwareLidarReconstructionExport* export_paths) {
    HardwareLidarReconstructionExport paths;
    paths.csv_path = append_stem_suffix(json_report_path, "_lidar_reconstruction", ".csv");
    paths.ply_path = append_stem_suffix(json_report_path, "_lidar_reconstruction", ".ply");
    paths.png_path = append_stem_suffix(json_report_path, "_lidar_reconstruction", ".png");
    paths.note_path = append_stem_suffix(json_report_path, "_lidar_reconstruction", ".md");

    const bool csv_ok = write_hardware_lidar_reconstruction_csv(hardware, paths.csv_path);
    const bool ply_ok = write_hardware_lidar_reconstruction_ply(hardware, paths.ply_path);
    const bool png_ok = write_hardware_lidar_reconstruction_png(hardware, paths.png_path);
    const bool note_ok = write_hardware_lidar_reconstruction_note(
        hardware,
        paths.note_path,
        paths.csv_path,
        paths.ply_path,
        paths.png_path);
    if (csv_ok && ply_ok && png_ok && note_ok && export_paths != nullptr) {
        *export_paths = paths;
    }
    return csv_ok && ply_ok && png_ok && note_ok;
}

bool write_sim_markdown_summary(const PlannerDrivenVehicleSim& sim,
                                const std::string& status,
                                const std::string& report_path) {
    std::ofstream out(report_path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        return false;
    }
    const RunQualitySummary quality = summarize_run_quality(sim.history());
    out << std::fixed << std::setprecision(3);
    out << "# Planner Run Summary\n\n";
    out << "- Status: `" << status << "`\n";
    out << "- Environment: `" << thesis_sim::environment_mode_name(sim.environment_mode()) << "`\n";
    out << "- Map: `" << map_preset_name(sim.world()) << "`\n";
    out << "- Vehicle model: `" << thesis_sim::vehicle_model_kind_name(sim.vehicle_model_kind()) << "`\n";
    out << "- Controller: `" << thesis_sim::tracking_controller_mode_name(sim.tracking_controller_mode()) << "`\n";
    out << "- Dynamic gates: `" << (sim.config().dynamic_lidar_gates ? "enabled" : "disabled") << "`\n";
    out << "- Sensor profile: `" << thesis_sim::range_sensor_profile_name(sim.range_sensor_profile()) << "`\n\n";
    out << "| Metric | Value |\n";
    out << "| --- | ---: |\n";
    out << "| Runtime | " << sim.sim_time() << " s |\n";
    out << "| Steps | " << sim.step_count() << " |\n";
    out << "| Final goal distance | " << sim.distance_to_goal() << " m |\n";
    out << "| Passed gates | " << sim.passed_gate_count() << " |\n";
    out << "| Average speed | " << quality.avg_speed << " m/s |\n";
    out << "| Max cross-track error | " << quality.max_cross_track << " m |\n";
    out << "| Max heading error | " << quality.max_heading_error_deg << " deg |\n";
    out << "| Min LiDAR clearance | " << quality.min_lidar << " m |\n";
    out << "| Avg step time | " << quality.avg_step_ms << " ms |\n";
    out << "| Max step time | " << quality.max_step_ms << " ms |\n";
    out << "| Mixed switches | " << quality.mixed_switches << " |\n";
    out << "| Mixed aborts | " << quality.mixed_aborts << " |\n\n";

    const std::vector<TimelineEvent> events = build_sim_timeline(sim);
    if (!events.empty()) {
        out << "## Timeline\n\n";
        const size_t first = events.size() > 24 ? events.size() - 24 : 0;
        for (size_t i = first; i < events.size(); ++i) {
            out << "- " << events[i].time << " s: **" << events[i].label << "**";
            if (!events[i].detail.empty()) {
                out << " - " << events[i].detail;
            }
            out << '\n';
        }
    }
    return true;
}

bool write_hardware_markdown_summary(const HardwareViewerState& hardware,
                                     const UiState& ui_state,
                                     const LiveViewStreamServer& hardware_server,
                                     const std::string& report_path) {
    std::ofstream out(report_path, std::ios::out | std::ios::trunc);
    if (!out.is_open()) {
        return false;
    }
    const RunQualitySummary quality = summarize_run_quality(hardware.history);
    const HardwareThesisMetrics thesis_metrics = summarize_hardware_thesis_metrics(hardware);
    const HardwarePoseQuality pose_quality = estimate_hardware_pose_quality(hardware);
    const VehicleModelKind configured_model =
        static_cast<VehicleModelKind>(ui_state.hardware_vehicle_model);
    out << std::fixed << std::setprecision(3);
    out << "# Hardware Run Summary\n\n";
    out << "- Listener: `" << (hardware_server.listening() ? "listening" : "off") << "`\n";
    out << "- Connected: `" << (hardware_server.connected() ? "yes" : "no") << "`\n";
    out << "- Port: `" << hardware_server.port() << "`\n";
    out << "- Configured robot: `" << thesis_sim::vehicle_model_kind_name(configured_model) << "`\n";
    out << "- Live robot: `" << (hardware.has_scene ? hardware.scene.vehicle_model_name : "waiting for scene") << "`\n";
    out << "- Pose source: `" << pose_quality.source << "` (" << pose_quality.confidence << " confidence)\n";
    out << "- Launch command: `" << hardware_launch_hint(ui_state) << "`\n\n";
    out << "| Metric | Value |\n";
    out << "| --- | ---: |\n";
    out << "| Samples | " << hardware.history.size() << " |\n";
    out << "| LiDAR reconstruction points | " << hardware.lidar_reconstruction.size() << " |\n";
    out << "| Runtime | " << hardware.frame.sim_time << " s |\n";
    out << "| Goal distance | " << hardware.frame.distance_to_goal << " m |\n";
    out << "| Time to goal | " << thesis_metrics.time_to_goal_s << " s |\n";
    out << "| Passed gates | " << thesis_metrics.passed_gates << " |\n";
    out << "| Mixed switch events | " << thesis_metrics.switch_events << " |\n";
    out << "| Structured / gate time | " << thesis_metrics.structured_time_s << " / " << thesis_metrics.gate_time_s << " s |\n";
    out << "| Encoder distance | " << thesis_metrics.encoder_distance_m << " m |\n";
    out << "| Estimated path length | " << thesis_metrics.estimated_path_length_m << " m |\n";
    out << "| Odom-pose delta | " << thesis_metrics.odom_pose_delta_m << " m |\n";
    out << "| Average speed | " << quality.avg_speed << " m/s |\n";
    out << "| Max cross-track error | " << quality.max_cross_track << " m |\n";
    out << "| Max heading error | " << quality.max_heading_error_deg << " deg |\n";
    out << "| Min LiDAR clearance | " << quality.min_lidar << " m |\n";
    out << "| Avg step time | " << quality.avg_step_ms << " ms |\n";
    out << "| Max step time | " << quality.max_step_ms << " ms |\n";
    out << "| Planner reference | " << (hardware.frame.planner_has_reference ? "ready" : "waiting") << " |\n";
    out << "| Safety stop | " << (hardware.frame.safety_stop_active ? "active" : "clear") << " |\n\n";

    const std::vector<TimelineEvent> events = build_hardware_timeline(hardware);
    if (!events.empty()) {
        out << "## Timeline\n\n";
        const size_t first = events.size() > 24 ? events.size() - 24 : 0;
        for (size_t i = first; i < events.size(); ++i) {
            out << "- " << events[i].time << " s: **" << events[i].label << "**";
            if (!events[i].detail.empty()) {
                out << " - " << events[i].detail;
            }
            out << '\n';
        }
    }
    out << "\n## LiDAR Reconstruction\n\n";
    out << "The hardware thesis bundle includes passive LiDAR reconstruction files. The CSV/PLY store raw LiDAR hit points accumulated over the run, while the PNG rasterizes an occupancy-style map with free-space rays, occupied endpoints, and the estimated robot trajectory. These files are transformed with the onboard estimated pose, so they are useful for offline mapping and thesis figures but they are not external ground truth.\n";
    return true;
}
