std::vector<std::uint8_t> serialize_world_blob(const WorldMap& world) {
    std::vector<std::uint8_t> out;
    out.reserve(512);
    write_pod(&out, kWorldBlobMagic);
    write_pod(&out, kWorldBlobVersion);
    write_world(&out, world);
    return out;
}

bool deserialize_world_blob(const std::vector<std::uint8_t>& data, WorldMap* world) {
    if (world == nullptr) {
        return false;
    }

    std::size_t offset = 0;
    std::uint32_t magic = 0U;
    std::uint16_t version = 0U;
    if (!read_pod(data, &offset, &magic) ||
        !read_pod(data, &offset, &version) ||
        magic != kWorldBlobMagic ||
        version != kWorldBlobVersion ||
        !read_world(data, &offset, world)) {
        return false;
    }
    return offset == data.size();
}

LiveSceneSnapshot make_live_scene_snapshot(const HardwarePlannerRunner& runner) {
    const StructuredDisplayRemap remap = make_structured_display_remap(runner.world());
    const bool lidar_enabled = runner.lidar_enabled_for_current_mode();
    WorldMap scene_world = remap.display_world;
    if (runner.world().environment_mode() == EnvironmentMode::UnstructuredGates) {
        scene_world.editable_obstacles().clear();
        scene_world.editable_perception_obstacles().clear();
        scene_world.editable_gates().clear();
    }
    LiveSceneSnapshot scene;
    scene.stream_label = "Hardware live stream";
    scene.stream_profile = "planner";
    scene.world = std::move(scene_world);
    scene.geometry = runner.geometry();
    scene.imu_enabled = true;
    scene.lidar_enabled = lidar_enabled;
    const bool lidar_pose_correction = runner.lidar_pose_correction_enabled_for_current_mode();
    scene.localization_mode =
        !lidar_enabled
            ? "EKF (encoder/IMU + road constraint)"
            : (lidar_pose_correction
                   ? "EKF (encoder/IMU + LiDAR pose correction)"
                   : "EKF (encoder/IMU primary; LiDAR perception)");
    scene.heading_source = "IMU";
    scene.range_sensor_name =
        runner.world().environment_mode() == EnvironmentMode::UnstructuredGates
            ? "RPLidar A1 (perception-driven)"
            : (lidar_enabled
                   ? (lidar_pose_correction ? "RPLidar A1 (pose + perception)"
                                            : "RPLidar A1 (perception only)")
                   : "LiDAR disabled for structured planner");
    scene.vehicle_model_name = vehicle_model_kind_name(runner.config().vehicle_model);
    scene.tracking_controller_name = "MPC path follower";
    scene.active_lidar_beams = lidar_enabled ? 360 : 0;
    scene.active_lidar_fov_rad = lidar_enabled ? kTwoPi : 0.0;
    scene.active_lidar_range = lidar_enabled ? runner.config().localization.max_range_m : 0.0;
    return scene;
}

LiveFrameSnapshot make_live_frame_snapshot(const HardwarePlannerRunner& runner) {
    const StructuredDisplayRemap remap = make_structured_display_remap(runner.world());
    const RealRobotObservation& observation = runner.bridge().observation();
    LiveFrameSnapshot frame;
    frame.sim_time = runner.sim_time();
    frame.step_count = runner.step_count();
    frame.connected = runner.connected();
    frame.telemetry_ready = runner.telemetry_ready();
    frame.goal_reached = runner.goal_reached();
    frame.safety_stop_active = runner.safety_stop_active();
    frame.dynamic_gap_gates = runner.diagnostics().dynamic_gap_gates;
    frame.planner_has_reference = runner.diagnostics().planner_has_reference;
    frame.stall_boost_active = runner.diagnostics().stall_boost_active;
    frame.distance_to_goal = runner.distance_to_goal();
    frame.min_lidar_distance = runner.estimate().min_lidar_distance;
    frame.front_lidar_distance = runner.estimate().front_lidar_distance;
    frame.chosen_gate_distance =
        std::isfinite(runner.diagnostics().chosen_gate_distance) ? runner.diagnostics().chosen_gate_distance : -1.0;
    frame.last_j = runner.last_j();
    frame.last_r = runner.last_r();
    frame.planner_speed_ref = runner.planner_speed_reference();
    frame.tracker_cross_track_error = runner.tracker_cross_track_error();
    frame.tracker_heading_error_deg = runner.tracker_heading_error_deg();
    frame.valid_lidar_samples = runner.diagnostics().valid_lidar_points;
    frame.close_lidar_samples = runner.diagnostics().close_lidar_points;
    frame.front_close_lidar_samples = runner.diagnostics().front_close_lidar_points;
    frame.candidate_gates = runner.diagnostics().candidate_gates;
    frame.accumulated_lidar_points = runner.diagnostics().accumulated_lidar_points;
    frame.no_motion_command_cycles = runner.diagnostics().no_motion_command_cycles;
    frame.chosen_gate_index = runner.chosen_gate_index();
    frame.passed_gates = runner.passed_gate_count();
    frame.occupancy_cell_size_m = runner.config().gap_extraction.map_point_resolution_m;
    frame.navigation_position = runner.estimate().position;
    frame.navigation_yaw = runner.estimate().yaw;
    frame.navigation_yaw_rate = runner.estimate().yaw_rate;
    frame.navigation_curvature = runner.estimate().curvature;
    frame.navigation_speed = runner.estimate().speed;
    frame.navigation_accel = runner.estimate().accel;
    frame.visible_gate_indices = runner.visible_gate_indices();
    frame.trail = runner.trail();
    frame.planned_trajectory = runner.planned_trajectory();
    frame.slam_points = runner.lidar_map_points();
    frame.lidar_hits = runner.lidar_hits();

    const HardwarePlannerEstimate& estimate = runner.estimate();
    const HardwareControlCommand& command = runner.last_command();
    const VehicleGeometry& geometry = runner.geometry();
    const double half_track = geometry.track * 0.5;
    frame.vehicle.position = estimate.position;
    frame.vehicle.yaw = estimate.yaw;
    frame.vehicle.speed = estimate.speed;
    frame.vehicle.accel = estimate.accel;
    frame.vehicle.curvature = estimate.curvature;
    frame.vehicle.steer_angle = std::clamp(
        std::atan(geometry.wheelbase * estimate.curvature),
        -geometry.max_steer_angle,
        geometry.max_steer_angle);
    frame.vehicle.yaw_rate = estimate.yaw_rate;
    frame.vehicle.sideslip = 0.0;
    frame.vehicle.left_wheel_speed = estimate.speed - estimate.yaw_rate * half_track;
    frame.vehicle.right_wheel_speed = estimate.speed + estimate.yaw_rate * half_track;
    frame.vehicle.target_speed = command.target_speed;
    frame.vehicle.target_yaw_rate = command.target_yaw_rate;
    frame.vehicle.target_steer_angle = std::clamp(
        std::atan(geometry.wheelbase * command.target_curvature),
        -geometry.max_steer_angle,
        geometry.max_steer_angle);
    frame.vehicle.left_pwm = command.pwm_left;
    frame.vehicle.right_pwm = command.pwm_right;
    if (observation.have_controller_telemetry) {
        frame.vehicle.left_encoder_ticks = observation.controller.ticks_left;
        frame.vehicle.right_encoder_ticks = observation.controller.ticks_right;
        frame.vehicle.encoder_dt_ms = static_cast<double>(observation.controller.enc_dt_ms);
    }

    const auto& runtime_gate_specs = runner.gate_specs();
    const auto& planner_gates = runner.gates();
    frame.gates.reserve(runtime_gate_specs.size());
    for (std::size_t i = 0; i < runtime_gate_specs.size(); ++i) {
        LiveGateFrame gate_frame;
        gate_frame.spec = runtime_gate_specs[i];
        if (i < planner_gates.size()) {
            gate_frame.passed = planner_gates[i].passed;
        }
        frame.gates.push_back(std::move(gate_frame));
    }

    if (runner.last_mpc_command().has_value()) {
        frame.has_last_mpc_command = true;
        frame.last_mpc_command.accel_cmd = runner.last_mpc_command()->accel_cmd;
        frame.last_mpc_command.steer_rate_cmd = runner.last_mpc_command()->steer_rate_cmd;
    }

    if (!runner.history().empty()) {
        frame.has_latest_sample = true;
        frame.latest_sample = runner.history().back();
        frame.vehicle.left_encoder_ticks = frame.latest_sample.controller_left_encoder_ticks;
        frame.vehicle.right_encoder_ticks = frame.latest_sample.controller_right_encoder_ticks;
        frame.vehicle.left_encoder_delta = frame.latest_sample.controller_left_encoder_delta;
        frame.vehicle.right_encoder_delta = frame.latest_sample.controller_right_encoder_delta;
        frame.vehicle.encoder_dt_ms = frame.latest_sample.controller_encoder_dt_ms;
    }

    apply_structured_display_remap(remap, &frame);
    return frame;
}

