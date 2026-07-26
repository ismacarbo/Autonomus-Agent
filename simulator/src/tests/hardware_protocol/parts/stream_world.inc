bool stream_enabled(const TestOptions& options) {
    return !options.stream.host.empty() && options.stream.port > 0;
}

WorldMap make_selected_world(const TestOptions& options) {
    if (!options.world_file.empty()) {
        std::ifstream in(options.world_file, std::ios::binary);
        if (!in.is_open()) {
            throw std::runtime_error("could not open world file: " + options.world_file);
        }
        std::vector<std::uint8_t> blob(
            (std::istreambuf_iterator<char>(in)),
            std::istreambuf_iterator<char>());
        WorldMap world;
        if (!thesis_sim::deserialize_world_blob(blob, &world)) {
            throw std::runtime_error("could not decode world file: " + options.world_file);
        }
        return world;
    }
    if (options.environment_mode == EnvironmentMode::StructuredRoad) {
        return WorldMap::structured_demo(options.structured_preset);
    }
    return WorldMap::unstructured_demo(options.unstructured_preset, GateBehaviorMode::Static, 0);
}

void translate_world(WorldMap* world, const Vec2& delta) {
    if (world == nullptr) {
        return;
    }

    Rect bounds = world->bounds();
    bounds.min_x += delta.x;
    bounds.max_x += delta.x;
    bounds.min_y += delta.y;
    bounds.max_y += delta.y;
    world->set_bounds(bounds);
    world->set_start({world->start().x + delta.x, world->start().y + delta.y});
    world->set_goal({world->goal().x + delta.x, world->goal().y + delta.y});

    for (Rect& obstacle : world->editable_obstacles()) {
        obstacle.min_x += delta.x;
        obstacle.max_x += delta.x;
        obstacle.min_y += delta.y;
        obstacle.max_y += delta.y;
    }
    for (Rect& obstacle : world->editable_perception_obstacles()) {
        obstacle.min_x += delta.x;
        obstacle.max_x += delta.x;
        obstacle.min_y += delta.y;
        obstacle.max_y += delta.y;
    }
    for (auto& gate : world->editable_gates()) {
        gate.position = {gate.position.x + delta.x, gate.position.y + delta.y};
        gate.anchor_position = {gate.anchor_position.x + delta.x, gate.anchor_position.y + delta.y};
    }
    for (Vec2& point : world->editable_road_centerline()) {
        point = {point.x + delta.x, point.y + delta.y};
    }
}

WorldMap make_local_mapping_world(const TestOptions& options) {
    WorldMap world = make_selected_world(options);
    translate_world(&world, {-world.start().x, -world.start().y});
    return world;
}

WorldMap make_stream_world(const TestOptions& options) {
    WorldMap world = make_selected_world(options);
    world.editable_obstacles().clear();
    world.editable_perception_obstacles().clear();

    const double size = std::max(options.stream.world_size_m, 2.0);
    const Vec2 anchor{size * 0.5, size * 0.5};
    translate_world(&world, {anchor.x - world.start().x, anchor.y - world.start().y});

    Rect bounds = world.bounds();
    bounds.min_x = std::min(bounds.min_x, anchor.x - size * 0.5);
    bounds.max_x = std::max(bounds.max_x, anchor.x + size * 0.5);
    bounds.min_y = std::min(bounds.min_y, anchor.y - size * 0.5);
    bounds.max_y = std::max(bounds.max_y, anchor.y + size * 0.5);
    world.set_bounds(bounds);
    return world;
}

LiveSceneSnapshot make_smoke_scene(const TestOptions& options,
                                   const VehicleGeometry& geometry,
                                   const WorldMap& world) {
    LiveSceneSnapshot scene;
    scene.stream_label = options.infinite ? "Hardware Online SLAM" : "Hardware SLAM Smoke Test";
    scene.stream_profile = "slam";
    scene.world = world;
    scene.geometry = geometry;
    scene.imu_enabled = true;
    scene.lidar_enabled = options.simulate || !options.lidar_port.empty();
    scene.localization_mode = "Encoder + IMU EKF + LiDAR scan-to-map";
    scene.heading_source = "IMU fused in EKF";
    scene.range_sensor_name = scene.lidar_enabled ? "RPLidar A1" : "LiDAR disabled";
    scene.vehicle_model_name = "Differential-drive smoke test";
    scene.tracking_controller_name = "Direct PWM diagnostics";
    scene.active_lidar_beams = 360;
    scene.active_lidar_fov_rad = 2.0 * kPi;
    scene.active_lidar_range = options.map.lidar_max_range_m;
    return scene;
}

bool setup_stream_client(const TestOptions& options,
                         const LiveSceneSnapshot& scene,
                         LiveViewStreamClient* streamer) {
    if (!stream_enabled(options) || streamer == nullptr) {
        return true;
    }
    if (!streamer->connect_to(options.stream.host, static_cast<std::uint16_t>(options.stream.port))) {
        std::cerr << "live_stream_error=" << streamer->last_error() << '\n';
        return false;
    }
    if (!streamer->send_scene(scene)) {
        std::cerr << "live_stream_error=" << streamer->last_error() << '\n';
        return false;
    }
    return true;
}

void stream_frame_if_due(const TestOptions& options,
                         int step_count,
                         const LiveFrameSnapshot& frame,
                         LiveViewStreamClient* streamer,
                         bool force = false) {
    if (streamer == nullptr || !streamer->connected() || !stream_enabled(options)) {
        return;
    }
    const bool due = (step_count % std::max(options.stream.every_n_steps, 1)) == 0;
    if (!force && !due) {
        return;
    }
    if (!streamer->send_frame(frame)) {
        std::cerr << "live_stream_error=" << streamer->last_error() << '\n';
        streamer->disconnect();
    }
}

Vec2 stream_offset(const WorldMap& world) {
    return world.start();
}

std::vector<Vec2> offset_trail_for_stream(const std::vector<Pose2D>& trail,
                                          const Vec2& offset,
                                          std::size_t max_points) {
    std::vector<Vec2> out;
    if (trail.empty()) {
        return out;
    }
    const std::size_t begin = trail.size() > max_points ? trail.size() - max_points : 0U;
    out.reserve(trail.size() - begin);
    for (std::size_t i = begin; i < trail.size(); ++i) {
        out.push_back({trail[i].x + offset.x, trail[i].y + offset.y});
    }
    return out;
}

std::vector<Vec2> offset_points_for_stream(const std::vector<Vec2>& points,
                                           const Vec2& offset) {
    std::vector<Vec2> out;
    out.reserve(points.size());
    for (const Vec2& point : points) {
        out.push_back({point.x + offset.x, point.y + offset.y});
    }
    return out;
}

std::vector<LidarHit> offset_hits_for_stream(const std::vector<LidarHit>& hits,
                                             const Vec2& offset) {
    std::vector<LidarHit> out;
    out.reserve(hits.size());
    for (const LidarHit& hit : hits) {
        out.push_back({hit.angle, hit.distance, {hit.point.x + offset.x, hit.point.y + offset.y}, hit.hit});
    }
    return out;
}

LiveFrameSnapshot make_smoke_frame(double sim_time,
                                   int step_count,
                                   bool connected,
                                   bool telemetry_ready,
                                   bool safety_stop_active,
                                   double traveled_distance_m,
                                   double min_lidar_distance,
                                   double front_lidar_distance,
                                   const SlamEstimate& estimate,
                                   const WorldMap& stream_world,
                                   const VehicleGeometry& geometry,
                                   const DriveConfig& drive,
                                   const OdomUpdate& odom,
                                   const ControllerTelemetry* telemetry,
                                   int cmd_left,
                                   int cmd_right,
                                   const std::vector<Pose2D>& trail,
                                   const std::vector<LidarHit>& current_scan_hits,
                                   const std::vector<Vec2>& slam_points,
                                   const HardwareTelemetrySample& sample,
                                   const Vec2& offset) {
    LiveFrameSnapshot frame;
    frame.sim_time = sim_time;
    frame.step_count = step_count;
    frame.connected = connected;
    frame.telemetry_ready = telemetry_ready;
    frame.goal_reached = false;
    frame.safety_stop_active = safety_stop_active;
    frame.distance_to_goal = traveled_distance_m;
    frame.min_lidar_distance = min_lidar_distance;
    frame.front_lidar_distance = front_lidar_distance;
    frame.last_j = 0.0;
    frame.last_r = estimate_target_yaw_rate(cmd_left, cmd_right, drive);
    frame.planner_speed_ref = estimate_target_speed(cmd_left, cmd_right, drive);
    frame.tracker_cross_track_error = 0.0;
    frame.tracker_heading_error_deg = 0.0;
    frame.chosen_gate_index = -1;
    frame.navigation_position = {estimate.pose.x + offset.x, estimate.pose.y + offset.y};
    frame.navigation_yaw = estimate.pose.yaw;
    frame.navigation_yaw_rate = estimate.yaw_rate;
    frame.navigation_curvature = estimate.curvature;
    frame.navigation_speed = estimate.speed;
    frame.navigation_accel = estimate.accel;
    frame.trail = offset_trail_for_stream(trail, offset, 1600);
    frame.slam_points = offset_points_for_stream(slam_points, offset);
    frame.lidar_hits = offset_hits_for_stream(current_scan_hits, offset);
    frame.gates.reserve(stream_world.gates().size());
    for (const auto& gate : stream_world.gates()) {
        frame.gates.push_back({gate, false});
    }

    const double half_track = geometry.track * 0.5;
    LiveVehicleState vehicle;
    vehicle.position = frame.navigation_position;
    vehicle.yaw = estimate.pose.yaw;
    vehicle.speed = estimate.speed;
    vehicle.accel = estimate.accel;
    vehicle.curvature = estimate.curvature;
    vehicle.steer_angle = std::clamp(
        std::atan(geometry.wheelbase * estimate.curvature),
        -geometry.max_steer_angle,
        geometry.max_steer_angle);
    vehicle.yaw_rate = estimate.yaw_rate;
    vehicle.left_wheel_speed = estimate.speed - estimate.yaw_rate * half_track;
    vehicle.right_wheel_speed = estimate.speed + estimate.yaw_rate * half_track;
    vehicle.target_speed = estimate_target_speed(cmd_left, cmd_right, drive);
    vehicle.target_yaw_rate = estimate_target_yaw_rate(cmd_left, cmd_right, drive);
    vehicle.target_steer_angle = 0.0;
    vehicle.left_encoder_delta = odom.left_delta_ticks;
    vehicle.right_encoder_delta = odom.right_delta_ticks;
    vehicle.encoder_dt_ms = odom.encoder_dt_s * 1000.0;
    vehicle.left_pwm = telemetry != nullptr ? telemetry->target_pwm_l : cmd_left;
    vehicle.right_pwm = telemetry != nullptr ? telemetry->target_pwm_r : cmd_right;
    if (telemetry != nullptr) {
        vehicle.left_encoder_ticks = telemetry->ticks_left;
        vehicle.right_encoder_ticks = telemetry->ticks_right;
    }
    frame.vehicle = vehicle;
    frame.has_latest_sample = true;
    frame.latest_sample = sample;
    return frame;
}

