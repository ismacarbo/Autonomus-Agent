struct SimRobotState {
    Pose2D pose;
    double left_speed_mps = 0.0;
    double right_speed_mps = 0.0;
    double left_ticks_continuous = 0.0;
    double right_ticks_continuous = 0.0;
};

double wheel_speed_from_pwm(int pwm, const DriveConfig& drive) {
    if (std::abs(pwm) < drive.min_effective_pwm) {
        return 0.0;
    }
    const double magnitude =
        static_cast<double>(std::abs(pwm) - drive.min_effective_pwm) * drive.speed_estimate_per_pwm;
    return std::copysign(magnitude, static_cast<double>(pwm));
}

double estimate_target_speed(int pwm_left, int pwm_right, const DriveConfig& drive) {
    return 0.5 * (wheel_speed_from_pwm(pwm_left, drive) + wheel_speed_from_pwm(pwm_right, drive));
}

double estimate_target_yaw_rate(int pwm_left, int pwm_right, const DriveConfig& drive) {
    if (std::abs(drive.track_width) <= 1e-6) {
        return 0.0;
    }
    return (wheel_speed_from_pwm(pwm_right, drive) - wheel_speed_from_pwm(pwm_left, drive)) /
           drive.track_width;
}

std::vector<RPLidarA1::ScanPoint> make_sim_lidar_scan(const WorldMap& world,
                                                      const Pose2D& pose,
                                                      const MappingConfig& config) {
    constexpr int kBeams = 360;
    const Vec2 origin = lidar_origin_world(pose, config);
    const std::vector<LidarHit> hits = world.raycast(
        origin,
        pose.yaw + config.lidar_yaw_offset_rad,
        kBeams,
        2.0 * kPi,
        config.lidar_max_range_m,
        true);

    std::vector<RPLidarA1::ScanPoint> scan;
    scan.reserve(hits.size());
    for (std::size_t i = 0; i < hits.size(); ++i) {
        const double alpha = hits.size() > 1
                                 ? static_cast<double>(i) / static_cast<double>(hits.size() - 1)
                                 : 0.0;
        const double angle_deg = -180.0 + alpha * 360.0;
        const double angle_rad = deg_to_rad(angle_deg);
        const double distance_m = hits[i].distance;
        scan.push_back({
            static_cast<std::uint8_t>(distance_m < config.lidar_max_range_m ? 15 : 0),
            angle_deg,
            distance_m * 1000.0,
            distance_m,
            std::cos(angle_rad) * distance_m,
            std::sin(angle_rad) * distance_m,
        });
    }
    return scan;
}

RealRobotObservation make_sim_observation(const WorldMap& world,
                                          SimRobotState* state,
                                          int pwm_left,
                                          int pwm_right,
                                          double dt,
                                          const TestOptions& options,
                                          double host_time_s) {
    const double alpha = 1.0 - std::exp(-dt / std::max(options.drive.motor_time_constant_s, 1e-3));
    state->left_speed_mps += alpha * (wheel_speed_from_pwm(pwm_left, options.drive) - state->left_speed_mps);
    state->right_speed_mps += alpha * (wheel_speed_from_pwm(pwm_right, options.drive) - state->right_speed_mps);

    const double left_distance = state->left_speed_mps * dt;
    const double right_distance = state->right_speed_mps * dt;
    const double delta_yaw =
        std::abs(options.drive.track_width) > 1e-6 ? (right_distance - left_distance) / options.drive.track_width : 0.0;
    const double mid_yaw = wrap_angle(state->pose.yaw + 0.5 * delta_yaw);
    const double distance = 0.5 * (left_distance + right_distance);
    state->pose.x += distance * std::cos(mid_yaw);
    state->pose.y += distance * std::sin(mid_yaw);
    state->pose.yaw = wrap_angle(state->pose.yaw + delta_yaw);

    const double ticks_per_meter =
        static_cast<double>(std::max(options.drive.encoder_ticks_per_revolution, 1)) /
        (2.0 * kPi * std::max(options.drive.wheel_radius, 1e-4));
    state->left_ticks_continuous += left_distance * ticks_per_meter;
    state->right_ticks_continuous += right_distance * ticks_per_meter;

    ControllerTelemetry telemetry{};
    telemetry.ms = static_cast<std::uint32_t>(std::llround(host_time_s * 1000.0));
    telemetry.rx_timestamp_s = host_time_s;
    telemetry.imu_ms = telemetry.ms;
    telemetry.motor_ms = telemetry.ms;
    telemetry.encoder_ms = telemetry.ms;
    telemetry.heartbeat_ms = telemetry.ms;
    telemetry.yaw_mrad = static_cast<std::int32_t>(std::llround(state->pose.yaw * 1000.0));
    telemetry.yaw_rate_mrad_s = static_cast<std::int32_t>(std::llround(delta_yaw / std::max(dt, 1e-3) * 1000.0));
    telemetry.yaw_deg = rad_to_deg(state->pose.yaw);
    telemetry.yaw_rate_dps = rad_to_deg(delta_yaw / std::max(dt, 1e-3));
    telemetry.pwm_l = static_cast<std::int16_t>(pwm_left);
    telemetry.pwm_r = static_cast<std::int16_t>(pwm_right);
    telemetry.target_pwm_l = static_cast<std::int16_t>(pwm_left);
    telemetry.target_pwm_r = static_cast<std::int16_t>(pwm_right);
    telemetry.ticks_left = static_cast<std::int32_t>(std::llround(state->left_ticks_continuous));
    telemetry.ticks_right = static_cast<std::int32_t>(std::llround(state->right_ticks_continuous));
    telemetry.enc_dt_ms = static_cast<std::uint16_t>(std::llround(dt * 1000.0));
    telemetry.have_imu = true;
    telemetry.have_motor = true;
    telemetry.have_encoder = true;
    telemetry.have_heartbeat = true;

    RealRobotObservation observation{};
    observation.host_timestamp_s = host_time_s;
    observation.have_controller_telemetry = true;
    observation.controller = telemetry;
    observation.lidar_scan = make_sim_lidar_scan(world, state->pose, options.map);
    observation.have_lidar_scan = !observation.lidar_scan.empty();
    observation.min_lidar_range_m = 0.0;
    for (const RPLidarA1::ScanPoint& point : observation.lidar_scan) {
        if (point.distance_m < options.map.lidar_min_range_m) {
            continue;
        }
        if (observation.min_lidar_range_m <= 0.0) {
            observation.min_lidar_range_m = point.distance_m;
        } else {
            observation.min_lidar_range_m = std::min(observation.min_lidar_range_m, point.distance_m);
        }
    }
    return observation;
}

double compute_min_scan_range(const std::vector<RPLidarA1::ScanPoint>& scan, const MappingConfig& config) {
    double best = std::numeric_limits<double>::infinity();
    for (const RPLidarA1::ScanPoint& point : scan) {
        if (point.distance_m < config.lidar_min_range_m || point.distance_m > config.lidar_max_range_m) {
            continue;
        }
        best = std::min(best, point.distance_m);
    }
    return std::isfinite(best) ? best : 0.0;
}

double compute_front_distance(const std::vector<RPLidarA1::ScanPoint>& scan, const MappingConfig& config) {
    double best = std::numeric_limits<double>::infinity();
    constexpr double kFrontHalfAngleRad = 0.40;
    for (const RPLidarA1::ScanPoint& point : scan) {
        if (point.distance_m < config.lidar_min_range_m || point.distance_m > config.lidar_max_range_m) {
            continue;
        }
        const double beam_yaw = wrap_angle(deg_to_rad(point.angle_deg) + config.lidar_yaw_offset_rad);
        if (std::abs(beam_yaw) <= kFrontHalfAngleRad) {
            best = std::min(best, point.distance_m);
        }
    }
    return std::isfinite(best) ? best : 0.0;
}

std::vector<LidarHit> build_lidar_hits_world(const Pose2D& pose,
                                             const std::vector<RPLidarA1::ScanPoint>& scan,
                                             const MappingConfig& config) {
    std::vector<LidarHit> hits;
    hits.reserve(scan.size());
    const Vec2 origin = lidar_origin_world(pose, config);
    for (const RPLidarA1::ScanPoint& point : scan) {
        if (point.distance_m < config.lidar_min_range_m || point.distance_m > config.lidar_max_range_m) {
            continue;
        }
        const Vec2 hit = scan_point_world(pose, point, config);
        const double angle_world = std::atan2(hit.y - origin.y, hit.x - origin.x);
        hits.push_back({angle_world, point.distance_m, hit, true});
    }
    return hits;
}

Pose2D match_scan_to_map(const OccupancyGrid& grid,
                         const Pose2D& predicted,
                         const std::vector<RPLidarA1::ScanPoint>& scan,
                         const MappingConfig& config) {
    if (!grid.ready_for_matching() || static_cast<int>(scan.size()) < config.lidar_min_points) {
        return predicted;
    }

    struct SearchWindow {
        double xy_window = 0.0;
        double xy_step = 0.0;
        double yaw_window = 0.0;
        double yaw_step = 0.0;
    };

    const std::vector<SearchWindow> windows{
        {config.match_xy_window_m, config.match_xy_step_m, config.match_yaw_window_rad, config.match_yaw_step_rad},
        {std::max(config.match_xy_step_m, 0.02), std::max(config.match_xy_step_m * 0.5, 0.02),
         std::max(config.match_yaw_step_rad, 0.02), std::max(config.match_yaw_step_rad * 0.5, 0.01)},
    };

    Pose2D best = predicted;
    double best_score = grid.score_scan(best, scan, config);
    for (const SearchWindow& window : windows) {
        const Pose2D seed = best;
        for (double dx = -window.xy_window; dx <= window.xy_window + 1e-9; dx += window.xy_step) {
            for (double dy = -window.xy_window; dy <= window.xy_window + 1e-9; dy += window.xy_step) {
                for (double dyaw = -window.yaw_window; dyaw <= window.yaw_window + 1e-9; dyaw += window.yaw_step) {
                    const Pose2D candidate{seed.x + dx, seed.y + dy, wrap_angle(seed.yaw + dyaw)};
                    double score = grid.score_scan(candidate, scan, config);
                    score -= 35.0 * (dx * dx + dy * dy);
                    score -= 0.90 * std::abs(dyaw);
                    if (score > best_score) {
                        best_score = score;
                        best = candidate;
                    }
                }
            }
        }
    }
    return best;
}

std::vector<MotionSegment> build_motion_script(const TestOptions& options) {
    std::vector<MotionSegment> script;
    script.push_back({"warmup", 1.0, 0, 0});
    for (int i = 0; i < std::max(options.loops, 1); ++i) {
        const std::string lap = std::to_string(i + 1);
        script.push_back({"forward_a_" + lap, options.forward_seconds, options.forward_pwm, options.forward_pwm});
        script.push_back({"pause_a_" + lap, options.stop_seconds, 0, 0});
        script.push_back({"spin_left_" + lap, options.turn_seconds, -options.turn_pwm, options.turn_pwm});
        script.push_back({"pause_b_" + lap, options.stop_seconds, 0, 0});
        script.push_back({"forward_b_" + lap, options.forward_seconds, options.forward_pwm, options.forward_pwm});
        script.push_back({"pause_c_" + lap, options.stop_seconds, 0, 0});
        script.push_back({"spin_right_" + lap, options.turn_seconds, options.turn_pwm, -options.turn_pwm});
        script.push_back({"pause_d_" + lap, options.stop_seconds, 0, 0});
    }
    script.push_back({"final_stop", 1.0, 0, 0});
    return script;
}

double total_script_duration(const std::vector<MotionSegment>& script) {
    double total = 0.0;
    for (const MotionSegment& segment : script) {
        total += std::max(segment.duration_s, 0.0);
    }
    return total;
}

std::size_t phase_index_at_time(const std::vector<MotionSegment>& script, double elapsed_s) {
    double cursor = 0.0;
    for (std::size_t i = 0; i < script.size(); ++i) {
        cursor += std::max(script[i].duration_s, 0.0);
        if (elapsed_s < cursor) {
            return i;
        }
    }
    return script.empty() ? 0U : (script.size() - 1U);
}

std::string classify_result(const RunStats& stats) {
    if (!stats.saw_controller_ready) {
        return "controller_not_ready";
    }
    if (stats.nonzero_command_cycles == 0) {
        return "no_motion_command_sent";
    }
    if (!stats.saw_pwm_echo) {
        return "command_not_echoed";
    }
    if (!stats.saw_encoder_motion) {
        return "pwm_echoed_but_no_encoder_motion";
    }
    if (stats.total_distance_m < 0.15 && stats.nonzero_command_cycles >= 10) {
        return "weak_motion_detected";
    }
    if (stats.saw_lidar && stats.lidar_frames <= 1) {
        return "drive_ok_but_lidar_missing";
    }
    return "low_level_path_alive";
}

