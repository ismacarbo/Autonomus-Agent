void print_usage(const char* argv0) {
    std::cout
        << "Usage: " << argv0 << " --controller-port /dev/ttyACM0 --lidar-port /dev/ttyUSB0 [options]\n"
        << "Or:    " << argv0 << " --simulate [options]\n"
        << "Options:\n"
        << "  --controller-baudrate N   default 115200\n"
        << "  --lidar-baudrate N        default 115200\n"
        << "  --dt SEC                  default 0.10\n"
        << "  --forward-pwm N           default 150\n"
        << "  --turn-pwm N              default 120\n"
        << "  --forward-seconds SEC     default 2.0\n"
        << "  --turn-seconds SEC        default 1.0\n"
        << "  --stop-seconds SEC        default 0.35\n"
        << "  --loops N                 default 3\n"
        << "  --obstacle-stop M         default 0.32\n"
        << "  --map-resolution M        default 0.05\n"
        << "  --map-size M              default 8.0\n"
        << "  --max-range M             default 3.5\n"
        << "  --min-range M             default 0.12\n"
        << "  --track-width M           default 0.28\n"
        << "  --wheel-radius M          default 0.04\n"
        << "  --ticks-per-rev N         default 360\n"
        << "  --min-effective-pwm N     default 55\n"
        << "  --speed-per-pwm V         default 0.0048\n"
        << "  --lidar-x M               default 0.0\n"
        << "  --lidar-y M               default 0.0\n"
        << "  --lidar-yaw-deg DEG       default 0.0\n"
        << "  --stream-host HOST        send live viewer updates to HOST\n"
        << "  --stream-port N           send live viewer updates to port N\n"
        << "  --stream-every N          send one live frame every N cycles (default 1)\n"
        << "  --stream-world-size M     default 8.0\n"
        << "  --scenario MODE           structured | unstructured\n"
        << "  --structured-map NAME     validation | circle | zigzag | hardware_track | figure_eight | tank_circuit\n"
        << "  --unstructured-map NAME   robot_validation | tight | slalom | lower | hardware_lab\n"
        << "  --world-file PATH         load a custom exported `.thmap` world file\n"
        << "  --infinite                keep the SLAM session online and loop the motion script forever\n"
        << "  --once                    run the script once even when streaming to the GUI\n"
        << "  --output-prefix PATH      default hardware_smoke_test\n"
        << "  --simulate                run the smoke test against a synthetic compact track\n"
        << "  --no-auto-mode            do not force AUTONOMOUS mode on connect\n"
        << "  --no-gyro-zero            do not send GYRO_ZERO on connect\n";
}

TestOptions parse_args(int argc, char** argv) {
    TestOptions options;
    bool finite_requested = false;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--controller-port" && i + 1 < argc) {
            options.controller_port = argv[++i];
        } else if (arg == "--lidar-port" && i + 1 < argc) {
            options.lidar_port = argv[++i];
        } else if (arg == "--controller-baudrate" && i + 1 < argc) {
            options.controller_baudrate = std::atoi(argv[++i]);
        } else if (arg == "--lidar-baudrate" && i + 1 < argc) {
            options.lidar_baudrate = std::atoi(argv[++i]);
        } else if (arg == "--dt" && i + 1 < argc) {
            options.dt = std::atof(argv[++i]);
        } else if (arg == "--forward-pwm" && i + 1 < argc) {
            options.forward_pwm = std::atoi(argv[++i]);
        } else if (arg == "--turn-pwm" && i + 1 < argc) {
            options.turn_pwm = std::atoi(argv[++i]);
        } else if (arg == "--forward-seconds" && i + 1 < argc) {
            options.forward_seconds = std::atof(argv[++i]);
        } else if (arg == "--turn-seconds" && i + 1 < argc) {
            options.turn_seconds = std::atof(argv[++i]);
        } else if (arg == "--stop-seconds" && i + 1 < argc) {
            options.stop_seconds = std::atof(argv[++i]);
        } else if (arg == "--loops" && i + 1 < argc) {
            options.loops = std::atoi(argv[++i]);
        } else if (arg == "--obstacle-stop" && i + 1 < argc) {
            options.obstacle_stop_distance_m = std::atof(argv[++i]);
        } else if (arg == "--map-resolution" && i + 1 < argc) {
            options.map.resolution_m = std::atof(argv[++i]);
        } else if (arg == "--map-size" && i + 1 < argc) {
            options.map.size_m = std::atof(argv[++i]);
        } else if (arg == "--max-range" && i + 1 < argc) {
            options.map.lidar_max_range_m = std::atof(argv[++i]);
        } else if (arg == "--min-range" && i + 1 < argc) {
            options.map.lidar_min_range_m = std::atof(argv[++i]);
        } else if (arg == "--track-width" && i + 1 < argc) {
            options.drive.track_width = std::atof(argv[++i]);
        } else if (arg == "--wheel-radius" && i + 1 < argc) {
            options.drive.wheel_radius = std::atof(argv[++i]);
        } else if (arg == "--ticks-per-rev" && i + 1 < argc) {
            options.drive.encoder_ticks_per_revolution = std::atoi(argv[++i]);
        } else if (arg == "--min-effective-pwm" && i + 1 < argc) {
            options.drive.min_effective_pwm = std::atoi(argv[++i]);
        } else if (arg == "--speed-per-pwm" && i + 1 < argc) {
            options.drive.speed_estimate_per_pwm = std::atof(argv[++i]);
        } else if (arg == "--lidar-x" && i + 1 < argc) {
            options.map.lidar_x_offset = std::atof(argv[++i]);
        } else if (arg == "--lidar-y" && i + 1 < argc) {
            options.map.lidar_y_offset = std::atof(argv[++i]);
        } else if (arg == "--lidar-yaw-deg" && i + 1 < argc) {
            options.map.lidar_yaw_offset_rad = deg_to_rad(std::atof(argv[++i]));
        } else if (arg == "--stream-host" && i + 1 < argc) {
            options.stream.host = argv[++i];
        } else if (arg == "--stream-port" && i + 1 < argc) {
            options.stream.port = std::atoi(argv[++i]);
        } else if (arg == "--stream-every" && i + 1 < argc) {
            options.stream.every_n_steps = std::atoi(argv[++i]);
        } else if (arg == "--stream-world-size" && i + 1 < argc) {
            options.stream.world_size_m = std::atof(argv[++i]);
        } else if (arg == "--scenario" && i + 1 < argc) {
            options.environment_mode = parse_environment_mode(argv[++i]);
        } else if (arg.rfind("--scenario=", 0) == 0) {
            options.environment_mode = parse_environment_mode(arg.substr(std::strlen("--scenario=")));
        } else if (arg == "--structured-map" && i + 1 < argc) {
            options.structured_preset = parse_structured_preset(argv[++i]);
        } else if (arg.rfind("--structured-map=", 0) == 0) {
            options.structured_preset = parse_structured_preset(arg.substr(std::strlen("--structured-map=")));
        } else if (arg == "--unstructured-map" && i + 1 < argc) {
            options.unstructured_preset = parse_unstructured_preset(argv[++i]);
        } else if (arg.rfind("--unstructured-map=", 0) == 0) {
            options.unstructured_preset = parse_unstructured_preset(arg.substr(std::strlen("--unstructured-map=")));
        } else if (arg == "--world-file" && i + 1 < argc) {
            options.world_file = argv[++i];
        } else if (arg.rfind("--world-file=", 0) == 0) {
            options.world_file = arg.substr(std::strlen("--world-file="));
        } else if (arg == "--infinite") {
            options.infinite = true;
        } else if (arg == "--once" || arg == "--finite") {
            options.infinite = false;
            finite_requested = true;
        } else if (arg == "--output-prefix" && i + 1 < argc) {
            options.output_prefix = argv[++i];
        } else if (arg == "--simulate") {
            options.simulate = true;
        } else if (arg == "--no-auto-mode") {
            options.auto_mode = false;
        } else if (arg == "--no-gyro-zero") {
            options.gyro_zero = false;
        } else if (arg == "--help" || arg == "-h") {
            print_usage(argv[0]);
            std::exit(0);
        }
    }

    options.dt = std::max(options.dt, 0.03);
    options.loops = std::max(options.loops, 1);
    options.forward_pwm = std::clamp(options.forward_pwm, -255, 255);
    options.turn_pwm = std::clamp(options.turn_pwm, -255, 255);
    options.map.lidar_min_points = std::max(options.map.lidar_min_points, 20);
    options.map.lidar_downsample = std::max(options.map.lidar_downsample, 1);
    options.drive.encoder_ticks_per_revolution =
        std::max(options.drive.encoder_ticks_per_revolution, 1);
    options.stream.every_n_steps = std::max(options.stream.every_n_steps, 1);
    options.stream.world_size_m = std::max(options.stream.world_size_m, 2.0);
    if (stream_enabled(options) && !finite_requested) {
        options.infinite = true;
    }
    return options;
}

void write_trail_csv(const std::string& path, const std::vector<Pose2D>& trail) {
    std::ofstream out(path);
    if (!out.is_open()) {
        return;
    }
    out << "index,x_m,y_m,yaw_deg\n";
    for (std::size_t i = 0; i < trail.size(); ++i) {
        out << i << ',' << trail[i].x << ',' << trail[i].y << ',' << rad_to_deg(trail[i].yaw) << '\n';
    }
}

void write_summary(const std::string& path,
                   const RunStats& stats,
                   const Pose2D& final_pose,
                   const ControllerTelemetry& last_telemetry,
                   bool have_last_telemetry,
                   std::size_t streamed_map_points) {
    std::ofstream out(path);
    if (!out.is_open()) {
        return;
    }
    out << "result=" << classify_result(stats) << '\n';
    out << "cycles=" << stats.cycles << '\n';
    out << "nonzero_command_cycles=" << stats.nonzero_command_cycles << '\n';
    out << "pwm_echo_cycles=" << stats.pwm_echo_cycles << '\n';
    out << "encoder_motion_cycles=" << stats.encoder_motion_cycles << '\n';
    out << "blocked_cycles=" << stats.blocked_cycles << '\n';
    out << "lidar_frames=" << stats.lidar_frames << '\n';
    out << "scan_match_updates=" << stats.scan_match_updates << '\n';
    out << "total_distance_m=" << stats.total_distance_m << '\n';
    out << "total_abs_left_ticks=" << stats.total_abs_left_ticks << '\n';
    out << "total_abs_right_ticks=" << stats.total_abs_right_ticks << '\n';
    out << "min_front_distance_m="
        << (std::isfinite(stats.min_front_distance_m) ? stats.min_front_distance_m : 0.0) << '\n';
    out << "final_x_m=" << final_pose.x << '\n';
    out << "final_y_m=" << final_pose.y << '\n';
    out << "final_yaw_deg=" << rad_to_deg(final_pose.yaw) << '\n';
    out << "streamed_map_points=" << streamed_map_points << '\n';
    if (have_last_telemetry) {
        out << "last_status_flags=" << last_telemetry.status_flags << '\n';
        out << "last_motor_flags=" << last_telemetry.motor_flags << '\n';
        out << "last_safety_flags=" << last_telemetry.safety_flags << '\n';
        out << "last_error_code=" << last_telemetry.error_code << '\n';
        out << "last_pwm_left=" << last_telemetry.pwm_l << '\n';
        out << "last_pwm_right=" << last_telemetry.pwm_r << '\n';
    }
}

