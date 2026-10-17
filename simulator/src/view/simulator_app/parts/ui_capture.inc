// Opt-in visual smoke capture: SDL's dummy driver can render every workspace
// without a display or a robot. This path never opens a listener or SLAM bridge.
struct UiCapture {
    std::string directory;
    int frame = 0;
    bool active() const { return !directory.empty(); }
    static constexpr int frames_per_view = 4;
    static constexpr int view_count = 22;
    int initial_steps = 0;
    bool check_playback = false;

    UiCapture() {
        if (const char* path = std::getenv("THESIS_UI_CAPTURE_DIR")) directory = path;
        if (active()) std::filesystem::create_directories(directory);
    }
    void prepare(PlannerDrivenVehicleSim& sim, HardwareViewerState* hardware, UiState* state) {
        if (!active()) return;
        if (frame == 0) {
            for (int i = 0; i < 80 && !sim.goal_reached() && !sim.collision(); ++i) sim.step();
            state->paused = true;
            initial_steps = sim.step_count();
            check_playback = !sim.goal_reached() && !sim.collision();
        }
        if (frame < 4 && check_playback) ImGui::GetIO().AddKeyEvent(ImGuiKey_Space, frame % 2 == 0);
        if (frame == 4 && check_playback && !sim.goal_reached() && !sim.collision() && (!state->paused || sim.step_count() <= initial_steps))
            throw std::runtime_error("UI playback shortcut did not run and pause the simulation");
        const int views[] = {kWorkspaceViewMission, kWorkspaceViewMission, kWorkspaceViewSetup,
            kWorkspaceViewAnalytics, kWorkspaceViewMap, kWorkspaceViewDiagnostics, kWorkspaceViewExport,
            kWorkspaceViewMission, kWorkspaceViewSetup, kWorkspaceViewDiagnostics};
        const int index = frame / frames_per_view;
        state->workspace_source = (index >= 7 && index <= 9) || index >= 15 ? kWorkspaceSourceHardwarePlanner : kWorkspaceSourceSimulation;
        state->workspace_view = index < 10 ? views[index] : index < 15 ? kWorkspaceViewAnalytics :
            index == 15 || index == 21 ? kWorkspaceViewMission : index == 20 ? kWorkspaceViewExport : kWorkspaceViewAnalytics;
        if (index >= 10 && index < 15) state->requested_signal_tab = index - 9;
        if (index >= 16 && index <= 19) {
            const int tabs[] = {0, 2, 4, 5};
            state->requested_signal_tab = tabs[index - 16];
        }
        if (index == 15 && frame % frames_per_view == 0) fill_hardware_fixture(sim, hardware, state);
        state->show_slam_map = index == 1 || index == 21;
        state->map_editor_enabled = state->workspace_view == kWorkspaceViewMap;
    }
    void fill_hardware_fixture(const PlannerDrivenVehicleSim& sim, HardwareViewerState* hardware, UiState* state) {
        hardware->has_scene = true;
        hardware->scene.world = sim.world();
        hardware->scene.geometry = sim.geometry();
        hardware->scene.vehicle_model_name = "Car-Like Bicycle";
        hardware->scene.stream_label = "Visual smoke fixture (synthetic)";
        hardware->scene.stream_profile = "planner";
        hardware->scene.localization_mode = "Encoders + IMU";
        hardware->scene.heading_source = "IMU";
        hardware->scene.range_sensor_name = "Synthetic LiDAR";
        state->hardware_environment_mode = static_cast<int>(sim.environment_mode());
        state->last_hardware_frame_at = ImGui::GetTime() - 3.0;
        hardware->history.clear();
        for (const auto& sample : sim.history()) {
            HardwareTelemetrySample h;
#define COPY_UI_SAMPLE(field) h.field = sample.field
            COPY_UI_SAMPLE(time); COPY_UI_SAMPLE(speed); COPY_UI_SAMPLE(target_speed);
            COPY_UI_SAMPLE(yaw_rate); COPY_UI_SAMPLE(target_yaw_rate); COPY_UI_SAMPLE(accel);
            COPY_UI_SAMPLE(jerk); COPY_UI_SAMPLE(curvature); COPY_UI_SAMPLE(command_r);
            COPY_UI_SAMPLE(planner_speed_ref); COPY_UI_SAMPLE(tracker_cross_track);
            COPY_UI_SAMPLE(tracker_heading_error_deg); COPY_UI_SAMPLE(planning_ms);
            COPY_UI_SAMPLE(tracking_ms); COPY_UI_SAMPLE(step_ms); COPY_UI_SAMPLE(min_lidar);
            COPY_UI_SAMPLE(distance_to_goal); COPY_UI_SAMPLE(passed_gates); COPY_UI_SAMPLE(planner_has_reference);
#undef COPY_UI_SAMPLE
            h.position_x = sample.x; h.position_y = sample.y; h.yaw = sample.yaw;
            h.front_lidar = sample.min_lidar;
            h.pwm_left = h.controller_pwm_left = h.controller_target_pwm_left = static_cast<int>(sample.left_pwm);
            h.pwm_right = h.controller_pwm_right = h.controller_target_pwm_right = static_cast<int>(sample.right_pwm);
            h.controller_left_encoder_delta = static_cast<int>(sample.left_encoder_delta);
            h.controller_right_encoder_delta = static_cast<int>(sample.right_encoder_delta);
            h.controller_encoder_dt_ms = 50;
            hardware->history.push_back(h);
        }
        auto& f = hardware->frame;
        f.has_latest_sample = !hardware->history.empty();
        if (f.has_latest_sample) f.latest_sample = hardware->history.back();
        f.step_count = sim.step_count(); f.sim_time = sim.sim_time();
        f.telemetry_ready = true; f.planner_has_reference = !sim.planned_trajectory().empty();
        f.navigation_position = f.vehicle.position = sim.navigation_position();
        f.navigation_yaw = f.vehicle.yaw = sim.navigation_yaw();
        f.navigation_speed = f.vehicle.speed = sim.vehicle().speed;
        f.tracker_cross_track_error = sim.tracker_cross_track_error();
        f.tracker_heading_error_deg = sim.tracker_heading_error_deg();
        f.control_source = sim.environment_mode() == EnvironmentMode::StructuredRoad
            ? thesis_sim::HardwareControlSource::StructuredMpc : thesis_sim::HardwareControlSource::GateMpc;
        f.distance_to_goal = sim.distance_to_goal(); f.passed_gates = sim.passed_gate_count();
        f.front_lidar_distance = sim.min_lidar_distance();
        f.lidar_hits = sim.lidar_hits(); f.valid_lidar_samples = static_cast<int>(f.lidar_hits.size());
        f.global_occupied_points = sim.slam_points();
        f.trail = sim.estimated_trail(); f.planned_trajectory = sim.planned_trajectory();
        f.vehicle.left_encoder_delta = f.latest_sample.controller_left_encoder_delta;
        f.vehicle.right_encoder_delta = f.latest_sample.controller_right_encoder_delta;
    }
    bool capture(SDL_Renderer* renderer) {
        if (!active()) return false;
        if (frame % frames_per_view == frames_per_view - 1) {
            int w = 0, h = 0;
            SDL_GetRendererOutputSize(renderer, &w, &h);
            SDL_Surface* surface = SDL_CreateRGBSurfaceWithFormat(0, w, h, 32, SDL_PIXELFORMAT_ARGB8888);
            if (!surface) throw std::runtime_error(SDL_GetError());
            const std::string path = directory + "/view_" + std::to_string(frame / frames_per_view) + ".bmp";
            const bool ok = SDL_RenderReadPixels(renderer, nullptr, SDL_PIXELFORMAT_ARGB8888, surface->pixels, surface->pitch) == 0 &&
                SDL_SaveBMP(surface, path.c_str()) == 0;
            SDL_FreeSurface(surface);
            if (!ok) throw std::runtime_error(SDL_GetError());
            std::cout << "ui_capture=" << path << '\n';
        }
        return ++frame >= frames_per_view * view_count;
    }
};
