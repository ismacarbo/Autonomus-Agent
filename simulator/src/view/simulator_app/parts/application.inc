int run_headless(const AppOptions& options) {
    thesis_sim::SimConfig sim_config;
    sim_config.vehicle_model = options.vehicle_model;
    sim_config.dynamic_lidar_gates = options.dynamic_lidar_gates;
    apply_sim_level_to_config(options, &sim_config);
    WorldMap world = make_world_from_mode(
        options.environment_mode,
        options.unstructured_preset,
        options.structured_preset,
        GateBehaviorMode::Static,
        7,
        options.mixed_preset);
    world = fit_simulation_structured_world(std::move(world), options.vehicle_model);
    apply_dynamic_obstacle_sim_config(world, &sim_config);
    PlannerDrivenVehicleSim sim(std::move(world), sim_config);
    apply_sim_tuning_overrides(&sim, options);
    const SimulationReport report = sim.run_headless(options.max_steps);
    const std::string status = report.goal_reached ? "goal_reached" : (report.collision ? "collision" : "timeout");
    const std::string report_path = default_report_path(sim, "headless");
    const bool report_written = write_json_report(sim, status, report_path);

    std::cout << "status=" << status << '\n';
    std::cout << "steps=" << report.steps << '\n';
    std::cout << "time=" << report.sim_time << '\n';
    std::cout << "final_x=" << report.final_position.x << '\n';
    std::cout << "final_y=" << report.final_position.y << '\n';
    std::cout << "distance_to_goal=" << report.distance_to_goal << '\n';
    std::cout << "passed_gates=" << report.passed_gates << '\n';
    if (report_written) {
        std::cout << "report_json=" << report_path << '\n';
    }

    return report.goal_reached ? 0 : 1;
}

int run_gui(const AppOptions& options) {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_TIMER) != 0) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << '\n';
        return 1;
    }

    SDL_Window* window = SDL_CreateWindow(
        "Autonomous Reasoning Engine",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        1500,
        920,
        SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);

    if (window == nullptr) {
        std::cerr << "SDL_CreateWindow failed: " << SDL_GetError() << '\n';
        SDL_Quit();
        return 1;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(
        window,
        -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

    if (renderer == nullptr) {
        std::cerr << "SDL_CreateRenderer failed: " << SDL_GetError() << '\n';
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImPlot::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    io.ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    apply_style();

    ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer2_Init(renderer);

    thesis_sim::SimConfig sim_config;
    sim_config.vehicle_model = options.vehicle_model;
    sim_config.dynamic_lidar_gates = options.dynamic_lidar_gates;
    apply_sim_level_to_config(options, &sim_config);
    WorldMap world = make_world_from_mode(
        options.environment_mode,
        options.unstructured_preset,
        options.structured_preset,
        GateBehaviorMode::Static,
        7,
        options.mixed_preset);
    world = fit_simulation_structured_world(std::move(world), options.vehicle_model);
    apply_dynamic_obstacle_sim_config(world, &sim_config);
    PlannerDrivenVehicleSim sim(std::move(world), sim_config);
    apply_sim_tuning_overrides(&sim, options);
    HardwareViewerState hardware_view;
    LiveViewStreamServer hardware_server;
    SlamToolboxBridgeClient slam_bridge;
    slam_bridge.open();
    std::uint64_t slam_sequence = 0;
    int last_slam_sim_step = -1;
    int last_slam_hardware_step = -1;
    bool running = true;
    UiState ui_state;
    sync_ui_state_from_sim(&ui_state, sim);

    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT) {
                running = false;
            }
        }

        const LiveViewStreamServer::PollResult hardware_updates = hardware_server.poll();
        const bool hardware_stream_connected = hardware_server.connected();
        if (hardware_stream_connected && !ui_state.hardware_stream_connected_prev) {
            queue_current_hardware_world(
                &ui_state,
                &hardware_server,
                "Current hardware world detected and queued automatically for the newly connected Raspberry runner. Waiting for runner ack.",
                "Could not auto-queue the current hardware world for the newly connected Raspberry runner.");
            queue_hardware_robot_profile(
                &ui_state,
                &hardware_server,
                static_cast<VehicleModelKind>(ui_state.hardware_vehicle_model),
                "Current robot support queued automatically for the newly connected Raspberry runner. Waiting for runner ack.",
                "Could not auto-queue the robot support profile for the Raspberry runner.");
        }
        ui_state.hardware_stream_connected_prev = hardware_stream_connected;
        if (hardware_updates.scene_received && hardware_updates.scene.has_value()) {
            hardware_view.scene = *hardware_updates.scene;
            hardware_view.has_scene = true;
            ui_state.hardware_vehicle_model =
                static_cast<int>(vehicle_model_from_live_name(hardware_view.scene.vehicle_model_name));
            hardware_view.frame = {};
            hardware_view.history.clear();
            hardware_view.lidar_reconstruction.clear();
            hardware_view.last_lidar_reconstruction_step = -1;
            ui_state.hardware_report_written = false;
            ui_state.last_hardware_report_path.clear();
            ui_state.last_hardware_report_error.clear();
            ui_state.last_hardware_lidar_reconstruction_csv_path.clear();
            ui_state.last_hardware_lidar_reconstruction_ply_path.clear();
            ui_state.last_hardware_lidar_reconstruction_png_path.clear();
        }
        if (hardware_updates.frame_received && hardware_updates.frame.has_value()) {
            hardware_view.frame = *hardware_updates.frame;
            append_lidar_reconstruction_frame(&hardware_view);
            if (hardware_view.frame.has_latest_sample) {
                const bool duplicate =
                    !hardware_view.history.empty() &&
                    std::abs(hardware_view.history.back().time - hardware_view.frame.latest_sample.time) < 1e-9;
                if (!duplicate) {
                    hardware_view.history.push_back(hardware_view.frame.latest_sample);
                }
                constexpr int kMaxHardwareHistory = 2400;
                if (static_cast<int>(hardware_view.history.size()) > kMaxHardwareHistory) {
                    hardware_view.history.erase(hardware_view.history.begin());
                }
            }
        }
        if (hardware_updates.control_ack_received && hardware_updates.control_ack.has_value()) {
            ui_state.hardware_world_sync_pending = false;
            ui_state.hardware_world_sync_ok = hardware_updates.control_ack->ok;
            ui_state.last_hardware_world_sync_status = hardware_updates.control_ack->message;
        }

        if (ui_state.workspace_source == kWorkspaceSourceSimulation && !ui_state.paused && !sim.goal_reached() && !sim.collision()) {
            for (int i = 0; i < ui_state.steps_per_frame; ++i) {
                sim.step();
                if (sim.goal_reached() || sim.collision()) {
                    break;
                }
            }
        } else if (ui_state.workspace_source == kWorkspaceSourceSimulation && ui_state.single_step) {
            sim.step();
            ui_state.single_step = false;
        }

        if (ui_state.workspace_source == kWorkspaceSourceSimulation &&
            sim.step_count() != last_slam_sim_step &&
            !sim.lidar_hits().empty()) {
            slam_bridge.submit_scan(
                "simulation",
                ++slam_sequence,
                sim.sim_time(),
                sim.navigation_position(),
                sim.navigation_yaw(),
                sim.lidar_hits(),
                0.05,
                std::max(sim.active_lidar_range(), 0.10));
            last_slam_sim_step = sim.step_count();
        } else if (ui_state.workspace_source != kWorkspaceSourceSimulation &&
                   hardware_view.has_scene &&
                   hardware_view.frame.step_count != last_slam_hardware_step &&
                   !hardware_view.frame.lidar_hits.empty()) {
            slam_bridge.submit_scan(
                "hardware",
                ++slam_sequence,
                hardware_view.frame.sim_time,
                hardware_view.frame.navigation_position,
                hardware_view.frame.navigation_yaw,
                hardware_view.frame.lidar_hits,
                0.05,
                std::max(hardware_view.scene.active_lidar_range, 0.10));
            last_slam_hardware_step = hardware_view.frame.step_count;
        }
        slam_bridge.poll();

        if ((sim.goal_reached() || sim.collision()) && !ui_state.report_written) {
            const std::string report_path = default_report_path(sim, "gui_auto");
            if (write_json_report(sim, report_status_string(sim), report_path)) {
                ui_state.report_written = true;
                ui_state.last_report_path = report_path;
            }
        }

        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        render_workspace(
            sim,
            &hardware_view,
            &ui_state,
            &hardware_server,
            &slam_bridge.snapshot());

        ImGui::Render();
        SDL_SetRenderDrawColor(renderer, 9, 12, 15, 255);
        SDL_RenderClear(renderer);
        ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer);
        SDL_RenderPresent(renderer);
    }

    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImPlot::DestroyContext();
    ImGui::DestroyContext();
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}

}  // namespace

int main(int argc, char** argv) {
    const AppOptions options = parse_app_options(argc, argv);
    if (options.headless) {
        return run_headless(options);
    }
    return run_gui(options);
}
