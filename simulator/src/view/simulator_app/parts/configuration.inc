bool has_tuning_overrides(const VehicleTuningOverrides& overrides) {
    return overrides.min_effective_pwm.has_value() ||
           overrides.speed_estimate_per_pwm.has_value() ||
           overrides.pwm_slew_rate.has_value() ||
           overrides.motor_time_constant.has_value() ||
           overrides.max_linear_speed.has_value() ||
           overrides.max_curvature.has_value() ||
           overrides.max_steer_angle.has_value() ||
           overrides.max_steer_rate.has_value() ||
           overrides.max_yaw_rate.has_value() ||
           overrides.linear_feedback_gain.has_value() ||
           overrides.yaw_feedback_gain.has_value() ||
           overrides.left_pwm_scale.has_value() ||
           overrides.right_pwm_scale.has_value() ||
           overrides.yaw_response_scale.has_value() ||
           overrides.cruise_speed_limit.has_value();
}

void apply_sim_tuning_overrides(PlannerDrivenVehicleSim* sim, const AppOptions& options) {
    if (sim == nullptr || !has_tuning_overrides(options.tuning_overrides)) {
        return;
    }
    sim->set_tuning_overrides(options.tuning_overrides);
}

void fill_structured_tank_ideal_tuning_defaults(VehicleTuningOverrides* tuning) {
    if (tuning == nullptr) {
        return;
    }
    // Best-case tracked response for the structured ideal toggle; mixed maps
    // keep their existing baseline tuning.
    tuning->min_effective_pwm = 35.0;
    tuning->speed_estimate_per_pwm = 0.0018;
    tuning->pwm_slew_rate = 720.0;
    tuning->motor_time_constant = 0.10;
    tuning->max_linear_speed = 0.18;
    tuning->max_curvature = 6.5;
    tuning->max_steer_angle = 1.35;
    tuning->max_steer_rate = 5.5;
    tuning->max_yaw_rate = 1.60;
    tuning->linear_feedback_gain = 60.0;
    tuning->yaw_feedback_gain = 40.0;
    tuning->yaw_response_scale = 1.0;
    tuning->cruise_speed_limit = 0.12;
}

void apply_sim_level_to_config(const AppOptions& options, thesis_sim::SimConfig* config) {
    if (config == nullptr) {
        return;
    }
    config->calibrated_conditions = options.calibrated_simulation;
    if (!options.ideal_simulation) {
        return;
    }
    constexpr double kPi = 3.14159265358979323846;
    config->ideal_conditions = true;
    config->calibrated_conditions = false;
    config->imu_enabled = true;
    config->lidar_enabled = true;
    config->range_sensor_profile = RangeSensorProfile::IdealLidar2D;
    config->lidar_beams = 360;
    config->lidar_fov_rad = 2.0 * kPi;
    config->lidar_range = 12.0;
    config->control_interval_steps = 1;
}

bool structured_preset_is_ideal(StructuredMapPreset preset) {
    return preset == StructuredMapPreset::IdealCircle;
}

bool unstructured_preset_is_ideal(UnstructuredMapPreset preset) {
    return preset == UnstructuredMapPreset::IdealValidation;
}

bool mixed_preset_is_ideal(int preset) {
    return preset == 2;
}

bool mixed_preset_is_dynamic_obstacle(int preset) {
    return preset == 4;
}

bool world_uses_dynamic_obstacle_reveal(const WorldMap& world) {
    return world.environment_mode() == EnvironmentMode::MixedRoadGates &&
           !world.dynamic_obstacles().empty();
}

void apply_dynamic_obstacle_sim_config(const WorldMap& world, thesis_sim::SimConfig* config) {
    if (config == nullptr || config->ideal_conditions || !world_uses_dynamic_obstacle_reveal(world)) {
        return;
    }
    config->lidar_enabled = true;
    config->range_sensor_profile = RangeSensorProfile::RplidarA1;
    config->lidar_range = kDynamicObstacleSimulationLidarRangeM;
    config->cruise_speed_limit = std::min(config->cruise_speed_limit, 0.10);
}

void apply_dynamic_obstacle_sim_config(PlannerDrivenVehicleSim& sim) {
    if (sim.config().ideal_conditions || !world_uses_dynamic_obstacle_reveal(sim.world())) {
        return;
    }
    sim.set_sensor_suite(sim.imu_enabled(), true, RangeSensorProfile::RplidarA1);
    sim.set_lidar_range(kDynamicObstacleSimulationLidarRangeM);
}

void set_canonical_simulation_map_for_mode(UiState* ui_state) {
    if (ui_state == nullptr) {
        return;
    }
    const EnvironmentMode selected_mode = static_cast<EnvironmentMode>(ui_state->environment_mode);
    if (selected_mode == EnvironmentMode::StructuredRoad) {
        ui_state->structured_preset = static_cast<int>(StructuredMapPreset::ValidationRoad);
    } else if (selected_mode == EnvironmentMode::UnstructuredGates) {
        ui_state->unstructured_preset = static_cast<int>(UnstructuredMapPreset::RobotValidation);
    } else if (selected_mode == EnvironmentMode::MixedRoadGates) {
        ui_state->mixed_preset = 1;
    }
}

void set_canonical_hardware_map_for_mode(UiState* ui_state) {
    if (ui_state == nullptr) {
        return;
    }
    const EnvironmentMode selected_mode =
        static_cast<EnvironmentMode>(ui_state->hardware_environment_mode);
    if (selected_mode == EnvironmentMode::StructuredRoad) {
        ui_state->hardware_structured_preset =
            static_cast<int>(StructuredMapPreset::ValidationRoad);
    } else if (selected_mode == EnvironmentMode::UnstructuredGates) {
        ui_state->hardware_unstructured_preset =
            static_cast<int>(UnstructuredMapPreset::HardwareLab);
    } else if (selected_mode == EnvironmentMode::MixedRoadGates) {
        ui_state->hardware_mixed_preset = 1;
    }
}

void apply_gui_stack_for_selected_map(PlannerDrivenVehicleSim& sim, UiState* ui_state) {
    if (ui_state == nullptr) {
        return;
    }

    const EnvironmentMode selected_mode = static_cast<EnvironmentMode>(ui_state->environment_mode);
    if (ui_state->ideal_simulation) {
        ui_state->calibrated_simulation = false;
        if (selected_mode == EnvironmentMode::MixedRoadGates ||
            selected_mode == EnvironmentMode::UnstructuredGates) {
            sim.set_dynamic_lidar_gates(true);
        }
        VehicleTuningOverrides ideal_tuning;
        if (selected_mode == EnvironmentMode::StructuredRoad &&
            static_cast<VehicleModelKind>(ui_state->vehicle_model) == VehicleModelKind::TrackedVehicle) {
            fill_structured_tank_ideal_tuning_defaults(&ideal_tuning);
        }
        sim.set_tuning_overrides(ideal_tuning);
        sim.set_ideal_conditions(true);
    } else {
        if (sim.config().ideal_conditions) {
            sim.set_tuning_overrides(VehicleTuningOverrides{});
            sim.set_ideal_conditions(false);
        }
        if (ui_state->calibrated_simulation) {
            if (!sim.config().calibrated_conditions) {
                sim.set_calibrated_conditions(true);
            }
        } else if (sim.config().calibrated_conditions) {
            sim.set_calibrated_conditions(false);
        }
        if (selected_mode == EnvironmentMode::MixedRoadGates && !sim.config().dynamic_lidar_gates) {
            sim.set_dynamic_lidar_gates(true);
        }
        if (selected_mode == EnvironmentMode::MixedRoadGates &&
            mixed_preset_is_dynamic_obstacle(ui_state->mixed_preset)) {
            apply_dynamic_obstacle_sim_config(sim);
        }
    }

    sync_ui_state_from_sim(ui_state, sim);
    ui_state->paused = true;
    ui_state->report_written = false;
    ui_state->last_report_path.clear();
}

const std::array<ValidationPreset, 7>& validation_presets() {
    static const std::array<ValidationPreset, 7> presets{{
        {"Bicycle structured",
         "Baseline structured-road validation.",
         EnvironmentMode::StructuredRoad,
         UnstructuredMapPreset::RobotValidation,
         StructuredMapPreset::ValidationRoad,
         0,
         VehicleModelKind::CarLikeBicycle,
         false,
         false},
        {"Tank structured",
         "Tracked vehicle on the same structured validation road.",
         EnvironmentMode::StructuredRoad,
         UnstructuredMapPreset::RobotValidation,
         StructuredMapPreset::ValidationRoad,
         0,
         VehicleModelKind::TrackedVehicle,
         false,
         false},
        {"Bicycle mixed",
         "Road/gate arbiter with LiDAR gaps and bicycle support.",
         EnvironmentMode::MixedRoadGates,
         UnstructuredMapPreset::RobotValidation,
         StructuredMapPreset::ValidationRoad,
         0,
         VehicleModelKind::CarLikeBicycle,
         true,
         false},
        {"Bicycle dynamic",
         "Mixed road where obstacles appear online in the LiDAR layer.",
         EnvironmentMode::MixedRoadGates,
         UnstructuredMapPreset::RobotValidation,
         StructuredMapPreset::TankCircuit,
         4,
         VehicleModelKind::CarLikeBicycle,
         true,
         false},
        {"Tank mixed",
         "Tracked vehicle with Sabrina angular primitive on mixed validation.",
         EnvironmentMode::MixedRoadGates,
         UnstructuredMapPreset::RobotValidation,
         StructuredMapPreset::ValidationRoad,
         0,
         VehicleModelKind::TrackedVehicle,
         true,
         false},
        {"Tank hardware lab",
         "Small hardware-aligned mixed map for the cingolato.",
         EnvironmentMode::MixedRoadGates,
         UnstructuredMapPreset::RobotValidation,
         StructuredMapPreset::HardwareTrack,
         5,
         VehicleModelKind::TrackedVehicle,
         true,
         false},
        {"Tank ideal aligned",
         "Ideal sensing plus hardware-aligned mixed map for paper figures.",
         EnvironmentMode::MixedRoadGates,
         UnstructuredMapPreset::IdealValidation,
         StructuredMapPreset::IdealCircle,
         2,
         VehicleModelKind::TrackedVehicle,
         true,
         true},
    }};
    return presets;
}

void apply_validation_preset(PlannerDrivenVehicleSim& sim,
                             UiState* ui_state,
                             const ValidationPreset& preset) {
    if (ui_state == nullptr) {
        return;
    }
    ui_state->environment_mode = static_cast<int>(preset.environment);
    ui_state->unstructured_preset = static_cast<int>(preset.unstructured);
    ui_state->structured_preset = static_cast<int>(preset.structured);
    ui_state->mixed_preset = preset.mixed_preset;
    ui_state->vehicle_model = static_cast<int>(preset.vehicle);
    ui_state->ideal_simulation = preset.ideal;
    ui_state->dynamic_lidar_gates = preset.dynamic_gates;

    WorldMap world = make_world_from_mode(
        preset.environment,
        preset.unstructured,
        preset.structured,
        sim.gate_behavior(),
        sim.gate_seed(),
        preset.mixed_preset);
    world = fit_simulation_structured_world(std::move(world), preset.vehicle);
    sim.load_world(std::move(world));
    sim.set_vehicle_stack(preset.vehicle, sim.tracking_controller_mode());
    if (preset.ideal) {
        VehicleTuningOverrides tuning;
        if (preset.environment == EnvironmentMode::StructuredRoad &&
            preset.vehicle == VehicleModelKind::TrackedVehicle) {
            fill_structured_tank_ideal_tuning_defaults(&tuning);
        }
        sim.set_tuning_overrides(tuning);
        sim.set_ideal_conditions(true);
    } else {
        sim.set_tuning_overrides(VehicleTuningOverrides{});
        sim.set_ideal_conditions(false);
    }
    sim.set_dynamic_lidar_gates(preset.dynamic_gates);
    apply_dynamic_obstacle_sim_config(sim);
    sync_ui_state_from_sim(ui_state, sim);
    ui_state->workspace_source = kWorkspaceSourceSimulation;
    ui_state->workspace_view = kWorkspaceViewMission;
    ui_state->simulation_panel_tab = 0;
    ui_state->requested_simulation_panel_tab = 0;
    ui_state->paused = true;
}

std::string run_vehicle_benchmark(const PlannerDrivenVehicleSim& sim, const UiState& ui_state) {
    auto run_one = [&](VehicleModelKind model) {
        thesis_sim::SimConfig config = sim.config();
        config.vehicle_model = model;
        UiState candidate_ui = ui_state;
        candidate_ui.vehicle_model = static_cast<int>(model);
        PlannerDrivenVehicleSim candidate(world_from_ui_selection(sim, candidate_ui), config);
        const int max_steps =
            ui_state.environment_mode == static_cast<int>(EnvironmentMode::MixedRoadGates) ? 2200 : 1800;
        return candidate.run_headless(max_steps);
    };

    const SimulationReport bicycle = run_one(VehicleModelKind::CarLikeBicycle);
    const SimulationReport tank = run_one(VehicleModelKind::TrackedVehicle);
    auto status = [](const SimulationReport& report) {
        return report.goal_reached ? "goal" : (report.collision ? "collision" : "timeout");
    };

    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    out << "Bicycle: " << status(bicycle)
        << ", t " << bicycle.sim_time
        << " s, goal " << bicycle.distance_to_goal
        << " m | Tank: " << status(tank)
        << ", t " << tank.sim_time
        << " s, goal " << tank.distance_to_goal
        << " m";
    return out.str();
}

const char* map_editor_handle_type_name(MapEditorHandleType type) {
    switch (type) {
        case MapEditorHandleType::Start:
            return "Start";
        case MapEditorHandleType::Goal:
            return "Goal";
        case MapEditorHandleType::Obstacle:
            return "Obstacle";
        case MapEditorHandleType::Gate:
            return "Gate";
        case MapEditorHandleType::RoadPoint:
            return "Road Point";
        default:
            return "None";
    }
}

void draw_slam_reconstruction(ImDrawList* draw_list,
                              const WorldMap& world,
                              const ImVec2& canvas_pos,
                              const ImVec2& canvas_size,
                              const std::vector<Vec2>& slam_points,
                              const std::vector<Vec2>& estimated_trail,
                              const Vec2* robot_position,
                              bool show_grid,
                              const SlamToolboxSnapshot* toolbox_snapshot = nullptr) {
    draw_viewport_background(draw_list, canvas_pos, canvas_size, true);
    draw_list->PushClipRect(
        canvas_pos,
        ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y),
        true);
    const CanvasTransform tx = make_transform(world, canvas_pos, canvas_size);
    if (show_grid) {
        draw_world_grid(draw_list, tx, world.bounds());
    }

    if (!world.road_centerline().empty()) {
        draw_polyline(draw_list, tx, world.road_centerline(), IM_COL32(255, 226, 116, 120), 1.8f);
    }
    for (const Rect& obstacle : world.obstacles()) {
        draw_list->AddRect(
            world_to_screen(tx, {obstacle.min_x, obstacle.max_y}),
            world_to_screen(tx, {obstacle.max_x, obstacle.min_y}),
            IM_COL32(173, 183, 190, 115),
            2.0f,
            0,
            1.2f);
    }

    const size_t stride = slam_points.size() > 30000 ? 3 : (slam_points.size() > 12000 ? 2 : 1);
    for (size_t i = 0; i < slam_points.size(); i += stride) {
        const ImVec2 point = world_to_screen(tx, slam_points[i]);
        draw_list->AddRectFilled(
            ImVec2(point.x - 1.5f, point.y - 1.5f),
            ImVec2(point.x + 1.5f, point.y + 1.5f),
            IM_COL32(220, 240, 233, 235));
    }
    draw_polyline(draw_list, tx, estimated_trail, IM_COL32(88, 193, 255, 235), 2.2f);
    const bool toolbox_active =
        toolbox_snapshot != nullptr && toolbox_snapshot->connected;
    const Vec2* displayed_robot_position = robot_position;
    if (toolbox_active && toolbox_snapshot->pose_valid) {
        displayed_robot_position = &toolbox_snapshot->corrected_position;
    }
    if (displayed_robot_position != nullptr) {
        draw_list->AddCircleFilled(
            world_to_screen(tx, *displayed_robot_position),
            4.2f,
            toolbox_active ? IM_COL32(132, 232, 215, 255) : kColorHeading);
    }

    char point_count[64];
    char graph_status[96];
    std::snprintf(point_count, sizeof(point_count), "%d occupied cells", static_cast<int>(slam_points.size()));
    std::snprintf(
        graph_status,
        sizeof(graph_status),
        toolbox_active ? "scan matching | pose graph active | map updates %d"
                       : "start tools/slam_toolbox_bridge/run.sh for graph SLAM",
        toolbox_active ? toolbox_snapshot->map_updates : 0);
    draw_overlay_panel(
        draw_list,
        ImVec2(canvas_pos.x + 12.0f, canvas_pos.y + 12.0f),
        std::min(canvas_size.x - 24.0f, 310.0f),
        {{toolbox_active ? "SLAM Toolbox" : "LiDAR reconstruction",
          IM_COL32(132, 232, 215, 255)},
         {toolbox_active
              ? "scan matching + loop closure + graph optimization"
              : "estimated-pose endpoint accumulation",
          IM_COL32(213, 223, 223, 235)},
         {graph_status, IM_COL32(160, 177, 181, 235)},
         {point_count, IM_COL32(160, 177, 181, 235)}});
    draw_list->PopClipRect();
}

bool workspace_source_is_hardware(int source) {
    return source == kWorkspaceSourceHardwarePlanner;
}

const char* workspace_source_label(int source) {
    switch (source) {
        case kWorkspaceSourceHardwarePlanner:
            return "Hardware Planner";
        case kWorkspaceSourceSimulation:
        default:
            return "Autonomous Reasoning Engine";
    }
}

const char* unstructured_map_cli_name(UnstructuredMapPreset preset) {
    switch (preset) {
        case UnstructuredMapPreset::TightCorridor:
            return "tight";
        case UnstructuredMapPreset::WideSlalom:
            return "slalom";
        case UnstructuredMapPreset::LowerBypass:
            return "lower";
        case UnstructuredMapPreset::HardwareLab:
            return "hardware_lab";
        case UnstructuredMapPreset::Custom:
            return "custom";
        case UnstructuredMapPreset::RobotValidation:
        default:
            return "robot_validation";
    }
}

const char* structured_map_cli_name(StructuredMapPreset preset) {
    switch (preset) {
        case StructuredMapPreset::CircleLoop:
            return "circle";
        case StructuredMapPreset::ZigZag:
            return "zigzag";
        case StructuredMapPreset::HardwareTrack:
            return "hardware_track";
        case StructuredMapPreset::FigureEight:
            return "figure_eight";
        case StructuredMapPreset::TankCircuit:
            return "tank_circuit";
        case StructuredMapPreset::Custom:
            return "custom";
        case StructuredMapPreset::ValidationRoad:
        default:
            return "validation";
    }
}

std::string stream_profile_label(const std::string& stream_profile) {
    if (stream_profile.empty() || stream_profile == "planner") {
        return "Planner";
    }
    return "Unsupported";
}

VehicleModelKind vehicle_model_from_live_name(const std::string& name) {
    if (name.find("Tracked") != std::string::npos ||
        name.find("tracked") != std::string::npos ||
        name.find("Tank") != std::string::npos ||
        name.find("tank") != std::string::npos) {
        return VehicleModelKind::TrackedVehicle;
    }
    return VehicleModelKind::CarLikeBicycle;
}

std::string hardware_goal_distance_label(const LiveFrameSnapshot& frame, EnvironmentMode mode) {
    char goal_buf[32];
    if (mode == EnvironmentMode::UnstructuredGates &&
        (frame.distance_to_goal < 0.0 || frame.chosen_gate_index < 0 || !frame.planner_has_reference)) {
        if (frame.goal_reached) {
            return "done";
        }
        return frame.dynamic_gap_gates ? "n/a" : "idle";
    }

    std::snprintf(goal_buf, sizeof(goal_buf), "%.2f m", frame.distance_to_goal);
    return goal_buf;
}

std::string hardware_launch_hint(const UiState& ui_state) {
    std::ostringstream cmd;
    const EnvironmentMode mode = static_cast<EnvironmentMode>(ui_state.hardware_environment_mode);
    const bool structured = mode == EnvironmentMode::StructuredRoad;
    const bool mixed = mode == EnvironmentMode::MixedRoadGates;
    bool custom_world = false;
    if (structured) {
        custom_world =
            static_cast<StructuredMapPreset>(ui_state.hardware_structured_preset) == StructuredMapPreset::Custom;
    } else if (!mixed) {
        custom_world =
            static_cast<UnstructuredMapPreset>(ui_state.hardware_unstructured_preset) == UnstructuredMapPreset::Custom;
    }
    cmd << "thesis_robot_runner"
        << " --controller-port /dev/ttyACM0";
    if (!structured) {
        cmd << " --lidar-port /dev/ttyUSB0"
            << " --enable-planner-safety";
    }
    cmd
        << " --stream-host <pc-ip>"
        << " --stream-port " << std::max(ui_state.hardware_listen_port, 1)
        << " --scenario " << environment_mode_slug(mode);
    if (static_cast<VehicleModelKind>(ui_state.hardware_vehicle_model) == VehicleModelKind::TrackedVehicle) {
        cmd << " --vehicle-model tank";
    } else {
        cmd << " --vehicle-model car";
    }
    cmd << " --pose-fusion "
        << thesis_sim::hardware_localization_policy_cli_name(
               static_cast<HardwareLocalizationPolicy>(ui_state.hardware_localization_policy));
    if (custom_world) {
        cmd << " --world-file <copied-custom-map.thmap>";
    } else if (structured) {
        cmd << " --structured-map "
            << structured_map_cli_name(static_cast<StructuredMapPreset>(ui_state.hardware_structured_preset));
    } else if (mixed) {
        cmd << " --mixed-map "
            << hardware_mixed_map_cli_name(ui_state.hardware_mixed_preset);
    } else {
        cmd << " --unstructured-map "
            << unstructured_map_cli_name(static_cast<UnstructuredMapPreset>(ui_state.hardware_unstructured_preset));
    }
    return cmd.str();
}

void load_hardware_editor_from_selection(UiState* ui_state) {
    if (ui_state == nullptr) {
        return;
    }
    const EnvironmentMode mode = static_cast<EnvironmentMode>(ui_state->hardware_environment_mode);
    WorldMap selected_world =
        mode == EnvironmentMode::MixedRoadGates
            ? hardware_mixed_world_from_preset(ui_state->hardware_mixed_preset)
            : make_world_from_mode(
                  mode,
                  static_cast<UnstructuredMapPreset>(ui_state->hardware_unstructured_preset),
                  static_cast<StructuredMapPreset>(ui_state->hardware_structured_preset),
                  GateBehaviorMode::Static,
                  0);
    WorldMap world = apply_hardware_track_scale(*ui_state, std::move(selected_world));
    ui_state->hardware_editor_world = sanitize_hardware_unstructured_world(std::move(world));
    ui_state->hardware_editor_dirty = false;
    reset_editor_interaction(ui_state);
}

void clear_hardware_world_sync(UiState* ui_state,
                               LiveViewStreamServer* hardware_server,
                               const char* status_message = nullptr) {
    if (ui_state == nullptr) {
        return;
    }
    if (hardware_server != nullptr) {
        hardware_server->clear_pending_world();
        hardware_server->clear_pending_robot_profile();
    }
    ui_state->hardware_world_sync_pending = false;
    ui_state->hardware_world_sync_ok = false;
    ui_state->last_hardware_world_sync_status =
        status_message != nullptr ? status_message : "";
}

bool queue_current_hardware_world(UiState* ui_state,
                                  LiveViewStreamServer* hardware_server,
                                  const char* queued_message,
                                  const char* failure_message) {
    if (ui_state == nullptr || hardware_server == nullptr) {
        return false;
    }
    WorldMap streamed_world = hardware_world_from_ui_selection(*ui_state);
    // `hardware_world_from_ui_selection()` already finalizes true custom maps.
    // Avoid finalizing again here, otherwise built-in structured presets such as
    // Hardware Track get converted into StructuredMapPreset::Custom before they
    // are streamed to the Raspberry runner and later saved in reports.
    std::string validation_error;
    if (!validate_hardware_world(streamed_world, &validation_error)) {
        hardware_server->clear_pending_world();
        ui_state->hardware_world_sync_pending = false;
        ui_state->hardware_world_sync_ok = false;
        ui_state->last_hardware_world_error = validation_error;
        ui_state->last_hardware_world_sync_status = validation_error;
        return false;
    }
    if (hardware_server->queue_world(streamed_world)) {
        ui_state->hardware_editor_world = streamed_world;
        ui_state->hardware_editor_dirty = false;
        ui_state->hardware_world_sync_pending = true;
        ui_state->hardware_world_sync_ok = false;
        ui_state->last_hardware_world_sync_status =
            queued_message != nullptr ? queued_message : "Hardware map queued for Raspberry sync.";
        return true;
    }

    ui_state->hardware_world_sync_pending = false;
    ui_state->hardware_world_sync_ok = false;
    ui_state->last_hardware_world_sync_status =
        hardware_server->last_error().empty()
            ? (failure_message != nullptr ? failure_message : "Could not queue the hardware map for Raspberry sync.")
            : hardware_server->last_error();
    return false;
}

bool queue_hardware_robot_profile(UiState* ui_state,
                                  LiveViewStreamServer* hardware_server,
                                  VehicleModelKind vehicle_model,
                                  const char* queued_message,
                                  const char* failure_message) {
    if (ui_state == nullptr || hardware_server == nullptr) {
        return false;
    }
    ui_state->hardware_vehicle_model = static_cast<int>(vehicle_model);
    if (hardware_server->queue_robot_profile(vehicle_model)) {
        ui_state->hardware_world_sync_pending = true;
        ui_state->hardware_world_sync_ok = false;
        ui_state->last_hardware_world_sync_status =
            queued_message != nullptr ? queued_message : "Robot profile queued for Raspberry sync.";
        return true;
    }

    ui_state->hardware_world_sync_pending = false;
    ui_state->hardware_world_sync_ok = false;
    ui_state->last_hardware_world_sync_status =
        hardware_server->last_error().empty()
            ? (failure_message != nullptr ? failure_message : "Could not queue the robot profile for Raspberry sync.")
            : hardware_server->last_error();
    return false;
}

