void render_run_quality_cards(const RunQualitySummary& quality, const char* id_prefix) {
    char runtime_buf[32];
    char clearance_buf[32];
    char tracking_buf[32];
    char compute_buf[32];
    std::snprintf(runtime_buf, sizeof(runtime_buf), "%.1f s", quality.duration_s);
    std::snprintf(clearance_buf, sizeof(clearance_buf), "%.2f m", quality.min_lidar);
    std::snprintf(tracking_buf, sizeof(tracking_buf), "%.2f m", quality.max_cross_track);
    std::snprintf(compute_buf, sizeof(compute_buf), "%.1f / %.1f ms", quality.avg_step_ms, quality.max_step_ms);

    if (ImGui::BeginTable((std::string(id_prefix) + "_quality_cards").c_str(), 4, ImGuiTableFlags_SizingStretchSame)) {
        ImGui::TableNextColumn();
        metric_card((std::string(id_prefix) + "_runtime").c_str(), "Runtime", runtime_buf, "Run duration", ImVec4(0.43f, 0.82f, 0.96f, 1.0f), 56.0f);
        ImGui::TableNextColumn();
        metric_card((std::string(id_prefix) + "_clearance").c_str(), "Clearance", clearance_buf, "Minimum LiDAR distance", ImVec4(0.48f, 0.88f, 0.62f, 1.0f), 56.0f);
        ImGui::TableNextColumn();
        metric_card((std::string(id_prefix) + "_tracking").c_str(), "Max CTE", tracking_buf, "Worst cross-track error", ImVec4(0.97f, 0.89f, 0.45f, 1.0f), 56.0f);
        ImGui::TableNextColumn();
        metric_card((std::string(id_prefix) + "_compute").c_str(), "Step ms", compute_buf, "Average / max", ImVec4(0.96f, 0.66f, 0.28f, 1.0f), 56.0f);
        ImGui::EndTable();
    }
}

void render_sim_mission_readiness(const PlannerDrivenVehicleSim& sim) {
    const RunQualitySummary quality = summarize_run_quality(sim.history());
    ImGui::SeparatorText("Mission Readiness");
    if (!quality.has_data) {
        ImGui::TextDisabled("Run or step the simulation to populate validation metrics.");
    } else {
        render_run_quality_cards(quality, "sim_ready");
    }
    ImGui::Spacing();
    ImGui::SeparatorText("Execution Context");
    const Rect& bounds = sim.world().bounds();
    char arena_buf[48];
    char footprint_buf[48];
    std::snprintf(arena_buf,
                  sizeof(arena_buf),
                  "%.2f x %.2f m",
                  bounds.max_x - bounds.min_x,
                  bounds.max_y - bounds.min_y);
    std::snprintf(footprint_buf,
                  sizeof(footprint_buf),
                  "%.2f x %.2f m",
                  sim.geometry().body_length,
                  sim.geometry().body_width);
    const char* fidelity = sim.config().ideal_conditions
                               ? "Sim-Ideal"
                               : (sim.config().calibrated_conditions
                                      ? "Sim-Calibrated"
                                      : "Sim-Reference");
    status_line("Validation level",
                fidelity,
                sim.config().calibrated_conditions
                    ? ImVec4(0.96f, 0.66f, 0.28f, 1.0f)
                    : ImVec4(0.43f, 0.82f, 0.96f, 1.0f));
    status_line("Arena", arena_buf, ImVec4(0.48f, 0.88f, 0.62f, 1.0f));
    status_line("Robot footprint", footprint_buf, ImVec4(0.87f, 0.79f, 0.39f, 1.0f));
    if (sim.environment_mode() == EnvironmentMode::MixedRoadGates) {
        const bool gate_active =
            !sim.history().empty() && sim.history().back().mixed_mode > 0.5;
        status_line("Mixed arbiter",
                    gate_active ? "gate bypass" : "structured road",
                    gate_active ? ImVec4(0.96f, 0.66f, 0.28f, 1.0f)
                                : ImVec4(0.48f, 0.88f, 0.62f, 1.0f));
    }
    const bool tank = sim.vehicle_model_kind() == VehicleModelKind::TrackedVehicle;
    status_line("Vehicle model",
                thesis_sim::vehicle_model_kind_name(sim.vehicle_model_kind()),
                tank ? ImVec4(0.96f, 0.66f, 0.28f, 1.0f) : ImVec4(0.43f, 0.82f, 0.96f, 1.0f));
    status_line("Planner primitive",
                "j longitudinal jerk + r rotational jerk",
                tank ? ImVec4(0.96f, 0.66f, 0.28f, 1.0f) : ImVec4(0.48f, 0.88f, 0.62f, 1.0f));
    status_line("MPC layer",
                tank ? "MPC path tracking -> differential tracks"
                     : "MPC path tracking -> bicycle steering",
                ImVec4(0.48f, 0.88f, 0.62f, 1.0f));
    status_line("Dynamic gates",
                sim.config().dynamic_lidar_gates ? "enabled" : "disabled",
                sim.config().dynamic_lidar_gates ? ImVec4(0.48f, 0.88f, 0.62f, 1.0f)
                                                  : ImVec4(0.87f, 0.79f, 0.39f, 1.0f));
}

void render_validation_preset_grid(PlannerDrivenVehicleSim& sim, UiState* ui_state) {
    if (ui_state == nullptr) {
        return;
    }
    ImGui::SeparatorText("Validation Presets");
    const auto& presets = validation_presets();
    if (ImGui::BeginTable("ValidationPresetGrid", 2, ImGuiTableFlags_SizingStretchSame | ImGuiTableFlags_BordersInnerV)) {
        for (size_t i = 0; i < presets.size(); ++i) {
            const ValidationPreset& preset = presets[i];
            ImGui::TableNextColumn();
            ImGui::PushID(static_cast<int>(i));
            const bool current =
                sim.environment_mode() == preset.environment &&
                sim.vehicle_model_kind() == preset.vehicle &&
                sim.config().dynamic_lidar_gates == preset.dynamic_gates &&
                sim.config().ideal_conditions == preset.ideal &&
                (preset.environment != EnvironmentMode::MixedRoadGates || ui_state->mixed_preset == preset.mixed_preset);
            if (current) {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.43f, 0.52f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.22f, 0.56f, 0.66f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.11f, 0.33f, 0.40f, 1.0f));
            }
            if (ImGui::Button(preset.name, ImVec2(-1.0f, 0.0f))) {
                apply_validation_preset(sim, ui_state, preset);
            }
            if (current) {
                ImGui::PopStyleColor(3);
            }
            ImGui::TextWrapped("%s", preset.detail);
            ImGui::PopID();
        }
        ImGui::EndTable();
    }
    if (ImGui::Button("Run Bicycle / Tank Benchmark", ImVec2(-1.0f, 0.0f))) {
        ui_state->last_validation_benchmark = run_vehicle_benchmark(sim, *ui_state);
    }
    if (!ui_state->last_validation_benchmark.empty()) {
        ImGui::TextWrapped("%s", ui_state->last_validation_benchmark.c_str());
    }
}

void render_hardware_readiness_panel(const HardwareViewerState& hardware,
                                     const UiState& ui_state,
                                     const LiveViewStreamServer& hardware_server) {
    ImGui::SeparatorText("Hardware Readiness");
    const VehicleModelKind configured_model =
        static_cast<VehicleModelKind>(ui_state.hardware_vehicle_model);
    const bool tank_configured = configured_model == VehicleModelKind::TrackedVehicle;
    const bool live_tank =
        hardware.has_scene &&
        vehicle_model_from_live_name(hardware.scene.vehicle_model_name) == VehicleModelKind::TrackedVehicle;
    status_line("Selected robot",
                thesis_sim::vehicle_model_kind_name(configured_model),
                tank_configured ? ImVec4(0.96f, 0.66f, 0.28f, 1.0f)
                                : ImVec4(0.43f, 0.82f, 0.96f, 1.0f));
    status_line("Runner model",
                hardware.has_scene ? hardware.scene.vehicle_model_name.c_str() : "waiting scene",
                hardware.has_scene
                    ? (live_tank == tank_configured ? ImVec4(0.48f, 0.88f, 0.62f, 1.0f)
                                                    : ImVec4(0.95f, 0.60f, 0.34f, 1.0f))
                    : ImVec4(0.87f, 0.79f, 0.39f, 1.0f));
    status_line("Profile sync",
                hardware_server.has_pending_robot_profile() ? "queued" : "idle",
                hardware_server.has_pending_robot_profile() ? ImVec4(0.43f, 0.82f, 0.96f, 1.0f)
                                                            : ImVec4(0.48f, 0.88f, 0.62f, 1.0f));
    status_line("Map sync",
                hardware_server.has_pending_world() ? "queued" : "idle",
                hardware_server.has_pending_world() ? ImVec4(0.43f, 0.82f, 0.96f, 1.0f)
                                                    : ImVec4(0.48f, 0.88f, 0.62f, 1.0f));
    status_line("Connection",
                hardware_server.connected() ? "connected" : (hardware_server.listening() ? "listening" : "off"),
                hardware_server.connected() ? ImVec4(0.48f, 0.88f, 0.62f, 1.0f)
                                            : (hardware_server.listening() ? ImVec4(0.87f, 0.79f, 0.39f, 1.0f)
                                                                          : ImVec4(0.95f, 0.60f, 0.34f, 1.0f)));
    if (hardware.has_scene) {
        const Rect& bounds = hardware.scene.world.bounds();
        char arena_buf[48];
        char footprint_buf[48];
        std::snprintf(arena_buf,
                      sizeof(arena_buf),
                      "%.2f x %.2f m",
                      bounds.max_x - bounds.min_x,
                      bounds.max_y - bounds.min_y);
        std::snprintf(footprint_buf,
                      sizeof(footprint_buf),
                      "%.2f x %.2f m",
                      hardware.scene.geometry.body_length,
                      hardware.scene.geometry.body_width);
        status_line("Live arena", arena_buf, ImVec4(0.48f, 0.88f, 0.62f, 1.0f));
        status_line("Live footprint", footprint_buf, ImVec4(0.87f, 0.79f, 0.39f, 1.0f));
        if (hardware.scene.world.environment_mode() == EnvironmentMode::MixedRoadGates) {
            const bool gate_active = hardware.frame.chosen_gate_index >= 0;
            status_line("Mixed arbiter",
                        gate_active ? "gate bypass" : "structured road",
                        gate_active ? ImVec4(0.96f, 0.66f, 0.28f, 1.0f)
                                    : ImVec4(0.48f, 0.88f, 0.62f, 1.0f));
        }
    }
    if (tank_configured) {
        ImGui::TextWrapped("Tracked hardware path: shared j/r planner -> differential MPC path follower -> speed/yaw-rate targets -> track-speed and PWM mapping.");
    }
}

void render_hardware_workflow_strip(UiState* ui_state) {
    if (ui_state == nullptr) {
        return;
    }
    const struct Step {
        const char* label;
        int tab;
        int view;
    } steps[] = {
        {"Setup", 0, kWorkspaceViewMission},
        {"Launch", 1, kWorkspaceViewExport},
        {"Record", 1, kWorkspaceViewExport},
        {"Export", 1, kWorkspaceViewExport},
    };
    if (ImGui::BeginTable("HardwareWorkflowStrip", 4, ImGuiTableFlags_SizingStretchSame)) {
        for (const Step& step : steps) {
            ImGui::TableNextColumn();
            const bool selected =
                ui_state->hardware_panel_tab == step.tab &&
                ui_state->workspace_view == step.view;
            if (selected) {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.43f, 0.52f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.22f, 0.56f, 0.66f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.11f, 0.33f, 0.40f, 1.0f));
            }
            if (ImGui::Button(step.label, ImVec2(-1.0f, 0.0f))) {
                ui_state->hardware_panel_tab = step.tab;
                ui_state->requested_hardware_panel_tab = step.tab;
                ui_state->workspace_view = step.view;
            }
            if (selected) {
                ImGui::PopStyleColor(3);
            }
        }
        ImGui::EndTable();
    }
}

void render_hardware_pose_confidence_panel(const HardwareViewerState& hardware) {
    const HardwarePoseQuality pose_quality = estimate_hardware_pose_quality(hardware);
    char confidence_buf[32];
    std::snprintf(confidence_buf, sizeof(confidence_buf), "%.0f%%", pose_quality.confidence * 100.0);
    ImGui::SeparatorText("Pose State");
    status_line("Pose source", pose_quality.source.c_str(), pose_quality.color);
    status_line("Pose confidence", confidence_buf, pose_quality.color);
    ImGui::ProgressBar(static_cast<float>(pose_quality.confidence), ImVec2(-1.0f, 0.0f), confidence_buf);
    ImGui::TextWrapped("%s", pose_quality.detail.c_str());
}

void render_hardware_preflight_panel(const HardwareViewerState& hardware,
                                     const UiState& ui_state,
                                     const LiveViewStreamServer& hardware_server) {
    const HardwarePreflightSummary summary =
        summarize_hardware_preflight(hardware, ui_state, hardware_server);
    const bool has_sample = hardware.frame.has_latest_sample;
    const HardwareTelemetrySample& sample = hardware.frame.latest_sample;
    const VehicleModelKind configured_model =
        static_cast<VehicleModelKind>(ui_state.hardware_vehicle_model);
    const bool live_model_ok =
        hardware.has_scene &&
        vehicle_model_from_live_name(hardware.scene.vehicle_model_name) == configured_model;
    const bool imu_ready =
        has_sample && flag_set(sample.controller_status_flags, thesis_sim::StatusFlag::ImuReady);
    const bool encoders_ready =
        has_sample &&
        flag_set(sample.controller_status_flags, thesis_sim::StatusFlag::EncodersReady) &&
        sample.controller_encoder_dt_ms > 0.0;
    const bool motors_ready =
        has_sample && flag_set(sample.controller_status_flags, thesis_sim::StatusFlag::MotorsReady);
    const bool lidar_required = hardware.has_scene && hardware.scene.lidar_enabled;
    const bool lidar_ready = !lidar_required || hardware.frame.valid_lidar_samples >= 32;
    const bool planner_ready = hardware.frame.planner_has_reference || hardware.frame.step_count == 0;
    const bool map_ready = !hardware_server.has_pending_world();

    ImGui::SeparatorText("Preflight");
    const ImVec4 summary_color =
        summary.ready ? ImVec4(0.48f, 0.88f, 0.62f, 1.0f)
                      : (summary.warning ? ImVec4(0.87f, 0.79f, 0.39f, 1.0f)
                                         : ImVec4(0.95f, 0.40f, 0.34f, 1.0f));
    char summary_buf[48];
    std::snprintf(summary_buf, sizeof(summary_buf), "%d / %d ready", summary.passed, summary.total);
    status_line("Checklist", summary_buf, summary_color);

    auto row = [](const char* label, const char* value, bool ok) {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::TextUnformatted(label);
        ImGui::TableNextColumn();
        ImGui::PushStyleColor(ImGuiCol_Text,
                              ok ? ImVec4(0.48f, 0.88f, 0.62f, 1.0f)
                                 : ImVec4(0.95f, 0.60f, 0.34f, 1.0f));
        ImGui::TextUnformatted(ok ? "ready" : "check");
        ImGui::PopStyleColor();
        ImGui::TableNextColumn();
        ImGui::TextWrapped("%s", value);
    };

    if (ImGui::BeginTable("HardwarePreflightTable", 3, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_BordersInnerH)) {
        ImGui::TableSetupColumn("Item", ImGuiTableColumnFlags_WidthFixed, 92.0f);
        ImGui::TableSetupColumn("State", ImGuiTableColumnFlags_WidthFixed, 56.0f);
        ImGui::TableSetupColumn("Detail", ImGuiTableColumnFlags_WidthStretch);
        row("Listener",
            hardware_server.listening() ? "TCP listener active" : "start the listener",
            hardware_server.listening());
        row("Link",
            hardware_server.connected() ? hardware_server.remote_endpoint().c_str() : "waiting for Raspberry runner",
            hardware_server.connected() || hardware.frame.connected);
        row("Scene",
            hardware.has_scene ? "runner metadata received" : "waiting for live scene packet",
            hardware.has_scene);
        row("Robot",
            hardware.has_scene ? hardware.scene.vehicle_model_name.c_str() : "waiting scene",
            live_model_ok);
        row("Map",
            map_ready ? "no pending map sync" : "map/profile sync queued",
            map_ready);
        row("Telemetry",
            hardware.frame.telemetry_ready ? "controller telemetry streaming" : "waiting controller telemetry",
            hardware.frame.telemetry_ready);
        row("IMU",
            imu_ready ? "orientation stream ready" : "BNO/IMU not ready",
            imu_ready);
        row("Encoders",
            encoders_ready ? "tick stream ready" : "encoder dt/flags missing",
            encoders_ready);
        row("Motors",
            motors_ready ? "controller reports motors ready" : "motor ready flag missing",
            motors_ready);
        row("LiDAR",
            lidar_required ? (lidar_ready ? "valid scan samples" : "scan weak or missing")
                           : "not required for this scenario",
            lidar_ready);
        row("Planner",
            planner_ready ? "reference available or waiting at start" : "planner reference missing",
            planner_ready);
        ImGui::EndTable();
    }
}

void render_hardware_thesis_metrics_panel(const HardwareViewerState& hardware) {
    const HardwareThesisMetrics metrics = summarize_hardware_thesis_metrics(hardware);
    const HardwarePoseQuality pose_quality = estimate_hardware_pose_quality(hardware);
    ImGui::SeparatorText("Thesis Metrics");
    status_line("Pose source", pose_quality.source.c_str(), pose_quality.color);
    if (!metrics.has_data) {
        ImGui::TextDisabled("Waiting for hardware history.");
        return;
    }

    char goal_buf[32];
    char gates_buf[32];
    char switches_buf[32];
    char mode_time_buf[48];
    char clearance_buf[32];
    char tracking_buf[48];
    char odom_buf[48];
    char speed_buf[48];
    if (metrics.time_to_goal_s >= 0.0) {
        std::snprintf(goal_buf, sizeof(goal_buf), "%.1f s", metrics.time_to_goal_s);
    } else {
        std::snprintf(goal_buf, sizeof(goal_buf), "%.2f m left", metrics.final_goal_distance_m);
    }
    std::snprintf(gates_buf, sizeof(gates_buf), "%d", metrics.passed_gates);
    std::snprintf(switches_buf,
                  sizeof(switches_buf),
                  "%d (%d/%d)",
                  metrics.switch_events,
                  metrics.structured_to_gate_switches,
                  metrics.gate_to_structured_switches);
    std::snprintf(mode_time_buf,
                  sizeof(mode_time_buf),
                  "%.1f / %.1f s",
                  metrics.structured_time_s,
                  metrics.gate_time_s);
    if (std::isfinite(metrics.min_clearance_m)) {
        std::snprintf(clearance_buf, sizeof(clearance_buf), "%.2f m", metrics.min_clearance_m);
    } else {
        std::snprintf(clearance_buf, sizeof(clearance_buf), "n/a");
    }
    std::snprintf(tracking_buf,
                  sizeof(tracking_buf),
                  "%.2f m / %.1f deg",
                  metrics.max_cross_track_m,
                  metrics.max_heading_error_deg);
    std::snprintf(odom_buf,
                  sizeof(odom_buf),
                  "%.2f / %.2f / %.2f m",
                  metrics.encoder_distance_m,
                  metrics.estimated_path_length_m,
                  metrics.odom_pose_delta_m);
    std::snprintf(speed_buf,
                  sizeof(speed_buf),
                  "%.2f / %.2f m/s",
                  metrics.avg_speed_mps,
                  metrics.max_speed_mps);

    const ImVec4 ok = ImVec4(0.48f, 0.88f, 0.62f, 1.0f);
    const ImVec4 warn = ImVec4(0.87f, 0.79f, 0.39f, 1.0f);
    const ImVec4 bad = ImVec4(0.95f, 0.40f, 0.34f, 1.0f);
    const ImVec4 clearance_color =
        !std::isfinite(metrics.min_clearance_m)
            ? warn
            : (metrics.min_clearance_m >= 0.12 ? ok
                                                : (metrics.min_clearance_m >= 0.06 ? warn : bad));
    const ImVec4 tracking_color =
        metrics.max_cross_track_m <= 0.12 ? ok : (metrics.max_cross_track_m <= 0.25 ? warn : bad);

    auto row = [](const char* label, const char* value, ImVec4 color) {
        ImGui::TableNextRow();
        ImGui::TableNextColumn();
        ImGui::TextUnformatted(label);
        ImGui::TableNextColumn();
        ImGui::PushStyleColor(ImGuiCol_Text, color);
        ImGui::TextUnformatted(value);
        ImGui::PopStyleColor();
    };

    if (ImGui::BeginTable("HardwareThesisMetricsTable", 2, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_BordersInnerH)) {
        ImGui::TableSetupColumn("Metric", ImGuiTableColumnFlags_WidthFixed, 110.0f);
        ImGui::TableSetupColumn("Value", ImGuiTableColumnFlags_WidthStretch);
        row("Goal", goal_buf, metrics.time_to_goal_s >= 0.0 ? ok : warn);
        row("Passed gates", gates_buf, metrics.passed_gates > 0 ? ok : warn);
        row("Switches", switches_buf, metrics.switch_events > 0 ? ok : warn);
        row("Road / gate", mode_time_buf, ok);
        row("Clearance", clearance_buf, clearance_color);
        row("Max tracking", tracking_buf, tracking_color);
        row("Enc / est / delta", odom_buf, ok);
        row("Avg / max speed", speed_buf, ok);
        ImGui::EndTable();
    }
}

void render_editor_inspector(WorldMap& editor_world, UiState* ui_state, bool* dirty_flag) {
    if (ui_state == nullptr || dirty_flag == nullptr) {
        return;
    }

    ImGui::SeparatorText("Inspector");
    const MapEditorHandle handle = ui_state->selected_editor_handle;
    if (handle.type == MapEditorHandleType::None) {
        ImGui::TextDisabled("Select a handle in the viewport or add a map element.");
    } else {
        ImGui::Text("Selected = %s", map_editor_handle_type_name(handle.type));
    }

    auto edit_vec2 = [&](const char* label, Vec2* value) {
        if (value == nullptr) {
            return false;
        }
        bool changed = false;
        ImGui::PushID(label);
        ImGui::TextDisabled("%s", label);
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.48f);
        changed |= ImGui::InputDouble("x", &value->x, 0.01, 0.10, "%.3f");
        ImGui::SameLine();
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
        changed |= ImGui::InputDouble("y", &value->y, 0.01, 0.10, "%.3f");
        ImGui::PopID();
        if (changed) {
            *dirty_flag = true;
        }
        return changed;
    };

    switch (handle.type) {
        case MapEditorHandleType::Start: {
            Vec2 start = editor_world.start();
            if (edit_vec2("Start", &start)) {
                editor_world.set_start(start);
            }
            break;
        }
        case MapEditorHandleType::Goal: {
            Vec2 goal = editor_world.goal();
            if (edit_vec2("Goal", &goal)) {
                editor_world.set_goal(goal);
            }
            break;
        }
        case MapEditorHandleType::Obstacle:
            if (handle.index >= 0 &&
                handle.index < static_cast<int>(editor_world.editable_obstacles().size())) {
                Rect& obstacle = editor_world.editable_obstacles()[handle.index];
                Vec2 center{
                    0.5 * (obstacle.min_x + obstacle.max_x),
                    0.5 * (obstacle.min_y + obstacle.max_y),
                };
                double size[2] = {
                    obstacle.max_x - obstacle.min_x,
                    obstacle.max_y - obstacle.min_y,
                };
                bool changed = edit_vec2("Center", &center);
                ImGui::TextDisabled("Size");
                ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.48f);
                changed |= ImGui::InputDouble("w##ObstacleSize", &size[0], 0.01, 0.10, "%.3f");
                ImGui::SameLine();
                ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x);
                changed |= ImGui::InputDouble("h##ObstacleSize", &size[1], 0.01, 0.10, "%.3f");
                size[0] = std::max(size[0], 0.05);
                size[1] = std::max(size[1], 0.05);
                if (changed) {
                    obstacle.min_x = center.x - 0.5 * size[0];
                    obstacle.max_x = center.x + 0.5 * size[0];
                    obstacle.min_y = center.y - 0.5 * size[1];
                    obstacle.max_y = center.y + 0.5 * size[1];
                    *dirty_flag = true;
                }
            }
            break;
        case MapEditorHandleType::Gate:
            if (handle.index >= 0 &&
                handle.index < static_cast<int>(editor_world.editable_gates().size())) {
                GateSpec& gate = editor_world.editable_gates()[handle.index];
                Vec2 position = gate.anchor_position;
                if (edit_vec2("Gate", &position)) {
                    gate.anchor_position = position;
                    gate.position = position;
                }
                double heading_deg = gate.heading_hint * 180.0 / 3.14159265358979323846;
                if (ImGui::InputDouble("Heading deg", &heading_deg, 1.0, 10.0, "%.2f")) {
                    gate.heading_hint = heading_deg * 3.14159265358979323846 / 180.0;
                    *dirty_flag = true;
                }
            }
            break;
        case MapEditorHandleType::RoadPoint:
            if (handle.index >= 0 &&
                handle.index < static_cast<int>(editor_world.editable_road_centerline().size())) {
                edit_vec2("Road point", &editor_world.editable_road_centerline()[handle.index]);
            }
            break;
        default:
            break;
    }

    const Rect bounds = editor_world.bounds();
    char size_buf[64];
    std::snprintf(size_buf,
                  sizeof(size_buf),
                  "%.2f x %.2f m",
                  bounds.max_x - bounds.min_x,
                  bounds.max_y - bounds.min_y);
    status_line("Map size", size_buf, ImVec4(0.43f, 0.82f, 0.96f, 1.0f));
    status_line("Dirty",
                *dirty_flag ? "unapplied changes" : "synchronized",
                *dirty_flag ? ImVec4(0.95f, 0.60f, 0.34f, 1.0f)
                            : ImVec4(0.48f, 0.88f, 0.62f, 1.0f));
}

void render_hardware_control_panel(const HardwareViewerState& hardware,
                                   UiState* ui_state,
                                   LiveViewStreamServer* hardware_server) {
    if (ui_state == nullptr || hardware_server == nullptr) {
        return;
    }

    if (!ImGui::BeginChild("HardwareConfigPanel", ImVec2(0.0f, 0.0f), true)) {
        ImGui::EndChild();
        return;
    }

    const std::string configured_profile = "Planner";
    const std::string live_profile =
        hardware.has_scene ? stream_profile_label(hardware.scene.stream_profile)
                           : configured_profile;
    const bool profile_mismatch =
        hardware.has_scene && !hardware.scene.stream_profile.empty() &&
        hardware.scene.stream_profile != "planner";

    const char* status = hardware_server->connected()
                             ? (hardware.frame.goal_reached ? "Goal reached" : (hardware.frame.safety_stop_active ? "Safety stop" : "Live"))
                             : (hardware_server->listening() ? "Listening" : "Idle");
    const ImVec4 status_color = hardware_server->connected()
                                    ? (hardware.frame.goal_reached ? ImVec4(0.45f, 0.86f, 0.53f, 1.0f)
                                                                   : (hardware.frame.safety_stop_active ? ImVec4(0.95f, 0.40f, 0.34f, 1.0f)
                                                                                                       : ImVec4(0.87f, 0.79f, 0.39f, 1.0f)))
                                    : (hardware_server->listening() ? ImVec4(0.43f, 0.82f, 0.96f, 1.0f)
                                                                    : ImVec4(0.74f, 0.79f, 0.84f, 1.0f));
    ImGui::TextUnformatted("Hardware Control");
    ImGui::TextColored(status_color, "%s", status);
    ImGui::TextDisabled("configured %s | incoming %s",
                        configured_profile.c_str(),
                        live_profile.c_str());
    if (profile_mismatch) {
        ImGui::TextColored(ImVec4(0.95f, 0.60f, 0.34f, 1.0f),
                           "This UI expects the planner stream, but the current stream is %s.",
                           live_profile.c_str());
    }

    const bool map_sync_queued =
        ui_state->hardware_world_sync_pending ||
        hardware_server->has_pending_world() ||
        hardware_server->has_pending_robot_profile();
    const char* robot_state = hardware_server->connected()
                                  ? "connected"
                                  : (hardware_server->listening() ? "waiting" : "listener off");
    const ImVec4 robot_color = hardware_server->connected()
                                   ? ImVec4(0.48f, 0.88f, 0.62f, 1.0f)
                                   : (hardware_server->listening() ? ImVec4(0.43f, 0.82f, 0.96f, 1.0f)
                                                                   : ImVec4(0.74f, 0.79f, 0.84f, 1.0f));
    const char* map_state = ui_state->hardware_editor_dirty
                                ? "edited locally"
                                : (map_sync_queued ? "sync queued"
                                                   : (ui_state->hardware_world_sync_ok ? "synced" : "ready"));
    const ImVec4 map_color = ui_state->hardware_editor_dirty
                                 ? ImVec4(0.95f, 0.60f, 0.34f, 1.0f)
                                 : (map_sync_queued ? ImVec4(0.43f, 0.82f, 0.96f, 1.0f)
                                                    : ImVec4(0.48f, 0.88f, 0.62f, 1.0f));
    const char* planner_state = hardware.frame.safety_stop_active
                                    ? "safety stop"
                                    : (hardware.frame.planner_has_reference
                                           ? "reference ready"
                                           : (hardware_server->connected() ? "waiting reference" : "preview"));
    const ImVec4 planner_color = hardware.frame.safety_stop_active
                                     ? ImVec4(0.95f, 0.40f, 0.34f, 1.0f)
                                     : (hardware.frame.planner_has_reference ? ImVec4(0.48f, 0.88f, 0.62f, 1.0f)
                                                                            : ImVec4(0.87f, 0.79f, 0.39f, 1.0f));

    ImGui::SeparatorText("Demo State");
    status_line("Robot", robot_state, robot_color);
    status_line("Map", map_state, map_color);
    status_line("Planner", planner_state, planner_color);
    status_line("Mode", live_profile.c_str(), status_color);

    render_hardware_workflow_strip(ui_state);
    render_hardware_pose_confidence_panel(hardware);
    const bool hardware_run_active =
        hardware_server->connected() &&
        hardware.frame.step_count > 0 &&
        !hardware.frame.goal_reached &&
        !hardware.frame.safety_stop_active;
    if (!hardware_run_active) {
        render_hardware_preflight_panel(hardware, *ui_state, *hardware_server);
    }

    if (ImGui::BeginTabBar("HardwareDeskTabs")) {
        if (ImGui::BeginTabItem("Setup", nullptr, requested_tab_flags(ui_state->requested_hardware_panel_tab, 0))) {
            activate_hardware_panel_tab(ui_state, 0, kWorkspaceViewMission, false);
            consume_requested_tab(&ui_state->requested_hardware_panel_tab, 0);
            ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
            if (ImGui::CollapsingHeader("Demo Setup", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::InputInt("Listen Port", &ui_state->hardware_listen_port);
        ui_state->hardware_listen_port = std::max(ui_state->hardware_listen_port, 1);

        const char* hardware_vehicle_items[] = {
            "Bicycle support",
            "Tank support",
        };
        int hardware_vehicle_selection =
            ui_state->hardware_vehicle_model == static_cast<int>(VehicleModelKind::TrackedVehicle) ? 1 : 0;
        if (ImGui::Combo("Robot Support", &hardware_vehicle_selection, hardware_vehicle_items, IM_ARRAYSIZE(hardware_vehicle_items))) {
            const VehicleModelKind next_model =
                hardware_vehicle_selection == 1 ? VehicleModelKind::TrackedVehicle : VehicleModelKind::CarLikeBicycle;
            queue_hardware_robot_profile(
                ui_state,
                hardware_server,
                next_model,
                hardware_server->connected()
                    ? "Robot support sent to the connected Raspberry runner. Waiting for runner ack."
                    : "Robot support queued in the GUI. It will be sent automatically to the next Raspberry runner that connects.",
                "Could not queue the robot support profile for Raspberry sync.");
            if (static_cast<EnvironmentMode>(ui_state->hardware_environment_mode) == EnvironmentMode::StructuredRoad) {
                load_hardware_editor_from_selection(ui_state);
                queue_current_hardware_world(
                    ui_state,
                    hardware_server,
                    "Structured map rescaled for the selected robot and queued for the Raspberry runner.",
                    "Robot support changed locally, but the rescaled structured map could not be queued.");
            }
        }
        if (static_cast<VehicleModelKind>(ui_state->hardware_vehicle_model) == VehicleModelKind::TrackedVehicle) {
            ImGui::TextWrapped("Tracked support uses the same MPC path follower; its speed and yaw-rate targets are converted into differential track velocities.");
        }

        const char* environment_items[] = {
            thesis_sim::environment_mode_name(EnvironmentMode::UnstructuredGates),
            thesis_sim::environment_mode_name(EnvironmentMode::StructuredRoad),
            thesis_sim::environment_mode_name(EnvironmentMode::MixedRoadGates),
        };
        int hardware_environment = std::clamp(
            ui_state->hardware_environment_mode,
            static_cast<int>(EnvironmentMode::UnstructuredGates),
            static_cast<int>(EnvironmentMode::MixedRoadGates));
        const char* environment_label = "Planner Scenario";
        if (ImGui::Combo(environment_label, &hardware_environment, environment_items, IM_ARRAYSIZE(environment_items))) {
            ui_state->hardware_environment_mode = hardware_environment;
            load_hardware_editor_from_selection(ui_state);
            queue_current_hardware_world(
                ui_state,
                hardware_server,
                "Scenario map queued for the Raspberry runner.",
                "Scenario changed locally, but the map could not be queued for the Raspberry runner.");
        }

        const EnvironmentMode active_hardware_mode =
            static_cast<EnvironmentMode>(ui_state->hardware_environment_mode);
        if (active_hardware_mode == EnvironmentMode::StructuredRoad) {
            const StructuredMapPreset previous_preset =
                static_cast<StructuredMapPreset>(ui_state->hardware_structured_preset);
            const char* structured_items[] = {
                thesis_sim::structured_map_preset_name(StructuredMapPreset::ValidationRoad),
                thesis_sim::structured_map_preset_name(StructuredMapPreset::CircleLoop),
                thesis_sim::structured_map_preset_name(StructuredMapPreset::ZigZag),
                thesis_sim::structured_map_preset_name(StructuredMapPreset::HardwareTrack),
                thesis_sim::structured_map_preset_name(StructuredMapPreset::FigureEight),
                thesis_sim::structured_map_preset_name(StructuredMapPreset::TankCircuit),
                thesis_sim::structured_map_preset_name(StructuredMapPreset::Custom),
            };
            int selection = 0;
            switch (previous_preset) {
                case StructuredMapPreset::ValidationRoad:
                    selection = 0;
                    break;
                case StructuredMapPreset::CircleLoop:
                    selection = 1;
                    break;
                case StructuredMapPreset::ZigZag:
                    selection = 2;
                    break;
                case StructuredMapPreset::HardwareTrack:
                    selection = 3;
                    break;
                case StructuredMapPreset::FigureEight:
                    selection = 4;
                    break;
                case StructuredMapPreset::TankCircuit:
                    selection = 5;
                    break;
                case StructuredMapPreset::Custom:
                    selection = 6;
                    break;
                default:
                    selection = 0;
                    break;
            }
            if (ImGui::Combo("Structured Map", &selection, structured_items, IM_ARRAYSIZE(structured_items))) {
                StructuredMapPreset next_preset = StructuredMapPreset::ValidationRoad;
                switch (selection) {
                    case 0:
                        next_preset = StructuredMapPreset::ValidationRoad;
                        break;
                    case 1:
                        next_preset = StructuredMapPreset::CircleLoop;
                        break;
                    case 2:
                        next_preset = StructuredMapPreset::ZigZag;
                        break;
                    case 3:
                        next_preset = StructuredMapPreset::HardwareTrack;
                        break;
                    case 4:
                        next_preset = StructuredMapPreset::FigureEight;
                        break;
                    case 5:
                        next_preset = StructuredMapPreset::TankCircuit;
                        break;
                    case 6:
                        next_preset = StructuredMapPreset::Custom;
                        break;
                    default:
                        next_preset = StructuredMapPreset::ValidationRoad;
                        break;
                }
                if (next_preset == StructuredMapPreset::Custom &&
                    ui_state->hardware_editor_world.environment_mode() != EnvironmentMode::StructuredRoad) {
                    ui_state->hardware_editor_world = make_world_from_mode(
                        EnvironmentMode::StructuredRoad,
                        UnstructuredMapPreset::Custom,
                        previous_preset == StructuredMapPreset::Custom ? StructuredMapPreset::ValidationRoad : previous_preset,
                        GateBehaviorMode::Static,
                        0);
                    ui_state->hardware_editor_dirty = false;
                    reset_editor_interaction(ui_state);
                }
                ui_state->hardware_structured_preset = static_cast<int>(next_preset);
                if (next_preset != StructuredMapPreset::Custom) {
                    load_hardware_editor_from_selection(ui_state);
                    queue_current_hardware_world(
                        ui_state,
                        hardware_server,
                        "Structured preset queued for the Raspberry runner.",
                        "Structured preset restored locally, but it could not be queued for the Raspberry runner.");
                }
            }
        } else if (active_hardware_mode == EnvironmentMode::MixedRoadGates) {
            const int previous_preset = ui_state->hardware_mixed_preset;
            const char* mixed_items[] = {
                hardware_mixed_map_preset_name(0),
                hardware_mixed_map_preset_name(1),
                hardware_mixed_map_preset_name(2),
                hardware_mixed_map_preset_name(3),
                hardware_mixed_map_preset_name(4),
                hardware_mixed_map_preset_name(5),
            };
            int selection = std::clamp(previous_preset, 0, 5);
            if (ImGui::Combo("Mixed Map", &selection, mixed_items, IM_ARRAYSIZE(mixed_items))) {
                ui_state->hardware_mixed_preset = std::clamp(selection, 0, 5);
                load_hardware_editor_from_selection(ui_state);
                queue_current_hardware_world(
                    ui_state,
                    hardware_server,
                    "Mixed preset queued for the Raspberry runner.",
                    "Mixed preset restored locally, but it could not be queued for the Raspberry runner.");
            }
        } else {
            const UnstructuredMapPreset previous_preset =
                static_cast<UnstructuredMapPreset>(ui_state->hardware_unstructured_preset);
            const char* unstructured_items[] = {
                thesis_sim::unstructured_map_preset_name(UnstructuredMapPreset::RobotValidation),
                thesis_sim::unstructured_map_preset_name(UnstructuredMapPreset::TightCorridor),
                thesis_sim::unstructured_map_preset_name(UnstructuredMapPreset::WideSlalom),
                thesis_sim::unstructured_map_preset_name(UnstructuredMapPreset::LowerBypass),
                thesis_sim::unstructured_map_preset_name(UnstructuredMapPreset::HardwareLab),
                thesis_sim::unstructured_map_preset_name(UnstructuredMapPreset::Custom),
            };
            int selection = 4;
            switch (previous_preset) {
                case UnstructuredMapPreset::RobotValidation:
                    selection = 0;
                    break;
                case UnstructuredMapPreset::TightCorridor:
                    selection = 1;
                    break;
                case UnstructuredMapPreset::WideSlalom:
                    selection = 2;
                    break;
                case UnstructuredMapPreset::LowerBypass:
                    selection = 3;
                    break;
                case UnstructuredMapPreset::HardwareLab:
                    selection = 4;
                    break;
                case UnstructuredMapPreset::Custom:
                    selection = 5;
                    break;
                default:
                    selection = 4;
                    break;
            }
            if (ImGui::Combo("Unstructured Map", &selection, unstructured_items, IM_ARRAYSIZE(unstructured_items))) {
                UnstructuredMapPreset next_preset = UnstructuredMapPreset::HardwareLab;
                switch (selection) {
                    case 0:
                        next_preset = UnstructuredMapPreset::RobotValidation;
                        break;
                    case 1:
                        next_preset = UnstructuredMapPreset::TightCorridor;
                        break;
                    case 2:
                        next_preset = UnstructuredMapPreset::WideSlalom;
                        break;
                    case 3:
                        next_preset = UnstructuredMapPreset::LowerBypass;
                        break;
                    case 4:
                        next_preset = UnstructuredMapPreset::HardwareLab;
                        break;
                    case 5:
                        next_preset = UnstructuredMapPreset::Custom;
                        break;
                    default:
                        next_preset = UnstructuredMapPreset::HardwareLab;
                        break;
                }
                if (next_preset == UnstructuredMapPreset::Custom &&
                    ui_state->hardware_editor_world.environment_mode() != EnvironmentMode::UnstructuredGates) {
                    ui_state->hardware_editor_world = make_world_from_mode(
                        EnvironmentMode::UnstructuredGates,
                        previous_preset == UnstructuredMapPreset::Custom ? UnstructuredMapPreset::Custom : previous_preset,
                        StructuredMapPreset::HardwareTrack,
                        GateBehaviorMode::Static,
                        0);
                    ui_state->hardware_editor_dirty = false;
                    reset_editor_interaction(ui_state);
                }
                ui_state->hardware_unstructured_preset = static_cast<int>(next_preset);
                if (next_preset != UnstructuredMapPreset::Custom) {
                    load_hardware_editor_from_selection(ui_state);
                    queue_current_hardware_world(
                        ui_state,
                        hardware_server,
                        "Unstructured preset queued for the Raspberry runner.",
                        "Unstructured preset restored locally, but it could not be queued for the Raspberry runner.");
                }
            }
        }

        ImGui::SeparatorText("Planner Policy");
        const char* pose_fusion_items[] = {
            "Auto",
            "Encoder/IMU primary",
            "LiDAR pose correction",
        };
        int pose_fusion_selection = std::clamp(
            ui_state->hardware_localization_policy,
            static_cast<int>(HardwareLocalizationPolicy::Auto),
            static_cast<int>(HardwareLocalizationPolicy::LidarPoseCorrection));
        if (ImGui::Combo("Pose Fusion",
                         &pose_fusion_selection,
                         pose_fusion_items,
                         IM_ARRAYSIZE(pose_fusion_items))) {
            ui_state->hardware_localization_policy = pose_fusion_selection;
        }
        const HardwareLocalizationPolicy pose_policy =
            static_cast<HardwareLocalizationPolicy>(ui_state->hardware_localization_policy);
        status_line("Launch arg",
                    thesis_sim::hardware_localization_policy_cli_name(pose_policy),
                    pose_policy == HardwareLocalizationPolicy::EncoderImuPrimary
                        ? ImVec4(0.48f, 0.88f, 0.62f, 1.0f)
                        : (pose_policy == HardwareLocalizationPolicy::LidarPoseCorrection
                               ? ImVec4(0.87f, 0.79f, 0.39f, 1.0f)
                               : ImVec4(0.43f, 0.82f, 0.96f, 1.0f)));

        ImGui::SeparatorText("Connection");
        if (ImGui::BeginTable("HardwareListenerButtons", 2, ImGuiTableFlags_SizingStretchSame)) {
            ImGui::TableNextColumn();
            if (ImGui::Button(hardware_server->listening() ? "Restart Listener" : "Start Listener", ImVec2(-1.0f, 0.0f))) {
                hardware_server->start(static_cast<std::uint16_t>(ui_state->hardware_listen_port));
            }
            ImGui::TableNextColumn();
            if (!hardware_server->listening()) {
                ImGui::BeginDisabled();
            }
            if (ImGui::Button("Stop Listener", ImVec2(-1.0f, 0.0f))) {
                hardware_server->stop();
            }
            if (!hardware_server->listening()) {
                ImGui::EndDisabled();
            }
            ImGui::EndTable();
        }

        status_line("Listener", hardware_server->listening() ? "active" : "off", robot_color);
        if (hardware_server->listening()) {
            ImGui::Text("Port = %u", hardware_server->port());
        }
        if (hardware_server->connected()) {
            ImGui::Text("Remote = %s", hardware_server->remote_endpoint().c_str());
        } else {
            ImGui::TextDisabled("Remote = waiting for connection");
        }
        if (!hardware_server->last_error().empty()) {
            ImGui::TextWrapped("Last stream error: %s", hardware_server->last_error().c_str());
        }
    }

            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Launch", nullptr, requested_tab_flags(ui_state->requested_hardware_panel_tab, 1))) {
            activate_hardware_panel_tab(ui_state, 1, kWorkspaceViewExport, false);
            consume_requested_tab(&ui_state->requested_hardware_panel_tab, 1);
            ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
            if (ImGui::CollapsingHeader("Raspberry Launch", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::TextWrapped("Launch on the Raspberry with:");
        ImGui::TextWrapped("%s", hardware_launch_hint(*ui_state).c_str());
        const bool custom_world_selected =
            static_cast<EnvironmentMode>(ui_state->hardware_environment_mode) == EnvironmentMode::StructuredRoad
                ? static_cast<StructuredMapPreset>(ui_state->hardware_structured_preset) == StructuredMapPreset::Custom
                : static_cast<UnstructuredMapPreset>(ui_state->hardware_unstructured_preset) == UnstructuredMapPreset::Custom;
        if (custom_world_selected) {
            if (ui_state->hardware_world_sync_pending || hardware_server->has_pending_world()) {
                ImGui::TextColored(ImVec4(0.43f, 0.82f, 0.96f, 1.0f),
                                   "Custom map sync is queued in the GUI and will be streamed automatically to the Raspberry runner on connect.");
            } else {
                ImGui::TextColored(ImVec4(0.48f, 0.88f, 0.62f, 1.0f),
                                   "The current hardware world will be streamed automatically to the Raspberry runner as soon as it connects.");
            }
        }
        ImGui::TextWrapped("If you want to transport it over SSH, expose this same port with a reverse tunnel.");
    }

            ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
            if (ImGui::CollapsingHeader("Telemetry Capture", ImGuiTreeNodeFlags_DefaultOpen)) {
        const bool can_capture =
            hardware.has_scene ||
            !hardware.history.empty() ||
            hardware.frame.connected ||
            hardware.frame.step_count > 0;
        ImGui::TextWrapped("Save a full hardware snapshot with controller flags, trajectory, LiDAR, gates, workstation telemetry and thesis metrics.");
        render_hardware_thesis_metrics_panel(hardware);
        if (!can_capture) {
            ImGui::BeginDisabled();
        }
        if (ImGui::Button("Save Hardware Telemetry", ImVec2(-1.0f, 0.0f))) {
            const std::string report_path = default_hardware_report_path(hardware, *ui_state, "gui_manual");
            HardwareLidarReconstructionExport lidar_export;
            const bool json_ok = write_hardware_json_report(hardware, *ui_state, *hardware_server, report_path);
            const bool lidar_ok = write_hardware_lidar_reconstruction_bundle(hardware, report_path, &lidar_export);
            if (json_ok && lidar_ok) {
                ui_state->hardware_report_written = true;
                ui_state->last_hardware_report_path = report_path;
                ui_state->last_hardware_csv_report_path.clear();
                ui_state->last_hardware_markdown_report_path.clear();
                ui_state->last_hardware_lidar_reconstruction_csv_path = lidar_export.csv_path;
                ui_state->last_hardware_lidar_reconstruction_ply_path = lidar_export.ply_path;
                ui_state->last_hardware_lidar_reconstruction_png_path = lidar_export.png_path;
                ui_state->last_hardware_report_error.clear();
            } else {
                ui_state->hardware_report_written = false;
                ui_state->last_hardware_report_error = "Could not write the hardware telemetry report and LiDAR reconstruction.";
            }
        }
        if (ImGui::Button("Save Thesis Bundle", ImVec2(-1.0f, 0.0f))) {
            const std::string json_path = default_hardware_report_path(hardware, *ui_state, "gui_bundle");
            const std::string csv_path = replace_extension(json_path, ".csv");
            const std::string markdown_path = replace_extension(json_path, ".md");
            HardwareLidarReconstructionExport lidar_export;
            const bool json_ok = write_hardware_json_report(hardware, *ui_state, *hardware_server, json_path);
            const bool csv_ok = write_hardware_csv_report(hardware, csv_path);
            const bool markdown_ok = write_hardware_markdown_summary(hardware, *ui_state, *hardware_server, markdown_path);
            const bool lidar_ok = write_hardware_lidar_reconstruction_bundle(hardware, json_path, &lidar_export);
            if (json_ok && csv_ok && markdown_ok && lidar_ok) {
                ui_state->hardware_report_written = true;
                ui_state->last_hardware_report_path = json_path;
                ui_state->last_hardware_csv_report_path = csv_path;
                ui_state->last_hardware_markdown_report_path = markdown_path;
                ui_state->last_hardware_lidar_reconstruction_csv_path = lidar_export.csv_path;
                ui_state->last_hardware_lidar_reconstruction_ply_path = lidar_export.ply_path;
                ui_state->last_hardware_lidar_reconstruction_png_path = lidar_export.png_path;
                ui_state->last_hardware_report_error.clear();
            } else {
                ui_state->hardware_report_written = false;
                ui_state->last_hardware_report_error = "Could not write the complete hardware thesis bundle.";
            }
        }
        if (!can_capture) {
            ImGui::EndDisabled();
            ImGui::TextDisabled("Waiting for live hardware data before capture.");
        }
        ImGui::Text("History samples = %d", static_cast<int>(hardware.history.size()));
        ImGui::Text("Current LiDAR hits = %d   map points = %d",
                    static_cast<int>(hardware.frame.lidar_hits.size()),
                    static_cast<int>(hardware.frame.slam_points.size()));
        ImGui::Text("Reconstruction points = %d", static_cast<int>(hardware.lidar_reconstruction.size()));
        if (!ui_state->last_hardware_report_path.empty()) {
            ImGui::TextWrapped("JSON: %s", ui_state->last_hardware_report_path.c_str());
        }
        if (!ui_state->last_hardware_csv_report_path.empty()) {
            ImGui::TextWrapped("CSV: %s", ui_state->last_hardware_csv_report_path.c_str());
        }
        if (!ui_state->last_hardware_markdown_report_path.empty()) {
            ImGui::TextWrapped("Markdown: %s", ui_state->last_hardware_markdown_report_path.c_str());
        }
        if (!ui_state->last_hardware_lidar_reconstruction_csv_path.empty()) {
            ImGui::TextWrapped("LiDAR reconstruction CSV: %s", ui_state->last_hardware_lidar_reconstruction_csv_path.c_str());
        }
        if (!ui_state->last_hardware_lidar_reconstruction_ply_path.empty()) {
            ImGui::TextWrapped("LiDAR reconstruction PLY: %s", ui_state->last_hardware_lidar_reconstruction_ply_path.c_str());
        }
        if (!ui_state->last_hardware_lidar_reconstruction_png_path.empty()) {
            ImGui::TextWrapped("LiDAR reconstruction PNG: %s", ui_state->last_hardware_lidar_reconstruction_png_path.c_str());
        }
        if (!ui_state->last_hardware_report_error.empty()) {
            ImGui::TextColored(ImVec4(0.95f, 0.40f, 0.34f, 1.0f), "%s", ui_state->last_hardware_report_error.c_str());
        }
    }

            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Map", nullptr, requested_tab_flags(ui_state->requested_hardware_panel_tab, 2))) {
            activate_hardware_panel_tab(ui_state, 2, kWorkspaceViewMap, true);
            consume_requested_tab(&ui_state->requested_hardware_panel_tab, 2);
            ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
            if (ImGui::CollapsingHeader("Track Map", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SameLine();
        help_marker("Preview and reshape the hardware map before syncing it to the Raspberry runner.");

        WorldMap& editor_world = ui_state->hardware_editor_world;
        const bool hardware_structured_mode =
            static_cast<EnvironmentMode>(ui_state->hardware_environment_mode) == EnvironmentMode::StructuredRoad;
        const bool hardware_track_selected =
            static_cast<StructuredMapPreset>(ui_state->hardware_structured_preset) == StructuredMapPreset::HardwareTrack;
        const Rect editor_bounds = editor_world.bounds();
        char editor_size_buf[64];
        char road_points_buf[32];
        std::snprintf(editor_size_buf,
                      sizeof(editor_size_buf),
                      "%.2f x %.2f m",
                      editor_bounds.max_x - editor_bounds.min_x,
                      editor_bounds.max_y - editor_bounds.min_y);
        std::snprintf(road_points_buf,
                      sizeof(road_points_buf),
                      "%d",
                      static_cast<int>(editor_world.road_centerline().size()));
        const auto apply_editor_world = [&]() -> bool {
            const WorldMap previous_world = ui_state->hardware_editor_world;
            const bool previous_dirty = ui_state->hardware_editor_dirty;
            WorldMap applied_world = ui_state->hardware_editor_world;
            applied_world.finalize_editor_changes();
            applied_world = sanitize_hardware_unstructured_world(std::move(applied_world));
            applied_world = fit_hardware_structured_world(
                std::move(applied_world),
                static_cast<VehicleModelKind>(ui_state->hardware_vehicle_model));
            std::string validation_error;
            if (!validate_hardware_world(applied_world, &validation_error)) {
                ui_state->hardware_editor_world = previous_world;
                ui_state->hardware_editor_dirty = previous_dirty;
                ui_state->last_hardware_world_error = validation_error;
                ui_state->last_hardware_world_sync_status = validation_error;
                return false;
            }

            ui_state->hardware_editor_world = applied_world;
            ui_state->hardware_editor_dirty = false;
            ui_state->last_hardware_world_error.clear();
            if (applied_world.environment_mode() == EnvironmentMode::StructuredRoad) {
                ui_state->hardware_environment_mode = static_cast<int>(EnvironmentMode::StructuredRoad);
                ui_state->hardware_structured_preset = static_cast<int>(StructuredMapPreset::Custom);
            } else if (applied_world.environment_mode() == EnvironmentMode::MixedRoadGates) {
                ui_state->hardware_environment_mode = static_cast<int>(EnvironmentMode::MixedRoadGates);
                ui_state->hardware_mixed_preset = hardware_mixed_preset_from_world(applied_world);
            } else {
                ui_state->hardware_environment_mode = static_cast<int>(EnvironmentMode::UnstructuredGates);
                ui_state->hardware_unstructured_preset = static_cast<int>(UnstructuredMapPreset::Custom);
            }
            return true;
        };
        if (hardware_structured_mode && hardware_track_selected) {
            float track_scale = ui_state->hardware_track_scale;
            if (ImGui::SliderFloat("Track Scale", &track_scale, kHardwareTrackMinScale, kHardwareTrackMaxScale, "%.2fx")) {
                ui_state->hardware_track_scale = track_scale;
                load_hardware_editor_from_selection(ui_state);
                queue_current_hardware_world(
                    ui_state,
                    hardware_server,
                    "Hardware Track scale queued for the Raspberry runner.",
                    "Hardware Track scale updated locally, but it could not be queued for the Raspberry runner.");
            }
        }
        ImGui::Checkbox(hardware_structured_mode ? "Edit road by dragging" : "Edit map by dragging",
                        &ui_state->map_editor_enabled);

        if (ImGui::BeginTable("HardwareMapActions", 2, ImGuiTableFlags_SizingStretchSame)) {
            ImGui::TableNextColumn();
            const bool hardware_mixed_mode =
                editor_world.environment_mode() == EnvironmentMode::MixedRoadGates ||
                static_cast<EnvironmentMode>(ui_state->hardware_environment_mode) == EnvironmentMode::MixedRoadGates;
            const bool can_reset_track = hardware_structured_mode || hardware_mixed_mode;
            if (!can_reset_track) {
                ImGui::BeginDisabled();
            }
            const char* reset_label = hardware_mixed_mode ? "Reset Mixed" : "Reset Track";
            if (ImGui::Button(reset_label, ImVec2(-1.0f, 0.0f))) {
                if (hardware_mixed_mode) {
                    ui_state->hardware_environment_mode = static_cast<int>(EnvironmentMode::MixedRoadGates);
                } else {
                    ui_state->hardware_environment_mode = static_cast<int>(EnvironmentMode::StructuredRoad);
                    ui_state->hardware_structured_preset = static_cast<int>(StructuredMapPreset::HardwareTrack);
                }
                load_hardware_editor_from_selection(ui_state);
                queue_current_hardware_world(
                    ui_state,
                    hardware_server,
                    hardware_mixed_mode
                        ? "Mixed hardware road queued for the Raspberry runner."
                        : "Hardware Track queued for the Raspberry runner.",
                    hardware_mixed_mode
                        ? "Mixed hardware road restored locally, but it could not be queued for the Raspberry runner."
                        : "Hardware Track restored locally, but it could not be queued for the Raspberry runner.");
            }
            if (!can_reset_track) {
                ImGui::EndDisabled();
            }

            ImGui::TableNextColumn();
            const char* sync_label = ui_state->hardware_editor_dirty ? "Apply + Sync" : "Sync Map";
            if (ImGui::Button(sync_label, ImVec2(-1.0f, 0.0f))) {
                if (!ui_state->hardware_editor_dirty || apply_editor_world()) {
                    queue_current_hardware_world(
                        ui_state,
                        hardware_server,
                        hardware_server->connected()
                            ? "Current hardware world sent to the connected Raspberry runner. Waiting for runner ack."
                            : "Current hardware world queued in the GUI. It will be sent automatically to the next Raspberry runner that connects.",
                        "Could not queue the current hardware world for Raspberry sync.");
                }
            }
            ImGui::EndTable();
        }

        status_line("Track size", editor_size_buf, ImVec4(0.43f, 0.82f, 0.96f, 1.0f));
        status_line("Road points", road_points_buf, ImVec4(0.97f, 0.89f, 0.45f, 1.0f));
        status_line("Sync", map_state, map_color);

        if (ui_state->map_editor_enabled &&
            ImGui::CollapsingHeader("Edit Handles", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (editor_world.environment_mode() == EnvironmentMode::UnstructuredGates) {
                if (ImGui::Button("Add Obstacle", ImVec2(-1.0f, 0.0f))) {
                    const Vec2 center{
                        0.5 * (editor_world.start().x + editor_world.goal().x),
                        0.5 * (editor_world.start().y + editor_world.goal().y),
                    };
                    editor_world.editable_obstacles().push_back({center.x - 1.0, center.y - 1.0, center.x + 1.0, center.y + 1.0});
                    ui_state->hardware_editor_dirty = true;
                    ui_state->selected_editor_handle = {MapEditorHandleType::Obstacle, static_cast<int>(editor_world.editable_obstacles().size()) - 1};
                }
                if (ImGui::Button("Add Gate", ImVec2(-1.0f, 0.0f))) {
                    const Vec2 midpoint{
                        0.5 * (editor_world.start().x + editor_world.goal().x),
                        0.5 * (editor_world.start().y + editor_world.goal().y),
                    };
                    GateSpec gate;
                    gate.name = "gate_" + std::to_string(editor_world.editable_gates().size() + 1);
                    gate.position = midpoint;
                    gate.anchor_position = midpoint;
                    editor_world.editable_gates().push_back(gate);
                    ui_state->hardware_editor_dirty = true;
                    ui_state->selected_editor_handle = {MapEditorHandleType::Gate, static_cast<int>(editor_world.editable_gates().size()) - 1};
                }
            } else {
                if (ImGui::Button("Add Road Point", ImVec2(-1.0f, 0.0f))) {
                    std::vector<Vec2>& road = editor_world.editable_road_centerline();
                    const bool closed_loop =
                        road.size() >= 3 && thesis_sim::distance(road.front(), road.back()) < 0.5;
                    if (closed_loop) {
                        road.insert(road.end() - 1, editor_world.goal());
                        ui_state->selected_editor_handle = {
                            MapEditorHandleType::RoadPoint,
                            static_cast<int>(road.size()) - 2,
                        };
                    } else {
                        road.push_back(editor_world.goal());
                        ui_state->selected_editor_handle = {
                            MapEditorHandleType::RoadPoint,
                            static_cast<int>(road.size()) - 1,
                        };
                    }
                    ui_state->hardware_editor_dirty = true;
                }
            }

            if (ui_state->selected_editor_handle.type != MapEditorHandleType::None) {
                ImGui::Text("Selected = %s", map_editor_handle_type_name(ui_state->selected_editor_handle.type));
                if (ImGui::Button("Remove Selected", ImVec2(-1.0f, 0.0f))) {
                    switch (ui_state->selected_editor_handle.type) {
                        case MapEditorHandleType::Obstacle:
                            if (ui_state->selected_editor_handle.index >= 0 &&
                                ui_state->selected_editor_handle.index < static_cast<int>(editor_world.editable_obstacles().size())) {
                                editor_world.editable_obstacles().erase(
                                    editor_world.editable_obstacles().begin() + ui_state->selected_editor_handle.index);
                                ui_state->hardware_editor_dirty = true;
                            }
                            break;
                        case MapEditorHandleType::Gate:
                            if (ui_state->selected_editor_handle.index >= 0 &&
                                ui_state->selected_editor_handle.index < static_cast<int>(editor_world.editable_gates().size())) {
                                editor_world.editable_gates().erase(
                                    editor_world.editable_gates().begin() + ui_state->selected_editor_handle.index);
                                ui_state->hardware_editor_dirty = true;
                            }
                            break;
                        case MapEditorHandleType::RoadPoint:
                            if (ui_state->selected_editor_handle.index > 0 &&
                                ui_state->selected_editor_handle.index + 1 < static_cast<int>(editor_world.editable_road_centerline().size())) {
                                editor_world.editable_road_centerline().erase(
                                    editor_world.editable_road_centerline().begin() + ui_state->selected_editor_handle.index);
                                ui_state->hardware_editor_dirty = true;
                            }
                            break;
                        default:
                            break;
                    }
                    reset_editor_interaction(ui_state);
                }
            } else {
                ImGui::TextDisabled("No editor handle selected.");
            }
        }

        render_editor_inspector(editor_world, ui_state, &ui_state->hardware_editor_dirty);

        if (!ui_state->last_hardware_world_path.empty()) {
            ImGui::TextWrapped("Last custom map: %s", ui_state->last_hardware_world_path.c_str());
        }
        if (!ui_state->last_hardware_world_error.empty()) {
            ImGui::TextColored(ImVec4(0.95f, 0.40f, 0.34f, 1.0f), "%s", ui_state->last_hardware_world_error.c_str());
        }
        if (!ui_state->last_hardware_world_sync_status.empty()) {
            const ImVec4 sync_color =
                ui_state->hardware_world_sync_pending
                    ? ImVec4(0.43f, 0.82f, 0.96f, 1.0f)
                    : (ui_state->hardware_world_sync_ok
                           ? ImVec4(0.48f, 0.88f, 0.62f, 1.0f)
                           : ImVec4(0.95f, 0.60f, 0.34f, 1.0f));
            ImGui::TextColored(sync_color, "%s", ui_state->last_hardware_world_sync_status.c_str());
        }
    }

            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Diagnostics", nullptr, requested_tab_flags(ui_state->requested_hardware_panel_tab, 3))) {
            activate_hardware_panel_tab(ui_state, 3, kWorkspaceViewDiagnostics, false);
            consume_requested_tab(&ui_state->requested_hardware_panel_tab, 3);
            ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
            if (ImGui::CollapsingHeader("Sensors & Localization", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (!hardware.has_scene) {
            ImGui::TextDisabled("Waiting for scene metadata...");
        } else {
            ImGui::Text("Stream profile = %s", live_profile.c_str());
            ImGui::Text("Localization = %s", hardware.scene.localization_mode.c_str());
            ImGui::Text("Heading source = %s", hardware.scene.heading_source.c_str());
            ImGui::Text("Stack = %s + %s",
                        hardware.scene.vehicle_model_name.c_str(),
                        hardware.scene.tracking_controller_name.c_str());
            ImGui::Text("LiDAR = %s, min %.2f m, front %.2f m",
                        hardware.scene.range_sensor_name.c_str(),
                        hardware.frame.min_lidar_distance,
                        hardware.frame.front_lidar_distance);
            ImGui::Text("LiDAR samples = %d valid, %d close, %d front-close",
                        hardware.frame.valid_lidar_samples,
                        hardware.frame.close_lidar_samples,
                        hardware.frame.front_close_lidar_samples);
            ImGui::Text("Telemetry ready = %s", hardware.frame.telemetry_ready ? "yes" : "no");
        }
    }

            ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
            if (ImGui::CollapsingHeader("Live Diagnostics", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::Text("t = %.2f s   step = %d", hardware.frame.sim_time, hardware.frame.step_count);
        ImGui::Text("pos = (%.2f, %.2f) m", hardware.frame.navigation_position.x, hardware.frame.navigation_position.y);
        ImGui::Text("yaw = %.1f deg   v = %.2f m/s", hardware.frame.navigation_yaw * 180.0 / 3.14159265358979323846, hardware.frame.navigation_speed);
        ImGui::Text("goal distance = %s",
                    hardware_goal_distance_label(hardware.frame, hardware.scene.world.environment_mode()).c_str());
        ImGui::Text("tracking: cte %.2f m   hdg %.2f deg", hardware.frame.tracker_cross_track_error, hardware.frame.tracker_heading_error_deg);
        ImGui::Text("controller pwm = %d / %d   target = %d / %d",
                    hardware.frame.has_latest_sample ? static_cast<int>(hardware.frame.latest_sample.controller_pwm_left) : 0,
                    hardware.frame.has_latest_sample ? static_cast<int>(hardware.frame.latest_sample.controller_pwm_right) : 0,
                    hardware.frame.has_latest_sample ? static_cast<int>(hardware.frame.latest_sample.controller_target_pwm_left) : 0,
                    hardware.frame.has_latest_sample ? static_cast<int>(hardware.frame.latest_sample.controller_target_pwm_right) : 0);
        ImGui::Text("enc ticks = %d / %d   dTicks = %d / %d   dt = %.0f ms",
                    hardware.frame.vehicle.left_encoder_ticks,
                    hardware.frame.vehicle.right_encoder_ticks,
                    hardware.frame.vehicle.left_encoder_delta,
                    hardware.frame.vehicle.right_encoder_delta,
                    hardware.frame.vehicle.encoder_dt_ms);
        ImGui::Text("planner ref = %s   dynamic gaps = %s   stall boost = %s",
                    hardware.frame.planner_has_reference ? "yes" : "no",
                    hardware.frame.dynamic_gap_gates ? "yes" : "no",
                    hardware.frame.stall_boost_active ? "yes" : "no");
        ImGui::Text("gates = %d candidates   chosen distance = %.2f m",
                    hardware.frame.candidate_gates,
                    hardware.frame.chosen_gate_distance);
        ImGui::Text("LiDAR map pts = %d   no-motion cycles = %d",
                    hardware.frame.accumulated_lidar_points,
                    hardware.frame.no_motion_command_cycles);
        if (hardware.frame.has_last_mpc_command) {
            ImGui::Text("MPC: accel %.2f   steer rate %.2f deg/s",
                        hardware.frame.last_mpc_command.accel_cmd,
                        hardware.frame.last_mpc_command.steer_rate_cmd * 180.0 / 3.14159265358979323846);
        }
        if (hardware.has_scene && hardware.scene.world.environment_mode() == EnvironmentMode::UnstructuredGates) {
            const char* chosen_name =
                hardware.frame.chosen_gate_index >= 0 &&
                        hardware.frame.chosen_gate_index < static_cast<int>(hardware.frame.gates.size())
                    ? hardware.frame.gates[static_cast<std::size_t>(hardware.frame.chosen_gate_index)].spec.name.c_str()
                    : "none";
            ImGui::Text("chosen gate = %s   visible = %d", chosen_name, static_cast<int>(hardware.frame.visible_gate_indices.size()));
        }
    }

            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }

    ImGui::EndChild();
}

