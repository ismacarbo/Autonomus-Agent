void select_workspace_view(UiState* state, int view) {
    state->workspace_view = view;
    if (view == kWorkspaceViewDiagnostics) state->requested_signal_tab = 2;
    else if (view == kWorkspaceViewAnalytics) state->requested_signal_tab = 0;
    state->map_editor_enabled = view == kWorkspaceViewMap;
    if (!workspace_source_is_hardware(state->workspace_source) && (view == kWorkspaceViewSetup || view == kWorkspaceViewMap))
        state->paused = true;
    reset_editor_interaction(state);
}

void render_workspace_top_bar(PlannerDrivenVehicleSim& sim, const HardwareViewerState& hardware,
                              UiState* state, LiveViewStreamServer* server) {
    const bool hw = workspace_source_is_hardware(state->workspace_source);
    const bool stale = hw && hardware.frame.has_latest_sample &&
        (!server->connected() || state->last_hardware_frame_at < 0 ||
         ImGui::GetTime() - state->last_hardware_frame_at > 2.0);
    if (ImGui::BeginTable("AppHeader", 3, ImGuiTableFlags_SizingStretchProp)) {
        ImGui::TableSetupColumn("Brand", ImGuiTableColumnFlags_WidthStretch);
        ImGui::TableSetupColumn("Source", ImGuiTableColumnFlags_WidthFixed, 254);
        ImGui::TableSetupColumn("State", ImGuiTableColumnFlags_WidthFixed, 178);
        ImGui::TableNextColumn();
        ImGui::PushFont(ui_title_font);
        ImGui::TextUnformatted("Autonomous / Navigation Lab");
        ImGui::PopFont();
        ImGui::TextDisabled("%s  /  %s", hw ? "Robot workspace" : "Simulation workspace",
                           hw ? short_scenario_name(static_cast<EnvironmentMode>(state->hardware_environment_mode))
                              : short_scenario_name(sim.environment_mode()));
        ImGui::TableNextColumn();
        if (ui_choice("Simulation", !hw, 122)) state->workspace_source = kWorkspaceSourceSimulation;
        ImGui::SameLine();
        if (ui_choice("Robot", hw, 112)) state->workspace_source = kWorkspaceSourceHardwarePlanner;
        ImGui::TableNextColumn();
        if (hw) {
            if (stale) ui_badge("Last received data", kUiWarning);
            else if (!server->connected()) ui_badge(server->listening() ? "Awaiting robot" : "Offline preview", kUiMuted);
            else if (hardware.frame.safety_stop_active) ui_badge("Safety stop", kUiDanger);
            else if (hardware.frame.goal_reached) ui_badge("Goal reached", kUiAccent);
            else if (!hardware.frame.has_latest_sample) ui_badge("Awaiting telemetry", kUiWarning);
            else ui_badge("Live telemetry", kUiAccent);
        } else {
            ui_badge(sim.collision() ? "Collision" : sim.goal_reached() ? "Goal reached" :
                     state->paused ? "Paused" : "Running",
                     sim.collision() ? kUiDanger : state->paused ? kUiMuted : kUiAccent);
        }
        ImGui::EndTable();
    }
    ImGui::Separator();
    const char* labels[] = {"Overview", "Setup", "Analysis", "Map editor", "Diagnostics", "Export"};
    const int views[] = {kWorkspaceViewMission, kWorkspaceViewSetup, kWorkspaceViewAnalytics,
                         kWorkspaceViewMap, kWorkspaceViewDiagnostics, kWorkspaceViewExport};
    for (int i = 0; i < 6; ++i) {
        if (i) ImGui::SameLine();
        if (ui_choice(labels[i], state->workspace_view == views[i], i == 3 || i == 4 ? 126 : 106))
            select_workspace_view(state, views[i]);
    }
    if (ImGui::GetContentRegionAvail().x > 1000) {
        ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - 178);
        ImGui::TextDisabled("MPC  /  LiDAR  /  SLAM");
    }
    ImGui::Spacing();
}

void render_session_controls(PlannerDrivenVehicleSim& sim, UiState* state, LiveViewStreamServer* server) {
    const bool hw = workspace_source_is_hardware(state->workspace_source);
    if (hw) {
        if (ui_choice(server->listening() ? "Listener active" : "Start listener", server->listening(), 152)) {
            if (!server->listening()) server->start(static_cast<std::uint16_t>(state->hardware_listen_port));
        }
        ImGui::SameLine();
        ImGui::TextDisabled("TCP %d  /  %s", state->hardware_listen_port,
                            server->connected() ? server->remote_endpoint().c_str() : "Launch the runner from Setup");
    } else {
        const bool finished = sim.goal_reached() || sim.collision();
        ImGui::BeginDisabled(finished);
        if (ui_choice(state->paused ? "Run simulation" : "Pause", true, 152)) state->paused = !state->paused;
        ImGui::SameLine();
        ImGui::BeginDisabled(!state->paused);
        if (ImGui::Button("Step", ImVec2(64, 34))) state->single_step = true;
        ImGui::EndDisabled();
        ImGui::EndDisabled();
        ImGui::SameLine();
        if (ImGui::Button("Reset", ImVec2(72, 34))) {
            sim.reset();
            state->paused = true;
            state->single_step = false;
            state->report_written = false;
            state->last_report_path.clear();
            state->last_csv_report_path.clear();
            state->last_markdown_report_path.clear();
            state->last_export_error.clear();
        }
        ImGui::SameLine();
        ImGui::SetNextItemWidth(148);
        const int values[] = {1, 2, 4, 8};
        const char* labels[] = {"1 step/frame", "2 steps/frame", "4 steps/frame", "8 steps/frame"};
        int selected = state->steps_per_frame >= 8 ? 3 : state->steps_per_frame >= 4 ? 2 : state->steps_per_frame >= 2 ? 1 : 0;
        if (ImGui::Combo("##SimulationRate", &selected, labels, 4)) state->steps_per_frame = values[selected];
        ImGui::SameLine();
        ImGui::TextDisabled("Space: run/pause");
    }
    ImGui::SameLine(ImGui::GetWindowContentRegionMax().x - 142);
    ImGui::Checkbox("Live inspector", &state->show_inspector);
}

void render_session_metrics(const PlannerDrivenVehicleSim& sim, const HardwareViewerState& hw, const UiState& state) {
    const bool hardware = workspace_source_is_hardware(state.workspace_source);
    const bool data = hardware ? hw.frame.has_latest_sample : !sim.history().empty();
    const bool reference = hardware ? hw.frame.planner_has_reference : !sim.planned_trajectory().empty();
    const EnvironmentMode mode = hardware ? (hw.has_scene ? hw.scene.world.environment_mode() : static_cast<EnvironmentMode>(state.hardware_environment_mode)) : sim.environment_mode();
    const bool gates = mode == EnvironmentMode::UnstructuredGates;
    if (ImGui::BeginTable("SessionMetrics", 4, ImGuiTableFlags_SizingStretchSame)) {
        ImGui::TableNextColumn();
        ui_metric("time", "RUN TIME", data ? ui_number(hardware ? hw.frame.sim_time : sim.sim_time(), " s", 1) : "--",
                  hardware ? "Including initial matching" : "Simulation time");
        ImGui::TableNextColumn();
        ui_metric("speed", "BODY SPEED", data ? ui_number(hardware ? hw.frame.navigation_speed : sim.vehicle().speed, " m/s", 3) : "--",
                  hardware ? "Sensor estimate" : "Simulated vehicle", ImVec4(0.48f, 0.73f, 1, 1));
        ImGui::TableNextColumn();
        ui_metric("track", "CROSS-TRACK ERROR", data && reference ? ui_number(100 * (hardware ? hw.frame.tracker_cross_track_error : sim.tracker_cross_track_error()), " cm", 1) : "--",
                  reference ? "Relative to active reference" : "No active MPC reference", kUiWarning);
        ImGui::TableNextColumn();
        const std::string goal = gates ? (data ? std::to_string(hardware ? hw.frame.passed_gates : sim.passed_gate_count()) : "--") :
            (data ? (hardware ? hardware_goal_distance_label(hw.frame, mode) : ui_number(sim.distance_to_goal(), " m")) : "--");
        ui_metric("goal", gates ? "GATES CROSSED" : "DISTANCE TO GOAL", goal,
                  gates ? "Exploration progress" : "Mission target", ImVec4(0.75f, 0.67f, 0.96f, 1));
        ImGui::EndTable();
    }
}

void render_live_inspector(const PlannerDrivenVehicleSim& sim, const HardwareViewerState& hardware,
                           UiState* state, LiveViewStreamServer* server, const SlamToolboxSnapshot* slam) {
    const bool hw = workspace_source_is_hardware(state->workspace_source);
    const bool data = hw ? hardware.frame.has_latest_sample : !sim.history().empty();
    const auto& frame = hardware.frame;
    if (ImGui::BeginChild("LiveInspector", ImVec2(0, 0), true)) {
        ImGui::TextUnformatted("Live inspector");
        ImGui::TextDisabled(hw ? "Robot / latest received sample" : "Simulation / current state");
        ImGui::Spacing();
        if (!data) {
            ui_badge(hw ? "Waiting for telemetry" : "Ready to simulate", kUiMuted);
            ImGui::Spacing();
            ImGui::TextWrapped(hw ? "Select the scene in Setup, start the listener, then launch the runner. Live values appear after the first sample."
                                 : "Review the scene in Setup, then run the simulation or advance a single step.");
            ImGui::Spacing();
            if (ImGui::Button("Open setup", ImVec2(-1, 34))) select_workspace_view(state, kWorkspaceViewSetup);
        } else {
            ImGui::SeparatorText("Control");
            const bool reference = hw ? frame.planner_has_reference : !sim.planned_trajectory().empty();
            ui_detail("Reference", reference ? "Ready" : "Waiting", reference ? kUiAccent : kUiWarning);
            if (hw) {
                const char* control = "Hold";
                switch (frame.control_source) {
                    case thesis_sim::HardwareControlSource::StructuredMpc: control = "MPC / road"; break;
                    case thesis_sim::HardwareControlSource::GateMpc: control = "MPC / gate"; break;
                    case thesis_sim::HardwareControlSource::FrontierMpc: control = "MPC / frontier"; break;
                    case thesis_sim::HardwareControlSource::Recovery: control = "Recovery"; break;
                    case thesis_sim::HardwareControlSource::SafetyStop: control = "Safety stop"; break;
                    case thesis_sim::HardwareControlSource::StraightExploration: control = "Exploring forward"; break;
                    case thesis_sim::HardwareControlSource::ForwardSearch: control = "Searching for a gate"; break;
                    default: break;
                }
                ui_detail("Control source", control);
                ui_detail("Gates crossed", std::to_string(frame.passed_gates));
                ui_detail("Safety", frame.safety_stop_active ? "STOP ACTIVE" : "No stop reported",
                          frame.safety_stop_active ? kUiDanger : kUiAccent);
                if (!frame.reference_invalidation_reason.empty())
                    ImGui::TextWrapped("Reference: %s", frame.reference_invalidation_reason.c_str());
            } else {
                ui_detail("Controller", thesis_sim::tracking_controller_mode_name(sim.tracking_controller_mode()));
            }
            ui_detail("Heading error", reference ? ui_number(hw ? frame.tracker_heading_error_deg : sim.tracker_heading_error_deg(), " deg", 1) : "--");
            const bool lidar_valid = hw ? frame.valid_lidar_samples > 0 : sim.lidar_enabled() && !sim.lidar_hits().empty();
            ui_detail(hw ? "Front LiDAR" : "Nearest LiDAR", lidar_valid ? ui_number(hw ? frame.front_lidar_distance : sim.min_lidar_distance(), " m") : "--");
            ImGui::SeparatorText("Localization");
            const Vec2 position = hw ? frame.navigation_position : sim.navigation_position();
            ui_detail("Position x / y", ui_number(position.x, "") + " / " + ui_number(position.y, " m"));
            ui_detail("Yaw", ui_number((hw ? frame.navigation_yaw : sim.navigation_yaw()) * 180.0 / 3.141592653589793, " deg", 1));
            if (hw) {
                ui_detail("Start matching", !frame.start_matching_enabled ? "Disabled" : frame.start_matching_accepted ? "Accepted" : frame.start_matching_complete ? "Rejected" : "In progress",
                          frame.start_matching_accepted ? kUiAccent : kUiMuted);
                if (frame.start_matching_accepted) {
                    const auto& s = frame.latest_sample;
                    ui_detail("Initial dx / dy", ui_number(s.start_matching_offset_x_m * 100, "", 1) + " / " + ui_number(s.start_matching_offset_y_m * 100, " cm", 1));
                    ui_detail("Initial yaw offset", ui_number(s.start_matching_offset_yaw_deg, " deg", 1));
                }
                ui_detail("SLAM", !frame.slam_enabled ? "Disabled" : frame.slam_connected ? (frame.slam_pose_valid ? "Pose available" : "Waiting for pose") : "Local grid fallback");
                ImGui::SeparatorText("Drivetrain");
                ui_detail("PWM echo L / R", std::to_string(frame.latest_sample.controller_pwm_left) + " / " + std::to_string(frame.latest_sample.controller_pwm_right));
                ui_detail("Encoder delta L / R", std::to_string(frame.vehicle.left_encoder_delta) + " / " + std::to_string(frame.vehicle.right_encoder_delta));
                ui_detail("Received", state->last_hardware_frame_at >= 0 ? ui_number(std::max(0.0, ImGui::GetTime() - state->last_hardware_frame_at), " s ago", 1) : "--");
            } else {
                ui_detail("SLAM", slam && slam->connected ? "Bridge connected" : "LiDAR reconstruction");
                ui_detail("Gates crossed", std::to_string(sim.passed_gate_count()));
                ui_detail("Compute / step", ui_number(sim.history().back().step_ms, " ms", 1));
            }
            ImGui::Spacing();
            if (ImGui::Button("Inspect signals", ImVec2(-1, 34))) select_workspace_view(state, kWorkspaceViewAnalytics);
        }
        if (hw && !server->last_error().empty()) {
            ImGui::Separator();
            ImGui::TextWrapped("Connection: %s", server->last_error().c_str());
        }
    }
    ImGui::EndChild();
}

void render_run_review(const PlannerDrivenVehicleSim& sim, const HardwareViewerState& hardware, bool hw) {
    if (ImGui::BeginChild("RunReview", ImVec2(0, 0), true)) {
        ImGui::PushFont(ui_title_font);
        ImGui::TextUnformatted("Run summary");
        ImGui::PopFont();
        const auto quality = hw ? summarize_run_quality(hardware.history) : summarize_run_quality(sim.history());
        if (!quality.has_data) {
            ImGui::TextWrapped("Run a simulation or receive robot telemetry to create a report.");
        } else {
            ImGui::TextDisabled("Statistics over the available telemetry history");
            ui_detail("Samples", std::to_string(hw ? hardware.history.size() : sim.history().size()));
            ui_detail("Mean / peak speed", ui_number(quality.avg_speed, "") + " / " + ui_number(quality.max_speed, " m/s"));
            ui_detail("Max cross-track", ui_number(quality.max_cross_track * 100, " cm", 1));
            ui_detail("Max heading error", ui_number(quality.max_heading_error_deg, " deg", 1));
            ui_detail("Mean / peak compute", ui_number(quality.avg_step_ms, "", 1) + " / " + ui_number(quality.max_step_ms, " ms", 1));
            ImGui::SeparatorText("Mission events");
            render_timeline_events(hw ? build_hardware_timeline(hardware) : build_sim_timeline(sim), "ReviewEvents");
        }
    }
    ImGui::EndChild();
}

void render_workspace(PlannerDrivenVehicleSim& sim, HardwareViewerState* hardware, UiState* state,
                      LiveViewStreamServer* server, const SlamToolboxSnapshot* slam) {
    ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    const ImGuiWindowFlags flags = ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoSavedSettings;
    if (!ImGui::Begin("Navigation Lab", nullptr, flags)) { ImGui::End(); return; }
    render_workspace_top_bar(sim, *hardware, state, server);
    const bool hw = workspace_source_is_hardware(state->workspace_source);
    if (!hw && !ImGui::GetIO().WantTextInput && !ImGui::IsAnyItemActive() &&
        !ImGui::IsPopupOpen("", ImGuiPopupFlags_AnyPopupId | ImGuiPopupFlags_AnyPopupLevel) &&
        ImGui::IsKeyPressed(ImGuiKey_Space, false) && !sim.goal_reached() && !sim.collision())
        state->paused = !state->paused;
    const int view = state->workspace_view;
    if (view != kWorkspaceViewMap) state->map_editor_enabled = false;
    if (view == kWorkspaceViewMission) {
        render_session_controls(sim, state, server);
        render_session_metrics(sim, *hardware, *state);
    } else {
        const char* subtitles[] = {"", "Signals over time / zoom, pan and compare", "Edit the scene, then apply changes",
                                   "Sensors, localization and controller internals", "Save and review this run",
                                   "Configure the scenario, robot and sensors"};
        ImGui::TextDisabled("%s", subtitles[std::clamp(view, 0, 5)]);
        ImGui::Spacing();
    }
    auto map = [&]() {
        if (hw) render_hardware_world_tab(*hardware, state, slam);
        else render_world_tab(sim, state, slam);
    };
    auto config = [&]() {
        if (hw) render_hardware_control_panel(*hardware, state, server, slam);
        else render_control_panel(sim, state, server);
    };
    ImGui::PushID(view);
    if (view == kWorkspaceViewAnalytics) {
        if (hw) render_hardware_graphs_tab(*hardware, state);
        else render_graphs_tab(sim, state);
    } else if (view == kWorkspaceViewMission && !state->show_inspector) {
        map();
    } else if (ImGui::BeginTable("WorkspaceColumns", 2, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_Resizable)) {
        const bool overview = view == kWorkspaceViewMission;
        const float width = ImGui::GetContentRegionAvail().x;
        ImGui::TableSetupColumn(overview ? "Map" : "Configuration", overview ? ImGuiTableColumnFlags_WidthStretch : ImGuiTableColumnFlags_WidthFixed,
                               overview ? 1.0f : std::clamp(width * 0.35f, 360.0f, 480.0f));
        ImGui::TableSetupColumn(overview ? "Inspector" : "Content", overview ? ImGuiTableColumnFlags_WidthFixed : ImGuiTableColumnFlags_WidthStretch,
                               overview ? std::clamp(width * 0.25f, 296.0f, 350.0f) : 1.0f);
        ImGui::TableNextColumn();
        if (overview) map(); else config();
        ImGui::TableNextColumn();
        if (overview) render_live_inspector(sim, *hardware, state, server, slam);
        else if (view == kWorkspaceViewExport) render_run_review(sim, *hardware, hw);
        else if (view == kWorkspaceViewDiagnostics) {
            if (hw) render_hardware_graphs_tab(*hardware, state); else render_graphs_tab(sim, state);
        } else map();
        ImGui::EndTable();
    }
    ImGui::PopID();
    ImGui::End();
}
