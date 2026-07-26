void render_workspace_navigation_rail(UiState* ui_state) {
    if (ui_state == nullptr) {
        return;
    }

    ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 8.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(10.0f, 12.0f));
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.06f, 0.08f, 0.11f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.17f, 0.23f, 0.28f, 1.0f));
    if (ImGui::BeginChild("WorkspaceNavigationRail", ImVec2(116.0f, 0.0f), true)) {
        ImGui::TextDisabled("Views");
        ImGui::Spacing();

        auto nav_button = [&](const char* label,
                              int view,
                              int sim_panel_tab,
                              int hardware_panel_tab,
                              bool editor_mode) {
            const bool selected = ui_state->workspace_view == view;
            if (selected) {
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.43f, 0.52f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.22f, 0.56f, 0.66f, 1.0f));
                ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.11f, 0.33f, 0.40f, 1.0f));
            }
            if (ImGui::Button(label, ImVec2(-1.0f, 38.0f))) {
                ui_state->workspace_view = view;
                ui_state->simulation_panel_tab = sim_panel_tab;
                ui_state->hardware_panel_tab = hardware_panel_tab;
                ui_state->requested_simulation_panel_tab = sim_panel_tab;
                ui_state->requested_hardware_panel_tab = hardware_panel_tab;
                ui_state->map_editor_enabled = editor_mode;
            }
            if (selected) {
                ImGui::PopStyleColor(3);
            }
        };

        nav_button("Mission", kWorkspaceViewMission, 0, 0, false);
        nav_button("Analytics", kWorkspaceViewAnalytics, 0, 3, false);
        nav_button("Map", kWorkspaceViewMap, 3, 2, true);
        nav_button("Diagnostics", kWorkspaceViewDiagnostics, 2, 3, false);
        nav_button("Export", kWorkspaceViewExport, 4, 1, false);
    }
    ImGui::EndChild();
    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar(2);
}

void render_workspace(PlannerDrivenVehicleSim& sim,
                      HardwareViewerState* hardware,
                      UiState* ui_state,
                      LiveViewStreamServer* hardware_server,
                      const SlamToolboxSnapshot* slam_snapshot) {
    ImGuiViewport* viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    ImGuiWindowFlags flags =
        ImGuiWindowFlags_NoDecoration |
        ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoSavedSettings;

    if (!ImGui::Begin("Autonomous Reasoning Engine Workspace", nullptr, flags)) {
        ImGui::End();
        return;
    }

    render_workspace_top_bar(sim, hardware, ui_state, hardware_server);
    ImGui::Spacing();

    const bool hardware_mode = ui_state != nullptr && workspace_source_is_hardware(ui_state->workspace_source);
    if (ui_state != nullptr) {
        ui_state->workspace_view = std::clamp(
            ui_state->workspace_view,
            static_cast<int>(kWorkspaceViewMission),
            static_cast<int>(kWorkspaceViewExport));
        const bool graph_view =
            ui_state->workspace_view == kWorkspaceViewAnalytics ||
            ui_state->workspace_view == kWorkspaceViewDiagnostics ||
            ui_state->workspace_view == kWorkspaceViewExport;
        ui_state->workspace_tab = graph_view ? 1 : 0;
    }
    const float workspace_width = ImGui::GetContentRegionAvail().x;
    const float context_width = std::clamp(workspace_width * 0.26f, 315.0f, 390.0f);
    if (ImGui::BeginTable("WorkspaceLayout", 3, ImGuiTableFlags_SizingStretchProp | ImGuiTableFlags_BordersInnerV)) {
        ImGui::TableSetupColumn("Navigation", ImGuiTableColumnFlags_WidthFixed, 124.0f);
        ImGui::TableSetupColumn("MainArea", ImGuiTableColumnFlags_WidthStretch, 1.0f);
        ImGui::TableSetupColumn("ContextArea", ImGuiTableColumnFlags_WidthFixed, context_width);

        ImGui::TableNextColumn();
        render_workspace_navigation_rail(ui_state);

        ImGui::TableNextColumn();
        const int view = ui_state != nullptr ? ui_state->workspace_view : kWorkspaceViewMission;
        switch (view) {
            case kWorkspaceViewAnalytics:
            case kWorkspaceViewDiagnostics:
            case kWorkspaceViewExport:
                if (hardware_mode && hardware != nullptr) {
                    render_hardware_graphs_tab(*hardware);
                } else {
                    render_graphs_tab(sim);
                }
                break;
            case kWorkspaceViewMap:
            case kWorkspaceViewMission:
            default:
                if (hardware_mode && hardware != nullptr) {
                    render_hardware_world_tab(*hardware, ui_state, slam_snapshot);
                } else {
                    render_world_tab(sim, ui_state, slam_snapshot);
                }
                break;
        }

        ImGui::TableNextColumn();
        const bool telemetry_context = view == kWorkspaceViewAnalytics;
        if (telemetry_context) {
            if (hardware_mode && hardware != nullptr) {
                render_hardware_telemetry_side_panel(*hardware);
            } else {
                render_sim_telemetry_side_panel(sim);
            }
        } else {
            if (hardware_mode && hardware != nullptr) {
                if (view == kWorkspaceViewAnalytics) {
                    ui_state->hardware_panel_tab = 3;
                }
                render_hardware_control_panel(
                    *hardware, ui_state, hardware_server, slam_snapshot);
            } else {
                if (view == kWorkspaceViewAnalytics) {
                    ui_state->simulation_panel_tab = 0;
                }
                render_control_panel(sim, ui_state, hardware_server);
            }
        }
        ImGui::EndTable();
    }

    ImGui::End();
}
