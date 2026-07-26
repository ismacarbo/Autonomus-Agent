MapEditorHandle pick_editor_handle(const WorldMap& world, const CanvasTransform& tx, const ImVec2& mouse_pos) {
    constexpr float kHandleRadius = 12.0f;
    const float radius_sq = kHandleRadius * kHandleRadius;

    MapEditorHandle best;
    float best_dist_sq = radius_sq;

    const auto test_handle = [&](MapEditorHandleType type, int index, const Vec2& world_pos) {
        const float d_sq = distance_sq(world_to_screen(tx, world_pos), mouse_pos);
        if (d_sq <= best_dist_sq) {
            best = {type, index};
            best_dist_sq = d_sq;
        }
    };

    test_handle(MapEditorHandleType::Start, -1, world.start());
    test_handle(MapEditorHandleType::Goal, -1, world.goal());

    for (size_t i = 0; i < world.obstacles().size(); ++i) {
        const Rect& obstacle = world.obstacles()[i];
        const Vec2 center{
            0.5 * (obstacle.min_x + obstacle.max_x),
            0.5 * (obstacle.min_y + obstacle.max_y),
        };
        test_handle(MapEditorHandleType::Obstacle, static_cast<int>(i), center);
    }

    if (world.environment_mode() == EnvironmentMode::UnstructuredGates) {
        for (size_t i = 0; i < world.gates().size(); ++i) {
            const GateSpec& gate = world.gates()[i];
            if (gate.final) {
                continue;
            }
            test_handle(MapEditorHandleType::Gate, static_cast<int>(i), gate.anchor_position);
        }
    } else {
        for (size_t i = 1; i + 1 < world.road_centerline().size(); ++i) {
            test_handle(MapEditorHandleType::RoadPoint, static_cast<int>(i), world.road_centerline()[i]);
        }
    }

    return best;
}

void draw_editor_overlay(ImDrawList* draw_list, const CanvasTransform& tx, const WorldMap& world, const UiState& ui_state) {
    if (!ui_state.map_editor_enabled) {
        return;
    }

    for (const Rect& obstacle : world.obstacles()) {
        draw_list->AddRect(
            world_to_screen(tx, {obstacle.min_x, obstacle.max_y}),
            world_to_screen(tx, {obstacle.max_x, obstacle.min_y}),
            kColorEditorOverlay,
            4.0f,
            0,
            2.0f);

        const Vec2 center{
            0.5 * (obstacle.min_x + obstacle.max_x),
            0.5 * (obstacle.min_y + obstacle.max_y),
        };
        const ImVec2 screen_center = world_to_screen(tx, center);
        draw_list->AddCircleFilled(screen_center, 6.0f, kColorEditorHandle);
    }

    draw_list->AddCircleFilled(
        world_to_screen(tx, world.start()),
        7.0f,
        ui_state.selected_editor_handle.type == MapEditorHandleType::Start ? kColorEditorSelected : IM_COL32(120, 220, 150, 255));
    draw_list->AddCircleFilled(
        world_to_screen(tx, world.goal()),
        7.0f,
        ui_state.selected_editor_handle.type == MapEditorHandleType::Goal ? kColorEditorSelected : kColorGoal);

    if (world.environment_mode() == EnvironmentMode::UnstructuredGates) {
        for (size_t i = 0; i < world.gates().size(); ++i) {
            const GateSpec& gate = world.gates()[i];
            if (gate.final) {
                continue;
            }
            const bool selected =
                ui_state.selected_editor_handle.type == MapEditorHandleType::Gate &&
                ui_state.selected_editor_handle.index == static_cast<int>(i);
            draw_list->AddCircleFilled(
                world_to_screen(tx, gate.anchor_position),
                6.0f,
                selected ? kColorEditorSelected : kColorEditorHandle);
        }
    } else {
        draw_polyline(draw_list, tx, world.road_centerline(), kColorEditorOverlay, 2.0f);
        for (size_t i = 1; i + 1 < world.road_centerline().size(); ++i) {
            const bool selected =
                ui_state.selected_editor_handle.type == MapEditorHandleType::RoadPoint &&
                ui_state.selected_editor_handle.index == static_cast<int>(i);
            draw_list->AddCircleFilled(
                world_to_screen(tx, world.road_centerline()[i]),
                5.0f,
                selected ? kColorEditorSelected : kColorEditorHandle);
        }
    }
}

void update_editor_drag(UiState* ui_state, const CanvasTransform& tx, const ImVec2& mouse_pos) {
    if (ui_state == nullptr || !ui_state->map_editor_enabled) {
        return;
    }

    WorldMap* world_ptr = active_editor_world(ui_state);
    bool* dirty_flag = active_editor_dirty_flag(ui_state);
    if (world_ptr == nullptr || dirty_flag == nullptr) {
        return;
    }
    WorldMap& world = *world_ptr;
    const bool mouse_down = ImGui::IsMouseDown(ImGuiMouseButton_Left);

    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        ui_state->selected_editor_handle = pick_editor_handle(world, tx, mouse_pos);
        ui_state->active_drag_handle = ui_state->selected_editor_handle;

        const Vec2 world_mouse = screen_to_world(tx, mouse_pos);
        switch (ui_state->active_drag_handle.type) {
            case MapEditorHandleType::Start:
                ui_state->drag_offset = {world.start().x - world_mouse.x, world.start().y - world_mouse.y};
                break;
            case MapEditorHandleType::Goal:
                ui_state->drag_offset = {world.goal().x - world_mouse.x, world.goal().y - world_mouse.y};
                break;
            case MapEditorHandleType::Obstacle:
                if (ui_state->active_drag_handle.index >= 0 &&
                    ui_state->active_drag_handle.index < static_cast<int>(world.editable_obstacles().size())) {
                    const Rect& obstacle = world.editable_obstacles()[ui_state->active_drag_handle.index];
                    const Vec2 center{
                        0.5 * (obstacle.min_x + obstacle.max_x),
                        0.5 * (obstacle.min_y + obstacle.max_y),
                    };
                    ui_state->drag_offset = {center.x - world_mouse.x, center.y - world_mouse.y};
                }
                break;
            case MapEditorHandleType::Gate:
                if (ui_state->active_drag_handle.index >= 0 &&
                    ui_state->active_drag_handle.index < static_cast<int>(world.editable_gates().size())) {
                    const GateSpec& gate = world.editable_gates()[ui_state->active_drag_handle.index];
                    ui_state->drag_offset = {gate.anchor_position.x - world_mouse.x, gate.anchor_position.y - world_mouse.y};
                }
                break;
            case MapEditorHandleType::RoadPoint:
                if (ui_state->active_drag_handle.index >= 0 &&
                    ui_state->active_drag_handle.index < static_cast<int>(world.editable_road_centerline().size())) {
                    const Vec2& point = world.editable_road_centerline()[ui_state->active_drag_handle.index];
                    ui_state->drag_offset = {point.x - world_mouse.x, point.y - world_mouse.y};
                }
                break;
            default:
                break;
        }
    }

    if (!mouse_down) {
        ui_state->active_drag_handle = {};
        return;
    }

    if (ui_state->active_drag_handle.type == MapEditorHandleType::None) {
        return;
    }

    const Vec2 world_mouse = screen_to_world(tx, mouse_pos);
    const Vec2 new_pos{
        world_mouse.x + ui_state->drag_offset.x,
        world_mouse.y + ui_state->drag_offset.y,
    };

    switch (ui_state->active_drag_handle.type) {
        case MapEditorHandleType::Start:
            world.set_start(new_pos);
            *dirty_flag = true;
            break;
        case MapEditorHandleType::Goal:
            world.set_goal(new_pos);
            *dirty_flag = true;
            break;
        case MapEditorHandleType::Obstacle:
            if (ui_state->active_drag_handle.index >= 0 &&
                ui_state->active_drag_handle.index < static_cast<int>(world.editable_obstacles().size())) {
                Rect& obstacle = world.editable_obstacles()[ui_state->active_drag_handle.index];
                const double half_w = 0.5 * (obstacle.max_x - obstacle.min_x);
                const double half_h = 0.5 * (obstacle.max_y - obstacle.min_y);
                obstacle.min_x = new_pos.x - half_w;
                obstacle.max_x = new_pos.x + half_w;
                obstacle.min_y = new_pos.y - half_h;
                obstacle.max_y = new_pos.y + half_h;
                *dirty_flag = true;
            }
            break;
        case MapEditorHandleType::Gate:
            if (ui_state->active_drag_handle.index >= 0 &&
                ui_state->active_drag_handle.index < static_cast<int>(world.editable_gates().size())) {
                GateSpec& gate = world.editable_gates()[ui_state->active_drag_handle.index];
                gate.anchor_position = new_pos;
                gate.position = new_pos;
                *dirty_flag = true;
            }
            break;
        case MapEditorHandleType::RoadPoint:
            if (ui_state->active_drag_handle.index >= 0 &&
                ui_state->active_drag_handle.index < static_cast<int>(world.editable_road_centerline().size())) {
                world.editable_road_centerline()[ui_state->active_drag_handle.index] = new_pos;
                *dirty_flag = true;
            }
            break;
        default:
            break;
    }
}

void render_world_tab(PlannerDrivenVehicleSim& sim,
                      UiState* ui_state,
                      const SlamToolboxSnapshot* slam_snapshot) {
    if (ui_state == nullptr) {
        return;
    }

    if (!ImGui::BeginChild("WorldTabRoot", ImVec2(0.0f, 0.0f), false)) {
        ImGui::EndChild();
        return;
    }

    const std::string active_target =
        (sim.environment_mode() == EnvironmentMode::UnstructuredGates ||
         (sim.environment_mode() == EnvironmentMode::MixedRoadGates && sim.chosen_gate_index() >= 0))
            ? simulation_gate_label(sim, sim.chosen_gate_index())
            : (sim.environment_mode() == EnvironmentMode::MixedRoadGates ? "road / dynamic gate arbiter"
                                                                          : thesis_sim::structured_map_preset_name(sim.world().structured_preset()));

    char speed_buf[32];
    char goal_buf[32];
    char tracking_buf[32];
    char target_buf[48];
    std::snprintf(speed_buf, sizeof(speed_buf), "%.2f m/s", sim.vehicle().speed);
    std::snprintf(goal_buf, sizeof(goal_buf), "%.2f m", sim.distance_to_goal());
    std::snprintf(tracking_buf, sizeof(tracking_buf), "%.2f m / %.1f deg", sim.tracker_cross_track_error(), sim.tracker_heading_error_deg());
    std::snprintf(target_buf, sizeof(target_buf), "%s", active_target.c_str());

    if (ImGui::BeginChild("WorldToolbar", ImVec2(0.0f, 102.0f), true)) {
        if (ImGui::BeginTable("WorldToolbarLayout", 2, ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn("WorldMetrics", ImGuiTableColumnFlags_WidthStretch, 1.0f);
            ImGui::TableSetupColumn("WorldControls", ImGuiTableColumnFlags_WidthFixed, 320.0f);

            ImGui::TableNextColumn();
            if (ImGui::BeginTable("WorldMetricCards", 4, ImGuiTableFlags_SizingStretchSame)) {
                ImGui::TableNextColumn();
                metric_card("world_speed", "Vehicle", speed_buf, "Live speed", ImVec4(0.43f, 0.82f, 0.96f, 1.0f), 64.0f);
                ImGui::TableNextColumn();
                metric_card("world_goal", "Goal", goal_buf, "Distance remaining", ImVec4(0.99f, 0.70f, 0.32f, 1.0f), 64.0f);
                ImGui::TableNextColumn();
                metric_card("world_tracking", "Tracking", tracking_buf, "cte / heading", ImVec4(0.97f, 0.89f, 0.45f, 1.0f), 64.0f);
                ImGui::TableNextColumn();
                metric_card("world_target", "Target", target_buf,
                            sim.environment_mode() == EnvironmentMode::UnstructuredGates
                                ? "Active planner gate"
                                : (sim.environment_mode() == EnvironmentMode::MixedRoadGates ? "Mixed arbitration" : "Active structured loop"),
                            ImVec4(0.48f, 0.88f, 0.62f, 1.0f),
                            64.0f);
                ImGui::EndTable();
            }

            ImGui::TableNextColumn();
            ImGui::TextDisabled("VIEWPORT LAYERS");
            ImGui::Checkbox("Grid", &ui_state->show_grid);
            ImGui::SameLine();
            ImGui::Checkbox("Trails", &ui_state->show_trails);
            ImGui::Checkbox("LiDAR Rays", &ui_state->show_lidar_rays);
            ImGui::SameLine();
            ImGui::Checkbox("LiDAR Hits", &ui_state->show_lidar_hits);
            ImGui::Checkbox("Labels", &ui_state->show_gate_labels);
            ImGui::Checkbox("HUD", &ui_state->show_world_hud);
            ImGui::SameLine();
            ImGui::TextDisabled("%s", ui_state->map_editor_enabled ? "Editor enabled" : "Editor off");
            ImGui::TextWrapped("Use the viewport for live diagnosis first, then enable drag editing only when you want to reshape the map.");
            ImGui::EndTable();
        }
    }
    ImGui::EndChild();

    if (!ImGui::BeginChild("SimulationViewport", ImVec2(0.0f, 0.0f), true, ImGuiWindowFlags_NoScrollbar)) {
        ImGui::EndChild();
        ImGui::EndChild();
        return;
    }

    const ImVec2 viewport_pos = ImGui::GetCursorScreenPos();
    ImVec2 viewport_size = ImGui::GetContentRegionAvail();
    viewport_size.x = std::max(viewport_size.x, 640.0f);
    viewport_size.y = std::max(viewport_size.y, 320.0f);
    const float panel_gap = 10.0f;
    const float slam_width = std::clamp(viewport_size.x * 0.34f, 260.0f, 390.0f);
    const ImVec2 canvas_pos = viewport_pos;
    const ImVec2 canvas_size(viewport_size.x - slam_width - panel_gap, viewport_size.y);
    const ImVec2 slam_pos(canvas_pos.x + canvas_size.x + panel_gap, canvas_pos.y);
    const ImVec2 slam_size(slam_width, viewport_size.y);

    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    draw_viewport_background(draw_list, canvas_pos, canvas_size);

    const CanvasTransform tx = make_transform(sim.world(), canvas_pos, canvas_size);
    const Rect bounds = sim.world().bounds();
    if (ui_state->show_grid) {
        draw_world_grid(draw_list, tx, bounds);
    }

    for (const Rect& obstacle : sim.world().obstacles()) {
        draw_list->AddRectFilled(world_to_screen(tx, {obstacle.min_x, obstacle.max_y}),
                                 world_to_screen(tx, {obstacle.max_x, obstacle.min_y}),
                                 kColorObstacle, 4.0f);
    }
    for (const Rect& obstacle : sim.world().perception_obstacles()) {
        draw_list->AddRectFilled(world_to_screen(tx, {obstacle.min_x, obstacle.max_y}),
                                 world_to_screen(tx, {obstacle.max_x, obstacle.min_y}),
                                 kColorObstacle, 4.0f);
    }

    if (!sim.world().road_centerline().empty()) {
        if (sim.world().environment_mode() == EnvironmentMode::StructuredRoad ||
            sim.world().environment_mode() == EnvironmentMode::MixedRoadGates) {
            draw_structured_road_map(draw_list, tx, sim.world());
        } else {
            draw_polyline(draw_list, tx, sim.world().road_centerline(), IM_COL32(113, 210, 255, 180), 4.0f);
        }
        if (sim.world().environment_mode() == EnvironmentMode::StructuredRoad ||
            sim.world().environment_mode() == EnvironmentMode::MixedRoadGates) {
            draw_goal_marker(draw_list, tx, sim.world().goal());
        }
    }

    if (ui_state->show_trails) {
        draw_polyline(draw_list, tx, sim.trail(), kColorTrail, 2.5f);
        draw_polyline(draw_list, tx, sim.estimated_trail(), kColorEstimateTrail, 1.8f);
        draw_polyline(draw_list, tx, sim.planned_trajectory(), kColorTrajectory, 3.0f);
    }

    if (sim.environment_mode() == EnvironmentMode::MixedRoadGates &&
        sim.config().dynamic_lidar_gates) {
        for (const GateSpec& gate : sim.world().gates()) {
            const bool checkpoint_marker =
                gate.final &&
                (gate.name.find("checkpoint") != std::string::npos ||
                 gate.name.find("finish") != std::string::npos);
            if (gate.final && !checkpoint_marker) {
                continue;
            }
            const ImVec2 screen_pos = world_to_screen(tx, gate.position);
            const ImU32 checkpoint_color =
                gate.name.find("finish") != std::string::npos
                    ? IM_COL32(124, 238, 151, 210)
                    : IM_COL32(255, 214, 112, 190);
            draw_list->AddCircle(screen_pos, checkpoint_marker ? 8.0f : 7.0f,
                                 checkpoint_marker ? checkpoint_color : IM_COL32(255, 184, 83, 170),
                                 0,
                                 2.0f);
            draw_list->AddLine(
                ImVec2(screen_pos.x - 6.0f, screen_pos.y),
                ImVec2(screen_pos.x + 6.0f, screen_pos.y),
                checkpoint_marker ? checkpoint_color : IM_COL32(255, 184, 83, 130),
                1.6f);
            if (ui_state->show_gate_labels) {
                draw_list->AddText(
                    ImVec2(screen_pos.x + 7.0f, screen_pos.y + 2.0f),
                    IM_COL32(255, 214, 152, 190),
                    gate.name.c_str());
            }
        }
    }

    if (sim.lidar_enabled() && (ui_state->show_lidar_rays || ui_state->show_lidar_hits)) {
        draw_lidar_scan(
            draw_list,
            tx,
            sim.vehicle().position,
            sim.lidar_hits(),
            ui_state->show_lidar_rays,
            ui_state->show_lidar_hits);
    }

    for (size_t i = 0; i < sim.gates().size(); ++i) {
        const Vec2 gate_pos = simulation_gate_position(sim, i);
        const bool final_gate = simulation_gate_is_final(sim, i);
        const std::string gate_label = simulation_gate_label(sim, static_cast<int>(i));
        const auto screen_pos = world_to_screen(tx, gate_pos);
        const bool visible =
            std::find(sim.visible_gate_indices().begin(), sim.visible_gate_indices().end(), static_cast<int>(i)) !=
            sim.visible_gate_indices().end();

        ImU32 color = final_gate ? kColorGoal : kColorGate;
        if (sim.gates()[i].passed) {
            color = kColorGatePassed;
        } else if (visible) {
            color = kColorGateVisible;
        }
        if (static_cast<int>(i) == sim.chosen_gate_index()) {
            color = IM_COL32(255, 233, 118, 255);
        }

        if (final_gate) {
            draw_goal_marker(draw_list, tx, gate_pos);
        } else {
            draw_oriented_gate(
                draw_list,
                tx,
                gate_pos,
                simulation_gate_heading(sim, i),
                std::max(sim.geometry().body_width * 1.35, 0.14),
                color,
                static_cast<int>(i) == sim.chosen_gate_index());
        }
        if (ui_state->show_gate_labels) {
            draw_list->AddText(ImVec2(screen_pos.x + 6.0f, screen_pos.y - 12.0f), IM_COL32(222, 227, 230, 255), gate_label.c_str());
        }
    }

    const float vehicle_visual_scale = simulation_vehicle_visual_scale_for_world(sim.world());
    draw_vehicle(draw_list,
                 tx,
                 sim.vehicle(),
                 sim.geometry(),
                 vehicle_visual_scale,
                 hardware_vehicle_body_color_for_world(sim.world()));
    const Vec2 nav_pos = sim.navigation_position();
    const Vec2 nav_nose{
        nav_pos.x + std::cos(sim.navigation_yaw()) * sim.geometry().body_length * 0.85 * vehicle_visual_scale,
        nav_pos.y + std::sin(sim.navigation_yaw()) * sim.geometry().body_length * 0.85 * vehicle_visual_scale,
    };
    draw_list->AddCircle(world_to_screen(tx, nav_pos), 11.0f, kColorEstimateTrail, 0, 2.8f);
    draw_list->AddLine(world_to_screen(tx, nav_pos), world_to_screen(tx, nav_nose), kColorEstimateTrail, 3.2f);

    const std::vector<Vec2>& rendered_slam_points =
        slam_snapshot != nullptr && slam_snapshot->connected &&
                !slam_snapshot->occupied_points.empty()
            ? slam_snapshot->occupied_points
            : sim.slam_points();
    draw_slam_reconstruction(
        draw_list,
        sim.world(),
        slam_pos,
        slam_size,
        rendered_slam_points,
        sim.estimated_trail(),
        &nav_pos,
        ui_state->show_grid,
        slam_snapshot);

    if (ui_state->show_world_hud) {
        char hud_line[96];
        std::vector<OverlayLine> top_left = {
            {"Autonomous Reasoning Engine", IM_COL32(240, 243, 235, 255)},
            {thesis_sim::environment_mode_name(sim.environment_mode()), IM_COL32(170, 179, 185, 255)},
            {map_preset_name(sim.world()),
             IM_COL32(170, 179, 185, 255)},
            {thesis_sim::range_sensor_profile_name(sim.range_sensor_profile()), IM_COL32(170, 179, 185, 255)},
        };
        draw_overlay_panel(draw_list, ImVec2(canvas_pos.x + 16.0f, canvas_pos.y + 16.0f), 228.0f, top_left);

        std::snprintf(hud_line, sizeof(hud_line), "speed %.2f m/s", sim.vehicle().speed);
        std::vector<OverlayLine> top_right = {
            {sim.goal_reached() ? "Mission complete" : (sim.collision() ? "Collision detected" : "Mission active"),
             sim.goal_reached() ? IM_COL32(124, 238, 151, 255)
                                : (sim.collision() ? IM_COL32(255, 124, 102, 255) : IM_COL32(255, 221, 113, 255))},
            {hud_line, IM_COL32(240, 243, 235, 255)},
        };
        std::snprintf(hud_line, sizeof(hud_line), "goal %.2f m", sim.distance_to_goal());
        top_right.push_back({hud_line, IM_COL32(170, 179, 185, 255)});
        if (sim.environment_mode() == EnvironmentMode::UnstructuredGates ||
            sim.environment_mode() == EnvironmentMode::MixedRoadGates) {
            std::snprintf(hud_line, sizeof(hud_line), "gate %s | visible %d", active_target.c_str(), static_cast<int>(sim.visible_gate_indices().size()));
        } else {
            std::snprintf(hud_line, sizeof(hud_line), "road pts %d", static_cast<int>(sim.world().road_centerline().size()));
        }
        top_right.push_back({hud_line, IM_COL32(170, 179, 185, 255)});
        draw_overlay_panel(draw_list, ImVec2(canvas_pos.x + canvas_size.x - 236.0f, canvas_pos.y + 16.0f), 220.0f, top_right);

        std::vector<OverlayLine> legend = {
            {"blue: ground truth trail", kColorTrail},
            {"orange: fused state / estimated trail", kColorEstimateTrail},
            {"yellow: selected planner trajectory", kColorTrajectory},
        };
        const bool aruco_reference_mixed_map =
            sim.world().environment_mode() == EnvironmentMode::MixedRoadGates &&
            sim.world().structured_preset() == StructuredMapPreset::TankCircuit &&
            !sim.world().obstacles().empty();
        if (!aruco_reference_mixed_map &&
            (sim.world().environment_mode() == EnvironmentMode::StructuredRoad ||
             sim.world().environment_mode() == EnvironmentMode::MixedRoadGates)) {
            legend.push_back({"red: structured road bounds", IM_COL32(255, 96, 96, 230)});
        }
        if (sim.lidar_enabled() && ui_state->show_lidar_rays) {
            legend.push_back({"green: LiDAR hit rays", kColorLidar});
        }
        if (sim.lidar_enabled() && ui_state->show_lidar_hits) {
            legend.push_back({"lime/cyan: LiDAR collision points", kColorLidarHit});
        }
        const float legend_height = 20.0f + static_cast<float>(legend.size()) * (ImGui::GetFontSize() + 5.0f);
        draw_overlay_panel(draw_list,
                           ImVec2(canvas_pos.x + 16.0f, canvas_pos.y + canvas_size.y - legend_height),
                           286.0f,
                           legend);
    }

    if (ui_state != nullptr && ui_state->map_editor_enabled) {
        if (ui_state->show_world_hud) {
            draw_overlay_panel(draw_list,
                               ImVec2(canvas_pos.x + 16.0f, canvas_pos.y + 130.0f),
                               286.0f,
                               {{"Map editor active", kColorEditorOverlay},
                                {"Drag handles in the viewport, then apply changes.", IM_COL32(170, 179, 185, 255)}});
        }
        draw_editor_overlay(draw_list, tx, ui_state->scenario_editor_world, *ui_state);

        const ImVec2 mouse_pos = ImGui::GetIO().MousePos;
        const bool inside_canvas =
            mouse_pos.x >= canvas_pos.x && mouse_pos.x <= canvas_pos.x + canvas_size.x &&
            mouse_pos.y >= canvas_pos.y && mouse_pos.y <= canvas_pos.y + canvas_size.y;
        if (inside_canvas && ImGui::IsWindowHovered(ImGuiHoveredFlags_AllowWhenBlockedByActiveItem)) {
            update_editor_drag(ui_state, tx, mouse_pos);
        } else if (ui_state->active_drag_handle.type != MapEditorHandleType::None &&
                   !ImGui::IsMouseDown(ImGuiMouseButton_Left)) {
            ui_state->active_drag_handle = {};
        }
    }

    ImGui::Dummy(viewport_size);
    ImGui::EndChild();
    ImGui::EndChild();
}

void render_plot_window(const char* title,
                        const char* y_axis_label,
                        const char* description,
                        const std::vector<double>& x,
                        const std::vector<double>& y0,
                        const char* label0,
                        const std::vector<double>* y1 = nullptr,
                        const char* label1 = nullptr,
                        const std::vector<double>* y2 = nullptr,
                        const char* label2 = nullptr) {
    ImGui::PushID(title);
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted(title);
    if (description != nullptr) {
        ImGui::SameLine();
        help_marker(description);
    }
    ImGui::SameLine();
    ImGui::TextDisabled("latest %.2fs", x.empty() ? 0.0 : x.back());

    if (!ImPlot::BeginPlot("##plot", ImVec2(-1, 220.0f), ImPlotFlags_NoMenus | ImPlotFlags_NoBoxSelect)) {
        ImGui::PopID();
        return;
    }

    const double x_min = x.empty() ? 0.0 : x.front();
    const double x_max = x.size() > 1 ? x.back() : (x_min + 1.0);
    ImPlot::SetupAxes("t [s]", y_axis_label, ImPlotAxisFlags_None, ImPlotAxisFlags_AutoFit);
    ImPlot::SetupLegend(ImPlotLocation_NorthEast, ImPlotLegendFlags_None);
    ImPlot::SetupAxisLimits(ImAxis_X1, x_min, x_max, ImGuiCond_Always);

    ImPlot::SetNextLineStyle(ImVec4(0.36f, 0.73f, 0.98f, 1.0f), 2.0f);
    ImPlot::PlotLine(label0, x.data(), y0.data(), static_cast<int>(x.size()));
    if (y1 != nullptr && label1 != nullptr) {
        ImPlot::SetNextLineStyle(ImVec4(0.96f, 0.66f, 0.28f, 1.0f), 2.0f);
        ImPlot::PlotLine(label1, x.data(), y1->data(), static_cast<int>(x.size()));
    }
    if (y2 != nullptr && label2 != nullptr) {
        ImPlot::SetNextLineStyle(ImVec4(0.48f, 0.87f, 0.60f, 1.0f), 2.0f);
        ImPlot::PlotLine(label2, x.data(), y2->data(), static_cast<int>(x.size()));
    }

    render_plot_hover_overlay(
        title,
        x,
        {
            {&y0, label0, ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
            {y1, label1, ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
            {y2, label2, ImVec4(0.48f, 0.87f, 0.60f, 1.0f)},
        });
    ImPlot::EndPlot();
    ImGui::PopID();
}

void plot_series(const std::vector<double>& time,
                 const std::vector<double>& series,
                 const char* label,
                 const ImVec4& color) {
    if (time.empty() || series.empty()) {
        return;
    }
    ImPlot::SetNextLineStyle(color, 2.0f);
    ImPlot::PlotLine(label, time.data(), series.data(), static_cast<int>(time.size()));
}

void render_plot_hover_overlay(const char* title,
                               const std::vector<double>& x,
                               std::initializer_list<PlotHoverSeries> series_list) {
    if (x.empty() || !ImPlot::IsPlotHovered()) {
        return;
    }

    const ImPlotPoint mouse = ImPlot::GetPlotMousePos();
    const auto it = std::lower_bound(x.begin(), x.end(), mouse.x);
    const size_t idx = static_cast<size_t>(std::clamp<long>(
        static_cast<long>(it == x.end() ? static_cast<long>(x.size()) - 1 : std::distance(x.begin(), it)),
        0L,
        static_cast<long>(x.size()) - 1));

    const double hovered_x = x[idx];
    const ImPlotRect limits = ImPlot::GetPlotLimits();
    ImDrawList* draw_list = ImPlot::GetPlotDrawList();
    const ImVec2 line_top = ImPlot::PlotToPixels(hovered_x, limits.Y.Max);
    const ImVec2 line_bottom = ImPlot::PlotToPixels(hovered_x, limits.Y.Min);
    draw_list->AddLine(line_top, line_bottom, IM_COL32(255, 255, 255, 120), 1.5f);
    ImPlot::TagX(hovered_x, ImVec4(0.95f, 0.96f, 0.98f, 1.0f), "t %.2f s", hovered_x);

    ImGui::BeginTooltip();
    ImGui::Text("%s", title);
    ImGui::Separator();
    ImGui::Text("t = %.2f s", hovered_x);
    for (const PlotHoverSeries& item : series_list) {
        if (item.values == nullptr || item.label == nullptr || idx >= item.values->size()) {
            continue;
        }
        const double y = (*item.values)[idx];
        const ImVec2 point_px = ImPlot::PlotToPixels(hovered_x, y);
        draw_list->AddCircleFilled(point_px, 4.0f, ImGui::ColorConvertFloat4ToU32(item.color));
        ImGui::PushStyleColor(ImGuiCol_Text, item.color);
        ImGui::Text("%s = %.3f", item.label, y);
        ImGui::PopStyleColor();
    }
    ImGui::EndTooltip();
}

