void render_control_panel(PlannerDrivenVehicleSim& sim, UiState* ui_state, LiveViewStreamServer* hardware_server) {
    if (!ImGui::BeginChild("ConfigPanel", ImVec2(0.0f, 0.0f), true)) {
        ImGui::EndChild();
        return;
    }

    ImGui::PushTextWrapPos(0);
    ImGui::PushItemWidth(ImGui::GetContentRegionAvail().x * 0.55f);
     {
        if (ui_state->workspace_view == kWorkspaceViewSetup) {
            ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
            if (ImGui::CollapsingHeader("Scenario & Planner", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SameLine();
        help_marker("Switch between gate-based and structured-road behaviour without changing the rest of the Autonomous Reasoning Engine stack.");

        int fidelity_selection = sim.config().ideal_conditions
                                     ? 0
                                     : (sim.config().calibrated_conditions ? 1 : 2);
        const char* fidelity_items[] = {
            "Sim-Ideal",
            "Sim-Calibrated",
            "Sim-Reference",
        };
        ImGui::SetNextItemWidth(ImGui::GetContentRegionAvail().x * 0.55f);
        if (ui_labeled_combo(
                "Validation Fidelity",
                &fidelity_selection,
                fidelity_items,
                IM_ARRAYSIZE(fidelity_items))) {
            ui_state->ideal_simulation = fidelity_selection == 0;
            ui_state->calibrated_simulation = fidelity_selection == 1;
            sim.load_world(world_from_ui_selection(sim, *ui_state));
            apply_gui_stack_for_selected_map(sim, ui_state);
        }
        if (sim.config().ideal_conditions) {
            ImGui::TextDisabled("Nominal dynamics and exact sensor observations.");
        } else if (sim.config().calibrated_conditions) {
            ImGui::TextDisabled("Seeded sensor noise, asymmetry, actuator lag and command delay.");
        } else {
            ImGui::TextDisabled("Hardware-scale geometry and sensor/update-rate reference.");
        }

        const char* environment_items[] = {
            thesis_sim::environment_mode_name(EnvironmentMode::UnstructuredGates),
            thesis_sim::environment_mode_name(EnvironmentMode::StructuredRoad),
            thesis_sim::environment_mode_name(EnvironmentMode::MixedRoadGates),
        };
        if (ui_labeled_combo("Environment", &ui_state->environment_mode, environment_items, IM_ARRAYSIZE(environment_items))) {
            sim.load_world(world_from_ui_selection(sim, *ui_state));
            apply_gui_stack_for_selected_map(sim, ui_state);
        }

        const char* vehicle_items[] = {
            "Bicycle support",
            "Tank support",
        };
        int vehicle_selection = ui_state->vehicle_model == static_cast<int>(VehicleModelKind::TrackedVehicle) ? 1 : 0;
        if (ui_labeled_combo("Robot model", &vehicle_selection, vehicle_items, IM_ARRAYSIZE(vehicle_items))) {
            const VehicleModelKind next_model =
                vehicle_selection == 1 ? VehicleModelKind::TrackedVehicle : VehicleModelKind::CarLikeBicycle;
            ui_state->vehicle_model = static_cast<int>(next_model);
            sim.set_vehicle_stack(next_model, sim.tracking_controller_mode());
            if (static_cast<EnvironmentMode>(ui_state->environment_mode) == EnvironmentMode::StructuredRoad) {
                sim.load_world(world_from_ui_selection(sim, *ui_state));
            }
            apply_gui_stack_for_selected_map(sim, ui_state);
            sync_ui_state_from_sim(ui_state, sim);
            ui_state->paused = true;
        }
        if (sim.vehicle_model_kind() == VehicleModelKind::TrackedVehicle) {
            ImGui::TextWrapped("Tracked support uses the same MPC path follower; its speed and yaw-rate targets are converted into differential track velocities.");
        }

        if (sim.environment_mode() == EnvironmentMode::UnstructuredGates) {
            if (sim.world().unstructured_preset() == UnstructuredMapPreset::Custom) {
                ImGui::TextWrapped("Autonomous LiDAR exploration active: no gate is predeclared and no driving input is manual. The optional map editor is only a debug tool for changing the test fixture.");
            } else {
                ImGui::TextWrapped("Named gates are the planner's passage hypotheses through the free openings of the map. Keeping this section visible is useful when you debug wrong gate selection.");
            }

            const char* preset_items[] = {
                thesis_sim::unstructured_map_preset_name(UnstructuredMapPreset::RobotValidation),
                thesis_sim::unstructured_map_preset_name(UnstructuredMapPreset::TightCorridor),
                thesis_sim::unstructured_map_preset_name(UnstructuredMapPreset::WideSlalom),
                thesis_sim::unstructured_map_preset_name(UnstructuredMapPreset::LowerBypass),
                thesis_sim::unstructured_map_preset_name(UnstructuredMapPreset::Custom),
                thesis_sim::unstructured_map_preset_name(UnstructuredMapPreset::HardwareLab),
                thesis_sim::unstructured_map_preset_name(UnstructuredMapPreset::IdealValidation),
            };
            if (ui_labeled_combo("Unstructured Map", &ui_state->unstructured_preset, preset_items, IM_ARRAYSIZE(preset_items))) {
                const bool manual_editor = ui_state->unstructured_preset == static_cast<int>(UnstructuredMapPreset::Custom);
                sim.load_world(world_from_ui_selection(sim, *ui_state));
                apply_gui_stack_for_selected_map(sim, ui_state);
                ui_state->map_editor_enabled = manual_editor;
            }
            ImGui::Text("Preset = %s", thesis_sim::unstructured_map_preset_name(sim.world().unstructured_preset()));
            bool dynamic_lidar_gates = sim.config().dynamic_lidar_gates;
            if (sim.config().ideal_conditions) {
                ImGui::TextColored(ImVec4(0.48f, 0.88f, 0.62f, 1.0f),
                                   "Ideal matched map: dynamic gates, Ideal 2D LiDAR, baseline vehicle limits");
            } else if (ImGui::Checkbox("LiDAR dynamic gates", &dynamic_lidar_gates)) {
                const bool was_paused = ui_state->paused;
                sim.set_dynamic_lidar_gates(dynamic_lidar_gates);
                sync_ui_state_from_sim(ui_state, sim);
                ui_state->paused = was_paused;
            }

            if (sim.world().unstructured_preset() == UnstructuredMapPreset::Custom) {
                ImGui::TextColored(ImVec4(1.0f, 0.82f, 0.42f, 1.0f), "Autonomous LiDAR gates are runtime detections, not editor targets.");
                ImGui::TextWrapped("The Map Editor is optional test-fixture tooling. Runtime gates are always discovered autonomously from LiDAR and are never taken from editor annotations.");
            } else {
                int gate_behavior = static_cast<int>(sim.gate_behavior());
                bool gate_changed = false;
                const char* gate_items[] = {
                    thesis_sim::gate_behavior_mode_name(GateBehaviorMode::Static),
                    thesis_sim::gate_behavior_mode_name(GateBehaviorMode::Randomized),
                    thesis_sim::gate_behavior_mode_name(GateBehaviorMode::Mobile),
                };
                if (ui_labeled_combo("Gate Layout", &gate_behavior, gate_items, IM_ARRAYSIZE(gate_items))) {
                    gate_changed = true;
                }
                if (ImGui::InputInt("Gate Seed", &ui_state->gate_seed_input)) {
                    ui_state->gate_seed_input = std::max(ui_state->gate_seed_input, 0);
                    gate_changed = true;
                }
                if (gate_changed) {
                    sim.set_gate_behavior(
                        static_cast<GateBehaviorMode>(std::clamp(gate_behavior, 0, static_cast<int>(IM_ARRAYSIZE(gate_items)) - 1)),
                        static_cast<std::uint32_t>(ui_state->gate_seed_input));
                    ui_state->report_written = false;
                    ui_state->last_report_path.clear();
                }
                if (ImGui::Button("Regenerate Gates", ImVec2(-1.0f, 0.0f))) {
                    ui_state->gate_seed_input = std::max(ui_state->gate_seed_input + 1, 0);
                    sim.regenerate_gate_layout(static_cast<std::uint32_t>(ui_state->gate_seed_input));
                    ui_state->report_written = false;
                    ui_state->last_report_path.clear();
                }
                ImGui::Text("Gate mode = %s", thesis_sim::gate_behavior_mode_name(sim.gate_behavior()));
                ImGui::Text("Gate seed = %u", sim.gate_seed());
            }
        } else if (sim.environment_mode() == EnvironmentMode::MixedRoadGates) {
            ImGui::TextWrapped("Follow the road, bypass obstacles through detected gates, then rejoin.");
            const char* mixed_items[] = {
                mixed_map_preset_name(0),
                mixed_map_preset_name(1),
                mixed_map_preset_name(2),
                mixed_map_preset_name(3),
                mixed_map_preset_name(4),
                mixed_map_preset_name(5),
            };
            int mixed_selection = std::clamp(ui_state->mixed_preset, 0, static_cast<int>(IM_ARRAYSIZE(mixed_items)) - 1);
            if (ui_labeled_combo("Mixed Map", &mixed_selection, mixed_items, IM_ARRAYSIZE(mixed_items))) {
                ui_state->mixed_preset = mixed_selection;
                sim.load_world(world_from_ui_selection(sim, *ui_state));
                apply_gui_stack_for_selected_map(sim, ui_state);
            }
            ImGui::Text("Preset = %s", map_preset_name(sim.world()).c_str());
            bool dynamic_lidar_gates = sim.config().dynamic_lidar_gates;
            if (sim.config().ideal_conditions) {
                ImGui::TextColored(ImVec4(0.48f, 0.88f, 0.62f, 1.0f),
                                   "Ideal matched map: dynamic gates, Ideal 2D LiDAR, baseline vehicle limits");
            } else if (ImGui::Checkbox("LiDAR dynamic gates", &dynamic_lidar_gates)) {
                const bool was_paused = ui_state->paused;
                sim.set_dynamic_lidar_gates(dynamic_lidar_gates);
                sync_ui_state_from_sim(ui_state, sim);
                ui_state->paused = was_paused;
            }
            if (!sim.config().dynamic_lidar_gates) {
                ImGui::TextColored(ImVec4(1.0f, 0.82f, 0.42f, 1.0f), "Dynamic LiDAR gates are disabled; the mixed validation will remain on the structured road.");
            }
        } else {
            ImGui::TextWrapped("Follow the authored road using the selected vehicle and sensor model.");
            const char* structured_items[] = {
                thesis_sim::structured_map_preset_name(StructuredMapPreset::ValidationRoad),
                thesis_sim::structured_map_preset_name(StructuredMapPreset::CircleLoop),
                thesis_sim::structured_map_preset_name(StructuredMapPreset::ZigZag),
                thesis_sim::structured_map_preset_name(StructuredMapPreset::HardwareTrack),
                thesis_sim::structured_map_preset_name(StructuredMapPreset::FigureEight),
                thesis_sim::structured_map_preset_name(StructuredMapPreset::TankCircuit),
                thesis_sim::structured_map_preset_name(StructuredMapPreset::IdealCircle),
            };
            StructuredMapPreset previous_structured_preset =
                static_cast<StructuredMapPreset>(ui_state->structured_preset);
            int structured_selection = 0;
            switch (previous_structured_preset) {
                case StructuredMapPreset::ValidationRoad:
                    structured_selection = 0;
                    break;
                case StructuredMapPreset::CircleLoop:
                    structured_selection = 1;
                    break;
                case StructuredMapPreset::ZigZag:
                    structured_selection = 2;
                    break;
                case StructuredMapPreset::HardwareTrack:
                    structured_selection = 3;
                    break;
                case StructuredMapPreset::FigureEight:
                    structured_selection = 4;
                    break;
                case StructuredMapPreset::TankCircuit:
                    structured_selection = 5;
                    break;
                case StructuredMapPreset::IdealCircle:
                    structured_selection = 6;
                    break;
                case StructuredMapPreset::Custom:
                default:
                    structured_selection = 0;
                    break;
            }
            if (ui_labeled_combo("Structured Map", &structured_selection, structured_items, IM_ARRAYSIZE(structured_items))) {
                StructuredMapPreset next_structured_preset = StructuredMapPreset::ValidationRoad;
                switch (structured_selection) {
                    case 1:
                        next_structured_preset = StructuredMapPreset::CircleLoop;
                        break;
                    case 2:
                        next_structured_preset = StructuredMapPreset::ZigZag;
                        break;
                    case 3:
                        next_structured_preset = StructuredMapPreset::HardwareTrack;
                        break;
                    case 4:
                        next_structured_preset = StructuredMapPreset::FigureEight;
                        break;
                    case 5:
                        next_structured_preset = StructuredMapPreset::TankCircuit;
                        break;
                    case 6:
                        next_structured_preset = StructuredMapPreset::IdealCircle;
                        break;
                    case 0:
                    default:
                        next_structured_preset = StructuredMapPreset::ValidationRoad;
                        break;
                }
                ui_state->structured_preset = static_cast<int>(next_structured_preset);
                sim.load_world(world_from_ui_selection(sim, *ui_state));
                apply_gui_stack_for_selected_map(sim, ui_state);
            }
            ImGui::Text("Preset = %s", thesis_sim::structured_map_preset_name(sim.world().structured_preset()));
            ImGui::Text("Road points = %d", static_cast<int>(sim.world().road_centerline().size()));
            if (sim.vehicle_model_kind() == VehicleModelKind::TrackedVehicle) {
                bool tank_ideal =
                    static_cast<StructuredMapPreset>(ui_state->structured_preset) == StructuredMapPreset::IdealCircle;
                if (ImGui::Checkbox("Tank ideal simulation", &tank_ideal)) {
                    ui_state->structured_preset = static_cast<int>(
                        tank_ideal ? StructuredMapPreset::IdealCircle : StructuredMapPreset::ValidationRoad);
                    ui_state->ideal_simulation = tank_ideal;
                    sim.load_world(world_from_ui_selection(sim, *ui_state));
                    apply_gui_stack_for_selected_map(sim, ui_state);
                }
                ImGui::PushStyleColor(ImGuiCol_Text, ImGui::GetStyleColorVec4(ImGuiCol_TextDisabled));
                ImGui::TextWrapped("Tank baseline uses the validated 1 m structured road; ideal keeps the same road with ideal sensing and best-case tracked tuning.");
                ImGui::PopStyleColor();
            }
            if (sim.world().structured_preset() == StructuredMapPreset::Custom) {
                ImGui::TextColored(ImVec4(1.0f, 0.82f, 0.42f, 1.0f), "Current road = Custom edited scenario");
                ImGui::TextWrapped("The last edited structured road is active. Built-in presets in the combo restore the known-good loops.");
                if (ImGui::Button("Restore Validation Road", ImVec2(-1.0f, 0.0f))) {
                    ui_state->structured_preset = static_cast<int>(StructuredMapPreset::ValidationRoad);
                    sim.load_world(world_from_ui_selection(sim, *ui_state));
                    apply_gui_stack_for_selected_map(sim, ui_state);
                }
            }
            ImGui::TextDisabled("Gate layout controls are disabled in structured mode.");
        }
    }

        }

        if (ui_state->workspace_view == kWorkspaceViewSetup || ui_state->workspace_view == kWorkspaceViewDiagnostics) {
            ImGui::SetNextItemOpen(ui_state->workspace_view == kWorkspaceViewDiagnostics, ImGuiCond_FirstUseEver);
            if (ImGui::CollapsingHeader("Sensors & Localization", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SameLine();
        help_marker("These toggles affect the simulated measurements that feed the EKF. The planner still works on the fused state estimate.");
        bool imu_enabled = sim.imu_enabled();
        bool lidar_enabled = sim.lidar_enabled();
        int sensor_profile = static_cast<int>(sim.range_sensor_profile());
        bool sensor_changed = false;

        if (sim.config().ideal_conditions || sim.config().calibrated_conditions) {
            ImGui::TextColored(ImVec4(0.48f, 0.88f, 0.62f, 1.0f),
                               sim.config().ideal_conditions
                                   ? "Sim-Ideal locks exact IMU/LiDAR observations."
                                   : "Sim-Calibrated locks the seeded hardware-derived sensor profile.");
        } else {
            if (ImGui::Checkbox("Enable IMU", &imu_enabled)) {
                sensor_changed = true;
            }
            if (ImGui::Checkbox("Enable LiDAR", &lidar_enabled)) {
                sensor_changed = true;
            }

            const char* sensor_items[] = {
                thesis_sim::range_sensor_profile_name(RangeSensorProfile::IdealLidar2D),
                thesis_sim::range_sensor_profile_name(RangeSensorProfile::RplidarA1),
                thesis_sim::range_sensor_profile_name(RangeSensorProfile::ShortRangeScanner),
            };
            if (ui_labeled_combo("Range Sensor", &sensor_profile, sensor_items, IM_ARRAYSIZE(sensor_items))) {
                sensor_changed = true;
            }
            if (sensor_changed) {
                sim.set_sensor_suite(
                    imu_enabled,
                    lidar_enabled,
                    static_cast<RangeSensorProfile>(std::clamp(sensor_profile, 0, static_cast<int>(IM_ARRAYSIZE(sensor_items)) - 1)));
                sync_ui_state_from_sim(ui_state, sim);
                ui_state->report_written = false;
                ui_state->last_report_path.clear();
            }
        }

        ImGui::Text("Localization = %s", sim.localization_mode_name());
        ImGui::Text("Heading source = %s", sim.heading_source_name());
        ImGui::Text("Stack = %s + %s",
                    thesis_sim::vehicle_model_kind_name(sim.vehicle_model_kind()),
                    thesis_sim::tracking_controller_mode_name(sim.tracking_controller_mode()));
        if (sim.lidar_enabled()) {
            ImGui::Text("LiDAR = %s, min %.2f m",
                        thesis_sim::range_sensor_profile_name(sim.range_sensor_profile()),
                        sim.min_lidar_distance());
        } else {
            ImGui::Text("LiDAR = disabled");
        }
    }

        }

        if (ui_state->workspace_view == kWorkspaceViewMap) {
            ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
            if (ImGui::CollapsingHeader("Map Editor", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SameLine();
        help_marker("Enable drag mode, move handles directly in the viewport, then apply the edited scenario.");
        WorldMap& editor_world = ui_state->scenario_editor_world;
        ImGui::Checkbox("Enable drag editing", &ui_state->map_editor_enabled);
        if (ImGui::Button("Load Current Scenario", ImVec2(-1.0f, 0.0f))) {
            ui_state->scenario_editor_world = sim.world();
            ui_state->scenario_editor_dirty = false;
            ui_state->selected_editor_handle = {};
            ui_state->active_drag_handle = {};
        }
        if (ImGui::Button("Apply Edited Map", ImVec2(-1.0f, 0.0f))) {
            ui_state->scenario_editor_world.finalize_editor_changes();
            sim.load_world(ui_state->scenario_editor_world);
            sync_ui_state_from_sim(ui_state, sim);
            ui_state->paused = true;
        }

        if (ImGui::CollapsingHeader("Manual Map Tools", ImGuiTreeNodeFlags_DefaultOpen)) {
            if (ImGui::Button("Add Obstacle", ImVec2(-1.0f, 0.0f))) {
                const Vec2 center{
                    0.5 * (editor_world.start().x + editor_world.goal().x),
                    0.5 * (editor_world.start().y + editor_world.goal().y),
                };
                const Rect bounds = editor_world.bounds();
                const double half = 0.075 * std::min(bounds.max_x - bounds.min_x, bounds.max_y - bounds.min_y);
                const Vec2 obstacle_center{std::clamp(center.x, bounds.min_x + half, bounds.max_x - half),
                                           std::clamp(center.y, bounds.min_y + half, bounds.max_y - half)};
                editor_world.editable_obstacles().push_back({obstacle_center.x - half, obstacle_center.y - half,
                                                             obstacle_center.x + half, obstacle_center.y + half});
                ui_state->scenario_editor_dirty = true;
                ui_state->selected_editor_handle = {MapEditorHandleType::Obstacle, static_cast<int>(editor_world.editable_obstacles().size()) - 1};
            }

            if (editor_world.environment_mode() == EnvironmentMode::UnstructuredGates) {
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
                    ui_state->scenario_editor_dirty = true;
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
                    ui_state->scenario_editor_dirty = true;
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
                                ui_state->scenario_editor_dirty = true;
                            }
                            break;
                        case MapEditorHandleType::Gate:
                            if (ui_state->selected_editor_handle.index >= 0 &&
                                ui_state->selected_editor_handle.index < static_cast<int>(editor_world.editable_gates().size())) {
                                editor_world.editable_gates().erase(
                                    editor_world.editable_gates().begin() + ui_state->selected_editor_handle.index);
                                ui_state->scenario_editor_dirty = true;
                            }
                            break;
                        case MapEditorHandleType::RoadPoint:
                            if (ui_state->selected_editor_handle.index > 0 &&
                                ui_state->selected_editor_handle.index + 1 < static_cast<int>(editor_world.editable_road_centerline().size())) {
                                editor_world.editable_road_centerline().erase(
                                    editor_world.editable_road_centerline().begin() + ui_state->selected_editor_handle.index);
                                ui_state->scenario_editor_dirty = true;
                            }
                            break;
                        default:
                            break;
                    }
                    ui_state->selected_editor_handle = {};
                    ui_state->active_drag_handle = {};
                }
            } else {
                ImGui::TextDisabled("No editor handle selected.");
            }

            render_editor_inspector(editor_world, ui_state, &ui_state->scenario_editor_dirty);
            ImGui::TextDisabled("%s", ui_state->scenario_editor_dirty ? "Edited scenario has unapplied changes." : "Edited scenario is synchronized.");
            if (ui_state->map_editor_enabled) {
                ImGui::TextWrapped("Drag start, goal, obstacle centers, gate anchors or interior road points directly in the viewport.");
            }
        }
    }

        }

        if (ui_state->workspace_view == kWorkspaceViewExport) {
            ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
            if (ImGui::CollapsingHeader("Reports", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SameLine();
        help_marker("Writes paper-ready artifacts: JSON for reproducibility, CSV for plots, Markdown for quick thesis notes.");
        if (ImGui::Button("Save JSON snapshot", ImVec2(-1.0f, 0.0f))) {
            const std::string report_path = default_report_path(sim, "gui");
            if (write_json_report(sim, report_status_string(sim), report_path)) {
                ui_state->report_written = true;
                ui_state->last_report_path = report_path;
                ui_state->last_export_error.clear();
            } else {
                ui_state->last_export_error = "Could not write JSON report.";
            }
        }
        if (ImGui::Button("Save complete bundle", ImVec2(-1.0f, 0.0f))) {
            const std::string json_path = default_report_path(sim, "gui_bundle");
            const std::string csv_path = replace_extension(json_path, ".csv");
            const std::string markdown_path = replace_extension(json_path, ".md");
            const bool json_ok = write_json_report(sim, report_status_string(sim), json_path);
            const bool csv_ok = write_sim_csv_report(sim, csv_path);
            const bool markdown_ok = write_sim_markdown_summary(sim, report_status_string(sim), markdown_path);
            if (json_ok && csv_ok && markdown_ok) {
                ui_state->report_written = true;
                ui_state->last_report_path = json_path;
                ui_state->last_csv_report_path = csv_path;
                ui_state->last_markdown_report_path = markdown_path;
                ui_state->last_export_error.clear();
            } else {
                ui_state->last_export_error = "Could not write the complete paper bundle.";
            }
        }
        if (!ui_state->last_report_path.empty()) {
            ImGui::TextWrapped("JSON: %s", ui_state->last_report_path.c_str());
        }
        if (!ui_state->last_csv_report_path.empty()) {
            ImGui::TextWrapped("CSV: %s", ui_state->last_csv_report_path.c_str());
        }
        if (!ui_state->last_markdown_report_path.empty()) {
            ImGui::TextWrapped("Markdown: %s", ui_state->last_markdown_report_path.c_str());
        }
        if (!ui_state->last_export_error.empty()) {
            ImGui::TextColored(ImVec4(0.95f, 0.40f, 0.34f, 1.0f), "%s", ui_state->last_export_error.c_str());
        }
    }

        }
    }

    ImGui::PopTextWrapPos();
    ImGui::PopItemWidth();
    ImGui::EndChild();
}
