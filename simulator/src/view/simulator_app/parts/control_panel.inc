void render_control_panel(PlannerDrivenVehicleSim& sim, UiState* ui_state, LiveViewStreamServer* hardware_server) {
    if (!ImGui::BeginChild("ConfigPanel", ImVec2(0.0f, 0.0f), true)) {
        ImGui::EndChild();
        return;
    }

    const char* status = sim.goal_reached() ? "Goal reached" : (sim.collision() ? "Collision" : (ui_state->paused ? "Paused" : "Running"));
    const ImVec4 status_color = sim.goal_reached() ? ImVec4(0.45f, 0.86f, 0.53f, 1.0f)
                                                   : (sim.collision() ? ImVec4(0.95f, 0.40f, 0.34f, 1.0f)
                                                                      : (ui_state->paused ? ImVec4(0.74f, 0.79f, 0.84f, 1.0f)
                                                                                          : ImVec4(0.87f, 0.79f, 0.39f, 1.0f)));
    const auto& vehicle = sim.vehicle();
    const std::string chosen_name = simulation_gate_label(sim, sim.chosen_gate_index());
    ImGui::TextUnformatted("Run Control");
    ImGui::TextColored(status_color, "%s", status);
    ImGui::TextDisabled("Playback, live state and scenario controls");


    if (ImGui::BeginTabBar("SimulationDeskTabs")) {
        if (ImGui::BeginTabItem("Run", nullptr, requested_tab_flags(ui_state->requested_simulation_panel_tab, 0))) {
            activate_simulation_panel_tab(ui_state, 0, kWorkspaceViewMission, false);
            consume_requested_tab(&ui_state->requested_simulation_panel_tab, 0);
            ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
            if (ImGui::CollapsingHeader("Playback", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::PushStyleColor(ImGuiCol_Button,
                              ui_state->paused ? ImVec4(0.15f, 0.43f, 0.52f, 1.0f) : ImVec4(0.66f, 0.47f, 0.16f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered,
                              ui_state->paused ? ImVec4(0.22f, 0.56f, 0.66f, 1.0f) : ImVec4(0.80f, 0.58f, 0.22f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive,
                              ui_state->paused ? ImVec4(0.11f, 0.33f, 0.40f, 1.0f) : ImVec4(0.51f, 0.35f, 0.10f, 1.0f));
        if (ImGui::Button(ui_state->paused ? "Run Engine" : "Pause Engine", ImVec2(-1.0f, 0.0f))) {
            ui_state->paused = !ui_state->paused;
        }
        ImGui::PopStyleColor(3);

        if (ImGui::BeginTable("PlaybackButtons", 2, ImGuiTableFlags_SizingStretchSame)) {
            ImGui::TableNextColumn();
            if (ImGui::Button("Step Once", ImVec2(-1.0f, 0.0f))) {
                ui_state->single_step = true;
            }
            ImGui::TableNextColumn();
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.43f, 0.17f, 0.17f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.58f, 0.23f, 0.23f, 1.0f));
            ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.34f, 0.13f, 0.13f, 1.0f));
            if (ImGui::Button("Reset", ImVec2(-1.0f, 0.0f))) {
                sim.reset();
                ui_state->paused = true;
                ui_state->report_written = false;
                ui_state->last_report_path.clear();
            }
            ImGui::PopStyleColor(3);
            ImGui::EndTable();
        }
        ImGui::SliderInt("Steps / frame", &ui_state->steps_per_frame, 1, 8);
        ImGui::TextDisabled("Higher values speed up playback but make collisions and gate transitions harder to inspect.");
    }

            ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
            if (ImGui::CollapsingHeader("Live Diagnostics", ImGuiTreeNodeFlags_DefaultOpen)) {
                ImGui::Text("t = %.2f s   step = %d", sim.sim_time(), sim.step_count());
                ImGui::Text("pos = (%.2f, %.2f) m", vehicle.position.x, vehicle.position.y);
                ImGui::Text("yaw = %.1f deg   v = %.2f m/s", vehicle.yaw * 180.0 / 3.14159265358979323846, vehicle.speed);
                ImGui::Text("goal distance = %.2f m", sim.distance_to_goal());
                ImGui::Text("tracking: cte %.2f m   hdg %.2f deg", sim.tracker_cross_track_error(), sim.tracker_heading_error_deg());
                if (sim.last_mpc_command().has_value()) {
                    ImGui::Text("MPC: accel %.2f   steer rate %.2f deg/s",
                                sim.last_mpc_command()->accel_cmd,
                                sim.last_mpc_command()->steer_rate_cmd * 180.0 / 3.14159265358979323846);
                }
                if (sim.environment_mode() == EnvironmentMode::UnstructuredGates ||
                    sim.environment_mode() == EnvironmentMode::MixedRoadGates) {
                    ImGui::Text("chosen gate = %s   visible = %d", chosen_name.c_str(), static_cast<int>(sim.visible_gate_indices().size()));
                    if (sim.environment_mode() == EnvironmentMode::MixedRoadGates) {
                        const TelemetrySample* latest = sim.history().empty() ? nullptr : &sim.history().back();
                        ImGui::Text("mixed: mode %.0f   gate score %.2f   road score %.2f",
                                    latest != nullptr ? latest->mixed_mode : 0.0,
                                    latest != nullptr ? latest->mixed_gate_score : 0.0,
                                    latest != nullptr ? latest->mixed_structured_score : 0.0);
                    }
                } else {
                    ImGui::Text("road points = %d", static_cast<int>(sim.world().road_centerline().size()));
                }
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Scenario", nullptr, requested_tab_flags(ui_state->requested_simulation_panel_tab, 1))) {
            activate_simulation_panel_tab(ui_state, 1, kWorkspaceViewMission, false);
            consume_requested_tab(&ui_state->requested_simulation_panel_tab, 1);
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
        ImGui::SetNextItemWidth(-1.0f);
        if (ImGui::Combo(
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
        if (ImGui::Combo("Environment", &ui_state->environment_mode, environment_items, IM_ARRAYSIZE(environment_items))) {
            sim.load_world(world_from_ui_selection(sim, *ui_state));
            apply_gui_stack_for_selected_map(sim, ui_state);
        }

        const char* vehicle_items[] = {
            "Bicycle support",
            "Tank support",
        };
        int vehicle_selection = ui_state->vehicle_model == static_cast<int>(VehicleModelKind::TrackedVehicle) ? 1 : 0;
        if (ImGui::Combo("Motion Primitive Support", &vehicle_selection, vehicle_items, IM_ARRAYSIZE(vehicle_items))) {
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
                ImGui::TextWrapped("Manual gate editor active: place gates and obstacles yourself, then let the planner reach each gate with the motion primitives on top of the mapped free space.");
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
            if (ImGui::Combo("Unstructured Map", &ui_state->unstructured_preset, preset_items, IM_ARRAYSIZE(preset_items))) {
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
                ImGui::TextColored(ImVec4(1.0f, 0.82f, 0.42f, 1.0f), "User-defined gates stay static in manual editor mode.");
                ImGui::TextWrapped("Use the Map Editor section to add gates, move them directly in the viewport, and place obstacles the robot must sense and avoid.");
            } else {
                int gate_behavior = static_cast<int>(sim.gate_behavior());
                bool gate_changed = false;
                const char* gate_items[] = {
                    thesis_sim::gate_behavior_mode_name(GateBehaviorMode::Static),
                    thesis_sim::gate_behavior_mode_name(GateBehaviorMode::Randomized),
                    thesis_sim::gate_behavior_mode_name(GateBehaviorMode::Mobile),
                };
                if (ImGui::Combo("Gate Layout", &gate_behavior, gate_items, IM_ARRAYSIZE(gate_items))) {
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
            ImGui::TextWrapped("Mixed mode follows the structured road while LiDAR gaps compete as dynamic gates. The arbiter only leaves the road when the gate is stable, aligned, safe and useful for progress.");
            const char* mixed_items[] = {
                mixed_map_preset_name(0),
                mixed_map_preset_name(1),
                mixed_map_preset_name(2),
                mixed_map_preset_name(3),
                mixed_map_preset_name(4),
                mixed_map_preset_name(5),
            };
            int mixed_selection = std::clamp(ui_state->mixed_preset, 0, static_cast<int>(IM_ARRAYSIZE(mixed_items)) - 1);
            if (ImGui::Combo("Mixed Map", &mixed_selection, mixed_items, IM_ARRAYSIZE(mixed_items))) {
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
            ImGui::TextWrapped("Structured mode keeps the planner on a fixed road loop while the EKF fuses encoder, IMU and LiDAR for the state estimate.");
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
            if (ImGui::Combo("Structured Map", &structured_selection, structured_items, IM_ARRAYSIZE(structured_items))) {
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

            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Sensors", nullptr, requested_tab_flags(ui_state->requested_simulation_panel_tab, 2))) {
            activate_simulation_panel_tab(ui_state, 2, kWorkspaceViewDiagnostics, false);
            consume_requested_tab(&ui_state->requested_simulation_panel_tab, 2);
            ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
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
            if (ImGui::Combo("Range Sensor", &sensor_profile, sensor_items, IM_ARRAYSIZE(sensor_items))) {
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

            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Map", nullptr, requested_tab_flags(ui_state->requested_simulation_panel_tab, 3))) {
            activate_simulation_panel_tab(ui_state, 3, kWorkspaceViewMap, true);
            consume_requested_tab(&ui_state->requested_simulation_panel_tab, 3);
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
                editor_world.editable_obstacles().push_back({center.x - 1.0, center.y - 1.0, center.x + 1.0, center.y + 1.0});
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

            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Export", nullptr, requested_tab_flags(ui_state->requested_simulation_panel_tab, 4))) {
            activate_simulation_panel_tab(ui_state, 4, kWorkspaceViewExport, false);
            consume_requested_tab(&ui_state->requested_simulation_panel_tab, 4);
            ImGui::SetNextItemOpen(true, ImGuiCond_FirstUseEver);
            if (ImGui::CollapsingHeader("Reports", ImGuiTreeNodeFlags_DefaultOpen)) {
        ImGui::SameLine();
        help_marker("Writes paper-ready artifacts: JSON for reproducibility, CSV for plots, Markdown for quick thesis notes.");
        if (ImGui::Button("Write JSON Report", ImVec2(-1.0f, 0.0f))) {
            const std::string report_path = default_report_path(sim, "gui");
            if (write_json_report(sim, report_status_string(sim), report_path)) {
                ui_state->report_written = true;
                ui_state->last_report_path = report_path;
                ui_state->last_export_error.clear();
            } else {
                ui_state->last_export_error = "Could not write JSON report.";
            }
        }
        if (ImGui::Button("Write Paper Bundle", ImVec2(-1.0f, 0.0f))) {
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

            ImGui::EndTabItem();
        }
        ImGui::EndTabBar();
    }

    ImGui::EndChild();
}

void render_workspace_top_bar(PlannerDrivenVehicleSim& sim,
                              const HardwareViewerState* hardware,
                              UiState* ui_state,
                              LiveViewStreamServer* hardware_server) {
    if (ui_state == nullptr) {
        return;
    }

    const bool hardware_mode = workspace_source_is_hardware(ui_state->workspace_source);
    const VehicleSnapshot& vehicle = sim.vehicle();
    const bool hardware_connected = hardware_server != nullptr && hardware_server->connected();
    const bool hardware_listening = hardware_server != nullptr && hardware_server->listening();

    const char* status_text = nullptr;
    ImVec4 status_color;
    std::string scenario_label;
    std::string map_label;
    std::string robot_label;
    char goal_buf[48];
    char speed_buf[48];

    if (hardware_mode) {
        const LiveFrameSnapshot* frame = hardware != nullptr ? &hardware->frame : nullptr;
        status_text = hardware_connected
                          ? ((frame != nullptr && frame->goal_reached) ? "Goal reached"
                                                                        : ((frame != nullptr && frame->safety_stop_active) ? "Safety stop" : "Live"))
                          : (hardware_listening ? "Listening" : "Preview");
        status_color = hardware_connected
                           ? ((frame != nullptr && frame->goal_reached) ? ImVec4(0.45f, 0.86f, 0.53f, 1.0f)
                                                                         : ((frame != nullptr && frame->safety_stop_active)
                                                                                ? ImVec4(0.95f, 0.40f, 0.34f, 1.0f)
                                                                                : ImVec4(0.87f, 0.79f, 0.39f, 1.0f)))
                           : (hardware_listening ? ImVec4(0.43f, 0.82f, 0.96f, 1.0f)
                                                 : ImVec4(0.74f, 0.79f, 0.84f, 1.0f));
        scenario_label = "Hardware Planner";
        map_label = hardware != nullptr && hardware->has_scene
                        ? map_preset_name(hardware->scene.world)
                        : "configured preview";
        robot_label = hardware != nullptr && hardware->has_scene
                          ? hardware->scene.vehicle_model_name
                          : thesis_sim::vehicle_model_kind_name(static_cast<VehicleModelKind>(ui_state->hardware_vehicle_model));
        const std::string goal_label =
            hardware != nullptr
                ? hardware_goal_distance_label(hardware->frame, hardware->scene.world.environment_mode())
                : "n/a";
        std::snprintf(goal_buf, sizeof(goal_buf), "%s", goal_label.c_str());
        std::snprintf(speed_buf,
                      sizeof(speed_buf),
                      "%.2f m/s",
                      frame != nullptr ? frame->vehicle.speed : 0.0);
    } else {
        status_text = sim.goal_reached() ? "Goal reached" : (sim.collision() ? "Collision" : (ui_state->paused ? "Paused" : "Running"));
        status_color = sim.goal_reached() ? ImVec4(0.45f, 0.86f, 0.53f, 1.0f)
                                          : (sim.collision() ? ImVec4(0.95f, 0.40f, 0.34f, 1.0f)
                                                             : (ui_state->paused ? ImVec4(0.74f, 0.79f, 0.84f, 1.0f)
                                                                                 : ImVec4(0.87f, 0.79f, 0.39f, 1.0f)));
        scenario_label = thesis_sim::environment_mode_name(sim.environment_mode());
        map_label = map_preset_name(sim.world());
        robot_label = thesis_sim::vehicle_model_kind_name(sim.vehicle_model_kind());
        std::snprintf(goal_buf, sizeof(goal_buf), "%.2f m", sim.distance_to_goal());
        std::snprintf(speed_buf, sizeof(speed_buf), "%.2f m/s", vehicle.speed);
    }

    ImGui::PushStyleVar(ImGuiStyleVar_ChildRounding, 8.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(14.0f, 10.0f));
    ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.07f, 0.10f, 0.13f, 1.0f));
    ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(0.18f, 0.25f, 0.29f, 1.0f));
    if (ImGui::BeginChild("WorkspaceTopBar", ImVec2(0.0f, 68.0f), true, ImGuiWindowFlags_NoScrollbar)) {
        if (ImGui::BeginTable("WorkspaceTopBarLayout", 4, ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn("Title", ImGuiTableColumnFlags_WidthStretch, 1.2f);
            ImGui::TableSetupColumn("Source", ImGuiTableColumnFlags_WidthFixed, 248.0f);
            ImGui::TableSetupColumn("State", ImGuiTableColumnFlags_WidthFixed, 230.0f);
            ImGui::TableSetupColumn("Vehicle", ImGuiTableColumnFlags_WidthFixed, 245.0f);
            ImGui::TableNextColumn();
            ImGui::TextUnformatted("Autonomous Reasoning Engine");
            ImGui::TextDisabled("%s  |  %s", scenario_label.c_str(), map_label.c_str());

            ImGui::TableNextColumn();
            ImGui::TextDisabled("Workspace");
            auto source_button = [&](const char* label, int source) {
                const bool selected = ui_state->workspace_source == source;
                if (selected) {
                    ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.15f, 0.43f, 0.52f, 1.0f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(0.22f, 0.56f, 0.66f, 1.0f));
                    ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(0.11f, 0.33f, 0.40f, 1.0f));
                }
                if (ImGui::Button(label, ImVec2(112.0f, 0.0f))) {
                    ui_state->workspace_source = source;
                    if (workspace_source_is_hardware(source) &&
                        hardware_server != nullptr &&
                        !hardware_server->listening() &&
                        ui_state->hardware_listen_port > 0) {
                        hardware_server->start(static_cast<std::uint16_t>(ui_state->hardware_listen_port));
                    }
                }
                if (selected) {
                    ImGui::PopStyleColor(3);
                }
            };
            source_button("Engine", kWorkspaceSourceSimulation);
            ImGui::SameLine();
            source_button("Hardware", kWorkspaceSourceHardwarePlanner);

            ImGui::TableNextColumn();
            ImGui::TextDisabled("Run State");
            ImGui::PushStyleColor(ImGuiCol_Text, status_color);
            ImGui::TextUnformatted(status_text);
            ImGui::PopStyleColor();
            ImGui::TextDisabled("goal %s", goal_buf);

            ImGui::TableNextColumn();
            ImGui::TextDisabled("Robot");
            ImGui::TextUnformatted(robot_label.c_str());
            ImGui::TextDisabled("speed %s", speed_buf);
            ImGui::EndTable();
        }
    }
    ImGui::EndChild();
    ImGui::PopStyleColor(2);
    ImGui::PopStyleVar(2);
}

