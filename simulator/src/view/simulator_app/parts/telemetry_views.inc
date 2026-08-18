void setup_time_plot_axes(const std::vector<double>& time, const char* y_axis_label) {
    const double x_min = time.empty() ? 0.0 : time.front();
    const double x_max = time.size() > 1 ? time.back() : (x_min + 1.0);
    ImPlot::SetupAxes("t [s]", y_axis_label, ImPlotAxisFlags_None, ImPlotAxisFlags_AutoFit);
    ImPlot::SetupLegend(ImPlotLocation_NorthEast, ImPlotLegendFlags_None);
    ImPlot::SetupAxisLimits(ImAxis_X1, x_min, x_max, ImGuiCond_Always);
}

void render_timeline_events(const std::vector<TimelineEvent>& events, const char* child_id) {
    if (events.empty()) {
        ImGui::TextDisabled("No mission events yet.");
        return;
    }

    const float line_height = ImGui::GetTextLineHeightWithSpacing();
    const float height = std::max(180.0f, ImGui::GetContentRegionAvail().y);
    if (!ImGui::BeginChild(child_id, ImVec2(0.0f, height), true)) {
        ImGui::EndChild();
        return;
    }

    const size_t max_events = 48;
    const size_t first = events.size() > max_events ? events.size() - max_events : 0;
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    for (size_t i = first; i < events.size(); ++i) {
        const TimelineEvent& event = events[i];
        ImGui::PushID(static_cast<int>(i));
        const ImVec2 cursor = ImGui::GetCursorScreenPos();
        const ImVec2 marker(cursor.x + 7.0f, cursor.y + line_height * 0.58f);
        if (i > first) {
            draw_list->AddLine(
                ImVec2(marker.x, cursor.y - 4.0f),
                ImVec2(marker.x, marker.y - 7.0f),
                IM_COL32(81, 91, 99, 170),
                1.0f);
        }
        if (i + 1 < events.size()) {
            draw_list->AddLine(
                ImVec2(marker.x, marker.y + 7.0f),
                ImVec2(marker.x, cursor.y + line_height * 2.0f),
                IM_COL32(81, 91, 99, 170),
                1.0f);
        }
        draw_list->AddCircleFilled(marker, 5.0f, ImGui::ColorConvertFloat4ToU32(event.color));
        ImGui::Dummy(ImVec2(18.0f, 0.0f));
        ImGui::SameLine();
        ImGui::BeginGroup();
        ImGui::PushStyleColor(ImGuiCol_Text, event.color);
        ImGui::Text("%.2f s  %s", event.time, event.label.c_str());
        ImGui::PopStyleColor();
        if (!event.detail.empty()) {
            ImGui::TextWrapped("%s", event.detail.c_str());
        }
        ImGui::EndGroup();
        ImGui::Spacing();
        ImGui::PopID();
    }
    ImGui::EndChild();
}

void render_sim_telemetry_side_panel(const PlannerDrivenVehicleSim& sim) {
    if (!ImGui::BeginChild("SimulationTelemetrySidePanel", ImVec2(0.0f, 0.0f), true)) {
        ImGui::EndChild();
        return;
    }

    ImGui::TextUnformatted("Telemetry");
    const std::vector<TelemetrySample>& history = sim.history();
    if (history.empty()) {
        ImGui::TextDisabled("No samples yet.");
        ImGui::EndChild();
        return;
    }

    const TelemetrySample& latest = history.back();
    char speed_buf[32];
    char goal_buf[32];
    char lidar_buf[32];
    char err_buf[32];
    char samples_buf[32];
    std::snprintf(speed_buf, sizeof(speed_buf), "%.2f m/s", latest.speed);
    std::snprintf(goal_buf, sizeof(goal_buf), "%.2f m", latest.distance_to_goal);
    std::snprintf(lidar_buf, sizeof(lidar_buf), "%.2f m", latest.min_lidar);
    std::snprintf(err_buf, sizeof(err_buf), "%.2f / %.1f", latest.nav_xy_error, latest.nav_yaw_error_deg);
    std::snprintf(samples_buf, sizeof(samples_buf), "%d", static_cast<int>(history.size()));

    if (ImGui::BeginTable("SimTelemetryCards", 2, ImGuiTableFlags_SizingStretchSame)) {
        ImGui::TableNextColumn();
        metric_card("side_sim_speed", "Speed", speed_buf, "", ImVec4(0.37f, 0.63f, 0.76f, 1.0f), 54.0f);
        ImGui::TableNextColumn();
        metric_card("side_sim_goal", "Goal", goal_buf, "", ImVec4(0.80f, 0.62f, 0.34f, 1.0f), 54.0f);
        ImGui::TableNextColumn();
        metric_card("side_sim_lidar", "LiDAR", lidar_buf, "", ImVec4(0.42f, 0.70f, 0.49f, 1.0f), 54.0f);
        ImGui::TableNextColumn();
        metric_card("side_sim_ekf", "EKF", err_buf, "", ImVec4(0.78f, 0.72f, 0.38f, 1.0f), 54.0f);
        ImGui::EndTable();
    }

    ImGui::SeparatorText("Run");
    status_line("Samples", samples_buf, ImVec4(0.43f, 0.82f, 0.96f, 1.0f));
    status_line("Scenario",
                thesis_sim::environment_mode_name(sim.environment_mode()),
                ImVec4(0.48f, 0.88f, 0.62f, 1.0f));
    status_line("State",
                sim.goal_reached() ? "goal reached" : (sim.collision() ? "collision" : "running"),
                sim.goal_reached() ? ImVec4(0.48f, 0.88f, 0.62f, 1.0f)
                                   : (sim.collision() ? ImVec4(0.95f, 0.40f, 0.34f, 1.0f)
                                                      : ImVec4(0.87f, 0.79f, 0.39f, 1.0f)));

    ImGui::SeparatorText("Focus");
    ImGui::TextDisabled("Main");
    ImGui::TextWrapped("Velocity, planner commands, clearance, compute time.");
    ImGui::TextDisabled("Tracking");
    ImGui::TextWrapped("Steering, speed reference, cross-track and heading error.");

    std::vector<double> time;
    std::vector<double> speed;
    std::vector<double> target_speed;
    std::vector<double> tracking;
    const size_t stride = plot_sample_stride(history.size());
    for (size_t i = 0; i < history.size(); i += stride) {
        const TelemetrySample& sample = history[i];
        time.push_back(sample.time);
        speed.push_back(sample.speed);
        target_speed.push_back(sample.target_speed);
        tracking.push_back(sample.tracker_cross_track);
    }
    if ((history.size() - 1) % stride != 0) {
        const TelemetrySample& sample = history.back();
        time.push_back(sample.time);
        speed.push_back(sample.speed);
        target_speed.push_back(sample.target_speed);
        tracking.push_back(sample.tracker_cross_track);
    }

    ImGui::SeparatorText("Main Plots");
    if (ImPlot::BeginPlot("Speed Tracking##side_sim", ImVec2(-1.0f, 132.0f), ImPlotFlags_NoTitle)) {
        setup_time_plot_axes(time, "m/s");
        plot_series(time, target_speed, "target", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
        plot_series(time, speed, "actual", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
        ImPlot::EndPlot();
    }
    if (ImPlot::BeginPlot("Cross-Track Error##side_sim", ImVec2(-1.0f, 132.0f), ImPlotFlags_NoTitle)) {
        setup_time_plot_axes(time, "m");
        plot_series(time, tracking, "cross-track error", ImVec4(0.48f, 0.87f, 0.60f, 1.0f));
        ImPlot::EndPlot();
    }
    ImGui::EndChild();
}

void render_hardware_telemetry_side_panel(const HardwareViewerState& hardware) {
    if (!ImGui::BeginChild("HardwareTelemetrySidePanel", ImVec2(0.0f, 0.0f), true)) {
        ImGui::EndChild();
        return;
    }

    ImGui::TextUnformatted("Telemetry");
    if (hardware.history.empty()) {
        ImGui::TextDisabled("Waiting for live samples.");
        ImGui::EndChild();
        return;
    }

    const HardwareTelemetrySample& latest = hardware.history.back();
    char speed_buf[32];
    char goal_buf[32];
    char lidar_buf[32];
    char err_buf[32];
    char samples_buf[32];
    const std::string goal_label =
        hardware_goal_distance_label(hardware.frame, hardware.scene.world.environment_mode());
    std::snprintf(speed_buf, sizeof(speed_buf), "%.2f m/s", latest.speed);
    std::snprintf(goal_buf, sizeof(goal_buf), "%s", goal_label.c_str());
    std::snprintf(lidar_buf, sizeof(lidar_buf), "%.2f / %.2f m", latest.min_lidar, latest.front_lidar);
    std::snprintf(err_buf, sizeof(err_buf), "%.2f / %.1f", latest.tracker_cross_track, latest.tracker_heading_error_deg);
    std::snprintf(samples_buf, sizeof(samples_buf), "%d", static_cast<int>(hardware.history.size()));

    if (ImGui::BeginTable("HardwareTelemetryCards", 2, ImGuiTableFlags_SizingStretchSame)) {
        ImGui::TableNextColumn();
        metric_card("side_hw_speed", "Speed", speed_buf, "", ImVec4(0.37f, 0.63f, 0.76f, 1.0f), 54.0f);
        ImGui::TableNextColumn();
        metric_card("side_hw_goal", "Goal", goal_buf, "", ImVec4(0.80f, 0.62f, 0.34f, 1.0f), 54.0f);
        ImGui::TableNextColumn();
        metric_card("side_hw_lidar", "LiDAR", lidar_buf, "", ImVec4(0.42f, 0.70f, 0.49f, 1.0f), 54.0f);
        ImGui::TableNextColumn();
        metric_card("side_hw_tracking", "Tracking", err_buf, "", ImVec4(0.78f, 0.72f, 0.38f, 1.0f), 54.0f);
        ImGui::EndTable();
    }

    ImGui::SeparatorText("Run");
    status_line("Samples", samples_buf, ImVec4(0.43f, 0.82f, 0.96f, 1.0f));
    status_line("Planner",
                hardware.frame.planner_has_reference ? "reference ready" : "waiting",
                hardware.frame.planner_has_reference ? ImVec4(0.48f, 0.88f, 0.62f, 1.0f)
                                                     : ImVec4(0.87f, 0.79f, 0.39f, 1.0f));
    status_line("Safety",
                hardware.frame.safety_stop_active ? "stop active" : "clear",
                hardware.frame.safety_stop_active ? ImVec4(0.95f, 0.40f, 0.34f, 1.0f)
                                                  : ImVec4(0.48f, 0.88f, 0.62f, 1.0f));

    ImGui::SeparatorText("Focus");
    ImGui::TextDisabled("Main");
    ImGui::TextWrapped("Speed, road tracking, clearance, compute time.");
    ImGui::TextDisabled("Drivetrain");
    ImGui::TextWrapped("PWM echo, encoder deltas and controller timing.");

    std::vector<double> time;
    std::vector<double> speed;
    std::vector<double> ref_speed;
    std::vector<double> tracking;
    std::vector<double> clearance;
    const size_t stride = plot_sample_stride(hardware.history.size());
    for (size_t i = 0; i < hardware.history.size(); i += stride) {
        const HardwareTelemetrySample& sample = hardware.history[i];
        time.push_back(sample.time);
        speed.push_back(sample.speed);
        ref_speed.push_back(sample.planner_speed_ref);
        tracking.push_back(sample.tracker_cross_track);
        clearance.push_back(sample.front_lidar);
    }
    if ((hardware.history.size() - 1) % stride != 0) {
        const HardwareTelemetrySample& sample = hardware.history.back();
        time.push_back(sample.time);
        speed.push_back(sample.speed);
        ref_speed.push_back(sample.planner_speed_ref);
        tracking.push_back(sample.tracker_cross_track);
        clearance.push_back(sample.front_lidar);
    }

    ImGui::SeparatorText("Main Plots");
    if (ImPlot::BeginPlot("Speed / Ref##side_hw", ImVec2(-1.0f, 132.0f), ImPlotFlags_NoTitle)) {
        setup_time_plot_axes(time, "m/s");
        plot_series(time, speed, "actual", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
        plot_series(time, ref_speed, "ref", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
        ImPlot::EndPlot();
    }
    if (ImPlot::BeginPlot("Tracking / Clearance##side_hw", ImVec2(-1.0f, 132.0f), ImPlotFlags_NoTitle)) {
        setup_time_plot_axes(time, "m");
        plot_series(time, tracking, "cte", ImVec4(0.48f, 0.87f, 0.60f, 1.0f));
        plot_series(time, clearance, "front", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
        ImPlot::EndPlot();
    }
    ImGui::EndChild();
}

void render_graphs_tab(PlannerDrivenVehicleSim& sim) {
    if (!ImGui::BeginChild("TelemetryViewport", ImVec2(0.0f, 0.0f), true)) {
        ImGui::EndChild();
        return;
    }

    const std::vector<TelemetrySample>& history = sim.history();
    if (history.empty()) {
        ImGui::TextUnformatted("No data yet.");
        ImGui::EndChild();
        return;
    }

    std::vector<double> time;
    std::vector<double> speed;
    std::vector<double> accel;
    std::vector<double> yaw_rate;
    std::vector<double> jerk;
    std::vector<double> command_r;
    std::vector<double> left_wheel_speed;
    std::vector<double> right_wheel_speed;
    std::vector<double> target_speed;
    std::vector<double> target_yaw_rate;
    std::vector<double> left_pwm;
    std::vector<double> right_pwm;
    std::vector<double> left_encoder_delta;
    std::vector<double> right_encoder_delta;
    std::vector<double> dist_goal;
    std::vector<double> min_lidar;
    std::vector<double> nav_xy_error;
    std::vector<double> nav_yaw_error_deg;
    std::vector<double> steer_angle;
    std::vector<double> target_steer_angle;
    std::vector<double> planner_speed_ref;
    std::vector<double> tracker_accel_cmd;
    std::vector<double> tracker_steer_rate_cmd;
    std::vector<double> tracker_cross_track;
    std::vector<double> tracker_heading_error_deg;
    std::vector<double> planning_ms;
    std::vector<double> tracking_ms;
    std::vector<double> lidar_ms;
    std::vector<double> estimator_ms;
    std::vector<double> step_ms;
    std::vector<double> visible_gates;
    std::vector<double> lidar_samples;
    time.reserve(history.size());
    speed.reserve(history.size());
    accel.reserve(history.size());
    yaw_rate.reserve(history.size());
    jerk.reserve(history.size());
    command_r.reserve(history.size());
    left_wheel_speed.reserve(history.size());
    right_wheel_speed.reserve(history.size());
    target_speed.reserve(history.size());
    target_yaw_rate.reserve(history.size());
    left_pwm.reserve(history.size());
    right_pwm.reserve(history.size());
    left_encoder_delta.reserve(history.size());
    right_encoder_delta.reserve(history.size());
    dist_goal.reserve(history.size());
    min_lidar.reserve(history.size());
    nav_xy_error.reserve(history.size());
    nav_yaw_error_deg.reserve(history.size());
    steer_angle.reserve(history.size());
    target_steer_angle.reserve(history.size());
    planner_speed_ref.reserve(history.size());
    tracker_accel_cmd.reserve(history.size());
    tracker_steer_rate_cmd.reserve(history.size());
    tracker_cross_track.reserve(history.size());
    tracker_heading_error_deg.reserve(history.size());
    planning_ms.reserve(history.size());
    tracking_ms.reserve(history.size());
    lidar_ms.reserve(history.size());
    estimator_ms.reserve(history.size());
    step_ms.reserve(history.size());
    visible_gates.reserve(history.size());
    lidar_samples.reserve(history.size());

    const size_t stride = plot_sample_stride(history.size());
    for (size_t i = 0; i < history.size(); i += stride) {
        const TelemetrySample& sample = history[i];
        time.push_back(sample.time);
        speed.push_back(sample.speed);
        accel.push_back(sample.accel);
        yaw_rate.push_back(sample.yaw_rate * 180.0 / 3.14159265358979323846);
        jerk.push_back(sample.jerk);
        command_r.push_back(sample.command_r);
        left_wheel_speed.push_back(sample.left_wheel_speed);
        right_wheel_speed.push_back(sample.right_wheel_speed);
        target_speed.push_back(sample.target_speed);
        target_yaw_rate.push_back(sample.target_yaw_rate * 180.0 / 3.14159265358979323846);
        left_pwm.push_back(sample.left_pwm);
        right_pwm.push_back(sample.right_pwm);
        left_encoder_delta.push_back(sample.left_encoder_delta);
        right_encoder_delta.push_back(sample.right_encoder_delta);
        dist_goal.push_back(sample.distance_to_goal);
        min_lidar.push_back(sample.min_lidar);
        nav_xy_error.push_back(sample.nav_xy_error);
        nav_yaw_error_deg.push_back(sample.nav_yaw_error_deg);
        steer_angle.push_back(sample.steer_angle * 180.0 / 3.14159265358979323846);
        target_steer_angle.push_back(sample.target_steer_angle * 180.0 / 3.14159265358979323846);
        planner_speed_ref.push_back(sample.planner_speed_ref);
        tracker_accel_cmd.push_back(sample.tracker_accel_cmd);
        tracker_steer_rate_cmd.push_back(sample.tracker_steer_rate_cmd * 180.0 / 3.14159265358979323846);
        tracker_cross_track.push_back(sample.tracker_cross_track);
        tracker_heading_error_deg.push_back(sample.tracker_heading_error_deg);
        planning_ms.push_back(sample.planning_ms);
        tracking_ms.push_back(sample.tracking_ms);
        lidar_ms.push_back(sample.lidar_ms);
        estimator_ms.push_back(sample.estimator_ms);
        step_ms.push_back(sample.step_ms);
        visible_gates.push_back(sample.visible_gates);
        lidar_samples.push_back(sample.lidar_samples);
    }
    if ((history.size() - 1) % stride != 0) {
        const TelemetrySample& sample = history.back();
        time.push_back(sample.time);
        speed.push_back(sample.speed);
        accel.push_back(sample.accel);
        yaw_rate.push_back(sample.yaw_rate * 180.0 / 3.14159265358979323846);
        jerk.push_back(sample.jerk);
        command_r.push_back(sample.command_r);
        left_wheel_speed.push_back(sample.left_wheel_speed);
        right_wheel_speed.push_back(sample.right_wheel_speed);
        target_speed.push_back(sample.target_speed);
        target_yaw_rate.push_back(sample.target_yaw_rate * 180.0 / 3.14159265358979323846);
        left_pwm.push_back(sample.left_pwm);
        right_pwm.push_back(sample.right_pwm);
        left_encoder_delta.push_back(sample.left_encoder_delta);
        right_encoder_delta.push_back(sample.right_encoder_delta);
        dist_goal.push_back(sample.distance_to_goal);
        min_lidar.push_back(sample.min_lidar);
        nav_xy_error.push_back(sample.nav_xy_error);
        nav_yaw_error_deg.push_back(sample.nav_yaw_error_deg);
        steer_angle.push_back(sample.steer_angle * 180.0 / 3.14159265358979323846);
        target_steer_angle.push_back(sample.target_steer_angle * 180.0 / 3.14159265358979323846);
        planner_speed_ref.push_back(sample.planner_speed_ref);
        tracker_accel_cmd.push_back(sample.tracker_accel_cmd);
        tracker_steer_rate_cmd.push_back(sample.tracker_steer_rate_cmd * 180.0 / 3.14159265358979323846);
        tracker_cross_track.push_back(sample.tracker_cross_track);
        tracker_heading_error_deg.push_back(sample.tracker_heading_error_deg);
        planning_ms.push_back(sample.planning_ms);
        tracking_ms.push_back(sample.tracking_ms);
        lidar_ms.push_back(sample.lidar_ms);
        estimator_ms.push_back(sample.estimator_ms);
        step_ms.push_back(sample.step_ms);
        visible_gates.push_back(sample.visible_gates);
        lidar_samples.push_back(sample.lidar_samples);
    }

    if (ImGui::BeginTabBar("GraphTabs", ImGuiTabBarFlags_Reorderable)) {
        if (ImGui::BeginTabItem("Main")) {
            if (ImPlot::BeginSubplots("OverviewSubplots", 2, 2, ImVec2(-1.0f, -1.0f), ImPlotSubplotFlags_LinkAllX)) {
                if (ImPlot::BeginPlot("Speed Tracking")) {
                    setup_time_plot_axes(time, "speed [m/s]");
                    plot_series(time, planner_speed_ref, "planner reference", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, target_speed, "MPC target", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    plot_series(time, speed, "vehicle", ImVec4(0.48f, 0.87f, 0.60f, 1.0f));
                    render_plot_hover_overlay("Speed Tracking", time, {
                                                                         {&planner_speed_ref, "planner reference", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                         {&target_speed, "MPC target", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                         {&speed, "vehicle", ImVec4(0.48f, 0.87f, 0.60f, 1.0f)},
                                                                     });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Yaw-Rate Tracking")) {
                    setup_time_plot_axes(time, "yaw rate [deg/s]");
                    plot_series(time, target_yaw_rate, "MPC target", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, yaw_rate, "vehicle", ImVec4(0.48f, 0.87f, 0.60f, 1.0f));
                    render_plot_hover_overlay("Yaw-Rate Tracking", time, {
                                                                            {&target_yaw_rate, "MPC target", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                            {&yaw_rate, "vehicle", ImVec4(0.48f, 0.87f, 0.60f, 1.0f)},
                                                                        });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Distance / LiDAR")) {
                    setup_time_plot_axes(time, "distance");
                    plot_series(time, dist_goal, "goal distance [m]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, min_lidar, "LiDAR min [m]", ImVec4(0.48f, 0.87f, 0.60f, 1.0f));
                    render_plot_hover_overlay("Distance / LiDAR", time, {
                                                                         {&dist_goal, "goal distance [m]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                         {&min_lidar, "LiDAR min [m]", ImVec4(0.48f, 0.87f, 0.60f, 1.0f)},
                                                                     });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Step Timing")) {
                    setup_time_plot_axes(time, "ms");
                    plot_series(time, step_ms, "step total", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    plot_series(time, planning_ms, "planning", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, tracking_ms, "tracking", ImVec4(0.48f, 0.87f, 0.60f, 1.0f));
                    render_plot_hover_overlay("Step Timing", time, {
                                                                      {&step_ms, "step total", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                      {&planning_ms, "planning", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                      {&tracking_ms, "tracking", ImVec4(0.48f, 0.87f, 0.60f, 1.0f)},
                                                                  });
                    ImPlot::EndPlot();
                }
                ImPlot::EndSubplots();
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Timeline")) {
            ImGui::TextUnformatted("Mission events");
            ImGui::TextDisabled("Planner references, mixed arbitration, clearance warnings and performance spikes.");
            render_timeline_events(build_sim_timeline(sim), "SimulationTimelineEvents");
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Drivetrain")) {
            if (ImPlot::BeginSubplots("DrivetrainSubplots", 2, 2, ImVec2(-1.0f, -1.0f), ImPlotSubplotFlags_LinkAllX)) {
                if (ImPlot::BeginPlot("Target / Wheel Speed")) {
                    setup_time_plot_axes(time, "speed [m/s]");
                    plot_series(time, target_speed, "target v", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, left_wheel_speed, "left wheel v", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    plot_series(time, right_wheel_speed, "right wheel v", ImVec4(0.48f, 0.87f, 0.60f, 1.0f));
                    render_plot_hover_overlay("Target / Wheel Speed", time, {
                                                                             {&target_speed, "target v", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                             {&left_wheel_speed, "left wheel v", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                             {&right_wheel_speed, "right wheel v", ImVec4(0.48f, 0.87f, 0.60f, 1.0f)},
                                                                         });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Yaw-Rate Tracking")) {
                    setup_time_plot_axes(time, "yaw rate [deg/s]");
                    plot_series(time, target_yaw_rate, "target wz [deg/s]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, yaw_rate, "actual wz [deg/s]", ImVec4(0.48f, 0.87f, 0.60f, 1.0f));
                    render_plot_hover_overlay("Yaw-Rate Tracking", time, {
                                                                          {&target_yaw_rate, "target wz [deg/s]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                          {&yaw_rate, "actual wz [deg/s]", ImVec4(0.48f, 0.87f, 0.60f, 1.0f)},
                                                                      });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Encoder Delta")) {
                    setup_time_plot_axes(time, "ticks / step");
                    plot_series(time, left_encoder_delta, "dTicks left", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, right_encoder_delta, "dTicks right", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    render_plot_hover_overlay("Encoder Delta", time, {
                                                                          {&left_encoder_delta, "dTicks left", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                          {&right_encoder_delta, "dTicks right", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                      });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Wheel Speed Balance")) {
                    setup_time_plot_axes(time, "wheel speed [m/s]");
                    plot_series(time, left_wheel_speed, "left wheel v", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, right_wheel_speed, "right wheel v", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    render_plot_hover_overlay("Wheel Speed Balance", time, {
                                                                               {&left_wheel_speed, "left wheel v", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                               {&right_wheel_speed, "right wheel v", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                           });
                    ImPlot::EndPlot();
                }
                ImPlot::EndSubplots();
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Localization")) {
            if (ImPlot::BeginSubplots("LocalizationSubplots", 2, 2, ImVec2(-1.0f, -1.0f), ImPlotSubplotFlags_LinkAllX)) {
                if (ImPlot::BeginPlot("Position Estimate Error")) {
                    setup_time_plot_axes(time, "error [m]");
                    plot_series(time, nav_xy_error, "xy err [m]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    render_plot_hover_overlay("Position Estimate Error", time, {
                                                                          {&nav_xy_error, "xy err [m]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                      });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Heading Estimate Error")) {
                    setup_time_plot_axes(time, "error [deg]");
                    plot_series(time, nav_yaw_error_deg, "yaw err [deg]", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    render_plot_hover_overlay("Heading Estimate Error", time, {
                                                                                 {&nav_yaw_error_deg, "yaw err [deg]", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                             });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Estimator Compute")) {
                    setup_time_plot_axes(time, "ms");
                    plot_series(time, estimator_ms, "EKF + fusion ms", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    render_plot_hover_overlay("Estimator Compute", time, {
                                                                            {&estimator_ms, "EKF + fusion ms", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                        });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Visible Gates")) {
                    setup_time_plot_axes(time, "gate count");
                    plot_series(time, visible_gates, "visible gates", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    render_plot_hover_overlay("Visible Gates", time, {
                                                                            {&visible_gates, "visible gates", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                        });
                    ImPlot::EndPlot();
                }
                ImPlot::EndSubplots();
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Tracking")) {
            if (ImPlot::BeginSubplots("TrackingSubplots", 2, 2, ImVec2(-1.0f, -1.0f), ImPlotSubplotFlags_LinkAllX)) {
                if (ImPlot::BeginPlot("Steering Tracking")) {
                    setup_time_plot_axes(time, "steer [deg]");
                    plot_series(time, steer_angle, "steer actual", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, target_steer_angle, "steer target", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    render_plot_hover_overlay("Steering Tracking", time, {
                                                                             {&steer_angle, "steer actual", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                             {&target_steer_angle, "steer target", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                         });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("MPC Acceleration Command")) {
                    setup_time_plot_axes(time, "acceleration [m/s^2]");
                    plot_series(time, tracker_accel_cmd, "accel cmd [m/s^2]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    render_plot_hover_overlay("MPC Acceleration Command", time, {
                                                                            {&tracker_accel_cmd, "accel cmd [m/s^2]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                        });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Cross-Track Error")) {
                    setup_time_plot_axes(time, "error [m]");
                    plot_series(time, tracker_cross_track, "cross-track [m]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    render_plot_hover_overlay("Cross-Track Error", time, {
                                                                                {&tracker_cross_track, "cross-track [m]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                            });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Planner Ref vs Target Speed")) {
                    setup_time_plot_axes(time, "speed [m/s]");
                    plot_series(time, planner_speed_ref, "planner v ref", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, target_speed, "target v", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    plot_series(time, speed, "actual v", ImVec4(0.48f, 0.87f, 0.60f, 1.0f));
                    render_plot_hover_overlay("Planner Ref vs Target Speed", time, {
                                                                                     {&planner_speed_ref, "planner v ref", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                                     {&target_speed, "target v", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                                     {&speed, "actual v", ImVec4(0.48f, 0.87f, 0.60f, 1.0f)},
                                                                                 });
                    ImPlot::EndPlot();
                }
                ImPlot::EndSubplots();
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Performance")) {
            ImGui::TextWrapped("This tab is for lag diagnosis. In unstructured mode, spikes here usually come from repeated gate clothoid evaluation, LiDAR raycasting or state-estimation corrections.");
            if (ImPlot::BeginSubplots("PerformanceSubplots", 2, 2, ImVec2(-1.0f, -1.0f), ImPlotSubplotFlags_LinkAllX)) {
                if (ImPlot::BeginPlot("Planner / Tracking Compute")) {
                    setup_time_plot_axes(time, "ms");
                    plot_series(time, planning_ms, "planning ms", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, tracking_ms, "tracking ms", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    render_plot_hover_overlay("Planner / Tracking Compute", time, {
                                                                                     {&planning_ms, "planning ms", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                                     {&tracking_ms, "tracking ms", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                                 });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Sensor / Estimator Compute")) {
                    setup_time_plot_axes(time, "ms");
                    plot_series(time, lidar_ms, "LiDAR ms", ImVec4(0.48f, 0.87f, 0.60f, 1.0f));
                    plot_series(time, estimator_ms, "EKF ms", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    plot_series(time, step_ms, "step total ms", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    render_plot_hover_overlay("Sensor / Estimator Compute", time, {
                                                                                     {&lidar_ms, "LiDAR ms", ImVec4(0.48f, 0.87f, 0.60f, 1.0f)},
                                                                                     {&estimator_ms, "EKF ms", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                                     {&step_ms, "step total ms", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                                 });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Visible Gates")) {
                    setup_time_plot_axes(time, "count");
                    plot_series(time, visible_gates, "visible gates", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    render_plot_hover_overlay("Visible Gates", time, {
                                                                          {&visible_gates, "visible gates", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                      });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("LiDAR Samples")) {
                    setup_time_plot_axes(time, "count");
                    plot_series(time, lidar_samples, "scan samples", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    render_plot_hover_overlay("LiDAR Samples", time, {
                                                                           {&lidar_samples, "scan samples", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                       });
                    ImPlot::EndPlot();
                }
                ImPlot::EndSubplots();
            }
            ImGui::EndTabItem();
        }

        ImGui::EndTabBar();
    }
    ImGui::EndChild();
}

void render_hardware_world_tab(const HardwareViewerState& hardware,
                               UiState* ui_state,
                               const SlamToolboxSnapshot* slam_snapshot) {
    if (ui_state == nullptr) {
        return;
    }

    if (!ImGui::BeginChild("HardwareWorldTabRoot", ImVec2(0.0f, 0.0f), false)) {
        ImGui::EndChild();
        return;
    }

    const bool show_live_scene = hardware.has_scene && !ui_state->hardware_world_sync_pending;
    const LiveFrameSnapshot& frame = hardware.frame;
    const WorldMap preview_world = show_live_scene ? hardware.scene.world : hardware_world_from_ui_selection(*ui_state);
    const WorldMap& world = preview_world;
    const char* active_target = "none";
    if (world.environment_mode() == EnvironmentMode::UnstructuredGates) {
        if (show_live_scene && frame.chosen_gate_index >= 0 && frame.chosen_gate_index < static_cast<int>(frame.gates.size())) {
            active_target = frame.gates[static_cast<std::size_t>(frame.chosen_gate_index)].spec.name.c_str();
        } else {
            active_target = thesis_sim::unstructured_map_preset_name(world.unstructured_preset());
        }
    } else {
        active_target = thesis_sim::structured_map_preset_name(world.structured_preset());
    }

    char speed_buf[32];
    char goal_buf[32];
    char tracking_buf[32];
    char target_buf[48];
    const std::string goal_label = hardware_goal_distance_label(frame, world.environment_mode());
    std::snprintf(speed_buf, sizeof(speed_buf), "%.2f m/s", frame.vehicle.speed);
    std::snprintf(goal_buf, sizeof(goal_buf), "%s", goal_label.c_str());
    std::snprintf(tracking_buf, sizeof(tracking_buf), "%.2f m / %.1f deg", frame.tracker_cross_track_error, frame.tracker_heading_error_deg);
    std::snprintf(target_buf, sizeof(target_buf), "%s", active_target);

    if (ImGui::BeginChild("HardwareWorldToolbar", ImVec2(0.0f, 84.0f), true)) {
        if (ImGui::BeginTable("HardwareWorldToolbarLayout", 2, ImGuiTableFlags_SizingStretchProp)) {
            ImGui::TableSetupColumn("WorldMetrics", ImGuiTableColumnFlags_WidthStretch, 1.0f);
            ImGui::TableSetupColumn("WorldControls", ImGuiTableColumnFlags_WidthFixed, 260.0f);

            ImGui::TableNextColumn();
            if (ImGui::BeginTable("HardwareMetricCards", 4, ImGuiTableFlags_SizingStretchSame)) {
                ImGui::TableNextColumn();
                metric_card("hw_world_speed", "Vehicle", speed_buf, "", ImVec4(0.43f, 0.82f, 0.96f, 1.0f), 56.0f);
                ImGui::TableNextColumn();
                metric_card("hw_world_goal", "Goal", goal_buf, "", ImVec4(0.99f, 0.70f, 0.32f, 1.0f), 56.0f);
                ImGui::TableNextColumn();
                metric_card("hw_world_tracking", "Tracking", tracking_buf, "", ImVec4(0.97f, 0.89f, 0.45f, 1.0f), 56.0f);
                ImGui::TableNextColumn();
                metric_card("hw_world_target", "Target", target_buf, "", ImVec4(0.48f, 0.88f, 0.62f, 1.0f), 56.0f);
                ImGui::EndTable();
            }

            ImGui::TableNextColumn();
            ImGui::TextDisabled("VIEW");
            ImGui::Checkbox("Grid", &ui_state->show_grid);
            ImGui::SameLine();
            ImGui::Checkbox("Path", &ui_state->show_trails);
            ImGui::SameLine();
            ImGui::Checkbox("HUD", &ui_state->show_world_hud);
            ImGui::Checkbox("Rays", &ui_state->show_lidar_rays);
            ImGui::SameLine();
            ImGui::Checkbox("Hits", &ui_state->show_lidar_hits);
            ImGui::SameLine();
            ImGui::Checkbox("Labels", &ui_state->show_gate_labels);
            ImGui::EndTable();
        }
    }
    ImGui::EndChild();

    if (!ImGui::BeginChild("HardwareViewport", ImVec2(0.0f, 0.0f), true, ImGuiWindowFlags_NoScrollbar)) {
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

    const CanvasTransform tx = make_transform(world, canvas_pos, canvas_size);
    const Rect bounds = world.bounds();
    if (ui_state->show_grid) {
        draw_world_grid(draw_list, tx, bounds);
    }

    for (const Rect& obstacle : world.obstacles()) {
        draw_list->AddRectFilled(world_to_screen(tx, {obstacle.min_x, obstacle.max_y}),
                                 world_to_screen(tx, {obstacle.max_x, obstacle.min_y}),
                                 kColorObstacle, 4.0f);
    }
    for (const Rect& obstacle : world.perception_obstacles()) {
        draw_list->AddRectFilled(world_to_screen(tx, {obstacle.min_x, obstacle.max_y}),
                                 world_to_screen(tx, {obstacle.max_x, obstacle.min_y}),
                                 kColorObstacle, 4.0f);
    }
    if (!world.road_centerline().empty()) {
        if (world.environment_mode() == EnvironmentMode::StructuredRoad ||
            world.environment_mode() == EnvironmentMode::MixedRoadGates) {
            draw_structured_road_map(draw_list, tx, world);
        } else {
            draw_polyline(draw_list, tx, world.road_centerline(), IM_COL32(113, 210, 255, 180), 4.0f);
        }
        if (world.environment_mode() == EnvironmentMode::StructuredRoad ||
            world.environment_mode() == EnvironmentMode::MixedRoadGates) {
            draw_goal_marker(draw_list, tx, world.goal());
        }
    }

    if (show_live_scene && ui_state->show_trails) {
        draw_polyline(draw_list, tx, hardware.frame.trail, kColorEstimateTrail, 2.5f);
        draw_polyline(draw_list, tx, hardware.frame.planned_trajectory, kColorTrajectory, 3.0f);
        std::vector<Vec2> external_pose_trail;
        external_pose_trail.reserve(hardware.history.size());
        for (const HardwareTelemetrySample& sample : hardware.history) {
            if (sample.external_pose_valid > 0.5 &&
                std::isfinite(sample.external_pose_x) &&
                std::isfinite(sample.external_pose_y)) {
                external_pose_trail.push_back({sample.external_pose_x, sample.external_pose_y});
            }
        }
        draw_polyline(
            draw_list,
            tx,
            external_pose_trail,
            IM_COL32(238, 112, 211, 245),
            3.0f);
    }

    if (show_live_scene && hardware.scene.lidar_enabled && (ui_state->show_lidar_rays || ui_state->show_lidar_hits)) {
        draw_lidar_scan(
            draw_list,
            tx,
            frame.navigation_position,
            frame.lidar_hits,
            ui_state->show_lidar_rays,
            ui_state->show_lidar_hits);
    }

    const size_t rendered_gate_count = show_live_scene ? frame.gates.size() : world.gates().size();
    for (size_t i = 0; i < rendered_gate_count; ++i) {
        const GateSpec& spec = show_live_scene ? frame.gates[i].spec : world.gates()[i];
        const bool passed = show_live_scene && frame.gates[i].passed;
        const bool visible =
            show_live_scene &&
            std::find(frame.visible_gate_indices.begin(), frame.visible_gate_indices.end(), static_cast<int>(i)) !=
                frame.visible_gate_indices.end();

        const ImVec2 screen_pos = world_to_screen(tx, spec.position);
        ImU32 color = spec.final ? kColorGoal : kColorGate;
        if (passed) {
            color = kColorGatePassed;
        } else if (visible) {
            color = kColorGateVisible;
        }
        if (show_live_scene && static_cast<int>(i) == frame.chosen_gate_index) {
            color = IM_COL32(255, 233, 118, 255);
        }

        if (spec.final) {
            draw_goal_marker(draw_list, tx, spec.position);
        } else {
            const double body_width = show_live_scene ? hardware.scene.geometry.body_width : 0.15;
            draw_oriented_gate(
                draw_list,
                tx,
                spec.position,
                spec.heading_hint,
                std::max(body_width * 1.35, 0.14),
                color,
                show_live_scene && static_cast<int>(i) == frame.chosen_gate_index);
        }
        if (ui_state->show_gate_labels) {
            draw_list->AddText(ImVec2(screen_pos.x + 6.0f, screen_pos.y - 12.0f), IM_COL32(222, 227, 230, 255), spec.name.c_str());
        }
    }

    if (show_live_scene) {
        const VehicleSnapshot vehicle = build_vehicle_snapshot_from_live(frame.vehicle, hardware.scene.geometry);
        draw_vehicle(draw_list,
                     tx,
                     vehicle,
                     hardware.scene.geometry,
                     hardware_vehicle_visual_scale_for_world(world),
                     hardware_vehicle_body_color_for_world(world));
        if (frame.has_latest_sample && frame.latest_sample.external_pose_valid > 0.5) {
            const HardwareTelemetrySample& external = frame.latest_sample;
            const Vec2 position{external.external_pose_x, external.external_pose_y};
            const double arrow_length = std::max(0.12, hardware.scene.geometry.body_length * 0.65);
            const Vec2 nose{
                position.x + std::cos(external.external_pose_yaw) * arrow_length,
                position.y + std::sin(external.external_pose_yaw) * arrow_length,
            };
            draw_list->AddCircleFilled(
                world_to_screen(tx, position),
                5.0f,
                IM_COL32(238, 112, 211, 255));
            draw_list->AddLine(
                world_to_screen(tx, position),
                world_to_screen(tx, nose),
                IM_COL32(238, 112, 211, 255),
                3.0f);
        }
    }

    static const std::vector<Vec2> empty_points;
    const std::vector<Vec2>& fallback_slam_points =
        show_live_scene
            ? (!frame.global_occupied_points.empty()
                   ? frame.global_occupied_points
                   : frame.local_safety_occupied_points)
            : empty_points;
    const std::vector<Vec2>& slam_points =
        slam_snapshot != nullptr && slam_snapshot->connected &&
                !slam_snapshot->occupied_points.empty()
            ? slam_snapshot->occupied_points
            : fallback_slam_points;
    const std::vector<Vec2>& slam_trail = show_live_scene ? frame.trail : empty_points;
    const Vec2* slam_robot_position = show_live_scene ? &frame.navigation_position : nullptr;
    draw_slam_reconstruction(
        draw_list,
        world,
        slam_pos,
        slam_size,
        slam_points,
        slam_trail,
        slam_robot_position,
        ui_state->show_grid,
        slam_snapshot);

    if (ui_state->show_world_hud) {
        char hud_line[96];
        char world_size_line[96];
        std::snprintf(world_size_line,
                      sizeof(world_size_line),
                      "area %.2f x %.2f m",
                      bounds.max_x - bounds.min_x,
                      bounds.max_y - bounds.min_y);
        std::vector<OverlayLine> top_left = {
            {show_live_scene
                 ? "Autonomous Reasoning Engine | hardware"
                 : "Autonomous Reasoning Engine | preview",
             IM_COL32(240, 243, 235, 255)},
            {thesis_sim::environment_mode_name(world.environment_mode()), IM_COL32(170, 179, 185, 255)},
            {world.environment_mode() == EnvironmentMode::UnstructuredGates
                 ? thesis_sim::unstructured_map_preset_name(world.unstructured_preset())
                 : thesis_sim::structured_map_preset_name(world.structured_preset()),
             IM_COL32(170, 179, 185, 255)},
            {show_live_scene ? stream_profile_label(hardware.scene.stream_profile)
                                : "Local preview",
             IM_COL32(170, 179, 185, 255)},
            {world_size_line, IM_COL32(170, 179, 185, 255)},
            {show_live_scene ? hardware.scene.range_sensor_name : "Edit and export before Raspberry launch",
             IM_COL32(170, 179, 185, 255)},
        };
        draw_overlay_panel(draw_list, ImVec2(canvas_pos.x + 16.0f, canvas_pos.y + 16.0f), 228.0f, top_left);

        std::snprintf(hud_line, sizeof(hud_line), "speed %.2f m/s", frame.vehicle.speed);
        std::vector<OverlayLine> top_right = {
            {show_live_scene
                 ? (frame.goal_reached ? "Mission complete"
                                       : (frame.safety_stop_active ? "Safety stop active"
                                                                   : "Hardware stream live"))
                 : "Preview ready",
             show_live_scene
                 ? (frame.goal_reached ? IM_COL32(124, 238, 151, 255)
                                       : (frame.safety_stop_active ? IM_COL32(255, 124, 102, 255) : IM_COL32(255, 221, 113, 255)))
                 : IM_COL32(132, 214, 255, 255)},
            {hud_line, IM_COL32(240, 243, 235, 255)},
        };
        std::snprintf(hud_line, sizeof(hud_line), "goal %s", goal_label.c_str());
        top_right.push_back({hud_line, IM_COL32(170, 179, 185, 255)});
        if (world.environment_mode() == EnvironmentMode::UnstructuredGates) {
            if (show_live_scene &&
                frame.dynamic_gap_gates &&
                !frame.goal_reached &&
                frame.chosen_gate_index < 0 &&
                !frame.planner_has_reference) {
                std::snprintf(hud_line,
                              sizeof(hud_line),
                              "sensing | map %d",
                              static_cast<int>(frame.global_occupied_points.size()));
            } else if (show_live_scene) {
                std::snprintf(hud_line,
                              sizeof(hud_line),
                              "gate %s | cand %d | map %d",
                              active_target,
                              frame.candidate_gates,
                              static_cast<int>(frame.global_occupied_points.size()));
            } else {
                std::snprintf(hud_line, sizeof(hud_line), "preview gates %d", static_cast<int>(world.gates().size()));
            }
        } else {
            std::snprintf(hud_line, sizeof(hud_line), "road pts %d", static_cast<int>(world.road_centerline().size()));
        }
        top_right.push_back({hud_line, IM_COL32(170, 179, 185, 255)});
        draw_overlay_panel(draw_list, ImVec2(canvas_pos.x + canvas_size.x - 236.0f, canvas_pos.y + 16.0f), 220.0f, top_right);

        std::vector<OverlayLine> legend = {
            {"orange: estimated hardware trail", kColorEstimateTrail},
            {"yellow: selected planner trajectory", kColorTrajectory},
        };
        if (world.environment_mode() == EnvironmentMode::StructuredRoad) {
            legend.push_back({"red: structured road bounds", IM_COL32(255, 96, 96, 230)});
        }
        if (show_live_scene && hardware.scene.lidar_enabled && ui_state->show_lidar_rays) {
            legend.push_back({"green: LiDAR hit rays", kColorLidar});
        }
        if (show_live_scene && hardware.scene.lidar_enabled && ui_state->show_lidar_hits) {
            legend.push_back({"lime/cyan: LiDAR collision points", kColorLidarHit});
        }
        if (!show_live_scene && ui_state->map_editor_enabled) {
            legend.push_back({"blue: editable preview road", kColorEditorOverlay});
        }
        const float legend_height = 20.0f + static_cast<float>(legend.size()) * (ImGui::GetFontSize() + 5.0f);
        draw_overlay_panel(draw_list,
                           ImVec2(canvas_pos.x + 16.0f, canvas_pos.y + canvas_size.y - legend_height),
                           304.0f,
                           legend);
    }

    if (!show_live_scene && ui_state->map_editor_enabled) {
        draw_editor_overlay(draw_list, tx, ui_state->hardware_editor_world, *ui_state);

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

void render_hardware_graphs_tab(const HardwareViewerState& hardware) {
    if (!ImGui::BeginChild("HardwareTelemetryViewport", ImVec2(0.0f, 0.0f), true)) {
        ImGui::EndChild();
        return;
    }

    if (hardware.history.empty()) {
        ImGui::TextUnformatted("Waiting for live hardware telemetry samples.");
        ImGui::EndChild();
        return;
    }

    std::vector<double> time;
    std::vector<double> speed;
    std::vector<double> accel;
    std::vector<double> yaw_rate;
    std::vector<double> jerk;
    std::vector<double> command_r;
    std::vector<double> target_speed;
    std::vector<double> target_yaw_rate;
    std::vector<double> left_pwm;
    std::vector<double> right_pwm;
    std::vector<double> controller_left_pwm;
    std::vector<double> controller_right_pwm;
    std::vector<double> controller_target_pwm_left;
    std::vector<double> controller_target_pwm_right;
    std::vector<double> controller_left_encoder_ticks;
    std::vector<double> controller_right_encoder_ticks;
    std::vector<double> controller_left_encoder_delta;
    std::vector<double> controller_right_encoder_delta;
    std::vector<double> controller_encoder_dt_ms;
    std::vector<double> dist_goal;
    std::vector<double> min_lidar;
    std::vector<double> front_lidar;
    std::vector<double> planner_speed_ref;
    std::vector<double> tracker_cross_track;
    std::vector<double> tracker_heading_error_deg;
    std::vector<double> planning_ms;
    std::vector<double> tracking_ms;
    std::vector<double> lidar_ms;
    std::vector<double> estimator_ms;
    std::vector<double> step_ms;
    std::vector<double> visible_gates;

    time.reserve(hardware.history.size());
    speed.reserve(hardware.history.size());
    accel.reserve(hardware.history.size());
    yaw_rate.reserve(hardware.history.size());
    jerk.reserve(hardware.history.size());
    command_r.reserve(hardware.history.size());
    target_speed.reserve(hardware.history.size());
    target_yaw_rate.reserve(hardware.history.size());
    left_pwm.reserve(hardware.history.size());
    right_pwm.reserve(hardware.history.size());
    controller_left_pwm.reserve(hardware.history.size());
    controller_right_pwm.reserve(hardware.history.size());
    controller_target_pwm_left.reserve(hardware.history.size());
    controller_target_pwm_right.reserve(hardware.history.size());
    controller_left_encoder_ticks.reserve(hardware.history.size());
    controller_right_encoder_ticks.reserve(hardware.history.size());
    controller_left_encoder_delta.reserve(hardware.history.size());
    controller_right_encoder_delta.reserve(hardware.history.size());
    controller_encoder_dt_ms.reserve(hardware.history.size());
    dist_goal.reserve(hardware.history.size());
    min_lidar.reserve(hardware.history.size());
    front_lidar.reserve(hardware.history.size());
    planner_speed_ref.reserve(hardware.history.size());
    tracker_cross_track.reserve(hardware.history.size());
    tracker_heading_error_deg.reserve(hardware.history.size());
    planning_ms.reserve(hardware.history.size());
    tracking_ms.reserve(hardware.history.size());
    lidar_ms.reserve(hardware.history.size());
    estimator_ms.reserve(hardware.history.size());
    step_ms.reserve(hardware.history.size());
    visible_gates.reserve(hardware.history.size());

    const size_t stride = plot_sample_stride(hardware.history.size());
    for (size_t i = 0; i < hardware.history.size(); i += stride) {
        const HardwareTelemetrySample& sample = hardware.history[i];
        time.push_back(sample.time);
        speed.push_back(sample.speed);
        accel.push_back(sample.accel);
        yaw_rate.push_back(sample.yaw_rate * 180.0 / 3.14159265358979323846);
        jerk.push_back(sample.jerk);
        command_r.push_back(sample.command_r);
        target_speed.push_back(sample.target_speed);
        target_yaw_rate.push_back(sample.target_yaw_rate * 180.0 / 3.14159265358979323846);
        left_pwm.push_back(static_cast<double>(sample.pwm_left));
        right_pwm.push_back(static_cast<double>(sample.pwm_right));
        controller_left_pwm.push_back(static_cast<double>(sample.controller_pwm_left));
        controller_right_pwm.push_back(static_cast<double>(sample.controller_pwm_right));
        controller_target_pwm_left.push_back(static_cast<double>(sample.controller_target_pwm_left));
        controller_target_pwm_right.push_back(static_cast<double>(sample.controller_target_pwm_right));
        controller_left_encoder_ticks.push_back(static_cast<double>(sample.controller_left_encoder_ticks));
        controller_right_encoder_ticks.push_back(static_cast<double>(sample.controller_right_encoder_ticks));
        controller_left_encoder_delta.push_back(static_cast<double>(sample.controller_left_encoder_delta));
        controller_right_encoder_delta.push_back(static_cast<double>(sample.controller_right_encoder_delta));
        controller_encoder_dt_ms.push_back(sample.controller_encoder_dt_ms);
        dist_goal.push_back(sample.distance_to_goal);
        min_lidar.push_back(sample.min_lidar);
        front_lidar.push_back(sample.front_lidar);
        planner_speed_ref.push_back(sample.planner_speed_ref);
        tracker_cross_track.push_back(sample.tracker_cross_track);
        tracker_heading_error_deg.push_back(sample.tracker_heading_error_deg);
        planning_ms.push_back(sample.planning_ms);
        tracking_ms.push_back(sample.tracking_ms);
        lidar_ms.push_back(sample.lidar_ms);
        estimator_ms.push_back(sample.estimator_ms);
        step_ms.push_back(sample.step_ms);
        visible_gates.push_back(sample.visible_gates);
    }
    if ((hardware.history.size() - 1) % stride != 0) {
        const HardwareTelemetrySample& sample = hardware.history.back();
        time.push_back(sample.time);
        speed.push_back(sample.speed);
        accel.push_back(sample.accel);
        yaw_rate.push_back(sample.yaw_rate * 180.0 / 3.14159265358979323846);
        jerk.push_back(sample.jerk);
        command_r.push_back(sample.command_r);
        target_speed.push_back(sample.target_speed);
        target_yaw_rate.push_back(sample.target_yaw_rate * 180.0 / 3.14159265358979323846);
        left_pwm.push_back(static_cast<double>(sample.pwm_left));
        right_pwm.push_back(static_cast<double>(sample.pwm_right));
        controller_left_pwm.push_back(static_cast<double>(sample.controller_pwm_left));
        controller_right_pwm.push_back(static_cast<double>(sample.controller_pwm_right));
        controller_target_pwm_left.push_back(static_cast<double>(sample.controller_target_pwm_left));
        controller_target_pwm_right.push_back(static_cast<double>(sample.controller_target_pwm_right));
        controller_left_encoder_ticks.push_back(static_cast<double>(sample.controller_left_encoder_ticks));
        controller_right_encoder_ticks.push_back(static_cast<double>(sample.controller_right_encoder_ticks));
        controller_left_encoder_delta.push_back(static_cast<double>(sample.controller_left_encoder_delta));
        controller_right_encoder_delta.push_back(static_cast<double>(sample.controller_right_encoder_delta));
        controller_encoder_dt_ms.push_back(sample.controller_encoder_dt_ms);
        dist_goal.push_back(sample.distance_to_goal);
        min_lidar.push_back(sample.min_lidar);
        front_lidar.push_back(sample.front_lidar);
        planner_speed_ref.push_back(sample.planner_speed_ref);
        tracker_cross_track.push_back(sample.tracker_cross_track);
        tracker_heading_error_deg.push_back(sample.tracker_heading_error_deg);
        planning_ms.push_back(sample.planning_ms);
        tracking_ms.push_back(sample.tracking_ms);
        lidar_ms.push_back(sample.lidar_ms);
        estimator_ms.push_back(sample.estimator_ms);
        step_ms.push_back(sample.step_ms);
        visible_gates.push_back(sample.visible_gates);
    }

    if (ImGui::BeginTabBar("HardwareGraphTabs", ImGuiTabBarFlags_Reorderable)) {
        if (ImGui::BeginTabItem("Main")) {
            if (ImPlot::BeginSubplots("HardwareOverviewSubplots", 2, 2, ImVec2(-1.0f, -1.0f), ImPlotSubplotFlags_LinkAllX)) {
                if (ImPlot::BeginPlot("Speed Tracking")) {
                    setup_time_plot_axes(time, "speed [m/s]");
                    plot_series(time, planner_speed_ref, "planner reference", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, target_speed, "MPC target", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    plot_series(time, speed, "measured", ImVec4(0.48f, 0.87f, 0.60f, 1.0f));
                    render_plot_hover_overlay("Speed Tracking", time, {
                                                                         {&planner_speed_ref, "planner reference", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                         {&target_speed, "MPC target", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                         {&speed, "measured", ImVec4(0.48f, 0.87f, 0.60f, 1.0f)},
                                                                     });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Yaw-Rate Tracking")) {
                    setup_time_plot_axes(time, "yaw rate [deg/s]");
                    plot_series(time, target_yaw_rate, "MPC target", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, yaw_rate, "measured", ImVec4(0.48f, 0.87f, 0.60f, 1.0f));
                    render_plot_hover_overlay("Yaw-Rate Tracking", time, {
                                                                            {&target_yaw_rate, "MPC target", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                            {&yaw_rate, "measured", ImVec4(0.48f, 0.87f, 0.60f, 1.0f)},
                                                                        });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Distance / LiDAR")) {
                    setup_time_plot_axes(time, "distance");
                    plot_series(time, dist_goal, "goal distance [m]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, min_lidar, "LiDAR min [m]", ImVec4(0.48f, 0.87f, 0.60f, 1.0f));
                    plot_series(time, front_lidar, "LiDAR front [m]", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    render_plot_hover_overlay("Distance / LiDAR", time, {
                                                                         {&dist_goal, "goal distance [m]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                         {&min_lidar, "LiDAR min [m]", ImVec4(0.48f, 0.87f, 0.60f, 1.0f)},
                                                                         {&front_lidar, "LiDAR front [m]", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                     });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Step Timing")) {
                    setup_time_plot_axes(time, "ms");
                    plot_series(time, step_ms, "step total", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    plot_series(time, planning_ms, "planning", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, tracking_ms, "tracking", ImVec4(0.48f, 0.87f, 0.60f, 1.0f));
                    render_plot_hover_overlay("Step Timing", time, {
                                                                      {&step_ms, "step total", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                      {&planning_ms, "planning", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                      {&tracking_ms, "tracking", ImVec4(0.48f, 0.87f, 0.60f, 1.0f)},
                                                                  });
                    ImPlot::EndPlot();
                }
                ImPlot::EndSubplots();
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Timeline")) {
            ImGui::TextUnformatted("Hardware events");
            ImGui::TextDisabled("Reference state, safety stops, front-clearance warnings and loop timing spikes.");
            render_timeline_events(build_hardware_timeline(hardware), "HardwareTimelineEvents");
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Tracking")) {
            if (ImPlot::BeginSubplots("HardwareTrackingSubplots", 2, 2, ImVec2(-1.0f, -1.0f), ImPlotSubplotFlags_LinkAllX)) {
                if (ImPlot::BeginPlot("Speed Tracking")) {
                    setup_time_plot_axes(time, "speed [m/s]");
                    plot_series(time, planner_speed_ref, "planner v ref", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, target_speed, "target v", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    plot_series(time, speed, "actual v", ImVec4(0.48f, 0.87f, 0.60f, 1.0f));
                    render_plot_hover_overlay("Speed Tracking", time, {
                                                                           {&planner_speed_ref, "planner v ref", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                           {&target_speed, "target v", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                           {&speed, "actual v", ImVec4(0.48f, 0.87f, 0.60f, 1.0f)},
                                                                       });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Cross-Track Error")) {
                    setup_time_plot_axes(time, "error [m]");
                    plot_series(time, tracker_cross_track, "cross-track [m]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    render_plot_hover_overlay("Cross-Track Error", time, {
                                                                                {&tracker_cross_track, "cross-track [m]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                            });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Yaw-Rate Tracking")) {
                    setup_time_plot_axes(time, "yaw rate [deg/s]");
                    plot_series(time, target_yaw_rate, "target wz [deg/s]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, yaw_rate, "measured wz [deg/s]", ImVec4(0.48f, 0.87f, 0.60f, 1.0f));
                    render_plot_hover_overlay("Yaw-Rate Tracking", time, {
                                                                          {&target_yaw_rate, "target wz [deg/s]", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                          {&yaw_rate, "measured wz [deg/s]", ImVec4(0.48f, 0.87f, 0.60f, 1.0f)},
                                                                      });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Wheel Commands")) {
                    setup_time_plot_axes(time, "PWM");
                    plot_series(time, left_pwm, "PWM left", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, right_pwm, "PWM right", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    render_plot_hover_overlay("Wheel Commands", time, {
                                                                          {&left_pwm, "PWM left", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                          {&right_pwm, "PWM right", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                      });
                    ImPlot::EndPlot();
                }
                ImPlot::EndSubplots();
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Drivetrain")) {
            if (ImPlot::BeginSubplots("HardwareDrivetrainSubplots", 2, 2, ImVec2(-1.0f, -1.0f), ImPlotSubplotFlags_LinkAllX)) {
                if (ImPlot::BeginPlot("Controller PWM Echo")) {
                    setup_time_plot_axes(time, "PWM");
                    plot_series(time, controller_target_pwm_left, "target PWM left", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, controller_target_pwm_right, "target PWM right", ImVec4(0.48f, 0.87f, 0.60f, 1.0f));
                    plot_series(time, controller_left_pwm, "PWM left", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    plot_series(time, controller_right_pwm, "PWM right", ImVec4(0.93f, 0.43f, 0.62f, 1.0f));
                    render_plot_hover_overlay("Controller PWM Echo", time, {
                                                                                {&controller_target_pwm_left, "target PWM left", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                                {&controller_target_pwm_right, "target PWM right", ImVec4(0.48f, 0.87f, 0.60f, 1.0f)},
                                                                                {&controller_left_pwm, "PWM left", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                                {&controller_right_pwm, "PWM right", ImVec4(0.93f, 0.43f, 0.62f, 1.0f)},
                                                                            });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Controller Encoder Delta")) {
                    setup_time_plot_axes(time, "ticks / sample");
                    plot_series(time, controller_left_encoder_delta, "dTicks left", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, controller_right_encoder_delta, "dTicks right", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    render_plot_hover_overlay("Controller Encoder Delta", time, {
                                                                                   {&controller_left_encoder_delta, "dTicks left", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                                   {&controller_right_encoder_delta, "dTicks right", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                               });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Controller Encoder Ticks")) {
                    setup_time_plot_axes(time, "ticks total");
                    plot_series(time, controller_left_encoder_ticks, "ticks left", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, controller_right_encoder_ticks, "ticks right", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    render_plot_hover_overlay("Controller Encoder Ticks", time, {
                                                                                   {&controller_left_encoder_ticks, "ticks left", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                                   {&controller_right_encoder_ticks, "ticks right", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                               });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Encoder Sample Period")) {
                    setup_time_plot_axes(time, "period [ms]");
                    plot_series(time, controller_encoder_dt_ms, "enc dt [ms]", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    render_plot_hover_overlay("Encoder Sample Period", time, {
                                                                              {&controller_encoder_dt_ms, "enc dt [ms]", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                          });
                    ImPlot::EndPlot();
                }
                ImPlot::EndSubplots();
            }
            ImGui::EndTabItem();
        }

        if (ImGui::BeginTabItem("Performance")) {
            if (ImPlot::BeginSubplots("HardwarePerformanceSubplots", 2, 2, ImVec2(-1.0f, -1.0f), ImPlotSubplotFlags_LinkAllX)) {
                if (ImPlot::BeginPlot("Planner / Tracking Compute")) {
                    setup_time_plot_axes(time, "ms");
                    plot_series(time, planning_ms, "planning ms", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    plot_series(time, tracking_ms, "tracking ms", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    render_plot_hover_overlay("Planner / Tracking Compute", time, {
                                                                                     {&planning_ms, "planning ms", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                                     {&tracking_ms, "tracking ms", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                                 });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Sensor / Estimator Compute")) {
                    setup_time_plot_axes(time, "ms");
                    plot_series(time, lidar_ms, "LiDAR ms", ImVec4(0.48f, 0.87f, 0.60f, 1.0f));
                    plot_series(time, estimator_ms, "EKF ms", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    plot_series(time, step_ms, "step total ms", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    render_plot_hover_overlay("Sensor / Estimator Compute", time, {
                                                                                     {&lidar_ms, "LiDAR ms", ImVec4(0.48f, 0.87f, 0.60f, 1.0f)},
                                                                                     {&estimator_ms, "EKF ms", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                                     {&step_ms, "step total ms", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                                 });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("Visible Gates")) {
                    setup_time_plot_axes(time, "gate count");
                    plot_series(time, visible_gates, "visible gates", ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
                    render_plot_hover_overlay("Visible Gates", time, {
                                                                             {&visible_gates, "visible gates", ImVec4(0.36f, 0.73f, 0.98f, 1.0f)},
                                                                         });
                    ImPlot::EndPlot();
                }
                if (ImPlot::BeginPlot("LiDAR Clearance")) {
                    setup_time_plot_axes(time, "distance [m]");
                    plot_series(time, min_lidar, "LiDAR min [m]", ImVec4(0.48f, 0.87f, 0.60f, 1.0f));
                    plot_series(time, front_lidar, "LiDAR front [m]", ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
                    render_plot_hover_overlay("LiDAR Clearance", time, {
                                                                           {&min_lidar, "LiDAR min [m]", ImVec4(0.48f, 0.87f, 0.60f, 1.0f)},
                                                                           {&front_lidar, "LiDAR front [m]", ImVec4(0.96f, 0.66f, 0.28f, 1.0f)},
                                                                       });
                    ImPlot::EndPlot();
                }
                ImPlot::EndSubplots();
            }
            ImGui::EndTabItem();
        }

        ImGui::EndTabBar();
    }
    ImGui::EndChild();
}
