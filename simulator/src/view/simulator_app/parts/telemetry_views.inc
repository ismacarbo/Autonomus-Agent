bool plot_follow_time = true;
int plot_window_seconds = 30;

void render_plot_controls(UiState* state) {
    ImGui::Checkbox("Follow time", &state->follow_plot_time);
    ImGui::SameLine();
    ImGui::SetNextItemWidth(184);
    const char* windows[] = {"Last 10 seconds", "Last 30 seconds", "Last 60 seconds", "Entire run"};
    const int seconds[] = {10, 30, 60, 0};
    int selected = state->plot_window_seconds == 10 ? 0 : state->plot_window_seconds == 30 ? 1 : state->plot_window_seconds == 60 ? 2 : 3;
    if (ImGui::Combo("##PlotWindow", &selected, windows, 4)) state->plot_window_seconds = seconds[selected];
    if (ImGui::GetContentRegionAvail().x > 400) ImGui::SameLine();
    ImGui::TextDisabled(state->follow_plot_time ? "Uncheck to zoom and pan" : "Scroll: zoom / drag: pan / double-click: fit");
    plot_follow_time = state->follow_plot_time;
    plot_window_seconds = state->plot_window_seconds;
}

bool begin_signal_tab(const char* label, int index, UiState* state) {
    const bool selected = ImGui::BeginTabItem(label, nullptr,
        state->requested_signal_tab == index ? ImGuiTabItemFlags_SetSelected : ImGuiTabItemFlags_None);
    if (selected && state->requested_signal_tab == index) state->requested_signal_tab = -1;
    return selected;
}

bool begin_signal_grid(const char* id) {
    const int columns = ImGui::GetContentRegionAvail().x < 820 ? 1 : 2;
    const int rows = 4 / columns;
    return ImPlot::BeginSubplots(id, rows, columns,
        ImVec2(-1, std::max(ImGui::GetContentRegionAvail().y, 215.0f * rows)),
        ImPlotSubplotFlags_LinkAllX);
}

void setup_time_plot_axes(const std::vector<double>& time, const char* y_axis_label) {
    const double first = time.empty() ? 0.0 : time.front();
    const double x_max = time.size() > 1 ? time.back() : (first + 1.0);
    const double x_min = plot_window_seconds > 0 ? std::max(first, x_max - plot_window_seconds) : first;
    ImPlot::SetupAxes("t [s]", y_axis_label, ImPlotAxisFlags_None, plot_follow_time ? (ImPlotAxisFlags_AutoFit | ImPlotAxisFlags_RangeFit) : ImPlotAxisFlags_None);
    ImPlot::SetupLegend(ImPlotLocation_NorthEast, ImPlotLegendFlags_None);
    ImPlot::SetupAxisLimits(ImAxis_X1, x_min, x_max, plot_follow_time ? ImGuiCond_Always : ImGuiCond_Once);
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

void render_graphs_tab(PlannerDrivenVehicleSim& sim, UiState* state) {
    if (!ImGui::BeginChild("TelemetryViewport", ImVec2(0.0f, 0.0f), true)) {
        ImGui::EndChild();
        return;
    }

    const std::vector<TelemetrySample>& history = sim.history();
    if (history.empty()) {
        ImGui::TextUnformatted("No simulation samples yet");
        ImGui::TextDisabled("Run the simulation from Overview to inspect these signals.");
        ImGui::EndChild();
        return;
    }

    static std::vector<double> time;
    time.clear();
    static std::vector<double> speed;
    speed.clear();
    static std::vector<double> accel;
    accel.clear();
    static std::vector<double> yaw_rate;
    yaw_rate.clear();
    static std::vector<double> jerk;
    jerk.clear();
    static std::vector<double> command_r;
    command_r.clear();
    static std::vector<double> left_wheel_speed;
    left_wheel_speed.clear();
    static std::vector<double> right_wheel_speed;
    right_wheel_speed.clear();
    static std::vector<double> target_speed;
    target_speed.clear();
    static std::vector<double> target_yaw_rate;
    target_yaw_rate.clear();
    static std::vector<double> left_pwm;
    left_pwm.clear();
    static std::vector<double> right_pwm;
    right_pwm.clear();
    static std::vector<double> left_encoder_delta;
    left_encoder_delta.clear();
    static std::vector<double> right_encoder_delta;
    right_encoder_delta.clear();
    static std::vector<double> dist_goal;
    dist_goal.clear();
    static std::vector<double> min_lidar;
    min_lidar.clear();
    static std::vector<double> nav_xy_error;
    nav_xy_error.clear();
    static std::vector<double> nav_yaw_error_deg;
    nav_yaw_error_deg.clear();
    static std::vector<double> steer_angle;
    steer_angle.clear();
    static std::vector<double> target_steer_angle;
    target_steer_angle.clear();
    static std::vector<double> planner_speed_ref;
    planner_speed_ref.clear();
    static std::vector<double> tracker_accel_cmd;
    tracker_accel_cmd.clear();
    static std::vector<double> tracker_steer_rate_cmd;
    tracker_steer_rate_cmd.clear();
    static std::vector<double> tracker_cross_track;
    tracker_cross_track.clear();
    static std::vector<double> tracker_heading_error_deg;
    tracker_heading_error_deg.clear();
    static std::vector<double> planning_ms;
    planning_ms.clear();
    static std::vector<double> tracking_ms;
    tracking_ms.clear();
    static std::vector<double> lidar_ms;
    lidar_ms.clear();
    static std::vector<double> estimator_ms;
    estimator_ms.clear();
    static std::vector<double> step_ms;
    step_ms.clear();
    static std::vector<double> visible_gates;
    visible_gates.clear();
    static std::vector<double> lidar_samples;
    lidar_samples.clear();
    time.reserve(602);
    speed.reserve(602);
    accel.reserve(602);
    yaw_rate.reserve(602);
    jerk.reserve(602);
    command_r.reserve(602);
    left_wheel_speed.reserve(602);
    right_wheel_speed.reserve(602);
    target_speed.reserve(602);
    target_yaw_rate.reserve(602);
    left_pwm.reserve(602);
    right_pwm.reserve(602);
    left_encoder_delta.reserve(602);
    right_encoder_delta.reserve(602);
    dist_goal.reserve(602);
    min_lidar.reserve(602);
    nav_xy_error.reserve(602);
    nav_yaw_error_deg.reserve(602);
    steer_angle.reserve(602);
    target_steer_angle.reserve(602);
    planner_speed_ref.reserve(602);
    tracker_accel_cmd.reserve(602);
    tracker_steer_rate_cmd.reserve(602);
    tracker_cross_track.reserve(602);
    tracker_heading_error_deg.reserve(602);
    planning_ms.reserve(602);
    tracking_ms.reserve(602);
    lidar_ms.reserve(602);
    estimator_ms.reserve(602);
    step_ms.reserve(602);
    visible_gates.reserve(602);
    lidar_samples.reserve(602);

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

    render_plot_controls(state);
    if (ImGui::BeginTabBar("GraphTabs", ImGuiTabBarFlags_Reorderable)) {
        if (begin_signal_tab("Overview", 0, state)) {
            if (begin_signal_grid("##OverviewSubplots")) {
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

        if (begin_signal_tab("Timeline", 1, state)) {
            ImGui::TextUnformatted("Mission events");
            ImGui::TextDisabled("Planner references, mixed arbitration, clearance warnings and performance spikes.");
            render_timeline_events(build_sim_timeline(sim), "SimulationTimelineEvents");
            ImGui::EndTabItem();
        }

        if (begin_signal_tab("Drivetrain", 2, state)) {
            if (begin_signal_grid("##DrivetrainSubplots")) {
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

        if (begin_signal_tab("Localization", 3, state)) {
            if (begin_signal_grid("##LocalizationSubplots")) {
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

        if (begin_signal_tab("MPC tracking", 4, state)) {
            if (begin_signal_grid("##TrackingSubplots")) {
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

        if (begin_signal_tab("Performance", 5, state)) {
            ImGui::TextWrapped("This tab is for lag diagnosis. In unstructured mode, spikes here usually come from repeated gate clothoid evaluation, LiDAR raycasting or state-estimation corrections.");
            if (begin_signal_grid("##PerformanceSubplots")) {
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

    const bool show_live_scene = hardware.has_scene && !ui_state->hardware_world_sync_pending && ui_state->workspace_view != kWorkspaceViewMap;
    const LiveFrameSnapshot& frame = hardware.frame;
    const WorldMap preview_world = show_live_scene ? WorldMap{} : hardware_world_from_ui_selection(*ui_state);
    const WorldMap& world = ui_state->workspace_view == kWorkspaceViewMap ? ui_state->hardware_editor_world : show_live_scene ? hardware.scene.world : preview_world;
    render_map_toolbar(ui_state, world, show_live_scene);

    if (!ImGui::BeginChild("HardwareViewport", ImVec2(0.0f, 0.0f), true, ImGuiWindowFlags_NoScrollbar)) {
        ImGui::EndChild();
        ImGui::EndChild();
        return;
    }

    const ImVec2 viewport_pos = ImGui::GetCursorScreenPos();
    ImVec2 viewport_size = ImGui::GetContentRegionAvail();
    viewport_size.x = std::max(viewport_size.x, 1.0f);
    viewport_size.y = std::max(viewport_size.y, 1.0f);
    const MapViewportLayout layout = map_viewport_layout(viewport_pos, viewport_size, ui_state->show_slam_map);
    const ImVec2 canvas_pos = layout.map_pos;
    const ImVec2 canvas_size = layout.map_size;
    const ImVec2 slam_pos = layout.slam_pos;
    const ImVec2 slam_size = layout.slam_size;

    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    draw_viewport_background(draw_list, canvas_pos, canvas_size);
    draw_list->PushClipRect(canvas_pos, ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y), true);

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
        external_pose_trail.reserve(602);
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
        if (ui_state->show_robot_footprint)
            draw_robot_footprint(draw_list, tx, frame.navigation_position, frame.navigation_yaw,
                                 hardware.scene.geometry.body_length, hardware.scene.geometry.body_width);
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
    draw_list->PopClipRect();
    if (ui_state->show_slam_map) {
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
    }
    draw_list->PushClipRect(canvas_pos, ImVec2(canvas_pos.x + canvas_size.x, canvas_pos.y + canvas_size.y), true);

    draw_metric_map_caption(draw_list, tx, world, canvas_pos, canvas_size);

    if (ui_state->show_world_hud) {
        char hud_line[96];
        char world_size_line[96];
        std::snprintf(world_size_line,
                      sizeof(world_size_line),
                      "area %.2f x %.2f m",
                      bounds.max_x - bounds.min_x,
                      bounds.max_y - bounds.min_y);
        const char* active_target = frame.chosen_gate_index >= 0 && frame.chosen_gate_index < static_cast<int>(frame.gates.size())
            ? frame.gates[static_cast<std::size_t>(frame.chosen_gate_index)].spec.name.c_str() : "none";
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
        std::snprintf(hud_line, sizeof(hud_line), "goal %s", hardware_goal_distance_label(frame, world.environment_mode()).c_str());
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
            legend.push_back({"slate: structured road bounds", IM_COL32(113, 145, 162, 190)});
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

    draw_list->PopClipRect();
    ImGui::Dummy(viewport_size);
    ImGui::EndChild();
    ImGui::EndChild();
}

void render_hardware_graphs_tab(const HardwareViewerState& hardware, UiState* state) {
    if (!ImGui::BeginChild("HardwareTelemetryViewport", ImVec2(0.0f, 0.0f), true)) {
        ImGui::EndChild();
        return;
    }

    if (hardware.history.empty()) {
        ImGui::TextUnformatted("Waiting for live hardware telemetry samples.");
        ImGui::EndChild();
        return;
    }

    static std::vector<double> time;
    time.clear();
    static std::vector<double> speed;
    speed.clear();
    static std::vector<double> accel;
    accel.clear();
    static std::vector<double> yaw_rate;
    yaw_rate.clear();
    static std::vector<double> jerk;
    jerk.clear();
    static std::vector<double> command_r;
    command_r.clear();
    static std::vector<double> target_speed;
    target_speed.clear();
    static std::vector<double> target_yaw_rate;
    target_yaw_rate.clear();
    static std::vector<double> left_pwm;
    left_pwm.clear();
    static std::vector<double> right_pwm;
    right_pwm.clear();
    static std::vector<double> controller_left_pwm;
    controller_left_pwm.clear();
    static std::vector<double> controller_right_pwm;
    controller_right_pwm.clear();
    static std::vector<double> controller_target_pwm_left;
    controller_target_pwm_left.clear();
    static std::vector<double> controller_target_pwm_right;
    controller_target_pwm_right.clear();
    static std::vector<double> controller_left_encoder_ticks;
    controller_left_encoder_ticks.clear();
    static std::vector<double> controller_right_encoder_ticks;
    controller_right_encoder_ticks.clear();
    static std::vector<double> controller_left_encoder_delta;
    controller_left_encoder_delta.clear();
    static std::vector<double> controller_right_encoder_delta;
    controller_right_encoder_delta.clear();
    static std::vector<double> controller_encoder_dt_ms;
    controller_encoder_dt_ms.clear();
    static std::vector<double> dist_goal;
    dist_goal.clear();
    static std::vector<double> min_lidar;
    min_lidar.clear();
    static std::vector<double> front_lidar;
    front_lidar.clear();
    static std::vector<double> planner_speed_ref;
    planner_speed_ref.clear();
    static std::vector<double> tracker_cross_track;
    tracker_cross_track.clear();
    static std::vector<double> tracker_heading_error_deg;
    tracker_heading_error_deg.clear();
    static std::vector<double> planning_ms;
    planning_ms.clear();
    static std::vector<double> tracking_ms;
    tracking_ms.clear();
    static std::vector<double> lidar_ms;
    lidar_ms.clear();
    static std::vector<double> estimator_ms;
    estimator_ms.clear();
    static std::vector<double> step_ms;
    step_ms.clear();
    static std::vector<double> visible_gates;
    visible_gates.clear();

    time.reserve(602);
    speed.reserve(602);
    accel.reserve(602);
    yaw_rate.reserve(602);
    jerk.reserve(602);
    command_r.reserve(602);
    target_speed.reserve(602);
    target_yaw_rate.reserve(602);
    left_pwm.reserve(602);
    right_pwm.reserve(602);
    controller_left_pwm.reserve(602);
    controller_right_pwm.reserve(602);
    controller_target_pwm_left.reserve(602);
    controller_target_pwm_right.reserve(602);
    controller_left_encoder_ticks.reserve(602);
    controller_right_encoder_ticks.reserve(602);
    controller_left_encoder_delta.reserve(602);
    controller_right_encoder_delta.reserve(602);
    controller_encoder_dt_ms.reserve(602);
    dist_goal.reserve(602);
    min_lidar.reserve(602);
    front_lidar.reserve(602);
    planner_speed_ref.reserve(602);
    tracker_cross_track.reserve(602);
    tracker_heading_error_deg.reserve(602);
    planning_ms.reserve(602);
    tracking_ms.reserve(602);
    lidar_ms.reserve(602);
    estimator_ms.reserve(602);
    step_ms.reserve(602);
    visible_gates.reserve(602);

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

    render_plot_controls(state);
    if (ImGui::BeginTabBar("HardwareGraphTabs", ImGuiTabBarFlags_Reorderable)) {
        if (begin_signal_tab("Overview", 0, state)) {
            if (begin_signal_grid("##HardwareOverviewSubplots")) {
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

        if (begin_signal_tab("Timeline", 1, state)) {
            ImGui::TextUnformatted("Hardware events");
            ImGui::TextDisabled("Reference state, safety stops, front-clearance warnings and loop timing spikes.");
            render_timeline_events(build_hardware_timeline(hardware), "HardwareTimelineEvents");
            ImGui::EndTabItem();
        }

        if (begin_signal_tab("MPC tracking", 4, state)) {
            if (begin_signal_grid("##HardwareTrackingSubplots")) {
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

        if (begin_signal_tab("Drivetrain", 2, state)) {
            if (begin_signal_grid("##HardwareDrivetrainSubplots")) {
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

        if (begin_signal_tab("Performance", 5, state)) {
            if (begin_signal_grid("##HardwarePerformanceSubplots")) {
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
