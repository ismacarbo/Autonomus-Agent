std::vector<TimelineEvent> build_sim_timeline(const PlannerDrivenVehicleSim& sim) {
    std::vector<TimelineEvent> events;
    const std::vector<TelemetrySample>& history = sim.history();
    if (history.empty()) {
        return events;
    }

    push_timeline_event(&events,
                        history.front().time,
                        "Run started",
                        std::string(thesis_sim::environment_mode_name(sim.environment_mode())) +
                            " / " + map_preset_name(sim.world()),
                        ImVec4(0.43f, 0.82f, 0.96f, 1.0f));

    bool had_reference = history.front().planner_has_reference > 0.5;
    bool in_mixed_gate = history.front().mixed_mode > 0.5;
    bool clearance_warning = history.front().min_lidar > 0.0 && history.front().min_lidar < 0.25;
    int chosen_gate = static_cast<int>(std::lround(history.front().chosen_gate_index));
    int mixed_switches = static_cast<int>(std::lround(history.front().mixed_switches));
    int mixed_aborts = static_cast<int>(std::lround(history.front().mixed_aborts));
    bool lag_warning = history.front().step_ms > 30.0;

    for (size_t i = 1; i < history.size(); ++i) {
        const TelemetrySample& sample = history[i];
        const bool has_reference = sample.planner_has_reference > 0.5;
        if (has_reference != had_reference) {
            push_timeline_event(&events,
                                sample.time,
                                has_reference ? "Planner reference acquired" : "Planner reference lost",
                                has_reference ? "MPC received a valid path reference." : "Controller is waiting for a usable reference.",
                                has_reference ? ImVec4(0.48f, 0.88f, 0.62f, 1.0f)
                                              : ImVec4(0.95f, 0.60f, 0.34f, 1.0f));
            had_reference = has_reference;
        }

        const int gate = static_cast<int>(std::lround(sample.chosen_gate_index));
        if (gate >= 0 && gate != chosen_gate) {
            push_timeline_event(&events,
                                sample.time,
                                "Gate selected",
                                "Gate index " + std::to_string(gate) +
                                    ", distance " + std::to_string(sample.chosen_gate_distance) + " m.",
                                ImVec4(0.97f, 0.89f, 0.45f, 1.0f));
        }
        chosen_gate = gate;

        const bool mixed_gate = sample.mixed_mode > 0.5;
        if (mixed_gate != in_mixed_gate) {
            push_timeline_event(&events,
                                sample.time,
                                mixed_gate ? "Mixed gate mode" : "Structured rejoin",
                                mixed_gate ? "The arbiter switched from road tracking to dynamic gate tracking."
                                           : "The vehicle returned to the structured road objective.",
                                mixed_gate ? ImVec4(0.96f, 0.66f, 0.28f, 1.0f)
                                           : ImVec4(0.36f, 0.73f, 0.98f, 1.0f));
            in_mixed_gate = mixed_gate;
        }

        const int next_switches = static_cast<int>(std::lround(sample.mixed_switches));
        if (next_switches > mixed_switches) {
            push_timeline_event(&events,
                                sample.time,
                                "Mixed switch counted",
                                "Gate score " + std::to_string(sample.mixed_gate_score) +
                                    ", road score " + std::to_string(sample.mixed_structured_score) + ".",
                                ImVec4(0.96f, 0.66f, 0.28f, 1.0f));
            mixed_switches = next_switches;
        }

        const int next_aborts = static_cast<int>(std::lround(sample.mixed_aborts));
        if (next_aborts > mixed_aborts) {
            push_timeline_event(&events,
                                sample.time,
                                "Mixed abort",
                                "Dynamic gate rejected; road tracking restored.",
                                ImVec4(0.95f, 0.40f, 0.34f, 1.0f));
            mixed_aborts = next_aborts;
        }

        const bool next_clearance_warning = sample.min_lidar > 0.0 && sample.min_lidar < 0.25;
        if (next_clearance_warning && !clearance_warning) {
            push_timeline_event(&events,
                                sample.time,
                                "Clearance warning",
                                "Minimum LiDAR distance below 0.25 m.",
                                ImVec4(0.95f, 0.60f, 0.34f, 1.0f));
        }
        clearance_warning = next_clearance_warning && !(sample.min_lidar > 0.36);

        const bool next_lag_warning = sample.step_ms > 30.0;
        if (next_lag_warning && !lag_warning) {
            push_timeline_event(&events,
                                sample.time,
                                "Frame budget spike",
                                "Step time exceeded 30 ms.",
                                ImVec4(0.80f, 0.62f, 0.34f, 1.0f));
        }
        lag_warning = next_lag_warning;
    }

    push_timeline_event(&events,
                        history.back().time,
                        sim.goal_reached() ? "Goal reached" : (sim.collision() ? "Collision" : "Latest sample"),
                        "Final goal distance " + std::to_string(history.back().distance_to_goal) + " m.",
                        sim.goal_reached() ? ImVec4(0.48f, 0.88f, 0.62f, 1.0f)
                                           : (sim.collision() ? ImVec4(0.95f, 0.40f, 0.34f, 1.0f)
                                                             : ImVec4(0.74f, 0.79f, 0.84f, 1.0f)));
    return events;
}

std::vector<TimelineEvent> build_hardware_timeline(const HardwareViewerState& hardware) {
    std::vector<TimelineEvent> events;
    const std::vector<HardwareTelemetrySample>& history = hardware.history;
    if (history.empty()) {
        return events;
    }

    push_timeline_event(&events,
                        history.front().time,
                        "Hardware stream started",
                        hardware.has_scene ? hardware.scene.vehicle_model_name : "Waiting for scene metadata.",
                        ImVec4(0.43f, 0.82f, 0.96f, 1.0f));

    bool had_reference = history.front().planner_has_reference > 0.5;
    bool safety_stop = history.front().safety_stop_active > 0.5;
    bool clearance_warning = history.front().front_lidar > 0.0 && history.front().front_lidar < 0.22;
    int chosen_gate = static_cast<int>(std::lround(history.front().chosen_gate_index));
    bool lag_warning = history.front().step_ms > 50.0;

    for (size_t i = 1; i < history.size(); ++i) {
        const HardwareTelemetrySample& sample = history[i];
        const bool has_reference = sample.planner_has_reference > 0.5;
        if (has_reference != had_reference) {
            push_timeline_event(&events,
                                sample.time,
                                has_reference ? "Reference ready" : "Reference lost",
                                has_reference ? "Planner trajectory is feeding the hardware controller."
                                              : "The runner is waiting for a valid path reference.",
                                has_reference ? ImVec4(0.48f, 0.88f, 0.62f, 1.0f)
                                              : ImVec4(0.95f, 0.60f, 0.34f, 1.0f));
            had_reference = has_reference;
        }

        const bool stop_active = sample.safety_stop_active > 0.5;
        if (stop_active != safety_stop) {
            push_timeline_event(&events,
                                sample.time,
                                stop_active ? "Safety stop active" : "Safety stop cleared",
                                stop_active ? "Controller command is blocked by clearance or safety policy."
                                            : "Motion commands are allowed again.",
                                stop_active ? ImVec4(0.95f, 0.40f, 0.34f, 1.0f)
                                            : ImVec4(0.48f, 0.88f, 0.62f, 1.0f));
            safety_stop = stop_active;
        }

        const int gate = static_cast<int>(std::lround(sample.chosen_gate_index));
        if (gate >= 0 && gate != chosen_gate) {
            push_timeline_event(&events,
                                sample.time,
                                "Hardware gate selected",
                                "Gate index " + std::to_string(gate) +
                                    ", distance " + std::to_string(sample.chosen_gate_distance) + " m.",
                                ImVec4(0.97f, 0.89f, 0.45f, 1.0f));
        }
        chosen_gate = gate;

        const bool next_clearance_warning = sample.front_lidar > 0.0 && sample.front_lidar < 0.22;
        if (next_clearance_warning && !clearance_warning) {
            push_timeline_event(&events,
                                sample.time,
                                "Front clearance warning",
                                "Front LiDAR distance below 0.22 m.",
                                ImVec4(0.95f, 0.60f, 0.34f, 1.0f));
        }
        clearance_warning = next_clearance_warning && !(sample.front_lidar > 0.34);

        const bool next_lag_warning = sample.step_ms > 50.0;
        if (next_lag_warning && !lag_warning) {
            push_timeline_event(&events,
                                sample.time,
                                "Hardware loop spike",
                                "Step time exceeded 50 ms.",
                                ImVec4(0.80f, 0.62f, 0.34f, 1.0f));
        }
        lag_warning = next_lag_warning;
    }

    push_timeline_event(&events,
                        history.back().time,
                        hardware.frame.goal_reached ? "Goal reached" : "Latest hardware sample",
                        "Current goal distance " + std::to_string(history.back().distance_to_goal) + " m.",
                        hardware.frame.goal_reached ? ImVec4(0.48f, 0.88f, 0.62f, 1.0f)
                                                    : ImVec4(0.74f, 0.79f, 0.84f, 1.0f));
    return events;
}

MetricSummary summarize_metric(const std::vector<TelemetrySample>& history, double TelemetrySample::*member) {
    MetricSummary summary;
    if (history.empty()) {
        return summary;
    }
    double total = 0.0;
    for (const TelemetrySample& sample : history) {
        const double value = sample.*member;
        total += value;
        summary.max = std::max(summary.max, value);
    }
    summary.avg = total / static_cast<double>(history.size());
    return summary;
}

MetricSummary summarize_metric(const std::vector<HardwareTelemetrySample>& history, double HardwareTelemetrySample::*member) {
    MetricSummary summary;
    if (history.empty()) {
        return summary;
    }
    double total = 0.0;
    for (const HardwareTelemetrySample& sample : history) {
        const double value = sample.*member;
        total += value;
        summary.max = std::max(summary.max, value);
    }
    summary.avg = total / static_cast<double>(history.size());
    return summary;
}

std::string default_hardware_report_path(const HardwareViewerState& hardware,
                                         const UiState& ui_state,
                                         const char* source_tag) {
    namespace fs = std::filesystem;
    std::error_code ec;
    fs::path report_dir = fs::current_path(ec) / "reports";
    if (!ec) {
        fs::create_directories(report_dir, ec);
    }

    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm{};
    if (const std::tm* tm_ptr = std::localtime(&now_time)) {
        local_tm = *tm_ptr;
    }
    const auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;

    const EnvironmentMode environment = hardware.has_scene
                                            ? hardware.scene.world.environment_mode()
                                            : static_cast<EnvironmentMode>(ui_state.hardware_environment_mode);
    const std::string preset_name =
        environment == EnvironmentMode::MixedRoadGates
            ? "Mixed Hardware Road Gate"
            : (environment == EnvironmentMode::StructuredRoad
                   ? (hardware.has_scene
                          ? thesis_sim::structured_map_preset_name(hardware.scene.world.structured_preset())
                          : thesis_sim::structured_map_preset_name(
                                static_cast<StructuredMapPreset>(ui_state.hardware_structured_preset)))
                   : (hardware.has_scene
                          ? thesis_sim::unstructured_map_preset_name(hardware.scene.world.unstructured_preset())
                          : thesis_sim::unstructured_map_preset_name(
                                static_cast<UnstructuredMapPreset>(ui_state.hardware_unstructured_preset))));

    std::ostringstream name;
    name << "thesis_hardware_"
         << environment_mode_slug(environment)
         << "_" << slugify(preset_name)
         << "_" << source_tag << "_"
         << std::put_time(&local_tm, "%Y%m%d_%H%M%S")
         << "_" << std::setw(3) << std::setfill('0') << millis
         << ".json";
    return (report_dir / name.str()).string();
}

