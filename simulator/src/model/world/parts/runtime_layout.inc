WorldMap WorldMap::thesis_demo(UnstructuredMapPreset preset,
                               GateBehaviorMode gate_behavior,
                               std::uint32_t gate_seed) {
    return WorldMap::unstructured_demo(preset, gate_behavior, gate_seed);
}

void WorldMap::set_gate_behavior(GateBehaviorMode gate_behavior, std::uint32_t gate_seed) {
    if (environment_mode_ == EnvironmentMode::StructuredRoad) {
        gate_behavior_ = GateBehaviorMode::Static;
        gate_seed_ = 0;
        gates_.clear();
        gate_templates_.clear();
        return;
    }
    gate_behavior_ = gate_behavior;
    gate_seed_ = gate_seed;
    reset_gate_layout(gate_seed_);
}

void WorldMap::reset_gate_layout(std::uint32_t gate_seed) {
    if (environment_mode_ == EnvironmentMode::StructuredRoad) {
        return;
    }
    gate_seed_ = gate_seed;
    gates_ = gate_templates_;

    if (gate_behavior_ == GateBehaviorMode::Static) {
        for (GateSpec& gate : gates_) {
            gate.position = gate.anchor_position;
        }
        recompute_gate_headings(&gates_, start_, goal_);
        return;
    }

    std::mt19937 rng(gate_seed_);
    for (GateSpec& gate : gates_) {
        if (gate.final) {
            gate.anchor_position = goal_;
            gate.position = goal_;
            continue;
        }

        const double dx_limit = gate.motion_amplitude.x;
        const double dy_limit = gate.motion_amplitude.y;
        std::uniform_real_distribution<double> dx_dist(-dx_limit, dx_limit);
        std::uniform_real_distribution<double> dy_dist(-dy_limit, dy_limit);

        Vec2 anchor{
            gate.anchor_position.x + dx_dist(rng),
            gate.anchor_position.y + dy_dist(rng),
        };

        const double span_x = std::max(bounds_.max_x - bounds_.min_x, 0.1);
        const double span_y = std::max(bounds_.max_y - bounds_.min_y, 0.1);
        const double margin_x = std::clamp(0.08 * span_x, 0.04, 1.0);
        const double margin_y = std::clamp(0.08 * span_y, 0.04, 1.0);
        anchor.x = clamp_value(anchor.x, bounds_.min_x + margin_x, bounds_.max_x - margin_x);
        anchor.y = clamp_value(anchor.y, bounds_.min_y + margin_y, bounds_.max_y - margin_y);
        gate.anchor_position = anchor;
        gate.position = anchor;
    }

    recompute_gate_headings(&gates_, start_, goal_);
}

void WorldMap::update_gate_layout(double sim_time_s) {
    if (environment_mode_ == EnvironmentMode::StructuredRoad) {
        return;
    }
    if (gate_behavior_ != GateBehaviorMode::Mobile) {
        recompute_gate_headings(&gates_, start_, goal_);
        return;
    }

    for (GateSpec& gate : gates_) {
        if (gate.final) {
            gate.position = goal_;
            continue;
        }

        const double omega = 2.0 * kPi * gate.motion_frequency_hz;
        const double phase = omega * sim_time_s + gate.motion_phase_rad;
        gate.position = {
            gate.anchor_position.x + gate.motion_amplitude.x * 0.45 * std::sin(phase),
            gate.anchor_position.y + gate.motion_amplitude.y * 0.45 * std::cos(phase),
        };
    }

    recompute_gate_headings(&gates_, start_, goal_);
}

void WorldMap::reset_perception_obstacles() {
    perception_obstacles_.clear();
    for (DynamicObstacleSpec& spec : dynamic_obstacles_) {
        spec.discovered = false;
    }
}

void WorldMap::update_perception_obstacles(double sim_time_s,
                                           double progress_s,
                                           const Vec2* sensor_origin,
                                           double detection_range_m) {
    if (dynamic_obstacles_.empty()) {
        return;
    }

    perception_obstacles_.clear();
    const bool has_sensor_range =
        sensor_origin != nullptr &&
        std::isfinite(detection_range_m) &&
        detection_range_m > 1e-6;
    for (DynamicObstacleSpec& spec : dynamic_obstacles_) {
        const bool time_ready = sim_time_s + kEps >= spec.activate_after_s;
        const bool progress_ready = progress_s + kEps >= spec.activate_after_progress_m;
        const bool still_active = spec.deactivate_after_s < 0.0 ||
                                  sim_time_s <= spec.deactivate_after_s + kEps;
        const bool inside_lidar_radius =
            has_sensor_range &&
            distance_point_to_rect(*sensor_origin, spec.obstacle) <= detection_range_m + kEps;
        if (!spec.discovered && time_ready && progress_ready && still_active && inside_lidar_radius) {
            spec.discovered = true;
        }
        if (spec.discovered && still_active) {
            perception_obstacles_.push_back(spec.obstacle);
        }
    }
}

void WorldMap::finalize_editor_changes() {
    start_ = clamp_to_bounds(bounds_, start_);
    goal_ = clamp_to_bounds(bounds_, goal_);

    auto sanitize_rects = [&](std::vector<Rect>* rects) {
        if (rects == nullptr) {
            return;
        }
        for (Rect& obstacle : *rects) {
            normalize_rect(&obstacle);
            obstacle.min_x = clamp_value(obstacle.min_x, bounds_.min_x, bounds_.max_x);
            obstacle.max_x = clamp_value(obstacle.max_x, bounds_.min_x, bounds_.max_x);
            obstacle.min_y = clamp_value(obstacle.min_y, bounds_.min_y, bounds_.max_y);
            obstacle.max_y = clamp_value(obstacle.max_y, bounds_.min_y, bounds_.max_y);
            normalize_rect(&obstacle);
        }
    };
    sanitize_rects(&obstacles_);
    sanitize_rects(&perception_obstacles_);
    for (DynamicObstacleSpec& spec : dynamic_obstacles_) {
        normalize_rect(&spec.obstacle);
        spec.obstacle.min_x = clamp_value(spec.obstacle.min_x, bounds_.min_x, bounds_.max_x);
        spec.obstacle.max_x = clamp_value(spec.obstacle.max_x, bounds_.min_x, bounds_.max_x);
        spec.obstacle.min_y = clamp_value(spec.obstacle.min_y, bounds_.min_y, bounds_.max_y);
        spec.obstacle.max_y = clamp_value(spec.obstacle.max_y, bounds_.min_y, bounds_.max_y);
        normalize_rect(&spec.obstacle);
        spec.activate_after_s = std::max(0.0, spec.activate_after_s);
        spec.activate_after_progress_m = std::max(0.0, spec.activate_after_progress_m);
    }

    if (environment_mode_ == EnvironmentMode::StructuredRoad ||
        environment_mode_ == EnvironmentMode::MixedRoadGates) {
        structured_preset_ = StructuredMapPreset::Custom;
        if (environment_mode_ == EnvironmentMode::StructuredRoad) {
            gates_.clear();
            gate_templates_.clear();
            gate_behavior_ = GateBehaviorMode::Static;
            gate_seed_ = 0;
        }

        road_centerline_ = sanitize_structured_centerline(bounds_, start_, goal_, road_centerline_);
        if (points_form_closed_loop(road_centerline_, 0.45) && road_centerline_.size() > 2) {
            std::vector<Vec2> base = road_centerline_;
            base.pop_back();
            const size_t start_index = find_nearest_point_index(base, start_);
            road_centerline_ = rotate_closed_polyline(base, start_index);

            base = road_centerline_;
            base.pop_back();
            start_ = base.front();
            goal_ = start_;
        }
        start_heading_ = heading_from_first_segment(road_centerline_, start_heading_);
        if (environment_mode_ == EnvironmentMode::StructuredRoad) {
            return;
        }
    }

    if (environment_mode_ == EnvironmentMode::UnstructuredGates) {
        road_centerline_.clear();
    }
    unstructured_preset_ = UnstructuredMapPreset::Custom;

    std::vector<GateSpec> sanitized;
    sanitized.reserve(gates_.size() + 1);
    GateSpec final_gate{
        "goal",
        goal_,
        goal_,
        {0.0, 0.0},
        0.0,
        0.0,
        0.0,
        true,
    };

    for (size_t i = 0; i < gates_.size(); ++i) {
        GateSpec gate = gates_[i];
        gate.position = clamp_to_bounds(bounds_, gate.position);
        gate.anchor_position = clamp_to_bounds(bounds_, gate.anchor_position);
        gate.motion_amplitude.x = std::max(0.0, gate.motion_amplitude.x);
        gate.motion_amplitude.y = std::max(0.0, gate.motion_amplitude.y);
        gate.motion_frequency_hz = std::max(0.0, gate.motion_frequency_hz);
        if (gate.name.empty()) {
            gate.name = gate.final ? "goal" : ("gate_" + std::to_string(i + 1));
        }

        if (gate.final) {
            final_gate = gate;
            final_gate.position = goal_;
            final_gate.anchor_position = goal_;
            final_gate.motion_amplitude = {0.0, 0.0};
            final_gate.motion_frequency_hz = 0.0;
            final_gate.final = true;
        } else {
            sanitized.push_back(gate);
        }
    }

    if (final_gate.name.empty()) {
        final_gate.name = "goal";
    }
    sanitized.push_back(final_gate);

    gates_ = sanitized;
    gate_templates_ = gates_;
    recompute_gate_headings(&gate_templates_, start_, goal_);
    gates_ = gate_templates_;

    if (gate_behavior_ == GateBehaviorMode::Static) {
        for (GateSpec& gate : gates_) {
            gate.position = gate.anchor_position;
        }
        recompute_gate_headings(&gates_, start_, goal_);
    } else {
        reset_gate_layout(gate_seed_);
    }
}

}  // namespace thesis_sim
