WorldMap WorldMap::unstructured_demo(UnstructuredMapPreset preset,
                                     GateBehaviorMode gate_behavior,
                                     std::uint32_t gate_seed) {
    WorldMap world;
    world.environment_mode_ = EnvironmentMode::UnstructuredGates;
    world.unstructured_preset_ = preset;
    world.bounds_ = {0.0, 0.0, 40.0, 24.0};
    world.start_ = {4.0, 11.0};
    world.goal_ = {36.0, 20.0};
    world.start_heading_ = 0.0;

    switch (preset) {
        case UnstructuredMapPreset::RobotValidation:
            world.obstacles_ = {
                {10.0, 0.0, 12.0, 7.0},
                {10.0, 15.0, 12.0, 24.0},
                {18.2, 8.5, 23.0, 14.5},
                {29.5, 0.0, 31.5, 8.0},
                {30.0, 18.6, 31.5, 24.0},
            };
            world.gate_templates_ = {
                {"gap_entry", {13.4, 12.6}, {13.4, 12.6}, {0.16, 0.45}, 0.07, 0.20, 0.0, false},
                {"upper_bypass", {17.4, 17.8}, {17.4, 17.8}, {0.32, 0.22}, 0.05, 1.40, 0.0, false},
                {"exit_gap", {28.4, 16.2}, {28.4, 16.2}, {0.22, 0.40}, 0.06, 2.20, 0.0, false},
                {"goal_approach", {33.0, 18.5}, {33.0, 18.5}, {0.32, 0.28}, 0.05, 2.90, 0.0, false},
                {"goal", world.goal_, world.goal_, {0.0, 0.0}, 0.0, 0.0, 0.0, true},
            };
            break;
        case UnstructuredMapPreset::IdealValidation:
            world.bounds_ = {0.0, 0.0, 2.70, 2.40};
            world.start_ = {0.28, 1.00};
            world.goal_ = {2.35, 1.15};
            world.start_heading_ = 0.0;
            world.obstacles_ = {
                {0.90, 0.00, 1.08, 0.55},
                {1.70, 0.00, 1.88, 0.60},
            };
            world.gate_templates_ = {
                {"gate_entry", {1.18, 1.00}, {1.18, 1.00}, {0.0, 0.0}, 0.0, 0.0, 0.0, false},
                {"exit_align", {2.00, 1.15}, {2.00, 1.15}, {0.0, 0.0}, 0.0, 0.0, 0.0, false},
                {"goal", world.goal_, world.goal_, {0.0, 0.0}, 0.0, 0.0, 0.0, true},
            };
            break;
        case UnstructuredMapPreset::Custom:
            world.bounds_ = {0.0, 0.0, 2.0, 2.0};
            world.start_ = {0.24, 1.00};
            world.goal_ = {1.76, 1.00};
            world.start_heading_ = 0.0;
            world.obstacles_.clear();
            world.gate_templates_ = {
                {"goal", world.goal_, world.goal_, {0.0, 0.0}, 0.0, 0.0, 0.0, true},
            };
            break;
        case UnstructuredMapPreset::TightCorridor:
            world.obstacles_ = {
                {10.0, 0.0, 12.0, 8.0},
                {10.0, 14.2, 12.0, 24.0},
                {18.0, 8.7, 23.4, 14.8},
                {29.3, 0.0, 31.6, 8.4},
                {29.7, 18.0, 31.6, 24.0},
            };
            world.gate_templates_ = {
                {"gap_entry", {12.6, 12.0}, {12.6, 12.0}, {0.12, 0.30}, 0.07, 0.20, 0.0, false},
                {"upper_bypass", {17.3, 17.1}, {17.3, 17.1}, {0.22, 0.16}, 0.05, 1.40, 0.0, false},
                {"exit_gap", {28.1, 16.1}, {28.1, 16.1}, {0.18, 0.24}, 0.06, 2.20, 0.0, false},
                // Stay below and to the right of the final block before
                // turning toward the goal.  The previous anchor intersected
                // the block once the definitive 25 x 15 cm car footprint was
                // applied, even though its centreline remained outside.
                {"goal_approach", {33.5, 16.7}, {33.5, 16.7}, {0.20, 0.18}, 0.05, 2.90, 0.0, false},
                {"goal", world.goal_, world.goal_, {0.0, 0.0}, 0.0, 0.0, 0.0, true},
            };
            break;
        case UnstructuredMapPreset::WideSlalom:
            world.obstacles_ = {
                {9.5, 0.0, 11.8, 6.2},
                {11.0, 17.8, 13.2, 24.0},
                {18.4, 6.8, 22.6, 11.2},
                {21.2, 16.4, 24.2, 21.8},
                {30.2, 0.0, 32.0, 6.4},
            };
            world.gate_templates_ = {
                {"gate_a", {12.8, 9.8}, {12.8, 9.8}, {0.28, 0.75}, 0.07, 0.10, 0.0, false},
                {"gate_b", {18.4, 14.4}, {18.4, 14.4}, {0.35, 0.80}, 0.05, 1.20, 0.0, false},
                {"gate_c", {25.8, 11.8}, {25.8, 11.8}, {0.35, 0.75}, 0.06, 2.00, 0.0, false},
                {"goal_approach", {32.8, 16.8}, {32.8, 16.8}, {0.40, 0.60}, 0.05, 2.70, 0.0, false},
                {"goal", world.goal_, world.goal_, {0.0, 0.0}, 0.0, 0.0, 0.0, true},
            };
            break;
        case UnstructuredMapPreset::LowerBypass:
            world.obstacles_ = {
                {10.0, 16.0, 12.2, 24.0},
                {10.0, 0.0, 12.0, 6.0},
                {18.0, 10.5, 23.4, 16.2},
                {29.5, 16.5, 31.5, 24.0},
                {29.4, 0.0, 31.5, 6.5},
            };
            world.gate_templates_ = {
                {"gap_entry", {13.2, 10.1}, {13.2, 10.1}, {0.18, 0.42}, 0.07, 0.20, 0.0, false},
                {"lower_bypass", {18.0, 6.8}, {18.0, 6.8}, {0.28, 0.30}, 0.05, 1.40, 0.0, false},
                {"exit_gap", {28.0, 8.6}, {28.0, 8.6}, {0.22, 0.35}, 0.06, 2.20, 0.0, false},
                {"goal_approach", {33.0, 13.8}, {33.0, 13.8}, {0.32, 0.34}, 0.05, 2.90, 0.0, false},
                {"goal", world.goal_, world.goal_, {0.0, 0.0}, 0.0, 0.0, 0.0, true},
            };
            break;
        case UnstructuredMapPreset::HardwareLab:
            world.bounds_ = {0.0, 0.0, 2.70, 2.40};
            world.start_ = {0.28, 1.00};
            world.goal_ = {2.35, 1.15};
            world.start_heading_ = 0.0;
            world.obstacles_ = {
                {0.90, 0.00, 1.08, 0.55},
                {1.70, 0.00, 1.88, 0.60},
            };
            world.gate_templates_ = {
                {"gate_entry", {1.18, 1.00}, {1.18, 1.00}, {0.0, 0.0}, 0.0, 0.0, 0.0, false},
                {"exit_align", {2.00, 1.15}, {2.00, 1.15}, {0.0, 0.0}, 0.0, 0.0, 0.0, false},
                {"goal", world.goal_, world.goal_, {0.0, 0.0}, 0.0, 0.0, 0.0, true},
            };
            break;
        default:
            break;
    }

    // The original AVEC-style presets use a 40 x 24 model-space canvas.
    // Convert those coordinates uniformly to metres: preserve the original
    // aspect ratio and obstacle/gate geometry instead of stretching the map
    // into a square. Native metric presets are already expressed in metres.
    const double source_width = world.bounds_.max_x - world.bounds_.min_x;
    const double source_height = world.bounds_.max_y - world.bounds_.min_y;
    if (source_width > 5.0 && source_height > kEps) {
        constexpr double kMetricHeightM = 3.00;
        const Rect source_bounds = world.bounds_;
        const double scale = kMetricHeightM / source_height;
        const auto transform_point = [&](const Vec2& point) {
            return Vec2{
                (point.x - source_bounds.min_x) * scale,
                (point.y - source_bounds.min_y) * scale,
            };
        };
        const auto transform_rect = [&](const Rect& rect) {
            const Vec2 min_point = transform_point({rect.min_x, rect.min_y});
            const Vec2 max_point = transform_point({rect.max_x, rect.max_y});
            return Rect{min_point.x, min_point.y, max_point.x, max_point.y};
        };
        world.start_ = transform_point(world.start_);
        world.goal_ = transform_point(world.goal_);
        for (Rect& obstacle : world.obstacles_) {
            obstacle = transform_rect(obstacle);
        }
        for (Rect& obstacle : world.perception_obstacles_) {
            obstacle = transform_rect(obstacle);
        }
        for (DynamicObstacleSpec& obstacle : world.dynamic_obstacles_) {
            obstacle.obstacle = transform_rect(obstacle.obstacle);
            obstacle.activate_after_progress_m *= scale;
        }
        for (GateSpec& gate : world.gate_templates_) {
            gate.position = transform_point(gate.position);
            gate.anchor_position = transform_point(gate.anchor_position);
            gate.motion_amplitude.x *= scale;
            gate.motion_amplitude.y *= scale;
        }
        world.bounds_ = {0.0, 0.0, source_width * scale, kMetricHeightM};
    }

    world.gates_ = world.gate_templates_;
    recompute_gate_headings(&world.gates_, world.start_, world.goal_);
    world.set_gate_behavior(gate_behavior, gate_seed);

    return world;
}
