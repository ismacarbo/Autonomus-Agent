const char* gate_behavior_mode_name(GateBehaviorMode mode) {
    switch (mode) {
        case GateBehaviorMode::Static:
            return "Static Gates";
        case GateBehaviorMode::Randomized:
            return "Randomized Gates";
        case GateBehaviorMode::Mobile:
            return "Mobile Gates";
        default:
            return "Unknown";
    }
}

const char* environment_mode_name(EnvironmentMode mode) {
    switch (mode) {
        case EnvironmentMode::UnstructuredGates:
            return "Unstructured Gates";
        case EnvironmentMode::StructuredRoad:
            return "Structured Road";
        case EnvironmentMode::MixedRoadGates:
            return "Mixed Road/Gates";
        default:
            return "Unknown";
    }
}

const char* unstructured_map_preset_name(UnstructuredMapPreset preset) {
    switch (preset) {
        case UnstructuredMapPreset::RobotValidation:
            return "Robot Validation";
        case UnstructuredMapPreset::TightCorridor:
            return "Tight Corridor";
        case UnstructuredMapPreset::WideSlalom:
            return "Wide Slalom";
        case UnstructuredMapPreset::LowerBypass:
            return "Lower Bypass";
        case UnstructuredMapPreset::Custom:
            return "Manual Gate Editor";
        case UnstructuredMapPreset::HardwareLab:
            return "Hardware Lab";
        case UnstructuredMapPreset::IdealValidation:
            return "Ideal Hardware Lab";
        default:
            return "Unknown";
    }
}

const char* structured_map_preset_name(StructuredMapPreset preset) {
    switch (preset) {
        case StructuredMapPreset::ValidationRoad:
            return "Validation Road";
        case StructuredMapPreset::CircleLoop:
            return "Circle Loop";
        case StructuredMapPreset::ZigZag:
            return "Zig-Zag";
        case StructuredMapPreset::Custom:
            return "Custom";
        case StructuredMapPreset::HardwareTrack:
            return "Hardware Track";
        case StructuredMapPreset::FigureEight:
            return "Figure Eight";
        case StructuredMapPreset::IdealCircle:
            return "Ideal Validation Road";
        case StructuredMapPreset::TankCircuit:
            return "Tank Circuit";
        default:
            return "Unknown";
    }
}

double distance(const Vec2& a, const Vec2& b) {
    return std::hypot(b.x - a.x, b.y - a.y);
}

double angle_to(const Vec2& from, const Vec2& to) {
    return std::atan2(to.y - from.y, to.x - from.x);
}

Vec2 rotate(const Vec2& point, double yaw) {
    const double c = std::cos(yaw);
    const double s = std::sin(yaw);
    return {point.x * c - point.y * s, point.x * s + point.y * c};
}

std::array<Vec2, 4> make_box_corners(const Vec2& center, double yaw, double length, double width) {
    const double half_l = length * 0.5;
    const double half_w = width * 0.5;
    const std::array<Vec2, 4> local{{
        {half_l, half_w},
        {half_l, -half_w},
        {-half_l, -half_w},
        {-half_l, half_w},
    }};

    std::array<Vec2, 4> corners{};
    for (size_t i = 0; i < local.size(); ++i) {
        const Vec2 rotated = rotate(local[i], yaw);
        corners[i] = {center.x + rotated.x, center.y + rotated.y};
    }
    return corners;
}

WorldMap normalize_structured_world(WorldMap world,
                                    double arena_span_m,
                                    double content_span_m) {
    if (world.environment_mode() != EnvironmentMode::StructuredRoad ||
        world.structured_preset() == StructuredMapPreset::Custom ||
        arena_span_m <= 0.0 || content_span_m <= 0.0) {
        return world;
    }

    Rect content{
        std::min(world.start().x, world.goal().x),
        std::min(world.start().y, world.goal().y),
        std::max(world.start().x, world.goal().x),
        std::max(world.start().y, world.goal().y),
    };
    const auto include_point = [&content](const Vec2& point) {
        content.min_x = std::min(content.min_x, point.x);
        content.min_y = std::min(content.min_y, point.y);
        content.max_x = std::max(content.max_x, point.x);
        content.max_y = std::max(content.max_y, point.y);
    };
    for (const Vec2& point : world.road_centerline()) {
        include_point(point);
    }
    for (const Rect& obstacle : world.obstacles()) {
        include_point({obstacle.min_x, obstacle.min_y});
        include_point({obstacle.max_x, obstacle.max_y});
    }

    const double source_span = std::max(content.max_x - content.min_x, content.max_y - content.min_y);
    if (source_span <= kEps) {
        return world;
    }
    const Vec2 source_center{
        0.5 * (content.min_x + content.max_x),
        0.5 * (content.min_y + content.max_y),
    };
    const Vec2 target_center{0.5 * arena_span_m, 0.5 * arena_span_m};
    const double scale = content_span_m / source_span;
    const auto transform_point = [&](const Vec2& point) {
        return Vec2{
            target_center.x + (point.x - source_center.x) * scale,
            target_center.y + (point.y - source_center.y) * scale,
        };
    };
    const auto transform_rect = [&](const Rect& rect) {
        const Vec2 min_point = transform_point({rect.min_x, rect.min_y});
        const Vec2 max_point = transform_point({rect.max_x, rect.max_y});
        return Rect{
            std::min(min_point.x, max_point.x),
            std::min(min_point.y, max_point.y),
            std::max(min_point.x, max_point.x),
            std::max(min_point.y, max_point.y),
        };
    };

    world.set_start(transform_point(world.start()));
    world.set_goal(transform_point(world.goal()));
    for (Rect& obstacle : world.editable_obstacles()) {
        obstacle = transform_rect(obstacle);
    }
    for (Rect& obstacle : world.editable_perception_obstacles()) {
        obstacle = transform_rect(obstacle);
    }
    for (DynamicObstacleSpec& obstacle : world.editable_dynamic_obstacles()) {
        obstacle.obstacle = transform_rect(obstacle.obstacle);
        obstacle.activate_after_progress_m *= scale;
    }
    for (GateSpec& gate : world.editable_gates()) {
        gate.position = transform_point(gate.position);
        gate.anchor_position = transform_point(gate.anchor_position);
        gate.motion_amplitude.x *= scale;
        gate.motion_amplitude.y *= scale;
    }
    for (Vec2& point : world.editable_road_centerline()) {
        point = transform_point(point);
    }
    world.set_bounds({0.0, 0.0, arena_span_m, arena_span_m});
    return world;
}

