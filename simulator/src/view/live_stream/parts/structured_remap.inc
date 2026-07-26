bool points_form_closed_loop(const std::vector<Vec2>& points, double threshold = 0.45) {
    return points.size() >= 3 && distance(points.front(), points.back()) <= threshold;
}

double wrap_angle(double angle) {
    const double kPi = 0.5 * kTwoPi;
    while (angle > kPi) {
        angle -= kTwoPi;
    }
    while (angle < -kPi) {
        angle += kTwoPi;
    }
    return angle;
}

struct PolylineLayout {
    std::vector<Vec2> points;
    std::vector<double> cumulative_s;
    double total_length = 0.0;
    bool closed_loop = false;
};

struct PolylineProjection {
    bool valid = false;
    Vec2 point;
    double s = 0.0;
    double heading = 0.0;
    double lateral = 0.0;
};

struct StructuredDisplayRemap {
    bool active = false;
    WorldMap display_world;
    PolylineLayout source_path;
    PolylineLayout display_path;
};

double clamp_value(double value, double lo, double hi) {
    return std::max(lo, std::min(value, hi));
}

PolylineLayout make_polyline_layout(const std::vector<Vec2>& points) {
    PolylineLayout layout;
    layout.points = points;
    layout.closed_loop = points_form_closed_loop(points);
    layout.cumulative_s.reserve(points.size());
    layout.cumulative_s.push_back(0.0);
    for (std::size_t i = 1; i < points.size(); ++i) {
        layout.total_length += distance(points[i - 1], points[i]);
        layout.cumulative_s.push_back(layout.total_length);
    }
    return layout;
}

PolylineProjection project_point_onto_polyline(const PolylineLayout& layout, const Vec2& point) {
    PolylineProjection best;
    double best_distance_sq = 0.0;
    bool have_best = false;

    if (layout.points.size() < 2) {
        return best;
    }

    for (std::size_t i = 0; i + 1 < layout.points.size(); ++i) {
        const Vec2& a = layout.points[i];
        const Vec2& b = layout.points[i + 1];
        const double dx = b.x - a.x;
        const double dy = b.y - a.y;
        const double segment_length_sq = dx * dx + dy * dy;
        if (segment_length_sq <= 1e-12) {
            continue;
        }

        const double t = clamp_value(((point.x - a.x) * dx + (point.y - a.y) * dy) / segment_length_sq, 0.0, 1.0);
        const Vec2 projected{a.x + dx * t, a.y + dy * t};
        const double error_x = point.x - projected.x;
        const double error_y = point.y - projected.y;
        const double distance_sq = error_x * error_x + error_y * error_y;
        if (!have_best || distance_sq < best_distance_sq) {
            const double heading = std::atan2(dy, dx);
            best.valid = true;
            best.point = projected;
            best.s = layout.cumulative_s[i] + std::sqrt(segment_length_sq) * t;
            best.heading = heading;
            best.lateral = -std::sin(heading) * error_x + std::cos(heading) * error_y;
            best_distance_sq = distance_sq;
            have_best = true;
        }
    }

    return best;
}

PolylineProjection sample_polyline_at_s(const PolylineLayout& layout, double s) {
    PolylineProjection sample;
    if (layout.points.empty()) {
        return sample;
    }
    if (layout.points.size() == 1 || layout.total_length <= 1e-9) {
        sample.valid = true;
        sample.point = layout.points.front();
        sample.heading = 0.0;
        sample.s = 0.0;
        return sample;
    }

    double wrapped_s = s;
    if (layout.closed_loop && layout.total_length > 1e-9) {
        wrapped_s = std::fmod(wrapped_s, layout.total_length);
        if (wrapped_s < 0.0) {
            wrapped_s += layout.total_length;
        }
    } else {
        wrapped_s = clamp_value(wrapped_s, 0.0, layout.total_length);
    }

    for (std::size_t i = 0; i + 1 < layout.points.size(); ++i) {
        const double start_s = layout.cumulative_s[i];
        const double end_s = layout.cumulative_s[i + 1];
        const double segment_length = end_s - start_s;
        if (wrapped_s > end_s && i + 2 < layout.points.size()) {
            continue;
        }

        const Vec2& a = layout.points[i];
        const Vec2& b = layout.points[i + 1];
        const double heading = std::atan2(b.y - a.y, b.x - a.x);
        const double alpha = segment_length > 1e-9 ? clamp_value((wrapped_s - start_s) / segment_length, 0.0, 1.0) : 0.0;
        sample.valid = true;
        sample.point = {a.x + (b.x - a.x) * alpha, a.y + (b.y - a.y) * alpha};
        sample.heading = heading;
        sample.s = wrapped_s;
        return sample;
    }

    sample.valid = true;
    sample.point = layout.points.back();
    sample.heading = std::atan2(
        layout.points.back().y - layout.points[layout.points.size() - 2].y,
        layout.points.back().x - layout.points[layout.points.size() - 2].x);
    sample.s = wrapped_s;
    return sample;
}

bool needs_structured_display_normalization(const WorldMap& world) {
    if (world.environment_mode() != EnvironmentMode::StructuredRoad) {
        return false;
    }
    // Keep structured hardware scenes in physical meters. Remapping compact
    // tracks onto the large validation road makes the robot footprint and
    // wheel placement appear to change scale between preview and live stream.
    return false;
}

StructuredDisplayRemap make_structured_display_remap(const WorldMap& source_world) {
    StructuredDisplayRemap remap;
    remap.display_world = source_world;
    if (!needs_structured_display_normalization(source_world)) {
        return remap;
    }

    remap.source_path = make_polyline_layout(source_world.road_centerline());
    remap.display_world = WorldMap::structured_demo(StructuredMapPreset::ValidationRoad);
    remap.display_path = make_polyline_layout(remap.display_world.road_centerline());
    remap.active = remap.source_path.total_length > 1e-6 && remap.display_path.total_length > 1e-6;
    if (!remap.active) {
        remap.display_world = source_world;
    }
    return remap;
}

Vec2 remap_point_to_display(const StructuredDisplayRemap& remap, const Vec2& point) {
    if (!remap.active) {
        return point;
    }
    const PolylineProjection source_projection = project_point_onto_polyline(remap.source_path, point);
    if (!source_projection.valid || remap.source_path.total_length <= 1e-9) {
        return point;
    }

    const double progress = source_projection.s / remap.source_path.total_length;
    const PolylineProjection display_projection =
        sample_polyline_at_s(remap.display_path, progress * remap.display_path.total_length);
    if (!display_projection.valid) {
        return point;
    }

    return {
        display_projection.point.x - std::sin(display_projection.heading) * source_projection.lateral,
        display_projection.point.y + std::cos(display_projection.heading) * source_projection.lateral,
    };
}

double remap_yaw_to_display(const StructuredDisplayRemap& remap, const Vec2& source_point, double yaw) {
    if (!remap.active) {
        return yaw;
    }
    const PolylineProjection source_projection = project_point_onto_polyline(remap.source_path, source_point);
    if (!source_projection.valid || remap.source_path.total_length <= 1e-9) {
        return yaw;
    }

    const double progress = source_projection.s / remap.source_path.total_length;
    const PolylineProjection display_projection =
        sample_polyline_at_s(remap.display_path, progress * remap.display_path.total_length);
    if (!display_projection.valid) {
        return yaw;
    }
    return wrap_angle(yaw - source_projection.heading + display_projection.heading);
}

std::vector<Vec2> remap_points_to_display(const StructuredDisplayRemap& remap,
                                          const std::vector<Vec2>& points) {
    if (!remap.active || points.empty()) {
        return points;
    }

    std::vector<Vec2> out;
    out.reserve(points.size());
    for (const Vec2& point : points) {
        out.push_back(remap_point_to_display(remap, point));
    }
    return out;
}

std::vector<LidarHit> remap_hits_to_display(const StructuredDisplayRemap& remap,
                                            const std::vector<LidarHit>& hits,
                                            const Vec2& source_origin) {
    if (!remap.active || hits.empty()) {
        return hits;
    }

    const Vec2 display_origin = remap_point_to_display(remap, source_origin);
    std::vector<LidarHit> out;
    out.reserve(hits.size());
    for (const LidarHit& hit : hits) {
        LidarHit mapped = hit;
        mapped.point = remap_point_to_display(remap, hit.point);
        mapped.distance = distance(display_origin, mapped.point);
        mapped.angle = std::atan2(mapped.point.y - display_origin.y, mapped.point.x - display_origin.x);
        out.push_back(std::move(mapped));
    }
    return out;
}

void apply_structured_display_remap(const StructuredDisplayRemap& remap, LiveFrameSnapshot* frame) {
    if (!remap.active || frame == nullptr) {
        return;
    }

    const Vec2 source_vehicle_position = frame->vehicle.position;
    const Vec2 source_navigation_position = frame->navigation_position;
    frame->vehicle.position = remap_point_to_display(remap, source_vehicle_position);
    frame->vehicle.yaw = remap_yaw_to_display(remap, source_vehicle_position, frame->vehicle.yaw);
    frame->navigation_position = remap_point_to_display(remap, source_navigation_position);
    frame->navigation_yaw = remap_yaw_to_display(remap, source_navigation_position, frame->navigation_yaw);
    frame->trail = remap_points_to_display(remap, frame->trail);
    frame->planned_trajectory = remap_points_to_display(remap, frame->planned_trajectory);
    frame->slam_points = remap_points_to_display(remap, frame->slam_points);
    frame->lidar_hits = remap_hits_to_display(remap, frame->lidar_hits, source_navigation_position);
    if (frame->has_latest_sample) {
        const Vec2 source_sample_position{frame->latest_sample.position_x, frame->latest_sample.position_y};
        const Vec2 mapped_sample_position = remap_point_to_display(remap, source_sample_position);
        frame->latest_sample.position_x = mapped_sample_position.x;
        frame->latest_sample.position_y = mapped_sample_position.y;
        frame->latest_sample.yaw = remap_yaw_to_display(remap, source_sample_position, frame->latest_sample.yaw);
    }
}

}  // namespace

