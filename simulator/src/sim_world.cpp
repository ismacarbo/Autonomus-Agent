#include "sim_world.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <random>

namespace thesis_sim {

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kEps = 1e-9;

double dot(const Vec2& a, const Vec2& b) {
    return a.x * b.x + a.y * b.y;
}

Vec2 subtract(const Vec2& a, const Vec2& b) {
    return {a.x - b.x, a.y - b.y};
}

std::array<Vec2, 4> rect_corners(const Rect& rect) {
    return {{
        {rect.min_x, rect.min_y},
        {rect.max_x, rect.min_y},
        {rect.max_x, rect.max_y},
        {rect.min_x, rect.max_y},
    }};
}

Rect expand(const Rect& rect, double padding) {
    return {
        rect.min_x - padding,
        rect.min_y - padding,
        rect.max_x + padding,
        rect.max_y + padding,
    };
}

bool point_in_rect(const Vec2& p, const Rect& rect) {
    return p.x >= rect.min_x && p.x <= rect.max_x && p.y >= rect.min_y && p.y <= rect.max_y;
}

bool axis_overlap(const std::array<Vec2, 4>& polygon, const Rect& rect, const Vec2& axis) {
    double poly_min = dot(polygon[0], axis);
    double poly_max = poly_min;
    for (size_t i = 1; i < polygon.size(); ++i) {
        const double projection = dot(polygon[i], axis);
        poly_min = std::min(poly_min, projection);
        poly_max = std::max(poly_max, projection);
    }

    const auto corners = rect_corners(rect);
    double rect_min = dot(corners[0], axis);
    double rect_max = rect_min;
    for (size_t i = 1; i < corners.size(); ++i) {
        const double projection = dot(corners[i], axis);
        rect_min = std::min(rect_min, projection);
        rect_max = std::max(rect_max, projection);
    }

    return !(poly_max < rect_min || rect_max < poly_min);
}

bool polygon_intersects_rect(const std::array<Vec2, 4>& polygon, const Rect& rect) {
    std::array<Vec2, 6> axes{};
    for (int i = 0; i < 4; ++i) {
        const Vec2 edge = subtract(polygon[(i + 1) % 4], polygon[i]);
        axes[i] = {-edge.y, edge.x};
    }
    axes[4] = {1.0, 0.0};
    axes[5] = {0.0, 1.0};

    for (const Vec2& axis_raw : axes) {
        const double norm = std::hypot(axis_raw.x, axis_raw.y);
        if (norm < kEps) {
            continue;
        }
        const Vec2 axis{axis_raw.x / norm, axis_raw.y / norm};
        if (!axis_overlap(polygon, rect, axis)) {
            return false;
        }
    }
    return true;
}

std::optional<double> ray_rect_distance(const Vec2& origin, const Vec2& dir, const Rect& rect) {
    double tmin = 0.0;
    double tmax = std::numeric_limits<double>::infinity();

    const auto update_axis = [&](double origin_coord, double dir_coord, double min_coord, double max_coord) -> bool {
        if (std::abs(dir_coord) < kEps) {
            return origin_coord >= min_coord && origin_coord <= max_coord;
        }
        double t1 = (min_coord - origin_coord) / dir_coord;
        double t2 = (max_coord - origin_coord) / dir_coord;
        if (t1 > t2) {
            std::swap(t1, t2);
        }
        tmin = std::max(tmin, t1);
        tmax = std::min(tmax, t2);
        return tmin <= tmax;
    };

    if (!update_axis(origin.x, dir.x, rect.min_x, rect.max_x)) {
        return std::nullopt;
    }
    if (!update_axis(origin.y, dir.y, rect.min_y, rect.max_y)) {
        return std::nullopt;
    }

    if (tmax < 0.0) {
        return std::nullopt;
    }

    if (tmin >= 0.0) {
        return tmin;
    }
    return tmax;
}

bool segment_intersects_rect(const Vec2& a, const Vec2& b, const Rect& rect) {
    const Vec2 dir{b.x - a.x, b.y - a.y};
    const auto hit = ray_rect_distance(a, dir, rect);
    if (!hit.has_value()) {
        return false;
    }
    return *hit >= 0.0 && *hit <= 1.0;
}

double clamp_value(double value, double lo, double hi) {
    return std::max(lo, std::min(value, hi));
}

void recompute_gate_headings(std::vector<GateSpec>* gates, const Vec2& goal) {
    if (gates == nullptr || gates->empty()) {
        return;
    }
    for (size_t i = 0; i < gates->size(); ++i) {
        GateSpec& current = (*gates)[i];
        if (current.final) {
            current.heading_hint = 0.0;
            continue;
        }
        const Vec2 target = (i + 1 < gates->size()) ? (*gates)[i + 1].position : goal;
        current.heading_hint = angle_to(current.position, target);
    }
    if (gates->back().final && gates->size() > 1) {
        gates->back().heading_hint = angle_to((*gates)[gates->size() - 2].position, goal);
    }
}

}  // namespace

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

WorldMap WorldMap::thesis_demo(GateBehaviorMode gate_behavior, std::uint32_t gate_seed) {
    WorldMap world;
    world.bounds_ = {0.0, 0.0, 40.0, 24.0};
    world.start_ = {4.0, 11.0};
    world.goal_ = {36.0, 20.0};
    world.start_heading_ = 0.0;

    world.obstacles_ = {
        {10.0, 0.0, 12.0, 7.0},
        {10.0, 15.0, 12.0, 24.0},
        {18.2, 8.5, 23.0, 14.5},
        {29.5, 0.0, 31.5, 8.0},
        {29.5, 17.0, 31.5, 24.0},
    };

    world.gate_templates_ = {
        {"gap_entry", {11.0, 11.0}, {11.0, 11.0}, {0.20, 1.10}, 0.07, 0.20, 0.0, false},
        {"upper_bypass", {21.0, 18.2}, {21.0, 18.2}, {0.90, 0.45}, 0.05, 1.40, 0.0, false},
        {"exit_gap", {30.5, 14.0}, {30.5, 14.0}, {0.35, 1.00}, 0.06, 2.20, 0.0, false},
        {"goal_approach", {34.0, 17.0}, {34.0, 17.0}, {0.70, 0.55}, 0.05, 2.90, 0.0, false},
        {"goal", world.goal_, world.goal_, {0.0, 0.0}, 0.0, 0.0, 0.0, true},
    };
    world.gates_ = world.gate_templates_;
    recompute_gate_headings(&world.gates_, world.goal_);
    world.set_gate_behavior(gate_behavior, gate_seed);

    return world;
}

void WorldMap::set_gate_behavior(GateBehaviorMode gate_behavior, std::uint32_t gate_seed) {
    gate_behavior_ = gate_behavior;
    gate_seed_ = gate_seed;
    reset_gate_layout(gate_seed_);
}

void WorldMap::reset_gate_layout(std::uint32_t gate_seed) {
    gate_seed_ = gate_seed;
    gates_ = gate_templates_;

    if (gate_behavior_ == GateBehaviorMode::Static) {
        for (GateSpec& gate : gates_) {
            gate.position = gate.anchor_position;
        }
        recompute_gate_headings(&gates_, goal_);
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

        anchor.x = clamp_value(anchor.x, bounds_.min_x + 1.0, bounds_.max_x - 1.0);
        anchor.y = clamp_value(anchor.y, bounds_.min_y + 1.0, bounds_.max_y - 1.0);
        gate.anchor_position = anchor;
        gate.position = anchor;
    }

    recompute_gate_headings(&gates_, goal_);
}

void WorldMap::update_gate_layout(double sim_time_s) {
    if (gate_behavior_ != GateBehaviorMode::Mobile) {
        recompute_gate_headings(&gates_, goal_);
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

    recompute_gate_headings(&gates_, goal_);
}

bool WorldMap::line_of_sight(const Vec2& from, const Vec2& to, double padding) const {
    if (!point_in_rect(from, bounds_) || !point_in_rect(to, bounds_)) {
        return false;
    }

    for (const Rect& obstacle : obstacles_) {
        if (segment_intersects_rect(from, to, expand(obstacle, padding))) {
            return false;
        }
    }
    return true;
}

std::vector<LidarHit> WorldMap::raycast(const Vec2& origin, double heading, int beams, double fov_rad, double max_range) const {
    std::vector<LidarHit> hits;
    hits.reserve(static_cast<size_t>(std::max(beams, 1)));

    const int beam_count = std::max(beams, 1);
    const double start_angle = heading - fov_rad * 0.5;
    const double angle_step = beam_count > 1 ? fov_rad / static_cast<double>(beam_count - 1) : 0.0;

    for (int i = 0; i < beam_count; ++i) {
        const double angle = start_angle + static_cast<double>(i) * angle_step;
        const Vec2 dir{std::cos(angle), std::sin(angle)};

        double best = max_range;
        bool hit = false;

        if (const auto bounds_hit = ray_rect_distance(origin, dir, bounds_)) {
            if (*bounds_hit >= 0.0) {
                best = std::min(best, *bounds_hit);
                hit = true;
            }
        }

        for (const Rect& obstacle : obstacles_) {
            if (const auto obstacle_hit = ray_rect_distance(origin, dir, obstacle)) {
                if (*obstacle_hit >= 0.0 && *obstacle_hit < best) {
                    best = *obstacle_hit;
                    hit = true;
                }
            }
        }

        if (best > max_range) {
            best = max_range;
            hit = false;
        }

        hits.push_back({
            angle,
            best,
            {origin.x + dir.x * best, origin.y + dir.y * best},
            hit,
        });
    }

    return hits;
}

bool WorldMap::collides(const std::array<Vec2, 4>& polygon, double padding) const {
    const Rect padded_bounds = expand(bounds_, -padding);
    for (const Vec2& corner : polygon) {
        if (!point_in_rect(corner, padded_bounds)) {
            return true;
        }
    }

    for (const Rect& obstacle : obstacles_) {
        if (polygon_intersects_rect(polygon, expand(obstacle, padding))) {
            return true;
        }
    }
    return false;
}

}  // namespace thesis_sim
