#include "mvc/model/world/world.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>

namespace thesis_sim {
namespace {

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
    const bool origin_inside = point_in_rect(origin, rect);
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

    if (origin_inside) {
        return tmax;
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


}  // namespace

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

std::vector<LidarHit> WorldMap::raycast(const Vec2& origin,
                                        double heading,
                                        int beams,
                                        double fov_rad,
                                        double max_range,
                                        bool include_perception_obstacles) const {
    std::vector<LidarHit> hits;
    hits.reserve(static_cast<size_t>(std::max(beams, 1)));

    const int beam_count = std::max(beams, 1);
    const double start_angle = heading - fov_rad * 0.5;
    const double angle_step = beam_count > 1 ? fov_rad / static_cast<double>(beam_count - 1) : 0.0;
    const bool bounds_are_lidar_walls =
        !(environment_mode_ == EnvironmentMode::MixedRoadGates &&
          unstructured_preset_ == UnstructuredMapPreset::HardwareLab &&
          obstacles_.empty());

    for (int i = 0; i < beam_count; ++i) {
        const double angle = start_angle + static_cast<double>(i) * angle_step;
        const Vec2 dir{std::cos(angle), std::sin(angle)};

        double best = max_range;
        bool hit = false;

        if (bounds_are_lidar_walls) {
            if (const auto bounds_hit = ray_rect_distance(origin, dir, bounds_)) {
                if (*bounds_hit >= 0.0 && *bounds_hit <= max_range) {
                    best = std::min(best, *bounds_hit);
                    hit = true;
                }
            }
        }

        for (const Rect& obstacle : obstacles_) {
            if (const auto obstacle_hit = ray_rect_distance(origin, dir, obstacle)) {
                if (*obstacle_hit >= 0.0 && *obstacle_hit <= max_range && *obstacle_hit < best) {
                    best = *obstacle_hit;
                    hit = true;
                }
            }
        }
        if (include_perception_obstacles) {
            for (const Rect& obstacle : perception_obstacles_) {
                if (const auto obstacle_hit = ray_rect_distance(origin, dir, obstacle)) {
                    if (*obstacle_hit >= 0.0 && *obstacle_hit <= max_range && *obstacle_hit < best) {
                        best = *obstacle_hit;
                        hit = true;
                    }
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
    for (const Rect& obstacle : perception_obstacles_) {
        if (polygon_intersects_rect(polygon, expand(obstacle, padding))) {
            return true;
        }
    }
    return false;
}


}  // namespace thesis_sim
