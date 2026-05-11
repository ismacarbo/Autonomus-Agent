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

std::vector<Vec2> close_polyline_loop(std::vector<Vec2> points, double threshold);
std::vector<Vec2> chaikin_closed_polyline(const std::vector<Vec2>& points, int passes);
std::vector<Vec2> resample_closed_polyline(const std::vector<Vec2>& points, double spacing);

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

double clamp_value(double value, double lo, double hi) {
    return std::max(lo, std::min(value, hi));
}

Vec2 clamp_to_bounds(const Rect& bounds, const Vec2& value, double margin = 0.0) {
    return {
        clamp_value(value.x, bounds.min_x + margin, bounds.max_x - margin),
        clamp_value(value.y, bounds.min_y + margin, bounds.max_y - margin),
    };
}

Vec2 interpolate(const Vec2& a, const Vec2& b, double alpha) {
    return {
        a.x + (b.x - a.x) * alpha,
        a.y + (b.y - a.y) * alpha,
    };
}

Vec2 perpendicular_unit(const Vec2& vector) {
    const double norm = std::hypot(vector.x, vector.y);
    if (norm < kEps) {
        return {0.0, 1.0};
    }
    return {-vector.y / norm, vector.x / norm};
}

void normalize_rect(Rect* rect) {
    if (rect == nullptr) {
        return;
    }
    if (rect->min_x > rect->max_x) {
        std::swap(rect->min_x, rect->max_x);
    }
    if (rect->min_y > rect->max_y) {
        std::swap(rect->min_y, rect->max_y);
    }
}

std::vector<Vec2> make_circle_road(const Vec2& center,
                                   double radius,
                                   double start_angle,
                                   double end_angle,
                                   int samples) {
    std::vector<Vec2> road;
    const int point_count = std::max(samples, 2);
    road.reserve(static_cast<size_t>(point_count));
    for (int i = 0; i < point_count; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(point_count - 1);
        const double angle = start_angle + (end_angle - start_angle) * t;
        road.push_back({
            center.x + radius * std::cos(angle),
            center.y + radius * std::sin(angle),
        });
    }
    return road;
}

std::vector<Vec2> make_circle_loop(const Vec2& center, double radius, int samples) {
    std::vector<Vec2> road;
    const int point_count = std::max(samples, 8);
    road.reserve(static_cast<size_t>(point_count));
    for (int i = 0; i < point_count; ++i) {
        const double angle = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(point_count);
        road.push_back({
            center.x + radius * std::cos(angle),
            center.y + radius * std::sin(angle),
        });
    }
    return road;
}

std::vector<Vec2> make_zigzag_road() {
    return {
        {4.0, 9.0},
        {8.0, 12.0},
        {12.0, 16.5},
        {16.0, 13.5},
        {20.0, 17.5},
        {24.0, 14.0},
        {28.0, 18.0},
        {32.0, 15.0},
        {36.0, 18.0},
        {35.0, 12.0},
        {31.0, 8.5},
        {27.0, 11.0},
        {23.0, 7.5},
        {19.0, 10.0},
        {15.0, 7.0},
        {11.0, 9.5},
        {7.0, 6.5},
    };
}

std::vector<Vec2> make_validation_loop() {
    constexpr int kSamples = 28;
    // Keep the same oval shape as the original 40 cm validation loop, but
    // shrink it uniformly to a 50%-scale indoor loop so hardware captures
    // can run in tighter rooms without manual repositioning.
    const Vec2 center{0.15, 0.15};
    const double radius_x = 0.0650;  // 0.13 * 0.50
    const double radius_y = 0.0550;  // 0.11 * 0.50
    std::vector<Vec2> loop;
    loop.reserve(kSamples);
    for (int i = 0; i < kSamples; ++i) {
        const double theta = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(kSamples);
        loop.push_back({
            center.x + radius_x * std::cos(theta),
            center.y + radius_y * std::sin(theta),
        });
    }
    loop = close_polyline_loop(std::move(loop), 0.015);
    return resample_closed_polyline(loop, 0.010);
}

std::vector<Vec2> make_indoor_circle_loop() {
    constexpr int kSamples = 36;
    const Vec2 center{0.375, 0.375};
    const double radius = 0.15;
    std::vector<Vec2> loop;
    loop.reserve(kSamples);
    for (int i = 0; i < kSamples; ++i) {
        const double theta = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(kSamples);
        loop.push_back({
            center.x + radius * std::cos(theta),
            center.y + radius * std::sin(theta),
        });
    }
    loop = close_polyline_loop(std::move(loop), 0.02);
    return resample_closed_polyline(loop, 0.015);
}

std::vector<Vec2> make_hardware_road_track() {
    constexpr int kSamples = 28;
    const Vec2 center{0.150, 0.150};
    const double radius_x = 0.095;
    const double radius_y = 0.075;
    std::vector<Vec2> track;
    track.reserve(kSamples);

    for (int i = 0; i < kSamples; ++i) {
        const double theta = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(kSamples);
        track.push_back({
            center.x + radius_x * std::cos(theta),
            center.y + radius_y * std::sin(theta),
        });
    }
    track = close_polyline_loop(std::move(track), 0.02);
    return resample_closed_polyline(track, 0.015);
}

std::vector<Vec2> make_hardware_figure_eight_track() {
    constexpr int kSamples = 96;
    constexpr double kStartPhase = 0.5 * kPi;
    const Vec2 center{0.150, 0.150};
    const double radius_x = 0.140;
    const double radius_y = 0.085;

    std::vector<Vec2> track;
    track.reserve(kSamples);
    for (int i = 0; i < kSamples; ++i) {
        const double phase =
            kStartPhase + 2.0 * kPi * static_cast<double>(i) / static_cast<double>(kSamples);
        // A 1:2 Lissajous curve yields a true figure-eight. The lobes are kept
        // large enough to use the indoor hardware workspace, reducing the
        // extreme entry curvature that previously made the robot either
        // understeer or pivot around the start point.
        track.push_back({
            center.x + radius_x * std::sin(phase),
            center.y + radius_y * std::sin(2.0 * phase),
        });
    }

    track = close_polyline_loop(std::move(track), 0.01);
    return resample_closed_polyline(track, 0.008);
}

bool points_form_closed_loop(const std::vector<Vec2>& points, double threshold) {
    if (points.size() < 3 || distance(points.front(), points.back()) > threshold) {
        return false;
    }
    double arc_length = 0.0;
    for (size_t i = 1; i < points.size(); ++i) {
        arc_length += distance(points[i - 1], points[i]);
    }
    return arc_length > 2.0 * threshold;
}

std::vector<Vec2> close_polyline_loop(std::vector<Vec2> points, double threshold) {
    if (points.empty()) {
        return points;
    }
    if (points.size() == 1) {
        points.push_back(points.front());
        return points;
    }
    if (points_form_closed_loop(points, threshold)) {
        points.back() = points.front();
        return points;
    }
    points.push_back(points.front());
    return points;
}

std::vector<Vec2> open_points_to_closed_seed(const Rect& bounds, const Vec2& start, const Vec2& goal) {
    const double width = std::max(bounds.max_x - bounds.min_x, 0.1);
    const double height = std::max(bounds.max_y - bounds.min_y, 0.1);
    const double min_span = std::max(std::min(width, height), 0.1);
    const double max_span = std::max(width, height);
    if (distance(start, goal) <= std::max(0.05, 0.08 * min_span)) {
        const Vec2 center = clamp_to_bounds(bounds, start);
        const double rx = std::clamp(0.22 * width, 0.08, 0.45 * width);
        const double ry = std::clamp(0.22 * height, 0.08, 0.45 * height);
        return {
            {center.x + rx, center.y},
            {center.x, center.y + ry},
            {center.x - rx, center.y},
            {center.x, center.y - ry},
        };
    }

    const Vec2 midpoint{
        0.5 * (start.x + goal.x),
        0.5 * (start.y + goal.y),
    };
    const Vec2 chord{
        goal.x - start.x,
        goal.y - start.y,
    };
    const double chord_length = std::max(std::hypot(chord.x, chord.y), std::max(0.25 * max_span, 0.08));
    const Vec2 normal = perpendicular_unit(chord);
    const double offset = std::clamp(0.35 * chord_length,
                                     std::max(0.08, 0.10 * min_span),
                                     std::max(0.16, 0.40 * min_span));
    return {
        start,
        {midpoint.x + normal.x * offset, midpoint.y + normal.y * offset},
        goal,
        {midpoint.x - normal.x * offset, midpoint.y - normal.y * offset},
    };
}

std::vector<Vec2> deduplicate_polyline(const std::vector<Vec2>& points, double min_spacing) {
    std::vector<Vec2> filtered;
    filtered.reserve(points.size());
    for (const Vec2& point : points) {
        if (filtered.empty() || distance(filtered.back(), point) >= min_spacing) {
            filtered.push_back(point);
        }
    }
    return filtered;
}

std::vector<Vec2> chaikin_open_polyline(const std::vector<Vec2>& points, int passes) {
    if (points.size() < 3 || passes <= 0) {
        return points;
    }

    std::vector<Vec2> current = points;
    for (int pass = 0; pass < passes; ++pass) {
        if (current.size() < 3) {
            break;
        }

        std::vector<Vec2> next;
        next.reserve(current.size() * 2);
        next.push_back(current.front());
        for (size_t i = 0; i + 1 < current.size(); ++i) {
            const Vec2& a = current[i];
            const Vec2& b = current[i + 1];
            next.push_back(interpolate(a, b, 0.25));
            next.push_back(interpolate(a, b, 0.75));
        }
        next.push_back(current.back());
        current = std::move(next);
    }

    return current;
}

std::vector<Vec2> chaikin_closed_polyline(const std::vector<Vec2>& points, int passes) {
    if (points.size() < 3 || passes <= 0) {
        return points;
    }

    std::vector<Vec2> current = points;
    if (points_form_closed_loop(current, 0.45) && current.size() > 2) {
        current.pop_back();
    }

    for (int pass = 0; pass < passes; ++pass) {
        if (current.size() < 3) {
            break;
        }

        std::vector<Vec2> next;
        next.reserve(current.size() * 2);
        for (size_t i = 0; i < current.size(); ++i) {
            const Vec2& a = current[i];
            const Vec2& b = current[(i + 1) % current.size()];
            next.push_back(interpolate(a, b, 0.25));
            next.push_back(interpolate(a, b, 0.75));
        }
        current = std::move(next);
    }

    return current;
}

std::vector<Vec2> resample_polyline(const std::vector<Vec2>& points, double spacing) {
    if (points.size() < 2) {
        return points;
    }

    spacing = std::max(spacing, 0.25);
    std::vector<double> cumulative(points.size(), 0.0);
    for (size_t i = 1; i < points.size(); ++i) {
        cumulative[i] = cumulative[i - 1] + distance(points[i - 1], points[i]);
    }

    const double total_length = cumulative.back();
    if (total_length < spacing) {
        return {points.front(), points.back()};
    }

    const int sample_count = std::clamp(
        static_cast<int>(std::ceil(total_length / spacing)) + 1,
        2,
        96);
    std::vector<Vec2> resampled;
    resampled.reserve(static_cast<size_t>(sample_count));

    size_t segment_index = 0;
    for (int i = 0; i < sample_count; ++i) {
        const double target_s =
            (i + 1 == sample_count) ? total_length : std::min(total_length, spacing * static_cast<double>(i));
        while (segment_index + 1 < cumulative.size() && cumulative[segment_index + 1] < target_s) {
            ++segment_index;
        }

        if (segment_index + 1 >= cumulative.size()) {
            resampled.push_back(points.back());
            continue;
        }

        const double segment_start_s = cumulative[segment_index];
        const double segment_end_s = cumulative[segment_index + 1];
        const double segment_length = std::max(segment_end_s - segment_start_s, kEps);
        const double alpha = clamp_value((target_s - segment_start_s) / segment_length, 0.0, 1.0);
        resampled.push_back(interpolate(points[segment_index], points[segment_index + 1], alpha));
    }

    return resampled;
}

std::vector<Vec2> resample_closed_polyline(const std::vector<Vec2>& points, double spacing) {
    std::vector<Vec2> base = points;
    if (points_form_closed_loop(base, 0.45) && base.size() > 2) {
        base.pop_back();
    }
    if (base.size() < 3) {
        return base;
    }

    spacing = std::max(spacing, 0.01);
    std::vector<double> cumulative(base.size() + 1, 0.0);
    for (size_t i = 0; i < base.size(); ++i) {
        cumulative[i + 1] = cumulative[i] + distance(base[i], base[(i + 1) % base.size()]);
    }

    const double total_length = cumulative.back();
    if (total_length < spacing) {
        return base;
    }

    const int sample_count = std::clamp(
        static_cast<int>(std::ceil(total_length / spacing)),
        6,
        128);
    std::vector<Vec2> resampled;
    resampled.reserve(static_cast<size_t>(sample_count));

    size_t segment_index = 0;
    for (int i = 0; i < sample_count; ++i) {
        const double target_s = total_length * static_cast<double>(i) / static_cast<double>(sample_count);
        while (segment_index + 1 < cumulative.size() && cumulative[segment_index + 1] < target_s) {
            ++segment_index;
        }

        const size_t start_index = segment_index % base.size();
        const size_t end_index = (start_index + 1) % base.size();
        const double segment_start_s = cumulative[segment_index];
        const double segment_end_s = cumulative[segment_index + 1];
        const double segment_length = std::max(segment_end_s - segment_start_s, kEps);
        const double alpha = clamp_value((target_s - segment_start_s) / segment_length, 0.0, 1.0);
        resampled.push_back(interpolate(base[start_index], base[end_index], alpha));
    }

    return resampled;
}

size_t find_nearest_point_index(const std::vector<Vec2>& points,
                                const Vec2& query,
                                std::optional<size_t> skip_index = std::nullopt) {
    size_t best_index = 0;
    double best_distance_sq = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < points.size(); ++i) {
        if (skip_index.has_value() && i == *skip_index) {
            continue;
        }
        const double dx = points[i].x - query.x;
        const double dy = points[i].y - query.y;
        const double distance_sq = dx * dx + dy * dy;
        if (distance_sq < best_distance_sq) {
            best_distance_sq = distance_sq;
            best_index = i;
        }
    }
    return best_index;
}

std::vector<Vec2> rotate_closed_polyline(const std::vector<Vec2>& points, size_t start_index) {
    std::vector<Vec2> base = points;
    if (points_form_closed_loop(base, 0.45) && base.size() > 2) {
        base.pop_back();
    }
    if (base.empty()) {
        return base;
    }
    start_index %= base.size();
    std::rotate(base.begin(), base.begin() + static_cast<std::ptrdiff_t>(start_index), base.end());
    return close_polyline_loop(std::move(base), 0.45);
}

double heading_from_first_segment(const std::vector<Vec2>& road, double fallback_heading) {
    for (size_t i = 1; i < road.size(); ++i) {
        if (distance(road[i - 1], road[i]) >= 0.25) {
            return angle_to(road[i - 1], road[i]);
        }
    }
    return fallback_heading;
}

std::vector<Vec2> sanitize_structured_centerline(const Rect& bounds,
                                                 const Vec2& start,
                                                 const Vec2& goal,
                                                 const std::vector<Vec2>& raw_points) {
    const double width = std::max(bounds.max_x - bounds.min_x, 0.1);
    const double height = std::max(bounds.max_y - bounds.min_y, 0.1);
    const double min_span = std::max(std::min(width, height), 0.1);
    const double max_span = std::max(width, height);
    const double loop_threshold = std::clamp(0.12 * min_span, 0.03, 0.45);
    const double first_dedup_spacing = std::clamp(0.08 * min_span, 0.02, 0.35);
    const double resample_spacing = std::clamp(0.10 * max_span, 0.04, 1.0);
    const double final_dedup_spacing = std::clamp(0.10 * min_span, 0.02, 0.45);

    std::vector<Vec2> sanitized = raw_points;
    if (points_form_closed_loop(sanitized, loop_threshold) && sanitized.size() > 2) {
        sanitized.pop_back();
    }
    if (sanitized.empty()) {
        sanitized = open_points_to_closed_seed(bounds, start, goal);
    }

    for (Vec2& point : sanitized) {
        point = clamp_to_bounds(bounds, point);
    }

    const bool already_closed_loop = raw_points.size() >= 6 &&
                                     points_form_closed_loop(raw_points, loop_threshold);
    if (already_closed_loop) {
        sanitized = deduplicate_polyline(sanitized, std::clamp(0.02 * min_span, 0.005, 0.10));
        if (sanitized.size() >= 6) {
            return close_polyline_loop(std::move(sanitized), loop_threshold);
        }
    }

    sanitized = deduplicate_polyline(sanitized, first_dedup_spacing);
    if (sanitized.size() < 3) {
        sanitized = open_points_to_closed_seed(bounds, start, goal);
        for (Vec2& point : sanitized) {
            point = clamp_to_bounds(bounds, point);
        }
    }

    sanitized = chaikin_closed_polyline(sanitized, sanitized.size() > 4 ? 2 : 1);
    if (sanitized.size() < 3) {
        sanitized = open_points_to_closed_seed(bounds, start, goal);
    }

    sanitized = resample_closed_polyline(sanitized, resample_spacing);
    sanitized = deduplicate_polyline(sanitized, final_dedup_spacing);
    if (sanitized.size() < 3) {
        sanitized = open_points_to_closed_seed(bounds, start, goal);
    }

    return close_polyline_loop(std::move(sanitized), loop_threshold);
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
                {"goal_approach", {33.0, 18.4}, {33.0, 18.4}, {0.20, 0.18}, 0.05, 2.90, 0.0, false},
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
    world.gates_ = world.gate_templates_;
    recompute_gate_headings(&world.gates_, world.goal_);
    world.set_gate_behavior(gate_behavior, gate_seed);

    return world;
}

WorldMap WorldMap::structured_demo(StructuredMapPreset preset) {
    WorldMap world;
    world.environment_mode_ = EnvironmentMode::StructuredRoad;
    world.structured_preset_ = preset;
    world.bounds_ = {0.0, 0.0, 40.0, 24.0};
    world.obstacles_.clear();

    switch (preset) {
        case StructuredMapPreset::ValidationRoad:
        case StructuredMapPreset::Custom:
            world.bounds_ = {0.0, 0.0, 0.30, 0.30};
            world.road_centerline_ = close_polyline_loop(make_validation_loop(), 0.45);
            world.start_ = world.road_centerline_.front();
            world.goal_ = world.start_;
            world.start_heading_ = angle_to(world.road_centerline_.front(), world.road_centerline_[1]);
            break;
        case StructuredMapPreset::CircleLoop:
            world.bounds_ = {0.0, 0.0, 0.75, 0.75};
            world.road_centerline_ = make_indoor_circle_loop();
            world.start_ = world.road_centerline_.front();
            world.goal_ = world.start_;
            world.start_heading_ = angle_to(world.road_centerline_.front(), world.road_centerline_[1]);
            break;
        case StructuredMapPreset::ZigZag:
            world.road_centerline_ = close_polyline_loop(make_zigzag_road(), 0.45);
            world.start_ = world.road_centerline_.front();
            world.goal_ = world.start_;
            world.start_heading_ = angle_to(world.road_centerline_.front(), world.road_centerline_[1]);
            break;
        case StructuredMapPreset::HardwareTrack:
            // Minimal indoor road validation: a smooth oval loop inside a
            // 30 cm workspace, smaller than Validation Road but still feasible
            // for the real robot's indoor follower.
            world.bounds_ = {0.0, 0.0, 0.30, 0.30};
            world.obstacles_.clear();
            world.road_centerline_ = make_hardware_road_track();
            world.start_ = world.road_centerline_.front();
            world.goal_ = world.start_;
            world.start_heading_ = angle_to(world.road_centerline_.front(), world.road_centerline_[1]);
            break;
        case StructuredMapPreset::FigureEight:
            // Stress test for structured tracking: two small lobes in a figure
            // eight, still bounded by the same 30 cm indoor workspace.
            world.bounds_ = {0.0, 0.0, 0.30, 0.30};
            world.obstacles_.clear();
            world.road_centerline_ = make_hardware_figure_eight_track();
            world.start_ = world.road_centerline_.front();
            world.goal_ = world.start_;
            world.start_heading_ = angle_to(world.road_centerline_.front(), world.road_centerline_[1]);
            break;
        default:
            break;
    }

    world.gates_.clear();
    world.gate_templates_.clear();
    world.gate_behavior_ = GateBehaviorMode::Static;
    world.gate_seed_ = 0;
    return world;
}

WorldMap WorldMap::mixed_demo() {
    WorldMap world;
    world.environment_mode_ = EnvironmentMode::MixedRoadGates;
    world.unstructured_preset_ = UnstructuredMapPreset::RobotValidation;
    world.structured_preset_ = StructuredMapPreset::ValidationRoad;
    world.bounds_ = {-1.00, -0.80, 9.00, 5.60};

    std::vector<Vec2> mixed_track_seed = {
        {0.55, 2.70},
        {1.20, 4.35},
        {2.55, 4.85},
        {3.85, 4.55},
        {4.75, 3.30},
        {5.85, 2.85},
        {7.25, 3.55},
        {8.20, 2.80},
        {7.70, 1.55},
        {6.35, 1.05},
        {4.75, 0.88},
        {3.00, 0.78},
        {1.42, 1.00},
        {0.48, 1.82},
    };
    mixed_track_seed = close_polyline_loop(std::move(mixed_track_seed), 0.45);
    mixed_track_seed = chaikin_closed_polyline(mixed_track_seed, 2);
    world.road_centerline_ = resample_closed_polyline(mixed_track_seed, 0.24);
    world.start_ = world.road_centerline_.front();
    world.goal_ = world.start_;
    world.start_heading_ = angle_to(world.road_centerline_.front(), world.road_centerline_[1]);

    world.obstacles_ = {
        // Mixed validation: the first obstacles shape the start gate, while
        // the center block cuts the road and leaves a safe lateral bypass.
        {-0.78, 1.90, -0.10, 3.55},
        {0.00, 1.90, 0.18, 3.55},
        {2.20, 2.05, 3.05, 2.78},
        {4.72, 2.72, 5.55, 3.52},
        {7.20, 4.78, 7.75, 5.25},
    };

    world.gate_templates_ = {
        {"left_gate", {0.56, 2.72}, {0.56, 2.72}, {0.0, 0.0}, 0.0, 0.0, 0.0, false},
        {"inner_shortcut", {3.42, 2.95}, {3.42, 2.95}, {0.0, 0.0}, 0.0, 0.0, 0.0, false},
        {"block_bypass", {5.30, 4.08}, {5.30, 4.08}, {0.0, 0.0}, 0.0, 0.0, 0.0, false},
        {"road_rejoin", {6.18, 3.36}, {6.18, 3.36}, {0.0, 0.0}, 0.0, 0.0, 0.0, false},
        {"outer_decoy", {7.28, 1.58}, {7.28, 1.58}, {0.0, 0.0}, 0.0, 0.0, 0.0, false},
        {"late_rejoin", {7.45, 3.15}, {7.45, 3.15}, {0.0, 0.0}, 0.0, 0.0, 0.0, false},
        {"goal", world.goal_, world.goal_, {0.0, 0.0}, 0.0, 0.0, 0.0, true},
    };
    world.gates_ = world.gate_templates_;
    recompute_gate_headings(&world.gates_, world.goal_);
    world.gate_templates_ = world.gates_;
    world.gate_behavior_ = GateBehaviorMode::Static;
    world.gate_seed_ = 0;
    return world;
}

WorldMap WorldMap::mixed_hardware_demo() {
    WorldMap world;
    world.environment_mode_ = EnvironmentMode::MixedRoadGates;
    world.unstructured_preset_ = UnstructuredMapPreset::HardwareLab;
    world.structured_preset_ = StructuredMapPreset::HardwareTrack;
    world.bounds_ = {0.0, 0.0, 1.65, 1.35};

    // Compact real-lab mixed validation: a scaled version of the large mixed
    // road/gate scene, kept well inside the hardware workspace. The lower
    // branch contains a small road block and a nearby bypass gate; the upper
    // branch closes the road with a gentle return curve.
    std::vector<Vec2> mixed_track_seed = {
        {0.33, 0.66},
        {0.50, 0.60},
        {0.72, 0.62},
        {0.90, 0.74},
        {0.84, 0.88},
        {0.63, 0.96},
        {0.41, 0.84},
    };
    mixed_track_seed = close_polyline_loop(std::move(mixed_track_seed), 0.45);
    world.road_centerline_ = resample_closed_polyline(mixed_track_seed, 0.045);
    world.start_ = world.road_centerline_.front();
    world.goal_ = world.start_;
    world.start_heading_ = angle_to(world.road_centerline_.front(), world.road_centerline_[1]);

    world.obstacles_ = {
        {0.76, 0.57, 0.94, 0.78},
    };

    world.gate_templates_ = {
        {"block_bypass", {0.78, 0.40}, {0.78, 0.40}, {0.0, 0.0}, 0.0, 0.0, 0.0, false},
        {"road_rejoin", {0.90, 0.76}, {0.90, 0.76}, {0.0, 0.0}, 0.0, 0.0, 0.0, false},
        {"goal", world.goal_, world.goal_, {0.0, 0.0}, 0.0, 0.0, 0.0, true},
    };
    world.gates_ = world.gate_templates_;
    recompute_gate_headings(&world.gates_, world.goal_);
    world.gate_templates_ = world.gates_;
    world.gate_behavior_ = GateBehaviorMode::Static;
    world.gate_seed_ = 0;
    return world;
}

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
    if (environment_mode_ == EnvironmentMode::StructuredRoad) {
        return;
    }
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

void WorldMap::finalize_editor_changes() {
    start_ = clamp_to_bounds(bounds_, start_);
    goal_ = clamp_to_bounds(bounds_, goal_);

    for (Rect& obstacle : obstacles_) {
        normalize_rect(&obstacle);
        obstacle.min_x = clamp_value(obstacle.min_x, bounds_.min_x, bounds_.max_x);
        obstacle.max_x = clamp_value(obstacle.max_x, bounds_.min_x, bounds_.max_x);
        obstacle.min_y = clamp_value(obstacle.min_y, bounds_.min_y, bounds_.max_y);
        obstacle.max_y = clamp_value(obstacle.max_y, bounds_.min_y, bounds_.max_y);
        normalize_rect(&obstacle);
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
    recompute_gate_headings(&gate_templates_, goal_);
    gates_ = gate_templates_;

    if (gate_behavior_ == GateBehaviorMode::Static) {
        for (GateSpec& gate : gates_) {
            gate.position = gate.anchor_position;
        }
        recompute_gate_headings(&gates_, goal_);
    } else {
        reset_gate_layout(gate_seed_);
    }
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
            if (*bounds_hit >= 0.0 && *bounds_hit <= max_range) {
                best = std::min(best, *bounds_hit);
                hit = true;
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
