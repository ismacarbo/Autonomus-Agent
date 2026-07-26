#include "mvc/model/world/world.h"

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
Rect centered_rect(const Vec2& center, double width, double height);

double dot(const Vec2& a, const Vec2& b) {
    return a.x * b.x + a.y * b.y;
}

Vec2 subtract(const Vec2& a, const Vec2& b) {
    return {a.x - b.x, a.y - b.y};
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
    constexpr int kSamples = 32;
    // Keep the source loop circular: the GUI scales it to the 1.20 m arena,
    // where its curvature remains feasible for the 25 cm car-like platform.
    const Vec2 center{0.20, 0.20};
    const double radius_x = 0.125;
    const double radius_y = 0.125;
    std::vector<Vec2> loop;
    loop.reserve(kSamples);
    for (int i = 0; i < kSamples; ++i) {
        const double theta = 2.0 * kPi * static_cast<double>(i) / static_cast<double>(kSamples);
        loop.push_back({
            center.x + radius_x * std::cos(theta),
            center.y + radius_y * std::sin(theta),
        });
    }
    loop = close_polyline_loop(std::move(loop), 0.02);
    return resample_closed_polyline(loop, 0.012);
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
    const Vec2 center{0.200, 0.200};
    const double radius_x = 0.115;
    const double radius_y = 0.095;
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

std::vector<Vec2> make_tank_practice_circuit() {
    std::vector<Vec2> track{
        {0.050, 0.145},
        {0.125, 0.145},
        {0.205, 0.148},
        {0.285, 0.164},
        {0.346, 0.216},
        {0.362, 0.282},
        {0.326, 0.340},
        {0.245, 0.360},
        {0.160, 0.330},
        {0.090, 0.270},
        {0.052, 0.205},
    };
    track = close_polyline_loop(std::move(track), 0.02);
    return resample_closed_polyline(track, 0.012);
}

std::vector<Vec2> make_hardware_figure_eight_track() {
    constexpr int kSamples = 96;
    constexpr double kStartPhase = 0.5 * kPi;
    const Vec2 center{0.200, 0.200};
    const double radius_x = 0.160;
    const double radius_y = 0.100;

    std::vector<Vec2> track;
    track.reserve(kSamples);
    for (int i = 0; i < kSamples; ++i) {
        const double phase =
            kStartPhase + 2.0 * kPi * static_cast<double>(i) /
                              static_cast<double>(kSamples);
        track.push_back({
            center.x + radius_x * std::sin(phase),
            center.y + radius_y * std::sin(2.0 * phase),
        });
    }

    track = close_polyline_loop(std::move(track), 0.02);
    return resample_closed_polyline(track, 0.012);
}

Vec2 map_unit_point(const Rect& content, const Vec2& point) {
    return {
        content.min_x + point.x * (content.max_x - content.min_x),
        content.min_y + point.y * (content.max_y - content.min_y),
    };
}

Rect map_unit_rect(const Rect& content, const Rect& rect) {
    const Vec2 min_point = map_unit_point(content, {rect.min_x, rect.min_y});
    const Vec2 max_point = map_unit_point(content, {rect.max_x, rect.max_y});
    return {
        std::min(min_point.x, max_point.x),
        std::min(min_point.y, max_point.y),
        std::max(min_point.x, max_point.x),
        std::max(min_point.y, max_point.y),
    };
}

std::vector<Vec2> make_closed_obstacle_mixed_loop(const Rect& content, double spacing) {
    std::vector<Vec2> track{
        map_unit_point(content, {0.10, 0.36}),
        map_unit_point(content, {0.23, 0.29}),
        map_unit_point(content, {0.43, 0.24}),
        map_unit_point(content, {0.62, 0.27}),
        map_unit_point(content, {0.79, 0.35}),
        map_unit_point(content, {0.92, 0.46}),
        map_unit_point(content, {0.96, 0.58}),
        map_unit_point(content, {0.88, 0.71}),
        map_unit_point(content, {0.72, 0.81}),
        map_unit_point(content, {0.56, 0.76}),
        map_unit_point(content, {0.40, 0.82}),
        map_unit_point(content, {0.23, 0.72}),
        map_unit_point(content, {0.11, 0.58}),
        map_unit_point(content, {0.06, 0.47}),
    };
    track = close_polyline_loop(std::move(track), 0.02);
    track = chaikin_closed_polyline(track, 2);
    return resample_closed_polyline(track, spacing);
}

std::vector<Rect> make_closed_obstacle_road_blocks(const Rect& content) {
    return {
        // This block sits on the nominal structured path. The mixed planner
        // must derive a side gate from the obstacle and the road boundary.
        map_unit_rect(content, {0.52, 0.22, 0.58, 0.29}),
        map_unit_rect(content, {0.56, 0.75, 0.64, 0.82}),
    };
}

constexpr double kArucoReferenceHeightM = 1.40;

Vec2 aruco_to_sim_point(const Vec2& point) {
    // ArUco ground-truth maps use image coordinates: x to the right, y down.
    // The simulator uses x to the right, y up, so the vertical axis is flipped.
    return {point.x, kArucoReferenceHeightM - point.y};
}

Rect centered_aruco_rect_as_sim(const Vec2& aruco_center, double width, double height) {
    return centered_rect(aruco_to_sim_point(aruco_center), width, height);
}

std::vector<Vec2> make_aruco_real_mixed_loop(double spacing) {
    const std::vector<Vec2> aruco_track{
        {1.000, 1.350},
        {1.180, 0.180},
        {0.880, 0.055},
        {0.520, -0.015},
        {0.220, 0.180},
        {0.200, 0.760},
        {0.240, 1.260},
        {0.640, 1.245},
        {0.720, 1.390},
    };
    std::vector<Vec2> track;
    track.reserve(aruco_track.size());
    for (const Vec2& point : aruco_track) {
        track.push_back(aruco_to_sim_point(point));
    }
    track = close_polyline_loop(std::move(track), 0.02);
    return resample_closed_polyline(track, spacing);
}

std::vector<Rect> make_aruco_real_obstacle_blocks() {
    return {
        // Obstacle positions are expressed in the ArUco reference frame used
        // for the 00058 ground-truth run: C is the top-left reference center,
        // x points right, and y points down. They are converted to simulator
        // coordinates here so A starts in the lower-right of the viewport.
        centered_aruco_rect_as_sim({0.050, 0.766}, 0.120, 0.120),
        centered_aruco_rect_as_sim({0.640, 1.194}, 0.170, 0.170),
        centered_aruco_rect_as_sim({0.663, 0.364}, 0.235, 0.220),
        centered_aruco_rect_as_sim({0.428, -0.025}, 0.150, 0.150),
    };
}

Rect centered_rect(const Vec2& center, double width, double height) {
    return {
        center.x - 0.5 * width,
        center.y - 0.5 * height,
        center.x + 0.5 * width,
        center.y + 0.5 * height,
    };
}

std::vector<Rect> make_offset_wall_blocks(const std::vector<Vec2>& loop,
                                          double offset,
                                          double block_size) {
    std::vector<Rect> blocks;
    if (loop.size() < 3) {
        return blocks;
    }
    const size_t last_index = loop.size() - 1;
    blocks.reserve(last_index);
    for (size_t i = 0; i < last_index; ++i) {
        const size_t prev = i == 0 ? last_index - 1 : i - 1;
        const size_t next = (i + 1) % last_index;
        const Vec2 tangent_raw{
            loop[next].x - loop[prev].x,
            loop[next].y - loop[prev].y,
        };
        const double tangent_norm = std::hypot(tangent_raw.x, tangent_raw.y);
        if (!(tangent_norm > 1e-9)) {
            continue;
        }
        const Vec2 tangent{
            tangent_raw.x / tangent_norm,
            tangent_raw.y / tangent_norm,
        };
        const Vec2 normal{-tangent.y, tangent.x};
        const Vec2 point{
            loop[i].x + offset * normal.x,
            loop[i].y + offset * normal.y,
        };
        blocks.push_back(centered_rect(point, block_size, block_size));
    }
    return blocks;
}

Vec2 sampled_loop_point(const std::vector<Vec2>& loop, double fraction) {
    if (loop.empty()) {
        return {};
    }
    const double clamped = std::clamp(fraction, 0.0, 1.0);
    const size_t max_index = loop.size() - 1;
    const size_t index = std::min(
        max_index,
        static_cast<size_t>(std::round(clamped * static_cast<double>(max_index))));
    return loop[index];
}

Vec2 sampled_polyline_point(const std::vector<Vec2>& points, double fraction) {
    if (points.empty()) {
        return {};
    }
    if (points.size() == 1) {
        return points.front();
    }

    const double clamped = std::clamp(fraction, 0.0, 1.0);
    std::vector<double> cumulative(points.size(), 0.0);
    for (size_t i = 1; i < points.size(); ++i) {
        cumulative[i] = cumulative[i - 1] + distance(points[i - 1], points[i]);
    }
    const double total = cumulative.back();
    if (!(total > 1e-9)) {
        return points.front();
    }

    const double target_s = clamped * total;
    auto upper = std::upper_bound(cumulative.begin(), cumulative.end(), target_s);
    if (upper == cumulative.begin()) {
        return points.front();
    }
    if (upper == cumulative.end()) {
        return points.back();
    }
    const size_t index = static_cast<size_t>(std::distance(cumulative.begin(), upper));
    const double s0 = cumulative[index - 1];
    const double s1 = cumulative[index];
    const double alpha = (s1 > s0) ? std::clamp((target_s - s0) / (s1 - s0), 0.0, 1.0) : 0.0;
    return interpolate(points[index - 1], points[index], alpha);
}

std::vector<Vec2> make_lab_square_annulus_reference(const Vec2& center,
                                                    double radius,
                                                    double start_angle,
                                                    double lap_angle,
                                                    double spacing) {
    const double arc_length = std::abs(lap_angle) * radius;
    const int samples = std::clamp(
        static_cast<int>(std::ceil(arc_length / std::max(spacing, 0.012))) + 1,
        24,
        220);
    std::vector<Vec2> path;
    path.reserve(static_cast<size_t>(samples));
    for (int i = 0; i < samples; ++i) {
        const double alpha = static_cast<double>(i) / static_cast<double>(samples - 1);
        const double angle = start_angle + lap_angle * alpha;
        path.push_back({
            center.x + radius * std::cos(angle),
            center.y + radius * std::sin(angle),
        });
    }
    return path;
}

std::vector<Rect> make_disc_obstacle_blocks(const Vec2& center, double radius, double strip_height) {
    std::vector<Rect> blocks;
    const int strip_count = std::max(5, static_cast<int>(std::ceil((2.0 * radius) / std::max(strip_height, 0.02))));
    blocks.reserve(static_cast<size_t>(strip_count));
    const double y_min = center.y - radius;
    const double actual_strip = (2.0 * radius) / static_cast<double>(strip_count);
    for (int i = 0; i < strip_count; ++i) {
        const double y0 = y_min + actual_strip * static_cast<double>(i);
        const double y1 = y_min + actual_strip * static_cast<double>(i + 1);
        const double y_mid = 0.5 * (y0 + y1);
        const double half_width = std::sqrt(std::max(radius * radius - (y_mid - center.y) * (y_mid - center.y), 0.0));
        blocks.push_back({
            center.x - half_width,
            y0,
            center.x + half_width,
            y1,
        });
    }
    return blocks;
}

std::vector<Rect> make_boundary_wall_blocks(const Rect& bounds, double wall_thickness) {
    const double t = std::max(wall_thickness, 0.004);
    return {
        {bounds.min_x, bounds.min_y, bounds.min_x + t, bounds.max_y},
        {bounds.max_x - t, bounds.min_y, bounds.max_x, bounds.max_y},
        {bounds.min_x, bounds.min_y, bounds.max_x, bounds.min_y + t},
        {bounds.min_x, bounds.max_y - t, bounds.max_x, bounds.max_y},
    };
}

double polyline_length(const std::vector<Vec2>& points) {
    double length = 0.0;
    for (size_t i = 1; i < points.size(); ++i) {
        length += distance(points[i - 1], points[i]);
    }
    return length;
}

double nearest_polyline_s(const std::vector<Vec2>& points, const Vec2& point) {
    if (points.size() < 2) {
        return 0.0;
    }
    double best_s = 0.0;
    double best_distance_sq = std::numeric_limits<double>::infinity();
    double s_base = 0.0;
    for (size_t i = 0; i + 1 < points.size(); ++i) {
        const Vec2 a = points[i];
        const Vec2 b = points[i + 1];
        const Vec2 segment{b.x - a.x, b.y - a.y};
        const double segment_len_sq = dot(segment, segment);
        if (!(segment_len_sq > 1e-12)) {
            continue;
        }
        const double segment_len = std::sqrt(segment_len_sq);
        const Vec2 from_a{point.x - a.x, point.y - a.y};
        const double t = clamp_value(dot(from_a, segment) / segment_len_sq, 0.0, 1.0);
        const Vec2 candidate{a.x + segment.x * t, a.y + segment.y * t};
        const Vec2 delta{point.x - candidate.x, point.y - candidate.y};
        const double d_sq = dot(delta, delta);
        if (d_sq < best_distance_sq) {
            best_distance_sq = d_sq;
            best_s = s_base + t * segment_len;
        }
        s_base += segment_len;
    }
    return best_s;
}

double distance_point_to_rect(const Vec2& point, const Rect& rect) {
    const double dx = point.x < rect.min_x ? rect.min_x - point.x
                     : (point.x > rect.max_x ? point.x - rect.max_x : 0.0);
    const double dy = point.y < rect.min_y ? rect.min_y - point.y
                     : (point.y > rect.max_y ? point.y - rect.max_y : 0.0);
    return std::hypot(dx, dy);
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

void recompute_gate_headings(std::vector<GateSpec>* gates,
                             const Vec2& start,
                             const Vec2& goal) {
    (void)start;
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
