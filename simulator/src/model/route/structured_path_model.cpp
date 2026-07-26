#include "mvc/model/route/structured_path_model.h"

#include <algorithm>
#include <cmath>

namespace thesis_sim::mvc::model {
namespace {

double clamp_value(double value, double lo, double hi) {
    return std::max(lo, std::min(value, hi));
}

double wrap_path_s(double s, double length) {
    if (!(length > 1e-6)) {
        return 0.0;
    }
    double wrapped = std::fmod(s, length);
    if (wrapped < 0.0) {
        wrapped += length;
    }
    return wrapped;
}

struct PolylineSegment {
    Vec2 a;
    Vec2 b;
    double s_base = 0.0;
    double length = 0.0;
};

}  // namespace

ClosedPathProjection project_closed_path(const std::vector<Vec2>& points,
                                         const Vec2& position,
                                         double s_hint) {
    ClosedPathProjection projection{};
    if (points.size() < 2) {
        return projection;
    }

    std::vector<PolylineSegment> segments;
    segments.reserve(points.size());
    double route_length = 0.0;
    const auto append_segment = [&](const Vec2& a, const Vec2& b) {
        const double segment_length = distance(a, b);
        if (!(segment_length > 1e-12)) {
            return;
        }
        segments.push_back({a, b, route_length, segment_length});
        route_length += segment_length;
    };
    for (size_t i = 0; i + 1 < points.size(); ++i) {
        append_segment(points[i], points[i + 1]);
    }
    if (distance(points.front(), points.back()) > 0.02) {
        append_segment(points.back(), points.front());
    }
    if (segments.empty() || !(route_length > 1e-6)) {
        return projection;
    }

    double best_distance_sq = std::numeric_limits<double>::infinity();
    const double wrapped_hint =
        std::isfinite(s_hint) ? wrap_path_s(s_hint, route_length) : 0.0;
    const double forward_window = std::clamp(0.20 * route_length, 0.24, 0.52);
    const double backward_window = std::clamp(0.05 * route_length, 0.08, 0.14);
    const auto consider_segment = [&](const PolylineSegment& path_segment,
                                      bool restrict_to_hint) {
        const Vec2 segment{
            path_segment.b.x - path_segment.a.x,
            path_segment.b.y - path_segment.a.y,
        };
        const double segment_len_sq = path_segment.length * path_segment.length;
        const Vec2 from_a{
            position.x - path_segment.a.x,
            position.y - path_segment.a.y,
        };
        const double t = clamp_value(
            (from_a.x * segment.x + from_a.y * segment.y) / segment_len_sq,
            0.0,
            1.0);
        const double candidate_s = path_segment.s_base + t * path_segment.length;
        if (restrict_to_hint) {
            const double forward_delta =
                wrap_path_s(candidate_s - wrapped_hint, route_length);
            const double backward_delta =
                wrap_path_s(wrapped_hint - candidate_s, route_length);
            if (forward_delta > forward_window && backward_delta > backward_window) {
                return;
            }
            const bool crosses_start_early =
                wrapped_hint > 0.65 * route_length &&
                candidate_s < 0.35 * route_length &&
                distance(position, points.front()) > 0.22;
            if (crosses_start_early) {
                return;
            }
        }

        const Vec2 candidate{
            path_segment.a.x + segment.x * t,
            path_segment.a.y + segment.y * t,
        };
        const Vec2 error{position.x - candidate.x, position.y - candidate.y};
        const double distance_sq = error.x * error.x + error.y * error.y;
        if (distance_sq >= best_distance_sq) {
            return;
        }
        const Vec2 tangent{
            segment.x / path_segment.length,
            segment.y / path_segment.length,
        };
        const Vec2 normal{-tangent.y, tangent.x};
        projection.valid = true;
        projection.s = candidate_s;
        projection.lateral = error.x * normal.x + error.y * normal.y;
        best_distance_sq = distance_sq;
    };

    const bool restrict_to_hint = std::isfinite(s_hint);
    for (const PolylineSegment& segment : segments) {
        consider_segment(segment, restrict_to_hint);
    }
    if (!projection.valid && restrict_to_hint) {
        for (const PolylineSegment& segment : segments) {
            consider_segment(segment, false);
        }
    }

    projection.length = route_length;
    if (projection.valid) {
        projection.s = wrap_path_s(projection.s, route_length);
    }
    return projection;
}

Vec2 sample_closed_path(const std::vector<Vec2>& points,
                        double s,
                        double length) {
    if (points.empty()) {
        return {};
    }
    if (points.size() == 1 || !(length > 1e-6)) {
        return points.front();
    }

    const double target_s = wrap_path_s(s, length);
    double s_base = 0.0;
    const auto sample_segment = [&](const Vec2& a,
                                    const Vec2& b,
                                    Vec2* out) {
        const double segment_length = distance(a, b);
        if (!(segment_length > 1e-12)) {
            return false;
        }
        if (target_s <= s_base + segment_length) {
            const double alpha = clamp_value(
                (target_s - s_base) / segment_length,
                0.0,
                1.0);
            *out = {
                a.x + (b.x - a.x) * alpha,
                a.y + (b.y - a.y) * alpha,
            };
            return true;
        }
        s_base += segment_length;
        return false;
    };

    Vec2 sample = points.front();
    for (size_t i = 0; i + 1 < points.size(); ++i) {
        if (sample_segment(points[i], points[i + 1], &sample)) {
            return sample;
        }
    }
    if (distance(points.front(), points.back()) > 0.02 &&
        sample_segment(points.back(), points.front(), &sample)) {
        return sample;
    }
    return points.front();
}

std::vector<Vec2> sample_closed_path_span(const std::vector<Vec2>& points,
                                          double s_start,
                                          double span,
                                          int sample_count,
                                          double length) {
    std::vector<Vec2> samples;
    if (points.size() < 2 || sample_count < 2 ||
        !(span > 0.0) || !(length > 1e-6)) {
        return samples;
    }
    samples.reserve(static_cast<size_t>(sample_count));
    for (int i = 0; i < sample_count; ++i) {
        const double alpha =
            static_cast<double>(i) / static_cast<double>(sample_count - 1);
        samples.push_back(
            sample_closed_path(points, s_start + alpha * span, length));
    }
    return samples;
}

}  // namespace thesis_sim::mvc::model
