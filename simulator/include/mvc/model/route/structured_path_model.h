#pragma once

#include "mvc/model/world/world.h"

#include <limits>
#include <vector>

namespace thesis_sim::mvc::model {

struct ClosedPathProjection {
    bool valid = false;
    double s = 0.0;
    double lateral = 0.0;
    double length = 0.0;
};

ClosedPathProjection project_closed_path(
    const std::vector<Vec2>& points,
    const Vec2& position,
    double s_hint = std::numeric_limits<double>::quiet_NaN());

Vec2 sample_closed_path(const std::vector<Vec2>& points,
                        double s,
                        double length);

std::vector<Vec2> sample_closed_path_span(const std::vector<Vec2>& points,
                                          double s_start,
                                          double span,
                                          int sample_count,
                                          double length);

}  // namespace thesis_sim::mvc::model
