#include "mvc/model/route/structured_path_model.h"

#include <cmath>
#include <iostream>
#include <vector>

namespace {

bool near(double a, double b, double tolerance) {
    return std::abs(a - b) <= tolerance;
}

}  // namespace

int main() {
    using thesis_sim::Vec2;
    using thesis_sim::mvc::model::project_closed_path;
    using thesis_sim::mvc::model::sample_closed_path_span;

    const std::vector<Vec2> square{
        {0.0, 0.0},
        {1.0, 0.0},
        {1.0, 1.0},
        {0.0, 1.0},
        {0.0, 0.0},
    };
    const auto initial = project_closed_path(square, {0.0, 0.0}, 0.0);
    if (!initial.valid || !near(initial.s, 0.0, 1e-6) || !near(initial.length, 4.0, 1e-6)) {
        std::cerr << "initial structured projection is not anchored at route start\n";
        return 1;
    }

    // A robot on the final side must not jump to the first side merely because
    // the latter is geometrically close. That jump was the short-track bug.
    const auto guarded = project_closed_path(square, {0.45, 0.55}, 3.55);
    if (!guarded.valid || guarded.s < 3.0) {
        std::cerr << "structured projection crossed the start before returning to it\n";
        return 2;
    }

    const auto samples = sample_closed_path_span(square, 3.8, 0.4, 5, 4.0);
    if (samples.size() != 5 ||
        distance(samples.front(), Vec2{0.0, 0.2}) > 1e-6 ||
        distance(samples.back(), Vec2{0.2, 0.0}) > 1e-6) {
        std::cerr << "closed path sampling does not wrap continuously\n";
        return 3;
    }

    std::cout << "structured_path_model: ok\n";
    return 0;
}
