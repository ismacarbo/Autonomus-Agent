#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

#include "mvc/model/world/world.h"
#include "mvc/model/world/scenario_model.h"
#include "mvc/view/live_stream/live_view_stream.h"

namespace {

bool near(double lhs, double rhs, double tolerance = 1e-9) {
    return std::abs(lhs - rhs) <= tolerance;
}

int fail(const char* message) {
    std::cerr << "world_stream_smoke: " << message << '\n';
    return 1;
}

}  // namespace

int main() {
    const thesis_sim::WorldMap source = thesis_sim::WorldMap::mixed_hardware_aligned_demo();
    const std::vector<std::uint8_t> payload = thesis_sim::serialize_world_blob(source);
    if (payload.empty()) {
        return fail("serialized payload is empty");
    }

    thesis_sim::WorldMap decoded;
    if (!thesis_sim::deserialize_world_blob(payload, &decoded)) {
        return fail("deserialization failed");
    }

    const thesis_sim::Rect& bounds = decoded.bounds();
    if (!near(bounds.min_x, 0.0) || !near(bounds.min_y, 0.0) ||
        !near(bounds.max_x, 1.20) || !near(bounds.max_y, 1.20)) {
        return fail("arena bounds changed during roundtrip");
    }
    if (!near(decoded.start().x, 0.15) || !near(decoded.start().y, 0.60) ||
        !near(decoded.goal().x, 1.00) || !near(decoded.goal().y, 0.60)) {
        return fail("start or goal changed during roundtrip");
    }
    if (decoded.environment_mode() != thesis_sim::EnvironmentMode::MixedRoadGates ||
        decoded.road_centerline().size() != source.road_centerline().size() ||
        decoded.obstacles().size() != source.obstacles().size()) {
        return fail("mixed scenario content changed during roundtrip");
    }

    const thesis_sim::WorldMap normalized = thesis_sim::normalize_structured_world(
        thesis_sim::WorldMap::structured_demo(thesis_sim::StructuredMapPreset::ValidationRoad));
    const thesis_sim::Rect& normalized_bounds = normalized.bounds();
    if (!near(normalized_bounds.min_x, 0.0) || !near(normalized_bounds.min_y, 0.0) ||
        !near(normalized_bounds.max_x, 1.20) || !near(normalized_bounds.max_y, 1.20)) {
        return fail("structured normalization did not produce a 1.20 m square arena");
    }
    double min_x = std::numeric_limits<double>::infinity();
    double min_y = std::numeric_limits<double>::infinity();
    double max_x = -std::numeric_limits<double>::infinity();
    double max_y = -std::numeric_limits<double>::infinity();
    for (const thesis_sim::Vec2& point : normalized.road_centerline()) {
        min_x = std::min(min_x, point.x);
        min_y = std::min(min_y, point.y);
        max_x = std::max(max_x, point.x);
        max_y = std::max(max_y, point.y);
    }
    const double normalized_span = std::max(max_x - min_x, max_y - min_y);
    if (!near(normalized_span, 0.90, 1e-6)) {
        return fail("structured centerline span is not normalized to 0.90 m");
    }

    const thesis_sim::WorldMap unstructured = thesis_sim::WorldMap::unstructured_demo();
    const thesis_sim::WorldMap untouched_unstructured = thesis_sim::normalize_structured_world(unstructured);
    if (!near(untouched_unstructured.bounds().min_x, unstructured.bounds().min_x) ||
        !near(untouched_unstructured.bounds().min_y, unstructured.bounds().min_y) ||
        !near(untouched_unstructured.bounds().max_x, unstructured.bounds().max_x) ||
        !near(untouched_unstructured.bounds().max_y, unstructured.bounds().max_y)) {
        return fail("structured normalization changed the unstructured preset");
    }

    const thesis_sim::UnstructuredMapPreset hardware_presets[] = {
        thesis_sim::UnstructuredMapPreset::RobotValidation,
        thesis_sim::UnstructuredMapPreset::TightCorridor,
        thesis_sim::UnstructuredMapPreset::WideSlalom,
        thesis_sim::UnstructuredMapPreset::LowerBypass,
        thesis_sim::UnstructuredMapPreset::HardwareLab,
        thesis_sim::UnstructuredMapPreset::IdealValidation,
    };
    for (const thesis_sim::UnstructuredMapPreset preset : hardware_presets) {
        const thesis_sim::WorldMap compact =
            thesis_sim::mvc::model::fit_hardware_unstructured_world(
                thesis_sim::WorldMap::unstructured_demo(preset));
        const thesis_sim::Rect& compact_bounds = compact.bounds();
        if (!near(compact_bounds.min_x, 0.0) ||
            !near(compact_bounds.min_y, 0.0) ||
            !near(compact_bounds.max_x, 1.20) ||
            !near(compact_bounds.max_y, 1.20)) {
            return fail("hardware unstructured preset is not in the common 1.20 m arena");
        }
        constexpr double kRobotCircumscribedRadiusM = 0.146;
        if (compact.start().x < kRobotCircumscribedRadiusM ||
            compact.start().x > 1.20 - kRobotCircumscribedRadiusM ||
            compact.start().y < kRobotCircumscribedRadiusM ||
            compact.start().y > 1.20 - kRobotCircumscribedRadiusM ||
            compact.goal().x < kRobotCircumscribedRadiusM ||
            compact.goal().x > 1.20 - kRobotCircumscribedRadiusM ||
            compact.goal().y < kRobotCircumscribedRadiusM ||
            compact.goal().y > 1.20 - kRobotCircumscribedRadiusM) {
            return fail("hardware unstructured start or goal does not clear the physical robot radius");
        }
        if (compact.unstructured_preset() != preset) {
            return fail("hardware unstructured normalization changed the preset identity");
        }
        const thesis_sim::WorldMap compact_again =
            thesis_sim::mvc::model::fit_hardware_unstructured_world(compact);
        if (!near(compact_again.start().x, compact.start().x) ||
            !near(compact_again.start().y, compact.start().y) ||
            !near(compact_again.goal().x, compact.goal().x) ||
            !near(compact_again.goal().y, compact.goal().y)) {
            return fail("hardware unstructured normalization is not idempotent");
        }
    }

    std::cout << "world_stream_smoke: ok"
              << " payload_bytes=" << payload.size()
              << " road_points=" << decoded.road_centerline().size()
              << " obstacles=" << decoded.obstacles().size()
              << " structured_span=" << normalized_span << '\n';
    return 0;
}
