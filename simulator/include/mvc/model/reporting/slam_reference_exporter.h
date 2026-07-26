#pragma once

#include <string>
#include <utility>
#include <vector>

#include "mvc/model/world/world.h"

namespace thesis_sim::mvc::model {

struct SlamReferenceArtifact {
    Rect bounds;
    std::vector<Rect> reference_obstacles;
    std::vector<Vec2> reference_road;
    std::vector<std::pair<Vec2, Vec2>> free_space_rays;
    std::vector<Vec2> occupied_points;
    std::vector<Vec2> estimated_trail;
    Vec2 start;
    Vec2 goal;
    Vec2 current;
    bool draw_reference_geometry = true;
    bool draw_mission_markers = true;
};

bool write_slam_reference_png(const SlamReferenceArtifact& artifact,
                              const std::string& path);

}  // namespace thesis_sim::mvc::model
