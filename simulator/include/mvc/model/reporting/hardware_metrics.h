#pragma once

#include <cmath>
#include <vector>
#include "mvc/controller/hardware_planner/runner.h"

namespace thesis_sim::mvc::model {

inline double hardware_path_increment(const HardwareTelemetrySample& previous,
                                      const HardwareTelemetrySample& current) {
    // A localization reset is a change of coordinates, not travelled distance.
    if (previous.pose_epoch != current.pose_epoch ||
        (current.start_matching_enabled > 0.5 &&
         previous.start_matching_accepted <= 0.5)) return 0.0;
    return std::hypot(current.position_x - previous.position_x,
                      current.position_y - previous.position_y);
}

inline double hardware_mission_time(const std::vector<HardwareTelemetrySample>& history,
                                    bool goal_reached, double frame_time) {
    if (!goal_reached) return -1.0;
    for (const auto& sample : history) {
        if (sample.mission_goal_reached > 0.5) return sample.time;
    }
    // Compatibility with older frames; proximity to a temporary gate/frontier
    // is never sufficient evidence of global mission completion.
    return frame_time;
}

}  // namespace thesis_sim::mvc::model
