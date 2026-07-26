#pragma once

#include "mvc/view/live_stream/live_view_stream.h"

#include <vector>

namespace thesis_sim::mvc::view {

struct HardwareViewerState {
    struct LidarReconstructionPoint {
        double time = 0.0;
        int step = 0;
        int beam_index = 0;
        double pose_x = 0.0;
        double pose_y = 0.0;
        double pose_yaw = 0.0;
        double range_m = 0.0;
        double angle_world_rad = 0.0;
        double hit_x = 0.0;
        double hit_y = 0.0;
    };

    bool has_scene = false;
    LiveSceneSnapshot scene;
    LiveFrameSnapshot frame;
    std::vector<HardwareTelemetrySample> history;
    std::vector<LidarReconstructionPoint> lidar_reconstruction;
    int last_lidar_reconstruction_step = -1;
};

}  // namespace thesis_sim::mvc::view
