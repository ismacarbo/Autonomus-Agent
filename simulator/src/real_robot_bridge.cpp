#include "real_robot_bridge.h"

#include <algorithm>
#include <chrono>
#include <utility>

namespace thesis_sim {

namespace {

double monotonic_seconds() {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

double compute_min_range(const std::vector<RPLidarA1::ScanPoint>& scan) {
    if (scan.empty()) {
        return 0.0;
    }
    double min_range = scan.front().distance_m;
    for (const RPLidarA1::ScanPoint& point : scan) {
        min_range = std::min(min_range, point.distance_m);
    }
    return min_range;
}

}  // namespace

RealRobotBridge::RealRobotBridge()
    : RealRobotBridge(Options{}) {}

RealRobotBridge::RealRobotBridge(Options options)
    : options_(std::move(options)),
      controller_(options_.controller),
      lidar_(options_.lidar_port, options_.lidar_baudrate, options_.lidar_timeout_s, options_.lidar_motor_pwm) {}

void RealRobotBridge::connect(bool start_lidar_scan) {
    if (!options_.controller.port.empty()) {
        controller_.connect();
    }
    if (!options_.lidar_port.empty()) {
        lidar_.connect();
        if (start_lidar_scan) {
            lidar_.start_scan();
        }
    }
    refresh_observation_timestamp();
}

void RealRobotBridge::disconnect() {
    if (lidar_.is_connected()) {
        lidar_.stop_scan(true);
        lidar_.disconnect();
    }
    if (controller_.is_connected()) {
        controller_.disconnect();
    }
}

void RealRobotBridge::poll_controller(double timeout_s) {
    if (!controller_.is_connected()) {
        return;
    }
    controller_.poll(timeout_s);
    refresh_controller_snapshot();
    refresh_observation_timestamp();
}

std::vector<RPLidarA1::ScanPoint> RealRobotBridge::read_lidar_scan(int min_points) {
    if (!lidar_.is_connected() || !lidar_.scanning()) {
        return {};
    }
    std::vector<RPLidarA1::ScanPoint> scan = lidar_.grab_scan(min_points);
    refresh_lidar_snapshot(scan);
    refresh_observation_timestamp();
    return scan;
}

void RealRobotBridge::pump(double controller_timeout_s, int lidar_min_points, bool fetch_lidar) {
    poll_controller(controller_timeout_s);
    if (fetch_lidar) {
        read_lidar_scan(lidar_min_points);
    }
}

void RealRobotBridge::refresh_observation_timestamp() {
    observation_.host_timestamp_s = monotonic_seconds();
}

void RealRobotBridge::refresh_controller_snapshot() {
    if (const ControllerTelemetry* telemetry = controller_.telemetry()) {
        observation_.have_controller_telemetry = true;
        observation_.controller = *telemetry;
    }
}

void RealRobotBridge::refresh_lidar_snapshot(std::vector<RPLidarA1::ScanPoint> scan) {
    observation_.have_lidar_scan = !scan.empty();
    observation_.min_lidar_range_m = compute_min_range(scan);
    observation_.lidar_scan = std::move(scan);
}

}  // namespace thesis_sim
