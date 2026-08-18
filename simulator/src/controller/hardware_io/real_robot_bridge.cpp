#include "mvc/controller/hardware_io/real_robot_bridge.h"

#include <algorithm>
#include <chrono>
#include <iostream>
#include <utility>

namespace thesis_sim {

namespace {

constexpr double kLidarReconnectRetryS = 1.0;

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

bool RealRobotBridge::reconnect_lidar(bool log_failures) {
    if (options_.lidar_port.empty()) {
        return false;
    }
    if (lidar_.is_connected() && lidar_.scanning()) {
        return true;
    }

    const double now_s = monotonic_seconds();
    if (now_s < next_lidar_reconnect_time_s_) {
        return false;
    }
    next_lidar_reconnect_time_s_ = now_s + kLidarReconnectRetryS;

    if (lidar_.is_connected()) {
        try {
            lidar_.stop_scan(true);
        } catch (const LidarError&) {
        }
        lidar_.disconnect();
    }
    lidar_device_info_.reset();

    try {
        lidar_.connect();
        lidar_device_info_ = lidar_.get_info();
        lidar_.start_scan();
        last_lidar_error_.clear();
        next_lidar_reconnect_time_s_ = 0.0;
        std::cout << "hardware_runner_status=LiDAR connected on " << options_.lidar_port << '\n';
        return true;
    } catch (const LidarError& e) {
        lidar_device_info_.reset();
        last_lidar_error_ = e.what();
        refresh_lidar_snapshot({});
        if (lidar_.is_connected()) {
            lidar_.disconnect();
        }
        if (log_failures) {
            std::cerr << "hardware_runner_warning=LiDAR unavailable, will retry automatically: "
                      << last_lidar_error_ << '\n';
        }
        return false;
    }
}

void RealRobotBridge::connect(bool start_lidar_scan) {
    last_lidar_error_.clear();
    if (!options_.controller.port.empty()) {
        controller_.connect();
    }
    if (!options_.lidar_port.empty()) {
        if (start_lidar_scan) {
            reconnect_lidar(true);
        } else {
            try {
                lidar_.connect();
                lidar_device_info_ = lidar_.get_info();
            } catch (const LidarError& e) {
                lidar_device_info_.reset();
                last_lidar_error_ = e.what();
                if (lidar_.is_connected()) {
                    lidar_.disconnect();
                }
                refresh_lidar_snapshot({});
                next_lidar_reconnect_time_s_ = monotonic_seconds() + kLidarReconnectRetryS;
                std::cerr << "hardware_runner_warning=LiDAR unavailable, will retry automatically: "
                          << last_lidar_error_ << '\n';
            }
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
    lidar_device_info_.reset();
}

void RealRobotBridge::poll_controller(double timeout_s) {
    if (!controller_.is_connected()) {
        return;
    }
    controller_.poll(timeout_s);
    refresh_controller_snapshot();
    refresh_observation_timestamp();
}

std::vector<RPLidarA1::ScanPoint> RealRobotBridge::recent_lidar_scan_or_empty(double max_age_s) const {
    if (last_good_lidar_scan_.empty()) {
        return {};
    }

    const double age_s = monotonic_seconds() - last_good_lidar_scan_time_s_;
    if (age_s > std::max(max_age_s, 0.0)) {
        return {};
    }

    return last_good_lidar_scan_;
}

std::vector<RPLidarA1::ScanPoint> RealRobotBridge::read_lidar_scan(int min_points) {
    const int strict_min_points = std::max(min_points, 1);
    const int fallback_min_points = std::max(18, strict_min_points / 4);

    if (!lidar_.is_connected() || !lidar_.scanning()) {
        reconnect_lidar(false);
    }
    if (!lidar_.is_connected() || !lidar_.scanning()) {
        std::vector<RPLidarA1::ScanPoint> cached_scan = recent_lidar_scan_or_empty(0.30);
        refresh_lidar_snapshot(
            cached_scan,
            last_good_lidar_scan_start_time_s_,
            last_good_lidar_scan_time_s_,
            true);
        return observation_.lidar_scan;
    }

    try {
        const double scan_start_s = monotonic_seconds();
        std::vector<RPLidarA1::ScanPoint> scan = lidar_.grab_scan(strict_min_points);
        if (scan.empty() && fallback_min_points < strict_min_points) {
            scan = lidar_.grab_scan(fallback_min_points);
        }
        last_lidar_error_.clear();
        if (!scan.empty()) {
            const double scan_end_s = monotonic_seconds();
            last_good_lidar_scan_ = scan;
            last_good_lidar_scan_start_time_s_ = scan_start_s;
            last_good_lidar_scan_time_s_ = scan_end_s;
        }
        refresh_lidar_snapshot(scan, scan_start_s, monotonic_seconds(), false);
        refresh_observation_timestamp();
        return scan;
    } catch (const LidarError& e) {
        if (fallback_min_points < strict_min_points) {
            try {
                const double scan_start_s = monotonic_seconds();
                std::vector<RPLidarA1::ScanPoint> fallback_scan = lidar_.grab_scan(fallback_min_points);
                last_lidar_error_.clear();
                if (!fallback_scan.empty()) {
                    const double scan_end_s = monotonic_seconds();
                    last_good_lidar_scan_ = fallback_scan;
                    last_good_lidar_scan_start_time_s_ = scan_start_s;
                    last_good_lidar_scan_time_s_ = scan_end_s;
                }
                refresh_lidar_snapshot(fallback_scan, scan_start_s, monotonic_seconds(), false);
                refresh_observation_timestamp();
                return fallback_scan;
            } catch (const LidarError&) {
            }
        }

        last_lidar_error_ = e.what();
        std::vector<RPLidarA1::ScanPoint> cached_scan = recent_lidar_scan_or_empty(0.30);
        refresh_lidar_snapshot(
            cached_scan,
            last_good_lidar_scan_start_time_s_,
            last_good_lidar_scan_time_s_,
            true);
        if (!cached_scan.empty()) {
            refresh_observation_timestamp();
            return observation_.lidar_scan;
        }
        if (lidar_.is_connected()) {
            try {
                lidar_.stop_scan(true);
            } catch (const LidarError&) {
            }
            lidar_.disconnect();
        }
        next_lidar_reconnect_time_s_ = monotonic_seconds() + 0.4;
        std::cerr << "hardware_runner_warning=LiDAR stream lost, retrying automatically: "
                  << last_lidar_error_ << '\n';
        return {};
    }
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

void RealRobotBridge::refresh_lidar_snapshot(std::vector<RPLidarA1::ScanPoint> scan,
                                             double scan_start_s,
                                             double scan_end_s,
                                             bool reused) {
    const double now_s = monotonic_seconds();
    if (scan_end_s <= 0.0) {
        scan_end_s = now_s;
    }
    if (scan_start_s <= 0.0 || scan_start_s > scan_end_s) {
        scan_start_s = scan_end_s;
    }
    observation_.have_lidar_scan = !scan.empty();
    observation_.min_lidar_range_m = compute_min_range(scan);
    observation_.lidar_scan = std::move(scan);
    observation_.lidar_scan_start_timestamp_s = scan_start_s;
    observation_.lidar_scan_end_timestamp_s = scan_end_s;
    observation_.lidar_scan_mid_timestamp_s = 0.5 * (scan_start_s + scan_end_s);
    observation_.lidar_scan_duration_s = std::max(0.0, scan_end_s - scan_start_s);
    observation_.lidar_scan_age_s = std::max(0.0, now_s - scan_end_s);
    observation_.lidar_scan_reused = reused && observation_.have_lidar_scan;
}

}  // namespace thesis_sim
