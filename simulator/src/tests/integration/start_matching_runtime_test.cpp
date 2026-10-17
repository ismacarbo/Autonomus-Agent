#include <chrono>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <thread>
#include <unistd.h>

#include "mvc/controller/hardware_planner/runner.h"
#include "mvc/model/reporting/hardware_metrics.h"

using namespace thesis_sim;
namespace {
void require(bool condition, const char* message) {
    if (!condition) throw std::runtime_error(message);
}

struct Fixture {
    std::filesystem::path path = std::filesystem::temp_directory_path() /
        ("thesis_start_runtime_" + std::to_string(getpid()) + ".csv");
    HardwarePlannerConfig config;
    std::vector<RPLidarA1::ScanPoint> scan;
    double time = 10.0;
    int ticks = 0;
    Fixture() {
        config.start_matching.enabled = true;
        config.start_matching.reference_path = path.string();
        config.slam_observe_only = true;
        config.localization.lidar_x_offset = config.localization.lidar_y_offset = 0.0;
        config.localization.lidar_yaw_offset = 0.0;
        config.localization.lidar_flip_left_right = false;
        config.localization.max_range_m = 3.0;
        // Small search volume keeps CI fast while exercising the real matcher.
        config.start_matching.matcher.search_translation_m = .10;
        config.start_matching.matcher.search_yaw_rad = .12;
        mvc::model::InitialLidarReferenceCloud reference;
        reference.metadata.stability_valid = true;
        reference.metadata.lidar_x_offset_m = reference.metadata.lidar_y_offset_m = 0.0;
        reference.metadata.lidar_yaw_offset_rad = 0.0;
        reference.metadata.lidar_flip_left_right = false;
        for (int i = 0; i < 160; ++i) {
            const double angle = i * 2.0 * 3.141592653589793 / 160;
            const double r = .75 + .13 * std::cos(3 * angle) + .09 * std::sin(2 * angle);
            const Vec2 p{r * std::cos(angle), r * std::sin(angle)};
            reference.points.push_back(p);
            // Known accepted physical offset relative to the reference.
            const double x = p.x - .03, y = p.y + .02;
            const double range = std::hypot(x, y);
            scan.push_back({15, std::atan2(y, x) * 180.0 / 3.141592653589793,
                            range * 1000, range, x, y});
        }
        std::string error;
        require(mvc::model::write_initial_lidar_reference(path.string(), reference, &error),
                "cannot write matcher fixture");
    }
    ~Fixture() { std::error_code ec; std::filesystem::remove(path, ec); }
    void step(HardwarePlannerRunner& runner, bool reused = false, double dt = .1) {
        time += dt;
        RealRobotObservation observation;
        observation.host_timestamp_s = time;
        observation.have_controller_telemetry = true;
        auto& c = observation.controller;
        c.have_imu = c.have_encoder = c.have_motor = c.have_heartbeat = true;
        c.fw_major = 1; c.fw_minor = 5; c.enc_flags = 3; c.status_flags = 0x99;
        c.ms = c.imu_ms = c.encoder_ms = static_cast<std::uint32_t>(time * 1000);
        c.rx_timestamp_s = c.imu_host_timestamp_s = c.encoder_host_timestamp_s = time;
        c.imu_rx_timestamp_s = c.encoder_rx_timestamp_s = c.motor_rx_timestamp_s = time;
        c.enc_dt_ms = 100;
        c.ticks_left = c.ticks_right = ticks;
        observation.have_lidar_scan = true;
        observation.lidar_scan = scan;
        observation.lidar_scan_end_timestamp_s = time;
        observation.lidar_scan_reused = reused;
        runner.step_with_observation(observation, dt, false);
    }
};

void check_matcher_runtime() {
    Fixture f;
    auto world = WorldMap::unstructured_demo(UnstructuredMapPreset::HardwareLab);
    HardwarePlannerRunner runner(world, RealRobotBridge::Options{}, f.config);
    require(runner.start_matching_status().reference_loaded, "reference not loaded");
    for (int i = 0; i < 5; ++i) f.step(runner, true);
    require(runner.start_matching_status().collected_scans == 0,
            "reused scans counted as independent confirmations");
    f.step(runner); f.step(runner);
    require(runner.start_matching_status().status == "matching_stationary_cloud" &&
            runner.start_matching_status().completed_matches == 0,
            "match ran synchronously in the controller loop");
    const auto before_reset = std::chrono::steady_clock::now();
    runner.reset();
    require(std::chrono::steady_clock::now() - before_reset < std::chrono::milliseconds(500),
            "reset waited for the previous match worker");
    for (int i = 0; i < 5; ++i) f.step(runner, true);
    require(!runner.start_matching_status().accepted &&
            runner.start_matching_status().completed_matches == 0,
            "old match survived a reset");
    f.step(runner); f.step(runner);
    f.ticks += 1;
    f.step(runner, true);
    require(runner.start_matching_status().status == "waiting_for_stationary_robot" &&
            runner.start_matching_status().collected_scans == 0 &&
            !runner.start_matching_status().accepted,
            "motion during matching retained stationary confirmations");

    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(15);
    while (!runner.start_matching_status().complete && std::chrono::steady_clock::now() < deadline) {
        f.step(runner);
        if (!runner.start_matching_status().accepted) {
            require(runner.last_command().pwm_left == 0 && runner.last_command().pwm_right == 0,
                    "PWM issued before start acceptance");
            require(!runner.slam_start_ready(), "SLAM started before pose confirmation");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    const auto& match = runner.start_matching_status();
    require(match.accepted && match.completed_matches == 3 && match.collected_scans == 6,
            "three independent asynchronous confirmations did not complete");
    require(std::abs(match.position_offset_m.x - .03) < .015 &&
            std::abs(match.position_offset_m.y + .02) < .015 && runner.slam_start_ready(),
            "accepted match offset or SLAM readiness incorrect");
    require(!runner.history().back().start_matching_reference_hash.empty() &&
            runner.history().back().start_matching_completed_matches == 3,
            "match audit data not recorded");
    const double time = runner.sim_time();
    f.step(runner, false, 2.75);
    require(std::abs(runner.sim_time() - time - 2.75) < 1e-9 &&
            std::abs(runner.history().back().time - runner.sim_time()) < 1e-9,
            "long hardware interval compressed to integration dt");
    runner.reset_pose({2, 2}, .3);
    require(runner.global_occupied_points().empty() && runner.lidar_map_points().empty() &&
            !runner.slam_toolbox_snapshot().pose_valid, "old map survived pose reset");
}

void check_metrics() {
    HardwareTelemetrySample before, after;
    before.pose_epoch = 1; after.pose_epoch = 2;
    after.start_matching_enabled = after.start_matching_accepted = 1;
    after.position_x = .06;
    require(mvc::model::hardware_path_increment(before, after) == 0,
            "localization reset counted as physical distance");
    before = after; after.position_x += .1;
    require(std::abs(mvc::model::hardware_path_increment(before, after) - .1) < 1e-9,
            "real displacement omitted after initialization");
    after.time = 43.65; after.distance_to_goal = 0;
    require(mvc::model::hardware_mission_time({before, after}, false, 100) < 0,
            "temporary gate proximity counted as global success");
    after.mission_goal_reached = 1;
    require(mvc::model::hardware_mission_time({before, after}, true, 100) == 43.65,
            "explicit mission completion time lost");
}
}

int main() {
    try {
        check_matcher_runtime(); check_metrics();
        std::cout << "start_matching_runtime: ok\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "start_matching_runtime: " << error.what() << '\n';
        return 1;
    }
}
