#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "mvc/model/world/world.h"

namespace thesis_sim {

struct SlamToolboxSnapshot {
    bool connected = false;
    bool pose_valid = false;
    Vec2 corrected_position{};
    double corrected_yaw = 0.0;
    int map_updates = 0;
    int graph_nodes = 0;
    int loop_edges = 0;
    double map_resolution_m = 0.03;
    std::vector<Vec2> occupied_points;
    std::string status = "LiDAR reconstruction fallback";
};

class SlamToolboxBridgeClient {
  public:
    SlamToolboxBridgeClient() = default;
    ~SlamToolboxBridgeClient();

    SlamToolboxBridgeClient(const SlamToolboxBridgeClient&) = delete;
    SlamToolboxBridgeClient& operator=(const SlamToolboxBridgeClient&) = delete;

    bool open(const std::string& host = "127.0.0.1", std::uint16_t port = 9760);
    void close();
    bool submit_scan(const std::string& session,
                     std::uint64_t sequence,
                     double timestamp_s,
                     const Vec2& odom_position,
                     double odom_yaw,
                     const std::vector<LidarHit>& scan,
                     double min_range_m,
                     double max_range_m);
    bool poll();

    bool active() const { return socket_fd_ >= 0; }
    const SlamToolboxSnapshot& snapshot() const { return snapshot_; }
    const std::string& last_error() const { return last_error_; }

  private:
    bool parse_response(const char* data, std::size_t size);

    int socket_fd_ = -1;
    std::string host_;
    std::uint16_t port_ = 0;
    std::string last_error_;
    SlamToolboxSnapshot snapshot_{};
    std::chrono::steady_clock::time_point last_response_time_{};
};

}  // namespace thesis_sim
