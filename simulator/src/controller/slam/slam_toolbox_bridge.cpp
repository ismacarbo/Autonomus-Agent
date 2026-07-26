#include "mvc/controller/slam/slam_toolbox_bridge.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netdb.h>
#include <sys/socket.h>
#include <unistd.h>

namespace thesis_sim {
namespace {

std::vector<std::string> split(const std::string& value, char delimiter) {
    std::vector<std::string> fields;
    std::istringstream stream(value);
    std::string field;
    while (std::getline(stream, field, delimiter)) {
        fields.push_back(field);
    }
    return fields;
}

}  // namespace

SlamToolboxBridgeClient::~SlamToolboxBridgeClient() {
    close();
}

bool SlamToolboxBridgeClient::open(const std::string& host, std::uint16_t port) {
    close();
    if (host.empty() || port == 0U) {
        last_error_ = "invalid bridge endpoint";
        return false;
    }
    socket_fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (socket_fd_ < 0) {
        last_error_ = std::string("socket: ") + std::strerror(errno);
        return false;
    }
    const int flags = ::fcntl(socket_fd_, F_GETFL, 0);
    if (flags < 0 || ::fcntl(socket_fd_, F_SETFL, flags | O_NONBLOCK) < 0) {
        last_error_ = std::string("fcntl: ") + std::strerror(errno);
        close();
        return false;
    }

    sockaddr_in local{};
    local.sin_family = AF_INET;
    local.sin_addr.s_addr = htonl(INADDR_ANY);
    local.sin_port = 0;
    if (::bind(socket_fd_, reinterpret_cast<const sockaddr*>(&local), sizeof(local)) < 0) {
        last_error_ = std::string("bind: ") + std::strerror(errno);
        close();
        return false;
    }
    host_ = host;
    port_ = port;
    snapshot_ = {};
    last_error_.clear();
    return true;
}

void SlamToolboxBridgeClient::close() {
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
    }
    socket_fd_ = -1;
    port_ = 0;
    snapshot_ = {};
}

bool SlamToolboxBridgeClient::submit_scan(const std::string& session,
                                          std::uint64_t sequence,
                                          double timestamp_s,
                                          const Vec2& odom_position,
                                          double odom_yaw,
                                          const std::vector<LidarHit>& scan,
                                          double min_range_m,
                                          double max_range_m) {
    if (socket_fd_ < 0 || scan.empty()) {
        return false;
    }
    std::ostringstream message;
    message << std::setprecision(9)
            << "SCAN|" << session << '|' << sequence << '|' << timestamp_s << '|'
            << odom_position.x << '|' << odom_position.y << '|' << odom_yaw << '|'
            << min_range_m << '|' << max_range_m << '|';
    for (std::size_t i = 0; i < scan.size(); ++i) {
        const LidarHit& hit = scan[i];
        message << hit.angle << ',' << hit.distance << ',' << (hit.hit ? 1 : 0);
        if (i + 1U < scan.size()) {
            message << ';';
        }
    }
    const std::string payload = message.str();
    if (payload.size() > 60000U) {
        last_error_ = "scan packet exceeds UDP payload budget";
        return false;
    }

    addrinfo hints{};
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    addrinfo* result = nullptr;
    const std::string port_text = std::to_string(port_);
    if (::getaddrinfo(host_.c_str(), port_text.c_str(), &hints, &result) != 0 || result == nullptr) {
        last_error_ = "could not resolve SLAM bridge host";
        return false;
    }
    const ssize_t sent = ::sendto(
        socket_fd_,
        payload.data(),
        payload.size(),
        0,
        result->ai_addr,
        result->ai_addrlen);
    ::freeaddrinfo(result);
    if (sent != static_cast<ssize_t>(payload.size())) {
        last_error_ = std::string("sendto: ") + std::strerror(errno);
        return false;
    }
    return true;
}

bool SlamToolboxBridgeClient::parse_response(const char* data, std::size_t size) {
    if (data == nullptr || size == 0U || size > 65507U) {
        return false;
    }
    const std::vector<std::string> fields = split(std::string(data, size), '|');
    if (fields.size() < 15U || fields[0] != "MAP") {
        return false;
    }
    try {
        SlamToolboxSnapshot parsed;
        parsed.connected = true;
        parsed.pose_valid = std::stoi(fields[3]) != 0;
        parsed.corrected_position.x = std::stod(fields[4]);
        parsed.corrected_position.y = std::stod(fields[5]);
        parsed.corrected_yaw = std::stod(fields[6]);
        parsed.map_updates = std::stoi(fields[7]);
        parsed.graph_nodes = std::stoi(fields[8]);
        parsed.loop_edges = std::stoi(fields[9]);
        parsed.map_resolution_m = std::stod(fields[10]);
        const double origin_x = std::stod(fields[11]);
        const double origin_y = std::stod(fields[12]);
        const int width = std::stoi(fields[13]);
        const int height = std::stoi(fields[14]);
        if (!(parsed.map_resolution_m > 0.0) || width <= 0 || height <= 0 ||
            width > 4096 || height > 4096) {
            return false;
        }
        if (fields.size() >= 16U && !fields[15].empty()) {
            const std::vector<std::string> indices = split(fields[15], ',');
            parsed.occupied_points.reserve(indices.size());
            for (const std::string& token : indices) {
                const int index = std::stoi(token);
                if (index < 0 || index >= width * height) {
                    continue;
                }
                const int cell_x = index % width;
                const int cell_y = index / width;
                parsed.occupied_points.push_back({
                    origin_x + (static_cast<double>(cell_x) + 0.5) * parsed.map_resolution_m,
                    origin_y + (static_cast<double>(cell_y) + 0.5) * parsed.map_resolution_m,
                });
            }
        }
        parsed.status = "slam_toolbox: scan matching + pose graph optimization";
        snapshot_ = std::move(parsed);
        last_response_time_ = std::chrono::steady_clock::now();
        return true;
    } catch (...) {
        return false;
    }
}

bool SlamToolboxBridgeClient::poll() {
    if (socket_fd_ < 0) {
        return false;
    }
    bool updated = false;
    std::array<char, 65507> buffer{};
    while (true) {
        const ssize_t bytes = ::recvfrom(socket_fd_, buffer.data(), buffer.size(), 0, nullptr, nullptr);
        if (bytes < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                break;
            }
            last_error_ = std::string("recvfrom: ") + std::strerror(errno);
            break;
        }
        updated = parse_response(buffer.data(), static_cast<std::size_t>(bytes)) || updated;
    }
    if (snapshot_.connected &&
        std::chrono::steady_clock::now() - last_response_time_ > std::chrono::seconds(2)) {
        snapshot_.connected = false;
        snapshot_.status = "SLAM bridge stale; LiDAR reconstruction fallback";
    }
    return updated;
}

}  // namespace thesis_sim
