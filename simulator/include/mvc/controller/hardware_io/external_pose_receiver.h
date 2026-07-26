#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace thesis_sim {

struct ExternalPoseTransform {
    double translation_x_m = 0.0;
    double translation_y_m = 0.0;
    double yaw_rad = 0.0;
    double scale = 1.0;
    double body_yaw_offset_rad = 0.0;
};

struct ExternalPoseSample {
    bool valid = false;
    double source_timestamp_s = 0.0;
    double position_x_m = 0.0;
    double position_y_m = 0.0;
    double yaw_rad = 0.0;
    double quality = 1.0;
};

// Non-blocking UDP receiver for an independent pose reference. Accepted CSV:
// timestamp_s,x_m,y_m,yaw_rad[,quality]
class ExternalPoseUdpReceiver {
  public:
    ExternalPoseUdpReceiver() = default;
    ~ExternalPoseUdpReceiver();

    ExternalPoseUdpReceiver(const ExternalPoseUdpReceiver&) = delete;
    ExternalPoseUdpReceiver& operator=(const ExternalPoseUdpReceiver&) = delete;

    bool open(std::uint16_t port, const ExternalPoseTransform& transform = {});
    void close();
    bool poll(ExternalPoseSample* sample);

    bool active() const { return socket_fd_ >= 0; }
    std::uint16_t port() const { return port_; }
    const std::string& last_error() const { return last_error_; }

  private:
    bool parse_packet(const char* data, std::size_t size, ExternalPoseSample* sample) const;

    int socket_fd_ = -1;
    std::uint16_t port_ = 0;
    ExternalPoseTransform transform_{};
    std::string last_error_;
};

}  // namespace thesis_sim
