#include "mvc/controller/hardware_io/external_pose_receiver.h"

#include <algorithm>
#include <array>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <sstream>
#include <string>

#include <arpa/inet.h>
#include <fcntl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace thesis_sim {
namespace {

constexpr double kPi = 3.14159265358979323846;

double wrap_angle(double angle) {
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

}  // namespace

ExternalPoseUdpReceiver::~ExternalPoseUdpReceiver() {
    close();
}

bool ExternalPoseUdpReceiver::open(std::uint16_t port, const ExternalPoseTransform& transform) {
    close();
    if (port == 0U || !std::isfinite(transform.scale) || transform.scale <= 0.0) {
        last_error_ = "invalid UDP port or pose transform";
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

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(port);
    if (::bind(socket_fd_, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) < 0) {
        last_error_ = std::string("bind: ") + std::strerror(errno);
        close();
        return false;
    }

    port_ = port;
    transform_ = transform;
    last_error_.clear();
    return true;
}

void ExternalPoseUdpReceiver::close() {
    if (socket_fd_ >= 0) {
        ::close(socket_fd_);
    }
    socket_fd_ = -1;
    port_ = 0;
}

bool ExternalPoseUdpReceiver::parse_packet(const char* data,
                                           std::size_t size,
                                           ExternalPoseSample* sample) const {
    if (data == nullptr || sample == nullptr || size == 0U || size > 2048U) {
        return false;
    }
    std::string packet(data, size);
    std::replace(packet.begin(), packet.end(), ';', ',');
    std::istringstream stream(packet);
    std::array<double, 5> values{};
    std::size_t count = 0;
    std::string token;
    while (std::getline(stream, token, ',') && count < values.size()) {
        try {
            std::size_t parsed = 0;
            values[count] = std::stod(token, &parsed);
            if (parsed == 0U) {
                return false;
            }
        } catch (...) {
            return false;
        }
        ++count;
    }
    if (count < 4U) {
        return false;
    }
    for (std::size_t i = 0; i < count; ++i) {
        if (!std::isfinite(values[i])) {
            return false;
        }
    }

    const double c = std::cos(transform_.yaw_rad);
    const double s = std::sin(transform_.yaw_rad);
    const double scaled_x = transform_.scale * values[1];
    const double scaled_y = transform_.scale * values[2];
    sample->valid = true;
    sample->source_timestamp_s = values[0];
    sample->position_x_m = transform_.translation_x_m + c * scaled_x - s * scaled_y;
    sample->position_y_m = transform_.translation_y_m + s * scaled_x + c * scaled_y;
    sample->yaw_rad = wrap_angle(values[3] + transform_.yaw_rad + transform_.body_yaw_offset_rad);
    sample->quality = std::clamp(count >= 5U ? values[4] : 1.0, 0.0, 1.0);
    return true;
}

bool ExternalPoseUdpReceiver::poll(ExternalPoseSample* sample) {
    if (socket_fd_ < 0 || sample == nullptr) {
        return false;
    }
    bool received = false;
    std::array<char, 2048> buffer{};
    while (true) {
        const ssize_t bytes = ::recvfrom(socket_fd_, buffer.data(), buffer.size(), 0, nullptr, nullptr);
        if (bytes < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                break;
            }
            last_error_ = std::string("recvfrom: ") + std::strerror(errno);
            break;
        }
        ExternalPoseSample candidate;
        if (parse_packet(buffer.data(), static_cast<std::size_t>(bytes), &candidate)) {
            *sample = candidate;
            received = true;
        }
    }
    return received;
}

}  // namespace thesis_sim
