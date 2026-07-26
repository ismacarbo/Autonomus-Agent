#include "mvc/controller/hardware_io/external_pose_receiver.h"

#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>
#include <thread>

#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

int main() {
    constexpr std::uint16_t kPort = 15991;
    thesis_sim::ExternalPoseTransform transform;
    transform.translation_x_m = 0.10;
    transform.translation_y_m = -0.20;
    transform.yaw_rad = 3.14159265358979323846 / 2.0;
    transform.scale = 2.0;
    transform.body_yaw_offset_rad = 0.10;

    thesis_sim::ExternalPoseUdpReceiver receiver;
    if (!receiver.open(kPort, transform)) {
        std::cerr << receiver.last_error() << '\n';
        return 1;
    }

    const int sender = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (sender < 0) {
        return 2;
    }
    sockaddr_in target{};
    target.sin_family = AF_INET;
    target.sin_port = htons(kPort);
    target.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    const char payload[] = "12.5,0.25,0.50,0.20,0.90";
    const ssize_t sent = ::sendto(
        sender,
        payload,
        std::strlen(payload),
        0,
        reinterpret_cast<const sockaddr*>(&target),
        sizeof(target));
    ::close(sender);
    if (sent <= 0) {
        return 3;
    }

    thesis_sim::ExternalPoseSample sample;
    for (int attempt = 0; attempt < 20 && !receiver.poll(&sample); ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    const auto close = [](double a, double b) { return std::abs(a - b) < 1e-6; };
    if (!sample.valid ||
        !close(sample.source_timestamp_s, 12.5) ||
        !close(sample.position_x_m, -0.90) ||
        !close(sample.position_y_m, 0.30) ||
        !close(sample.yaw_rad, 0.20 + transform.yaw_rad + 0.10) ||
        !close(sample.quality, 0.90)) {
        std::cerr << "unexpected transformed pose\n";
        return 4;
    }
    std::cout << "external_pose_udp_roundtrip=ok\n";
    return 0;
}
