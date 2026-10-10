#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#include "mvc/controller/hardware_planner/runner.h"

using namespace thesis_sim;

namespace {
void require(bool ok, const char* message) {
    if (!ok) throw std::runtime_error(message);
}
bool near(double a, double b) { return std::abs(a - b) < 1e-6; }

class UdpFixture {
public:
    int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    sockaddr_in client{};
    SlamToolboxBridgeClient bridge;
    UdpFixture() {
        require(fd >= 0, "UDP socket failed");
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        require(::bind(fd, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0,
                "UDP bind failed");
        socklen_t size = sizeof(address);
        require(::getsockname(fd, reinterpret_cast<sockaddr*>(&address), &size) == 0,
                "UDP port lookup failed");
        timeval timeout{1, 0};
        ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
        require(bridge.open("127.0.0.1", ntohs(address.sin_port)), "bridge open failed");
    }
    ~UdpFixture() { ::close(fd); }
    void scan(const std::string& session, int sequence, Vec2 pose) {
        std::vector<LidarHit> hits;
        for (int i = 0; i < 12; ++i) {
            const double angle = .4 * i;
            LidarHit hit{};
            hit.angle = angle; hit.distance = .5; hit.hit = true;
            hit.point = {pose.x + .5 * std::cos(angle), pose.y + .5 * std::sin(angle)};
            hits.push_back(hit);
        }
        require(bridge.submit_scan(session, sequence, sequence * .1, pose, 0.0, hits, .04, 1.2),
                "scan submission failed");
        char buffer[60000];
        socklen_t size = sizeof(client);
        require(::recvfrom(fd, buffer, sizeof(buffer), 0,
                           reinterpret_cast<sockaddr*>(&client), &size) > 0,
                "scan datagram not received");
    }
    bool reply(const std::string& session, int sequence, const std::string& x,
               const std::string& y, int updates = 1) {
        const std::string packet = "MAP|" + session + '|' + std::to_string(sequence) +
            "|1|" + x + '|' + y + "|1.5707963267948966|" + std::to_string(updates) +
            "|2|0|0.1|100|200|2|2|0|1,2|test";
        require(::sendto(fd, packet.data(), packet.size(), 0,
                         reinterpret_cast<sockaddr*>(&client), sizeof(client)) ==
                    static_cast<ssize_t>(packet.size()), "map reply failed");
        return bridge.poll();
    }
};

void check_observation(const SlamToolboxSnapshot& snapshot) {
    auto world = WorldMap::unstructured_demo(UnstructuredMapPreset::HardwareLab);
    HardwarePlannerConfig config;
    config.start_matching.enabled = false;
    config.slam_observe_only = true;
    HardwarePlannerRunner observer(world, RealRobotBridge::Options{}, config);
    const Vec2 before = observer.estimate().position;
    observer.apply_slam_toolbox_snapshot(snapshot);
    require(observer.diagnostics().map_source == ExplorationMapSource::LocalPersistentGrid &&
            observer.global_occupied_points().empty(), "observer modified the navigation map");
    require(near(before.x, observer.estimate().position.x) &&
            near(before.y, observer.estimate().position.y), "observer modified the estimated pose");
    RealRobotObservation observation;
    observation.host_timestamp_s = 10.0;
    observer.step_with_observation(observation, .1, false);
    const auto& sample = observer.history().back();
    require(sample.slam_pose_valid > .5 && sample.slam_correction_accepted == 0.0 &&
            near(sample.slam_pose_x, 1.3) && near(sample.slam_position_innovation_m, .2),
            "SLAM comparison missing or compared with receive-time odometry");

    config.slam_observe_only = false;
    HardwarePlannerRunner mapper(world, RealRobotBridge::Options{}, config);
    mapper.apply_slam_toolbox_snapshot(snapshot);
    require(mapper.global_occupied_points().size() == 1 &&
            near(mapper.global_occupied_points()[0].x, .85) &&
            near(mapper.global_occupied_points()[0].y, 1.95),
            "SLAM grid used unregistered map coordinates for navigation");
    auto disconnected = snapshot;
    disconnected.connected = false;
    observer.apply_slam_toolbox_snapshot(disconnected);
    observer.step_with_observation(observation, .1, false);
    require(observer.history().back().slam_pose_valid == 0.0,
            "disconnected SLAM pose remained valid in history");
}
}

int main() {
    try {
        UdpFixture fixture;
        fixture.scan("session_a", 1, {1.0, 2.0});
        require(fixture.reply("session_a", 1, "100", "200"), "first map rejected");
        require(near(fixture.bridge.snapshot().corrected_position.x, 1.0) &&
                near(fixture.bridge.snapshot().corrected_yaw, 0.0), "initial map alignment failed");
        fixture.scan("session_a", 2, {1.1, 2.0});
        require(fixture.reply("session_a", 2, "100", "200.3", 2), "second map rejected");
        require(near(fixture.bridge.snapshot().corrected_position.x, 1.3),
                "SLAM drift was erased by aligning every response");
        check_observation(fixture.bridge.snapshot());
        require(!fixture.reply("session_a", 1, "100", "200"), "out-of-order response accepted");
        require(!fixture.reply("old_session", 2, "100", "200"), "wrong-session response accepted");
        require(!fixture.reply("session_a", 3, "100", "200"), "unsolicited sequence accepted");
        require(!fixture.reply("session_a", 2, "nan", "200"), "nonfinite SLAM pose accepted");
        require(fixture.reply("session_a", 2, "100", "200.3", 3),
                "map update for the same scan incorrectly rejected");
        fixture.scan("session_b", 1, {4.0, 5.0});
        require(!fixture.reply("session_a", 2, "100", "200"), "old session survived a restart");
        require(fixture.reply("session_b", 1, "100", "200") &&
                near(fixture.bridge.snapshot().corrected_position.x, 4.0),
                "new session inherited the previous map alignment");
        std::cout << "slam_observation: ok\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "slam_observation: " << error.what() << '\n';
        return 1;
    }
}
