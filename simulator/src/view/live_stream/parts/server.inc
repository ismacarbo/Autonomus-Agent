LiveViewStreamServer::~LiveViewStreamServer() {
    stop();
}

bool LiveViewStreamServer::start(std::uint16_t port) {
    stop();
    last_error_.clear();

    listen_fd_ = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd_ < 0) {
        last_error_ = socket_error_message("socket failed");
        return false;
    }

    int reuse_addr = 1;
    if (setsockopt(listen_fd_, SOL_SOCKET, SO_REUSEADDR, &reuse_addr, sizeof(reuse_addr)) != 0) {
        last_error_ = socket_error_message("setsockopt failed");
        stop();
        return false;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    addr.sin_port = htons(port);

    if (bind(listen_fd_, reinterpret_cast<const sockaddr*>(&addr), sizeof(addr)) != 0) {
        last_error_ = socket_error_message("bind failed");
        stop();
        return false;
    }
    if (listen(listen_fd_, 1) != 0) {
        last_error_ = socket_error_message("listen failed");
        stop();
        return false;
    }
    if (!set_nonblocking(listen_fd_)) {
        last_error_ = socket_error_message("failed to set listener non-blocking");
        stop();
        return false;
    }

    port_ = port;
    return true;
}

void LiveViewStreamServer::close_client() {
    close_socket(&client_fd_);
    remote_endpoint_.clear();
    recv_buffer_.clear();
}

void LiveViewStreamServer::stop() {
    close_client();
    close_socket(&listen_fd_);
    port_ = 0;
}

bool LiveViewStreamServer::accept_client() {
    if (listen_fd_ < 0) {
        return false;
    }

    sockaddr_storage addr{};
    socklen_t addr_len = sizeof(addr);
    const int fd = ::accept(listen_fd_, reinterpret_cast<sockaddr*>(&addr), &addr_len);
    if (fd < 0) {
        if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
            last_error_ = socket_error_message("accept failed");
        }
        return false;
    }

    close_client();
    client_fd_ = fd;
    const std::vector<std::uint8_t> hello_packet = make_packet(kPacketHello, {});
    if (!send_all(client_fd_, hello_packet.data(), hello_packet.size())) {
        last_error_ = socket_error_message("failed to send live viewer handshake");
        close_client();
        return false;
    }
    if (!set_nonblocking(client_fd_)) {
        last_error_ = socket_error_message("failed to set client non-blocking");
        close_client();
        return false;
    }
    remote_endpoint_ = endpoint_to_string(addr);
    last_error_.clear();
    return true;
}

bool LiveViewStreamServer::send_packet(std::uint16_t type, const std::vector<std::uint8_t>& payload) {
    if (client_fd_ < 0) {
        last_error_ = "stream server has no connected client";
        return false;
    }

    const std::vector<std::uint8_t> packet = make_packet(type, payload);
    if (!send_all(client_fd_, packet.data(), packet.size())) {
        last_error_ = socket_error_message("send failed");
        close_client();
        return false;
    }
    return true;
}

bool LiveViewStreamServer::queue_world(const WorldMap& world) {
    pending_world_ = world;
    if (client_fd_ >= 0) {
        flush_pending_world();
        return client_fd_ >= 0;
    }
    return true;
}

void LiveViewStreamServer::clear_pending_world() {
    pending_world_.reset();
}

bool LiveViewStreamServer::queue_robot_profile(VehicleModelKind vehicle_model) {
    pending_robot_profile_ = vehicle_model;
    if (client_fd_ >= 0) {
        flush_pending_robot_profile();
        return client_fd_ >= 0;
    }
    return true;
}

void LiveViewStreamServer::clear_pending_robot_profile() {
    pending_robot_profile_.reset();
}

bool LiveViewStreamServer::queue_runtime_settings(
    const LiveRuntimeSettings& settings) {
    pending_runtime_settings_ = settings;
    if (client_fd_ >= 0) {
        flush_pending_runtime_settings();
        return client_fd_ >= 0;
    }
    return true;
}

void LiveViewStreamServer::clear_pending_runtime_settings() {
    pending_runtime_settings_.reset();
}

void LiveViewStreamServer::flush_pending_world() {
    if (client_fd_ < 0 || !pending_world_.has_value()) {
        return;
    }
    if (!send_packet(kPacketWorld, serialize_world_blob(*pending_world_))) {
        return;
    }
    pending_world_.reset();
}

void LiveViewStreamServer::flush_pending_robot_profile() {
    if (client_fd_ < 0 || !pending_robot_profile_.has_value()) {
        return;
    }
    if (!send_packet(kPacketRobotProfile, serialize_robot_profile(*pending_robot_profile_))) {
        return;
    }
    pending_robot_profile_.reset();
}

void LiveViewStreamServer::flush_pending_runtime_settings() {
    if (client_fd_ < 0 || !pending_runtime_settings_.has_value()) return;
    if (!send_packet(
            kPacketRuntimeSettings,
            serialize_runtime_settings(*pending_runtime_settings_))) {
        return;
    }
    pending_runtime_settings_.reset();
}

void LiveViewStreamServer::read_client_data() {
    if (client_fd_ < 0) {
        return;
    }

    std::array<std::uint8_t, 8192> chunk{};
    while (true) {
        const ssize_t bytes = ::recv(client_fd_, chunk.data(), chunk.size(), 0);
        if (bytes > 0) {
            recv_buffer_.insert(recv_buffer_.end(), chunk.begin(), chunk.begin() + bytes);
            continue;
        }
        if (bytes == 0) {
            close_client();
            return;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return;
        }
        if (errno == EINTR) {
            continue;
        }
        last_error_ = socket_error_message("recv failed");
        close_client();
        return;
    }
}

bool LiveViewStreamServer::parse_next_packet(PollResult* result) {
    if (recv_buffer_.size() < kPacketHeaderSize) {
        return false;
    }

    std::size_t offset = 0;
    std::uint32_t magic = 0U;
    std::uint16_t version = 0U;
    std::uint16_t type = 0U;
    std::uint32_t size = 0U;
    if (!read_pod(recv_buffer_, &offset, &magic) ||
        !read_pod(recv_buffer_, &offset, &version) ||
        !read_pod(recv_buffer_, &offset, &type) ||
        !read_pod(recv_buffer_, &offset, &size)) {
        return false;
    }

    if (magic != kPacketMagic || version != kPacketVersion) {
        last_error_ = "live stream protocol mismatch";
        close_client();
        return false;
    }
    if (size > kMaxPacketBytes) {
        last_error_ = "live stream packet too large";
        close_client();
        return false;
    }
    if (recv_buffer_.size() < kPacketHeaderSize + size) {
        return false;
    }

    std::vector<std::uint8_t> payload(size);
    if (size > 0) {
        std::memcpy(payload.data(), recv_buffer_.data() + kPacketHeaderSize, size);
    }
    recv_buffer_.erase(recv_buffer_.begin(), recv_buffer_.begin() + static_cast<std::ptrdiff_t>(kPacketHeaderSize + size));

    if (result == nullptr) {
        return true;
    }

    if (type == kPacketScene) {
        LiveSceneSnapshot scene;
        if (!deserialize_scene(payload, &scene)) {
            last_error_ = "failed to decode live scene snapshot";
            close_client();
            return false;
        }
        result->scene_received = true;
        result->scene = std::move(scene);
        return true;
    }
    if (type == kPacketFrame) {
        LiveFrameSnapshot frame;
        if (!deserialize_frame(payload, &frame)) {
            last_error_ = "failed to decode live frame snapshot";
            close_client();
            return false;
        }
        result->frame_received = true;
        result->frame = std::move(frame);
        return true;
    }
    if (type == kPacketControlAck) {
        LiveControlAck ack;
        if (!deserialize_control_ack(payload, &ack)) {
            last_error_ = "failed to decode live control ack";
            close_client();
            return false;
        }
        result->control_ack_received = true;
        result->control_ack = std::move(ack);
        return true;
    }

    last_error_ = "unknown live stream packet type";
    close_client();
    return false;
}

LiveViewStreamServer::PollResult LiveViewStreamServer::poll() {
    PollResult result;
    if (listen_fd_ < 0) {
        return result;
    }

    if (client_fd_ < 0) {
        accept_client();
    }
    flush_pending_world();
    flush_pending_robot_profile();
    flush_pending_runtime_settings();
    read_client_data();
    while (parse_next_packet(&result)) {
    }
    return result;
}

}  // namespace thesis_sim
