LiveViewStreamClient::~LiveViewStreamClient() {
    disconnect();
}

bool LiveViewStreamClient::connect_to(const std::string& host, std::uint16_t port) {
    disconnect();
    last_error_.clear();

    addrinfo hints{};
    hints.ai_family = AF_UNSPEC;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;

    addrinfo* results = nullptr;
    const std::string port_str = std::to_string(port);
    const int rc = getaddrinfo(host.c_str(), port_str.c_str(), &hints, &results);
    if (rc != 0) {
        last_error_ = "getaddrinfo failed: " + std::string(gai_strerror(rc));
        return false;
    }

    bool connected = false;
    for (addrinfo* it = results; it != nullptr && !connected; it = it->ai_next) {
        socket_fd_ = ::socket(it->ai_family, it->ai_socktype, it->ai_protocol);
        if (socket_fd_ < 0) {
            continue;
        }
        if (::connect(socket_fd_, it->ai_addr, it->ai_addrlen) == 0) {
            connected = true;
            break;
        }
        close_socket(&socket_fd_);
    }
    freeaddrinfo(results);

    if (!connected) {
        last_error_ = socket_error_message("connect failed");
        return false;
    }
    if (!receive_server_hello(socket_fd_, &last_error_)) {
        disconnect();
        return false;
    }
    if (!set_nonblocking(socket_fd_)) {
        last_error_ = socket_error_message("failed to set stream client non-blocking");
        disconnect();
        return false;
    }
    return true;
}

void LiveViewStreamClient::disconnect() {
    close_socket(&socket_fd_);
    recv_buffer_.clear();
}

bool LiveViewStreamClient::send_packet(std::uint16_t type, const std::vector<std::uint8_t>& payload) {
    if (socket_fd_ < 0) {
        last_error_ = "stream client is not connected";
        return false;
    }

    const std::vector<std::uint8_t> packet = make_packet(type, payload);

    if (!send_all(socket_fd_, packet.data(), packet.size())) {
        last_error_ = socket_error_message("send failed");
        disconnect();
        return false;
    }
    return true;
}

bool LiveViewStreamClient::send_scene(const LiveSceneSnapshot& scene) {
    return send_packet(kPacketScene, serialize_scene(scene));
}

bool LiveViewStreamClient::send_frame(const LiveFrameSnapshot& frame) {
    return send_packet(kPacketFrame, serialize_frame(frame));
}

bool LiveViewStreamClient::send_control_ack(bool ok, const std::string& message) {
    LiveControlAck ack;
    ack.ok = ok;
    ack.message = message;
    return send_packet(kPacketControlAck, serialize_control_ack(ack));
}

void LiveViewStreamClient::read_server_data() {
    if (socket_fd_ < 0) {
        return;
    }

    std::array<std::uint8_t, 8192> chunk{};
    while (true) {
        const ssize_t bytes = ::recv(socket_fd_, chunk.data(), chunk.size(), 0);
        if (bytes > 0) {
            recv_buffer_.insert(recv_buffer_.end(), chunk.begin(), chunk.begin() + bytes);
            continue;
        }
        if (bytes == 0) {
            disconnect();
            return;
        }
        if (errno == EAGAIN || errno == EWOULDBLOCK) {
            return;
        }
        if (errno == EINTR) {
            continue;
        }
        last_error_ = socket_error_message("recv failed");
        disconnect();
        return;
    }
}

bool LiveViewStreamClient::parse_next_packet(PollResult* result) {
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
        disconnect();
        return false;
    }
    if (size > kMaxPacketBytes) {
        last_error_ = "live stream packet too large";
        disconnect();
        return false;
    }
    if (recv_buffer_.size() < kPacketHeaderSize + size) {
        return false;
    }

    std::vector<std::uint8_t> payload(size);
    if (size > 0U) {
        std::memcpy(payload.data(), recv_buffer_.data() + kPacketHeaderSize, size);
    }
    recv_buffer_.erase(
        recv_buffer_.begin(),
        recv_buffer_.begin() + static_cast<std::ptrdiff_t>(kPacketHeaderSize + size));

    if (result == nullptr) {
        return true;
    }

    if (type == kPacketWorld) {
        WorldMap world;
        if (!deserialize_world_blob(payload, &world)) {
            last_error_ = "failed to decode live world command";
            disconnect();
            return false;
        }
        result->world_received = true;
        result->world = std::move(world);
        return true;
    }
    if (type == kPacketRobotProfile) {
        VehicleModelKind vehicle_model = VehicleModelKind::CarLikeBicycle;
        if (!deserialize_robot_profile(payload, &vehicle_model)) {
            last_error_ = "failed to decode live robot profile command";
            disconnect();
            return false;
        }
        result->robot_profile_received = true;
        result->robot_profile = vehicle_model;
        return true;
    }

    last_error_ = "unknown live stream packet type";
    disconnect();
    return false;
}

LiveViewStreamClient::PollResult LiveViewStreamClient::poll() {
    PollResult result;
    if (socket_fd_ < 0) {
        return result;
    }

    read_server_data();
    while (parse_next_packet(&result)) {
    }
    return result;
}

