bool set_nonblocking(int fd) {
    if (fd < 0) {
        return false;
    }
    const int flags = fcntl(fd, F_GETFL, 0);
    if (flags < 0) {
        return false;
    }
    return fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0;
}

void close_socket(int* fd) {
    if (fd != nullptr && *fd >= 0) {
        ::close(*fd);
        *fd = -1;
    }
}

std::string socket_error_message(const std::string& prefix) {
    return prefix + ": " + std::strerror(errno);
}

std::string endpoint_to_string(const sockaddr_storage& addr) {
    char host[NI_MAXHOST] = {};
    char service[NI_MAXSERV] = {};
    if (getnameinfo(reinterpret_cast<const sockaddr*>(&addr),
                    sizeof(addr),
                    host,
                    sizeof(host),
                    service,
                    sizeof(service),
                    NI_NUMERICHOST | NI_NUMERICSERV) == 0) {
        return std::string(host) + ":" + service;
    }
    return "unknown";
}

bool send_all(int fd, const std::uint8_t* data, std::size_t size) {
    std::size_t sent = 0;
    while (sent < size) {
        const ssize_t bytes = ::send(fd, data + sent, size - sent, 0);
        if (bytes <= 0) {
            if (errno == EINTR) {
                continue;
            }
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                pollfd pfd{};
                pfd.fd = fd;
                pfd.events = POLLOUT;
                const int rc = ::poll(&pfd, 1, 1000);
                if (rc > 0) {
                    continue;
                }
                if (rc == 0) {
                    errno = ETIMEDOUT;
                }
            }
            return false;
        }
        sent += static_cast<std::size_t>(bytes);
    }
    return true;
}

std::vector<std::uint8_t> make_packet(std::uint16_t type, const std::vector<std::uint8_t>& payload) {
    std::vector<std::uint8_t> packet;
    packet.reserve(kPacketHeaderSize + payload.size());
    write_pod(&packet, kPacketMagic);
    write_pod(&packet, kPacketVersion);
    write_pod(&packet, type);
    write_pod<std::uint32_t>(&packet, static_cast<std::uint32_t>(payload.size()));
    packet.insert(packet.end(), payload.begin(), payload.end());
    return packet;
}

std::vector<std::uint8_t> serialize_robot_profile(VehicleModelKind vehicle_model) {
    std::vector<std::uint8_t> payload;
    const std::uint8_t value =
        vehicle_model == VehicleModelKind::TrackedVehicle ? 1U : 0U;
    write_pod(&payload, value);
    return payload;
}

bool deserialize_robot_profile(const std::vector<std::uint8_t>& payload,
                               VehicleModelKind* vehicle_model) {
    if (vehicle_model == nullptr) {
        return false;
    }
    std::size_t offset = 0;
    std::uint8_t value = 0U;
    if (!read_pod(payload, &offset, &value) || offset != payload.size()) {
        return false;
    }
    if (value == 0U) {
        *vehicle_model = VehicleModelKind::CarLikeBicycle;
        return true;
    }
    if (value == 1U) {
        *vehicle_model = VehicleModelKind::TrackedVehicle;
        return true;
    }
    return false;
}

bool recv_all_with_timeout(int fd, std::uint8_t* data, std::size_t size, int timeout_ms) {
    if (fd < 0 || data == nullptr) {
        return false;
    }

    std::size_t received = 0;
    while (received < size) {
        pollfd pfd{};
        pfd.fd = fd;
        pfd.events = POLLIN;
        const int rc = ::poll(&pfd, 1, timeout_ms);
        if (rc == 0) {
            errno = ETIMEDOUT;
            return false;
        }
        if (rc < 0) {
            if (errno == EINTR) {
                continue;
            }
            return false;
        }
        if ((pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
            errno = ECONNRESET;
            return false;
        }

        const ssize_t bytes = ::recv(fd, data + received, size - received, 0);
        if (bytes <= 0) {
            if (bytes < 0 && errno == EINTR) {
                continue;
            }
            if (bytes == 0) {
                errno = ECONNRESET;
            }
            return false;
        }
        received += static_cast<std::size_t>(bytes);
    }
    return true;
}

bool receive_server_hello(int fd, std::string* error) {
    std::array<std::uint8_t, kPacketHeaderSize> header{};
    if (!recv_all_with_timeout(fd, header.data(), header.size(), kHelloTimeoutMs)) {
        if (error != nullptr) {
            if (errno == ETIMEDOUT) {
                *error = "live viewer handshake failed: timeout waiting for viewer hello";
            } else {
                *error = socket_error_message("live viewer handshake failed");
            }
        }
        return false;
    }

    std::vector<std::uint8_t> header_vec(header.begin(), header.end());
    std::size_t offset = 0;
    std::uint32_t magic = 0U;
    std::uint16_t version = 0U;
    std::uint16_t type = 0U;
    std::uint32_t size = 0U;
    if (!read_pod(header_vec, &offset, &magic) ||
        !read_pod(header_vec, &offset, &version) ||
        !read_pod(header_vec, &offset, &type) ||
        !read_pod(header_vec, &offset, &size)) {
        if (error != nullptr) {
            *error = "live viewer handshake failed: malformed header";
        }
        return false;
    }
    if (magic != kPacketMagic || version != kPacketVersion || type != kPacketHello) {
        if (error != nullptr) {
            *error = "live viewer handshake failed: protocol mismatch or wrong service on port";
        }
        return false;
    }
    if (size > kMaxPacketBytes) {
        if (error != nullptr) {
            *error = "live viewer handshake failed: packet too large";
        }
        return false;
    }
    if (size == 0U) {
        return true;
    }

    std::vector<std::uint8_t> payload(size);
    if (!recv_all_with_timeout(fd, payload.data(), payload.size(), kHelloTimeoutMs)) {
        if (error != nullptr) {
            *error = socket_error_message("live viewer handshake payload failed");
        }
        return false;
    }
    return true;
}

