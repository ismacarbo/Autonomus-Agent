#include "serial_port.h"

#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <sstream>
#include <thread>
#include <utility>

namespace thesis_sim {

namespace {

double monotonic_seconds() {
    using clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(clock::now().time_since_epoch()).count();
}

std::string errno_message(const std::string& context) {
    std::ostringstream oss;
    oss << context << ": " << std::strerror(errno);
    return oss.str();
}

void throw_if_negative(int rc, const std::string& context) {
    if (rc < 0) {
        throw SerialError(errno_message(context));
    }
}

void set_serial_speed(termios* tty, speed_t speed) {
    if (tty == nullptr) {
        throw SerialError("set_serial_speed: null termios");
    }
#if defined(__linux__) && defined(CBAUD)
    // Set the encoded baudrate directly to avoid binding against the newer
    // glibc cfset*speed symbols, which break execution on older Linux hosts.
    tty->c_cflag &= static_cast<tcflag_t>(~CBAUD);
    tty->c_cflag |= static_cast<tcflag_t>(speed);
    tty->c_ispeed = speed;
    tty->c_ospeed = speed;
#else
    throw_if_negative(cfsetispeed(tty, speed), "cfsetispeed");
    throw_if_negative(cfsetospeed(tty, speed), "cfsetospeed");
#endif
}

}  // namespace

SerialPort::SerialPort(std::string device, int baudrate, double timeout_s) {
    open(device, baudrate, timeout_s);
}

SerialPort::~SerialPort() {
    close();
}

SerialPort::SerialPort(SerialPort&& other) noexcept
    : fd_(other.fd_),
      baudrate_(other.baudrate_),
      timeout_s_(other.timeout_s_),
      device_(std::move(other.device_)) {
    other.fd_ = -1;
}

SerialPort& SerialPort::operator=(SerialPort&& other) noexcept {
    if (this == &other) {
        return *this;
    }
    close();
    fd_ = other.fd_;
    baudrate_ = other.baudrate_;
    timeout_s_ = other.timeout_s_;
    device_ = std::move(other.device_);
    other.fd_ = -1;
    return *this;
}

void SerialPort::open(const std::string& device, int baudrate, double timeout_s) {
    close();

    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
    if (fd_ < 0) {
        throw SerialError(errno_message("Cannot open serial port " + device));
    }

    termios tty{};
    throw_if_negative(tcgetattr(fd_, &tty), "tcgetattr");

    cfmakeraw(&tty);
    tty.c_cflag |= static_cast<tcflag_t>(CLOCAL | CREAD);
    tty.c_cflag &= static_cast<tcflag_t>(~HUPCL);
    tty.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    tty.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
    tty.c_cflag &= static_cast<tcflag_t>(~PARENB);
    tty.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    tty.c_cflag |= CS8;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 0;

    const speed_t speed = static_cast<speed_t>(resolve_baudrate(baudrate));
    set_serial_speed(&tty, speed);
    throw_if_negative(tcsetattr(fd_, TCSANOW, &tty), "tcsetattr");
    throw_if_negative(tcflush(fd_, TCIOFLUSH), "tcflush");

    device_ = device;
    baudrate_ = baudrate;
    timeout_s_ = timeout_s;
}

void SerialPort::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

void SerialPort::set_timeout(double timeout_s) {
    timeout_s_ = std::max(0.0, timeout_s);
}

void SerialPort::reset_input_buffer() {
    require_open();
    throw_if_negative(tcflush(fd_, TCIFLUSH), "tcflush(TCIFLUSH)");
}

void SerialPort::reset_output_buffer() {
    require_open();
    throw_if_negative(tcflush(fd_, TCOFLUSH), "tcflush(TCOFLUSH)");
}

void SerialPort::flush() {
    require_open();
    throw_if_negative(tcdrain(fd_), "tcdrain");
}

void SerialPort::set_dtr(bool asserted) {
    require_open();
    int bits = 0;
    throw_if_negative(ioctl(fd_, TIOCMGET, &bits), "ioctl(TIOCMGET)");
    if (asserted) {
        bits |= TIOCM_DTR;
    } else {
        bits &= ~TIOCM_DTR;
    }
    throw_if_negative(ioctl(fd_, TIOCMSET, &bits), "ioctl(TIOCMSET)");
}

void SerialPort::set_rts(bool asserted) {
    require_open();
    int bits = 0;
    throw_if_negative(ioctl(fd_, TIOCMGET, &bits), "ioctl(TIOCMGET)");
    if (asserted) {
        bits |= TIOCM_RTS;
    } else {
        bits &= ~TIOCM_RTS;
    }
    throw_if_negative(ioctl(fd_, TIOCMSET, &bits), "ioctl(TIOCMSET)");
}

std::size_t SerialPort::write(const std::uint8_t* data, std::size_t size) {
    require_open();
    if (data == nullptr || size == 0) {
        return 0;
    }

    std::size_t written = 0;
    while (written < size) {
        const ssize_t rc = ::write(fd_, data + written, size - written);
        if (rc < 0) {
            if (errno == EINTR) {
                continue;
            }
            throw SerialError(errno_message("write"));
        }
        written += static_cast<std::size_t>(rc);
    }
    return written;
}

std::size_t SerialPort::write(const std::vector<std::uint8_t>& data) {
    return write(data.data(), data.size());
}

std::size_t SerialPort::write_byte(std::uint8_t value) {
    return write(&value, 1);
}

std::size_t SerialPort::read(std::uint8_t* buffer, std::size_t size, double timeout_s) {
    require_open();
    if (buffer == nullptr || size == 0) {
        return 0;
    }

    const double effective_timeout = timeout_s >= 0.0 ? timeout_s : timeout_s_;
    pollfd pfd{};
    pfd.fd = fd_;
    pfd.events = POLLIN;

    const int poll_rc = ::poll(&pfd, 1, timeout_to_poll_ms(effective_timeout));
    if (poll_rc < 0) {
        if (errno == EINTR) {
            return 0;
        }
        throw SerialError(errno_message("poll"));
    }
    if (poll_rc == 0) {
        return 0;
    }

    const ssize_t rc = ::read(fd_, buffer, size);
    if (rc < 0) {
        if (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK) {
            return 0;
        }
        throw SerialError(errno_message("read"));
    }
    return static_cast<std::size_t>(rc);
}

std::vector<std::uint8_t> SerialPort::read(std::size_t size, double timeout_s) {
    std::vector<std::uint8_t> out(size);
    const std::size_t count = read(out.data(), out.size(), timeout_s);
    out.resize(count);
    return out;
}

std::vector<std::uint8_t> SerialPort::read_exact(std::size_t size, double timeout_s) {
    require_open();
    std::vector<std::uint8_t> out;
    out.reserve(size);

    const double effective_timeout = timeout_s >= 0.0 ? timeout_s : timeout_s_;
    const double deadline = monotonic_seconds() + std::max(0.01, effective_timeout);

    while (out.size() < size) {
        const double now = monotonic_seconds();
        if (now >= deadline) {
            std::ostringstream oss;
            oss << "Timeout reading " << size << " bytes (got " << out.size() << ")";
            throw SerialError(oss.str());
        }

        const double remaining = deadline - now;
        std::uint8_t chunk[256];
        const std::size_t want = std::min<std::size_t>(sizeof(chunk), size - out.size());
        const std::size_t got = read(chunk, want, remaining);
        if (got == 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        out.insert(out.end(), chunk, chunk + got);
    }

    return out;
}

std::size_t SerialPort::bytes_available() const {
    require_open();
    int available = 0;
    throw_if_negative(ioctl(fd_, FIONREAD, &available), "ioctl(FIONREAD)");
    return static_cast<std::size_t>(std::max(available, 0));
}

void SerialPort::require_open() const {
    if (!is_open()) {
        throw SerialError("Serial port not open");
    }
}

int SerialPort::timeout_to_poll_ms(double timeout_s) {
    if (timeout_s < 0.0) {
        return -1;
    }
    return static_cast<int>(std::llround(std::max(0.0, timeout_s) * 1000.0));
}

unsigned long SerialPort::resolve_baudrate(int baudrate) {
    switch (baudrate) {
        case 9600:
            return B9600;
        case 19200:
            return B19200;
        case 38400:
            return B38400;
        case 57600:
            return B57600;
        case 115200:
            return B115200;
        case 230400:
            return B230400;
        case 460800:
            return B460800;
        case 921600:
            return B921600;
        default:
            throw SerialError("Unsupported baudrate: " + std::to_string(baudrate));
    }
}

}  // namespace thesis_sim
