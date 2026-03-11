#pragma once

#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace thesis_sim {

class SerialError : public std::runtime_error {
  public:
    using std::runtime_error::runtime_error;
};

class SerialPort {
  public:
    SerialPort() = default;
    SerialPort(std::string device, int baudrate = 115200, double timeout_s = 1.0);
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    SerialPort(SerialPort&& other) noexcept;
    SerialPort& operator=(SerialPort&& other) noexcept;

    void open(const std::string& device, int baudrate = 115200, double timeout_s = 1.0);
    void close();

    bool is_open() const { return fd_ >= 0; }
    const std::string& device() const { return device_; }
    int baudrate() const { return baudrate_; }
    double timeout() const { return timeout_s_; }

    void set_timeout(double timeout_s);
    void reset_input_buffer();
    void reset_output_buffer();
    void flush();

    void set_dtr(bool asserted);
    void set_rts(bool asserted);

    std::size_t write(const std::uint8_t* data, std::size_t size);
    std::size_t write(const std::vector<std::uint8_t>& data);
    std::size_t write_byte(std::uint8_t value);

    std::size_t read(std::uint8_t* buffer, std::size_t size, double timeout_s = -1.0);
    std::vector<std::uint8_t> read(std::size_t size, double timeout_s = -1.0);
    std::vector<std::uint8_t> read_exact(std::size_t size, double timeout_s = -1.0);

    std::size_t bytes_available() const;

  private:
    void require_open() const;
    static int timeout_to_poll_ms(double timeout_s);
    static unsigned long resolve_baudrate(int baudrate);

    int fd_ = -1;
    int baudrate_ = 115200;
    double timeout_s_ = 1.0;
    std::string device_;
};

}  // namespace thesis_sim
