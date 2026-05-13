#pragma once
#include <string>
#include <cstdint>

class SerialPort {
public:
    SerialPort(const std::string& device, int baud = 1000000);
    ~SerialPort();

    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    SerialPort(SerialPort&& other) noexcept;
    SerialPort& operator=(SerialPort&& other) noexcept;

    void write(const uint8_t* data, size_t len);
    size_t read(uint8_t* buf, size_t max_len);
    void flush();
    bool is_open() const { return fd_ >= 0; }

private:
    int fd_ = -1;
    void set_baud(int baud);
};
