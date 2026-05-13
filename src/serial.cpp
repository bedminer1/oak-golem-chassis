#include "serial.hpp"
#include <cstring>
#include <stdexcept>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

SerialPort::SerialPort(const std::string& device, int baud) {
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0)
        throw std::runtime_error("Failed to open " + device);
    set_baud(baud);
}

SerialPort::~SerialPort() {
    if (fd_ >= 0) ::close(fd_);
}

SerialPort::SerialPort(SerialPort&& other) noexcept
    : fd_(other.fd_) { other.fd_ = -1; }

SerialPort& SerialPort::operator=(SerialPort&& other) noexcept {
    if (this != &other) {
        if (fd_ >= 0) ::close(fd_);
        fd_ = other.fd_;
        other.fd_ = -1;
    }
    return *this;
}

void SerialPort::write(const uint8_t* data, size_t len) {
    if (::write(fd_, data, len) < 0)
        throw std::runtime_error("Serial write failed");
}

size_t SerialPort::read(uint8_t* buf, size_t max_len) {
    auto n = ::read(fd_, buf, max_len);
    if (n < 0) throw std::runtime_error("Serial read failed");
    return n;
}

void SerialPort::flush() {
    tcflush(fd_, TCIOFLUSH);
}

static speed_t to_termios_baud(int baud) {
    switch (baud) {
        case 9600:    return B9600;
        case 19200:   return B19200;
        case 38400:   return B38400;
        case 57600:   return B57600;
        case 115200:  return B115200;
        case 1000000: return B1000000;
        default:      throw std::runtime_error("Unsupported baud rate");
    }
}

void SerialPort::set_baud(int baud) {
    struct termios tty{};
    tcgetattr(fd_, &tty);
    cfsetospeed(&tty, to_termios_baud(baud));
    cfsetispeed(&tty, to_termios_baud(baud));
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |= CREAD | CLOCAL;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(INLCR | ICRNL | IGNCR);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 10;
    tcsetattr(fd_, TCSANOW, &tty);
}
