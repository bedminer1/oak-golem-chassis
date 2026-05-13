#include "serial.hpp"
#include <cstring>
#include <stdexcept>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>

// ============================================================
// SERIAL PORT IMPLEMENTATION
//
// Linux serial ports are accessed through the POSIX terminal
// interface (termios). Key concepts:
//
//   - O_RDWR | O_NOCTTY : open read-write, don't become
//     the process's controlling terminal
//   - termios struct : holds all serial parameters
//   - c_cflag : control flags (baud, parity, stop bits)
//   - c_lflag : local flags (echo, canonical mode)
//   - c_iflag : input flags (flow control, newline mapping)
//   - c_oflag : output flags (raw vs processed output)
//   - c_cc[VMIN] : minimum bytes before read() returns
//   - c_cc[VTIME] : timeout in deciseconds
//
// The STS3215 servos use:
//   1,000,000 baud (1 Mbps)
//   8 data bits, no parity, 1 stop bit (8N1)
//   No hardware or software flow control
//   Raw mode (no line buffering, no echo)
// ============================================================

SerialPort::SerialPort(const std::string& device, int baud) {
    // Open the serial port device file.
    // O_RDWR = read and write
    // O_NOCTTY = don't make this the controlling terminal
    //   (important: without this, signals like CTRL-C could
    //    get sent to your robot process!)
    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0)
        throw std::runtime_error("Failed to open " + device);
    set_baud(baud);
}

SerialPort::~SerialPort() {
    if (fd_ >= 0) ::close(fd_);
}

// Move constructor: steal the fd, null out the source.
SerialPort::SerialPort(SerialPort&& other) noexcept
    : fd_(other.fd_) { other.fd_ = -1; }

// Move assignment: close our old fd if open, steal theirs.
SerialPort& SerialPort::operator=(SerialPort&& other) noexcept {
    if (this != &other) {
        if (fd_ >= 0) ::close(fd_);
        fd_ = other.fd_;
        other.fd_ = -1;
    }
    return *this;
}

// Write len bytes to the serial port.
// Throws if the write fails (e.g. device disconnected).
void SerialPort::write(const uint8_t* data, size_t len) {
    if (::write(fd_, data, len) < 0)
        throw std::runtime_error("Serial write failed");
}

// Read up to max_len bytes from the serial port.
// Returns the number of bytes actually read.
// With VMIN=1 and VTIME=10, this blocks until at least 1 byte
// arrives OR 1 second passes (10 deciseconds).
size_t SerialPort::read(uint8_t* buf, size_t max_len) {
    auto n = ::read(fd_, buf, max_len);
    if (n < 0) throw std::runtime_error("Serial read failed");
    return n;
}

// Discard any buffered data in both directions.
// Useful before sending a command so stale data doesn't
// corrupt the response parsing.
void SerialPort::flush() {
    tcflush(fd_, TCIOFLUSH);
}

// ============================================================
// BAUD RATE LOOKUP
//
// The termios API uses speed_t enum values, not raw integers.
// This table converts common baud rates to their termios codes.
// ============================================================
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

// ============================================================
// SERIAL PORT CONFIGURATION
//
// Sets up the serial port for the STS3215 protocol:
//   1,000,000 baud, 8N1, no flow control, raw mode.
//
// cflag settings explained:
//   PARENB=0  : no parity bit
//   CSTOPB=0  : 1 stop bit (not 2)
//   CS8       : 8 data bits
//   CRTSCTS=0 : no hardware flow control
//   CREAD     : enable receiver
//   CLOCAL    : ignore modem control lines
//
// lflag settings (local mode):
//   ICANON=0  : non-canonical (raw input, no line buffering)
//   ECHO=0    : don't echo sent characters back
//   ECHOE=0   : don't echo erase characters
//   ISIG=0    : don't generate signals (INTR, QUIT, SUSP)
//
// iflag settings (input processing):
//   IXON/OFF/ANY=0 : no software flow control (XON/XOFF)
//   INLCR/ICRNL/IGNCR=0 : don't translate newlines
//
// oflag settings (output processing):
//   OPOST=0  : raw output (no NL-to-CR-NL translation)
//
// VMIN and VTIME control read() behavior:
//   VMIN=1  : return at least 1 byte
//   VTIME=10 : wait up to 1 second (10 * 0.1s) if no data
// ============================================================
void SerialPort::set_baud(int baud) {
    struct termios tty{};        // zero-initialized
    tcgetattr(fd_, &tty);        // read current settings

    // Set both input and output baud rates
    cfsetospeed(&tty, to_termios_baud(baud));
    cfsetispeed(&tty, to_termios_baud(baud));

    // --- Control flags: 8N1, no flow control ---
    tty.c_cflag &= ~PARENB;      // no parity bit
    tty.c_cflag &= ~CSTOPB;      // 1 stop bit
    tty.c_cflag |= CS8;          // 8 data bits
    tty.c_cflag &= ~CRTSCTS;     // no hardware flow control
    tty.c_cflag |= CREAD;        // enable receiver
    tty.c_cflag |= CLOCAL;       // ignore modem lines

    // --- Local flags: raw mode ---
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    // --- Input flags: no software flow control, no NL mapping ---
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_iflag &= ~(INLCR | ICRNL | IGNCR);

    // --- Output flags: raw output ---
    tty.c_oflag &= ~OPOST;

    // --- Read behavior: return 1 byte minimum, 1 sec timeout ---
    tty.c_cc[VMIN] = 1;          // min bytes for read
    tty.c_cc[VTIME] = 10;        // timeout in deciseconds (10 = 1s)

    // Apply all settings immediately
    tcsetattr(fd_, TCSANOW, &tty);
}
