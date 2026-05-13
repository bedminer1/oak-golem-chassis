#pragma once
#include <string>
#include <cstdint>

// ============================================================
// SERIAL PORT WRAPPER (RAII)
//
// RAII = Resource Acquisition Is Initialization. It means the
// serial port file descriptor is opened in the constructor and
// closed in the destructor. You can't forget to close it.
//
// Copy is deleted because copying a file descriptor is ambiguous
// (who closes it?). Move transfers ownership cleanly.
//
// This wraps Linux's POSIX serial API (/dev/ttyACM0) into a
// simple C++ class with read/write/flush.
// ============================================================

class SerialPort {
public:
    // Opens device (e.g. "/dev/ttyACM0") at given baud rate.
    // Default baud is 1,000,000 (1Mbps) for STS3215 servos.
    SerialPort(const std::string& device, int baud = 1000000);

    // Closes the file descriptor automatically.
    ~SerialPort();

    // --- No copy ---
    // If we copied this, both copies would share the same fd.
    // The first one destroyed would close it, leaving the other
    // with a dangling fd (use-after-close bug).
    SerialPort(const SerialPort&) = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    // --- Move semantics ---
    // Moving transfers the fd to the new owner and nulls out
    // the old one. Safe way to pass SerialPort around.
    SerialPort(SerialPort&& other) noexcept;
    SerialPort& operator=(SerialPort&& other) noexcept;

    // Write raw bytes. Throws on failure.
    void write(const uint8_t* data, size_t len);

    // Read up to max_len bytes. Returns bytes actually read.
    // Blocks until at least 1 byte arrives (VMIN=1).
    size_t read(uint8_t* buf, size_t max_len);

    // Discard any buffered data in both directions.
    void flush();

    bool is_open() const { return fd_ >= 0; }

private:
    int fd_ = -1;  // File descriptor, -1 means "not open"

    // Configure the serial port parameters (baud, 8N1, raw mode).
    void set_baud(int baud);
};
