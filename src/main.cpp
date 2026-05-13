// ============================================================
// LEKIWI C++ WASD TELEOP — Main Entry Point
//
// This is the top-level program. The flow is:
//
//   1. Open serial port to motor driver board
//   2. Configure 3 STS3215 motors for velocity mode
//   3. Enter keyboard loop (WASD control)
//   4. On each keypress, compute inverse kinematics and send
//      velocity commands to all 3 wheels simultaneously
//   5. When no key pressed, send zero velocity (stop)
//   6. Clean up on quit
//
// Conceptually, the program is a simple control loop:
//
//   loop {
//     read keyboard
//     body_to_wheel(x, y, theta) -> [left, back, right]
//     sync_write(7: left, 8: back, 9: right)
//     sleep 50ms
//   }
//
// The 50ms loop rate = 20Hz control frequency. This is slower
// than the servos' internal PID loop (which runs at ~1kHz) but
// fast enough for human teleoperation. The watchdog in the
// lekiwi_host code would trigger at 500ms of no commands.
// ============================================================

#include "serial.hpp"
#include "feetech.hpp"
#include "kinematics.hpp"

#include <iostream>
#include <thread>
#include <chrono>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std::chrono_literals;  // for "50ms" syntax

int main(int argc, char** argv) {
    // Allow command-line override of serial port:
    //   ./lekiwi-wasd /dev/ttyACM0
    //   ./lekiwi-wasd /dev/ttyUSB0
    const char* device = argc > 1 ? argv[1] : "/dev/ttyACM0";

    try {
        // --- Step 1: Open serial port to motor controller ---
        // The constructor opens the port and configures it:
        //   1,000,000 baud, 8N1, raw mode, no flow control.
        // If the port doesn't exist or is busy, throws immediately.
        SerialPort port(device, 1000000);
        std::cout << "Connected to " << device << std::endl;

        // --- Step 2: Configure motors for velocity mode ---
        // On first power-up, STS3215s default to POSITION mode.
        // In position mode, Goal_Velocity writes are ignored.
        // We must switch each motor to VELOCITY mode, then
        // enable torque so they respond to commands.
        //
        // The sequence matters:
        //   disable_torque -> set mode -> enable_torque
        // If you set mode while torque is enabled, it may not
        // take effect until the next power cycle.
        int wheel_ids[] = {7, 8, 9};
        for (int id : wheel_ids) {
            set_op_mode(port, id, OpMode::Velocity);
            enable_torque(port, id, true);
            std::cout << "Motor " << id << ": ready" << std::endl;
        }
        // Flush any stale bytes from the serial buffer before
        // entering the control loop. Without this, we might read
        // leftover response bytes from motor configuration.
        port.flush();
        std::this_thread::sleep_for(50ms);

        // --- Step 3: Set up keyboard for raw input ---
        // By default, the terminal buffers input line-by-line
        // (canonical mode). We need character-by-character input
        // so WASD works instantly without pressing Enter.
        //
        // termios struct holds all terminal settings.
        // We save the old settings to restore on quit.
        struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);  // save current settings
        newt = oldt;
        // Disable canonical mode (ICANON), echo (ECHO), and
        // signal generation (ISIG = no Ctrl-C, Ctrl-Z, etc.)
        newt.c_lflag &= ~(ICANON | ECHO | ISIG);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        // Set stdin to non-blocking mode.
        // Without this, read() blocks until a key is pressed.
        // Non-blocking means read() returns 0 immediately when
        // no key is down — we use this to send "stop" commands
        // when the user isn't pressing anything (dead man's switch).
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

        // Initial speed: 0.25 m/s (~0.9 km/h).
        // This is the "medium" speed from LeRobot docs.
        double speed = 0.25;  // meters/second

        std::cout << "\n=== WASD DRIVE TEST (C++20) ===" << std::endl;
        std::cout << "  w/s = forward/back (x axis)" << std::endl;
        std::cout << "  a/d = strafe left/right (y axis)" << std::endl;
        std::cout << "  z/x = rotate left/right (theta axis)" << std::endl;
        std::cout << "  r/f = speed up/down (0.05-0.5 m/s)" << std::endl;
        std::cout << "  q = quit" << std::endl;
        std::cout << "================================" << std::endl;

        // --- Step 4: Main control loop ---
        // Reads keyboard at ~20Hz (50ms per iteration).
        // When a key is pressed, computes the 3 wheel velocities
        // via inverse kinematics and sends them in one sync write.
        // When no key is pressed, sends zero velocity (robot stops).
        //
        // This is called "dead man's switch" behavior — the robot
        // only moves while you hold a key. If the program crashes
        // or loses keyboard focus, the robot stops automatically
        // on the next loop iteration.
        bool running = true;
        while (running) {
            char c;
            if (read(STDIN_FILENO, &c, 1) > 0) {
                // --- Key pressed: compute desired motion ---
                std::array<int32_t, 3> cmd{0, 0, 0};
                switch (c) {
                    case 'q': running = false; break;
                    case 'w': cmd = body_to_wheel( speed, 0, 0); break;  // forward
                    case 's': cmd = body_to_wheel(-speed, 0, 0); break;  // backward
                    case 'a': cmd = body_to_wheel(0,  speed, 0); break;  // strafe left
                    case 'd': cmd = body_to_wheel(0, -speed, 0); break;  // strafe right
                    case 'z': cmd = body_to_wheel(0, 0,  90); break;     // rotate left (deg/s)
                    case 'x': cmd = body_to_wheel(0, 0, -90); break;     // rotate right (deg/s)
                    case 'r': speed = std::min(speed + 0.1, 0.5);
                              std::cout << "speed: " << speed << " m/s" << std::endl;
                              break;
                    case 'f': speed = std::max(speed - 0.1, 0.05);
                              std::cout << "speed: " << speed << " m/s" << std::endl;
                              break;
                    default: break;
                }
                // Send velocity if it was a movement key
                if (c == 'w' || c == 's' || c == 'a' || c == 'd' || c == 'z' || c == 'x') {
                    std::unordered_map<uint8_t, int32_t> vm;
                    vm[7] = cmd[0];  // left wheel
                    vm[8] = cmd[1];  // back wheel
                    vm[9] = cmd[2];  // right wheel
                    sync_write_goal_velocities(port, vm);
                } else if (c == 'q') {
                    // Stop motors before exiting
                    std::unordered_map<uint8_t, int32_t> stop;
                    stop[7] = 0; stop[8] = 0; stop[9] = 0;
                    sync_write_goal_velocities(port, stop);
                }
            } else {
                // --- No key pressed: send stop ---
                // This is the dead man's switch. Every 50ms that
                // no key is held, we send zero velocity. If the
                // cable disconnects or the process hangs, the
                // motors stop on the last command (the servo
                // maintains its current speed until told otherwise,
                // so this loop is essential for safety).
                std::unordered_map<uint8_t, int32_t> stop;
                stop[7] = 0; stop[8] = 0; stop[9] = 0;
                sync_write_goal_velocities(port, stop);
            }
            // 50ms = 20Hz loop rate. Fast enough for responsive
            // teleop, slow enough to not flood the serial bus.
            std::this_thread::sleep_for(50ms);
        }

        // --- Step 5: Clean shutdown ---
        std::unordered_map<uint8_t, int32_t> stop;
        stop[7] = 0; stop[8] = 0; stop[9] = 0;
        sync_write_goal_velocities(port, stop);
        std::this_thread::sleep_for(100ms);  // let motors stop

        // Restore terminal to its original settings
        // (canonical mode, echo enabled, etc.)
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
