#include "serial.hpp"
#include "feetech.hpp"
#include "kinematics.hpp"

#include <iostream>
#include <thread>
#include <chrono>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace std::chrono_literals;

int main(int argc, char** argv) {
    const char* device = argc > 1 ? argv[1] : "/dev/ttyACM0";
    try {
        SerialPort port(device, 1000000);
        std::cout << "Connected to " << device << std::endl;

        int wheel_ids[] = {7, 8, 9};
        for (int id : wheel_ids) {
            set_op_mode(port, id, OpMode::Velocity);
            enable_torque(port, id, true);
            std::cout << "Motor " << id << ": ready" << std::endl;
        }
        port.flush();
        std::this_thread::sleep_for(50ms);

        struct termios oldt, newt;
        tcgetattr(STDIN_FILENO, &oldt);
        newt = oldt;
        newt.c_lflag &= ~(ICANON | ECHO | ISIG);
        tcsetattr(STDIN_FILENO, TCSANOW, &newt);

        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);

        double speed = 0.25;
        std::cout << "\n=== WASD DRIVE TEST (C++20) ===" << std::endl;
        std::cout << "  w/s = fwd/back   a/d = strafe" << std::endl;
        std::cout << "  z/x = rotate     r/f = speed" << std::endl;
        std::cout << "  q = quit" << std::endl;
        std::cout << "================================" << std::endl;

        bool running = true;
        while (running) {
            char c;
            if (read(STDIN_FILENO, &c, 1) > 0) {
                std::array<int32_t, 3> cmd{0, 0, 0};
                switch (c) {
                    case 'q': running = false; break;
                    case 'w': cmd = body_to_wheel(speed, 0, 0); break;
                    case 's': cmd = body_to_wheel(-speed, 0, 0); break;
                    case 'a': cmd = body_to_wheel(0, speed, 0); break;
                    case 'd': cmd = body_to_wheel(0, -speed, 0); break;
                    case 'z': cmd = body_to_wheel(0, 0, 90); break;
                    case 'x': cmd = body_to_wheel(0, 0, -90); break;
                    case 'r': speed = std::min(speed + 0.1, 0.5);
                              std::cout << "speed: " << speed << " m/s" << std::endl;
                              break;
                    case 'f': speed = std::max(speed - 0.1, 0.05);
                              std::cout << "speed: " << speed << " m/s" << std::endl;
                              break;
                    default: break;
                }
                if (c == 'w' || c == 's' || c == 'a' || c == 'd' || c == 'z' || c == 'x') {
                    std::unordered_map<uint8_t, int32_t> vm;
                    vm[7] = cmd[0]; vm[8] = cmd[1]; vm[9] = cmd[2];
                    sync_write_goal_velocities(port, vm);
                } else if (c == 'q') {
                    std::unordered_map<uint8_t, int32_t> stop;
                    stop[7] = 0; stop[8] = 0; stop[9] = 0;
                    sync_write_goal_velocities(port, stop);
                }
            } else {
                std::unordered_map<uint8_t, int32_t> stop;
                stop[7] = 0; stop[8] = 0; stop[9] = 0;
                sync_write_goal_velocities(port, stop);
            }
            std::this_thread::sleep_for(50ms);
        }

        std::unordered_map<uint8_t, int32_t> stop;
        stop[7] = 0; stop[8] = 0; stop[9] = 0;
        sync_write_goal_velocities(port, stop);
        std::this_thread::sleep_for(100ms);
        tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    return 0;
}
