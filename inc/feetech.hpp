#pragma once
#include "serial.hpp"
#include <cstdint>
#include <array>
#include <unordered_map>

namespace FeetechReg {
    constexpr uint8_t ID                  = 5;
    constexpr uint8_t Operating_Mode      = 33;
    constexpr uint8_t Goal_Velocity_L     = 46;
    constexpr uint8_t Present_Velocity_L  = 38;
    constexpr uint8_t Present_Position_L  = 36;
    constexpr uint8_t Torque_Enable       = 40;
}

constexpr uint16_t STS3215_MODEL = 777;

enum class OpMode : uint8_t {
    Position = 0,
    Velocity = 1,
    PWM      = 2,
    Step     = 3,
};

uint8_t feetech_checksum(const uint8_t* pkt, size_t len);

void feetech_write_reg(SerialPort& port, uint8_t id, uint8_t addr,
                        const uint8_t* data, size_t len);

void sync_write_goal_velocities(SerialPort& port,
    const std::unordered_map<uint8_t, int32_t>& velocities);

void set_op_mode(SerialPort& port, uint8_t id, OpMode mode);
void enable_torque(SerialPort& port, uint8_t id, bool on);
void write_goal_velocity(SerialPort& port, uint8_t id, int32_t raw);
