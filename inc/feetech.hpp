#pragma once
#include "serial.hpp"
#include <cstdint>
#include <array>
#include <unordered_map>

// ============================================================
// FEETECH STS3215 PROTOCOL
//
// The STS3215 is a serial bus servo from Feetech (also sold as
// "SCServo"). Multiple servos share the same 2-wire bus, each
// identified by a unique ID (1-254).
//
// Packet format (all multi-byte values are little-endian):
//
//   [0xFF] [0xFF] [ID] [LENGTH] [INSTRUCTION] [PARAMS...] [CHECKSUM]
//
// Header: 0xFF 0xFF marks the start of a packet
// ID: target servo ID (0xFE = broadcast, 0xFB = all except one)
// LENGTH: number of bytes after this field (params + checksum)
// INSTRUCTION: what to do (0x02=read, 0x03=write, 0x83=sync write)
// PARAMS: depend on instruction
// CHECKSUM: ~(ID + LENGTH + INSTRUCTION + sum(PARAMS)) & 0xFF
//
// Register map (addresses in the STS3215 control table):
//   Address 5  (0x05) : ID                    (read/write)
//   Address 33 (0x21) : Operating_Mode        (read/write)
//      0 = Position mode (default)
//      1 = Velocity mode (what we use for wheels)
//      2 = PWM mode
//      3 = Step mode
//   Address 36 (0x24) : Present_Position      (read, 4 bytes)
//   Address 38 (0x26) : Present_Speed         (read, 4 bytes)
//   Address 40 (0x28) : Torque_Enable         (read/write)
//   Address 46 (0x2E) : Goal_Velocity         (write, 4 bytes)
//
// Model number for STS3215: 777 (0x0309)
// Communication: 1,000,000 baud, 8N1
// Position resolution: 4096 steps per revolution (0.088 deg/step)
// Velocity unit: steps per second (max ~32,767 signed)
// ============================================================

// Register addresses for the STS3215 control table.
// These are the "memory map" of the servo — you read and write
// to these addresses to control or query the motor.
namespace FeetechReg {
    constexpr uint8_t ID                  = 5;    // 0x05
    constexpr uint8_t Operating_Mode      = 33;   // 0x21
    constexpr uint8_t Goal_Velocity_L     = 46;   // 0x2E (lower byte addr, 4 bytes total)
    constexpr uint8_t Present_Velocity_L  = 38;   // 0x26
    constexpr uint8_t Present_Position_L  = 36;   // 0x24
    constexpr uint8_t Torque_Enable       = 40;   // 0x28
}

// STS3215 model number as reported by the servo
constexpr uint16_t STS3215_MODEL = 777;

// Operating modes for the STS3215.
// Wheels use VELOCITY — you set a target speed and the servo
// maintains it (closed-loop). Arm joints use POSITION.
enum class OpMode : uint8_t {
    Position = 0,
    Velocity = 1,
    PWM      = 2,
    Step     = 3,
};

// ---- Low-level functions (packet encoding) ----

// Calculate Feetech checksum: bitwise NOT of the sum
// of all bytes starting from ID (index 2).
// The checksum is the last byte of every packet.
uint8_t feetech_checksum(const uint8_t* pkt, size_t len);

// Write data to a single servo's register.
// Used for: set operating mode, enable torque, set single velocity.
void feetech_write_reg(SerialPort& port, uint8_t id, uint8_t addr,
                        const uint8_t* data, size_t len);

// Sync write: send velocity commands to multiple motors in ONE packet.
// Much more efficient than individual writes — all motors receive
// the command simultaneously on the bus.
//
// velocities: map of motor_id -> raw velocity value (int32_t)
void sync_write_goal_velocities(SerialPort& port,
    const std::unordered_map<uint8_t, int32_t>& velocities);

// ---- High-level helpers ----

// Set a servo to position or velocity mode.
// Must be called with torque disabled, before enabling torque.
void set_op_mode(SerialPort& port, uint8_t id, OpMode mode);

// Enable or disable torque (motor power).
// Disabled: motor free-spins (can be moved by hand).
// Enabled: motor holds position or follows velocity command.
void enable_torque(SerialPort& port, uint8_t id, bool on);

// Send velocity command to a single motor.
// raw: signed 32-bit, typically -3000 to +3000 for safety.
void write_goal_velocity(SerialPort& port, uint8_t id, int32_t raw);
