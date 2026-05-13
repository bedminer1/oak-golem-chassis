#include "feetech.hpp"

uint8_t feetech_checksum(const uint8_t* pkt, size_t len) {
    uint8_t sum = 0;
    for (size_t i = 2; i < len; ++i) sum += pkt[i];
    return ~sum;
}

void feetech_write_reg(SerialPort& port, uint8_t id, uint8_t addr,
                        const uint8_t* data, size_t len) {
    uint8_t pkt[256];
    size_t i = 0;
    pkt[i++] = 0xFF;
    pkt[i++] = 0xFF;
    pkt[i++] = id;
    pkt[i++] = 3 + len;
    pkt[i++] = 0x03;
    pkt[i++] = addr;
    for (size_t j = 0; j < len; ++j) pkt[i++] = data[j];
    pkt[i] = feetech_checksum(pkt, i);
    port.write(pkt, i + 1);
}

void sync_write_goal_velocities(SerialPort& port,
    const std::unordered_map<uint8_t, int32_t>& velocities) {
    uint8_t addr = FeetechReg::Goal_Velocity_L;
    uint8_t data_len = 4;
    size_t n = velocities.size();
    uint8_t pkt[256];
    size_t i = 0;
    pkt[i++] = 0xFF;
    pkt[i++] = 0xFF;
    pkt[i++] = 0xFE;
    pkt[i++] = 2 + (1 + data_len) * n;
    pkt[i++] = 0x83;
    pkt[i++] = addr;
    pkt[i++] = data_len;
    for (auto item : velocities) {
        uint8_t id = item.first;
        int32_t raw = item.second;
        pkt[i++] = id;
        pkt[i++] = raw & 0xFF;
        pkt[i++] = (raw >> 8) & 0xFF;
        pkt[i++] = (raw >> 16) & 0xFF;
        pkt[i++] = (raw >> 24) & 0xFF;
    }
    pkt[i] = feetech_checksum(pkt, i);
    port.write(pkt, i + 1);
}

void set_op_mode(SerialPort& port, uint8_t id, OpMode mode) {
    uint8_t v = static_cast<uint8_t>(mode);
    feetech_write_reg(port, id, FeetechReg::Operating_Mode, &v, 1);
}

void enable_torque(SerialPort& port, uint8_t id, bool on) {
    uint8_t v = on ? 1 : 0;
    feetech_write_reg(port, id, FeetechReg::Torque_Enable, &v, 1);
}

void write_goal_velocity(SerialPort& port, uint8_t id, int32_t raw) {
    uint8_t data[4];
    data[0] = raw & 0xFF;
    data[1] = (raw >> 8) & 0xFF;
    data[2] = (raw >> 16) & 0xFF;
    data[3] = (raw >> 24) & 0xFF;
    feetech_write_reg(port, id, FeetechReg::Goal_Velocity_L, data, 4);
}
