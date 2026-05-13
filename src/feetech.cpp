#include "feetech.hpp"
#include <vector>

// ============================================================
// FEETECH PACKET ENCODING
//
// Every command to a servo is a packet:
//
//   Byte 0:    0xFF       (header)
//   Byte 1:    0xFF       (header)
//   Byte 2:    ID         (target motor, 0xFE = broadcast)
//   Byte 3:    LENGTH     (bytes after this: instruction + params + checksum)
//   Byte 4:    INSTRUCTION (0x02=read, 0x03=write, 0x83=sync)
//   Byte 5..:  PARAMS     (depends on instruction)
//   Last byte: CHECKSUM   (~(sum of bytes 2..n-1) & 0xFF)
//
// The checksum is the bitwise NOT (~) of the sum of bytes
// starting from the ID byte. If the sum overflows 8 bits,
// only the lowest 8 bits matter (checksum catches corruption
// from electrical noise on the bus cable).
// ============================================================

// Checksum: XOR-style verification.
// Walk bytes from index 2 (ID) to len-1, add them up, return ~sum.
uint8_t feetech_checksum(const uint8_t* pkt, size_t len) {
    uint8_t sum = 0;
    for (size_t i = 2; i < len; ++i) sum += pkt[i];
    return ~sum;  // bitwise NOT: 0x00 -> 0xFF, 0xAB -> 0x54, etc.
}

// ============================================================
// WRITE REGISTER (Single Motor)
//
// Instruction: 0x03 (WRITE)
// Packet layout:
//   0xFF 0xFF [ID] [LEN=3+N] 0x03 [ADDR] [DATA0..DATAN] [CHECKSUM]
//
// Where N = number of data bytes (1 for Operating_Mode/Torque,
// 4 for Goal_Velocity).
//
// The servo receives this, writes the value to its control table
// at the given address, and sends back a status packet.
// ============================================================
void feetech_write_reg(SerialPort& port, uint8_t id, uint8_t addr,
                        const uint8_t* data, size_t len) {
    uint8_t pkt[256];  // Feetech max packet size
    size_t i = 0;

    // Header
    pkt[i++] = 0xFF;
    pkt[i++] = 0xFF;

    // ID + Length + Instruction
    pkt[i++] = id;          // target motor
    pkt[i++] = 3 + len;     // instr(1) + addr(1) + data(N)
    pkt[i++] = 0x03;        // WRITE instruction

    // Register address + data
    pkt[i++] = addr;
    for (size_t j = 0; j < len; ++j) pkt[i++] = data[j];

    // Checksum covers everything from ID byte onwards
    pkt[i] = feetech_checksum(pkt, i);
    port.write(pkt, i + 1);
}

// ============================================================
// SYNC WRITE (Multiple Motors, One Packet)
//
// Instruction: 0x83 (SYNC WRITE)
// Packet layout:
//   0xFF 0xFF 0xFE [LEN] 0x83 [ADDR] [DATA_LEN]
//     [ID1] [D0..D3] [ID2] [D0..D3] [ID3] [D0..D3]
//   [CHECKSUM]
//
// Key differences from single write:
//   - ID is always 0xFE (broadcast — all motors receive it)
//   - Each motor's data is prefixed by its unique ID
//   - Motors only respond (or execute) if their ID matches
//   - All motors receive the command simultaneously
//
// WHY SYNC WRITE MATTERS:
// If we sent 3 individual write packets, motor 7 would start
// moving before motor 9 even received its command (about 15ms
// total at 1Mbps). Sync write delivers all 3 commands in one
// ~200us burst — all wheels start at the same time.
// ============================================================
void sync_write_goal_velocities(SerialPort& port,
    const std::unordered_map<uint8_t, int32_t>& velocities) {

    uint8_t addr = FeetechReg::Goal_Velocity_L;  // 46 = 0x2E
    uint8_t data_len = 4;                          // 4 bytes per value
    size_t n = velocities.size();

    uint8_t pkt[256];
    size_t i = 0;

    // Header (broadcast on 0xFE)
    pkt[i++] = 0xFF;
    pkt[i++] = 0xFF;
    pkt[i++] = 0xFE;  // 0xFE = all motors on bus

    // Length = addr(1) + data_len(1) + N * (ID(1) + data_len(N))
    pkt[i++] = 2 + (1 + data_len) * n;

    // Instruction + base address + data length per motor
    pkt[i++] = 0x83;    // SYNC WRITE
    pkt[i++] = addr;
    pkt[i++] = data_len;

    // For each motor: [ID] [4 bytes little-endian velocity]
    for (auto item : velocities) {
        uint8_t id = item.first;
        int32_t raw = item.second;
        pkt[i++] = id;
        // Little-endian: least significant byte first
        pkt[i++] = raw & 0xFF;           // byte 0 (LSB)
        pkt[i++] = (raw >> 8) & 0xFF;    // byte 1
        pkt[i++] = (raw >> 16) & 0xFF;   // byte 2
        pkt[i++] = (raw >> 24) & 0xFF;   // byte 3 (MSB)
    }

    pkt[i] = feetech_checksum(pkt, i);
    port.write(pkt, i + 1);
}

// ============================================================
// HIGH-LEVEL HELPERS
// ============================================================

// Set operating mode (POSITION=0, VELOCITY=1).
// MUST be called while torque is disabled, or it won't take effect.
// The mode determines which register the servo responds to:
//   Position mode: Goal_Position (addr 42)
//   Velocity mode: Goal_Velocity (addr 46)
void set_op_mode(SerialPort& port, uint8_t id, OpMode mode) {
    uint8_t v = static_cast<uint8_t>(mode);
    feetech_write_reg(port, id, FeetechReg::Operating_Mode, &v, 1);
}

// Enable (true) or disable (false) torque.
// Torque ON = servo actively holds position or follows velocity.
// Torque OFF = servo free-spins, can be turned by hand.
void enable_torque(SerialPort& port, uint8_t id, bool on) {
    uint8_t v = on ? 1 : 0;
    feetech_write_reg(port, id, FeetechReg::Torque_Enable, &v, 1);
}

// Write velocity command to a single motor.
// raw: signed int32, typically -3000 to +3000.
// Positive = one direction, negative = opposite.
// Zero = stop (but torque stays enabled — servo resists movement).
void write_goal_velocity(SerialPort& port, uint8_t id, int32_t raw) {
    uint8_t data[4];
    data[0] = raw & 0xFF;
    data[1] = (raw >> 8) & 0xFF;
    data[2] = (raw >> 16) & 0xFF;
    data[3] = (raw >> 24) & 0xFF;
    feetech_write_reg(port, id, FeetechReg::Goal_Velocity_L, data, 4);
}
