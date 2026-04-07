/*
 * ibis-ssl binary protocol constants, data types, and packet functions.
 *
 * Ported from ibis-ssl/grSim fork (ibis_command_receiver.cpp / binary_feedback_sender.cpp).
 * No grSim-specific dependencies -- uses only standard C++ and stdint.
 */

#pragma once

#include <cmath>
#include <cstring>
#include <cstdint>

// ---------------------------------------------------------------------------
// Protocol constants
// ---------------------------------------------------------------------------

constexpr int IBIS_ROBOT_SLOTS   = 11;
constexpr int IBIS_CMD_SIZE      = 64;
constexpr int IBIS_SLOT_SIZE     = IBIS_CMD_SIZE + 1;
constexpr int IBIS_PACKET_SIZE   = IBIS_SLOT_SIZE * IBIS_ROBOT_SLOTS; // 715

constexpr int    IBIS_DEFAULT_PORT       = 12345;
constexpr double IBIS_CHIP_ANGLE_DEG     = 30.0;
constexpr double IBIS_CHIP_ANGLE_RAD     = IBIS_CHIP_ANGLE_DEG * M_PI / 180.0;
constexpr double IBIS_MAX_KICK_SPEED     = 8.0;  // m/s
constexpr double IBIS_THETA_P_GAIN       = 4.0;
constexpr double IBIS_DT                 = 1.0 / 30.0;
constexpr int    IBIS_FEEDBACK_SIZE      = 128;
constexpr int    IBIS_FEEDBACK_PORT_BASE = 50100;
constexpr double IBIS_POSITION_MATCH_THRESHOLD = 0.5; // metres

// ---------------------------------------------------------------------------
// Byte offsets in the 64-byte RobotCommandSerializedV2 (from crane's robot_packet.h)
// ---------------------------------------------------------------------------

enum IbisAddress {
    HEADER              = 0,
    CHECK_COUNTER       = 1,
    VISION_GLOBAL_X_H   = 2,
    VISION_GLOBAL_X_L   = 3,
    VISION_GLOBAL_Y_H   = 4,
    VISION_GLOBAL_Y_L   = 5,
    VISION_GLOBAL_TH_H  = 6,
    VISION_GLOBAL_TH_L  = 7,
    TARGET_GLOBAL_TH_H  = 8,
    TARGET_GLOBAL_TH_L  = 9,
    KICK_POWER          = 10,
    DRIBBLE_POWER       = 11,
    ACCEL_LIMIT_H       = 12,
    ACCEL_LIMIT_L       = 13,
    LINEAR_VEL_LIMIT_H  = 14,
    LINEAR_VEL_LIMIT_L  = 15,
    ANGULAR_VEL_LIMIT_H = 16,
    ANGULAR_VEL_LIMIT_L = 17,
    LATENCY_MS_H        = 18,
    LATENCY_MS_L        = 19,
    ELAPSED_VISION_H    = 20,
    ELAPSED_VISION_L    = 21,
    FLAGS               = 22,
    CONTROL_MODE        = 23,
    CONTROL_MODE_ARGS   = 24,
    // CONTROL_MODE_ARGS size = 8, args end at offset 31
    TARGET_POS_X_H      = 32,
    TARGET_POS_X_L      = 33,
    TARGET_POS_Y_H      = 34,
    TARGET_POS_Y_L      = 35,
    TERMINAL_VEL_H      = 36,
    TERMINAL_VEL_L      = 37,
};

enum IbisFlagBit {
    IS_VISION_AVAILABLE = 0,
    ENABLE_CHIP         = 1,
    STOP_EMERGENCY      = 3,
};

// ---------------------------------------------------------------------------
// Deserialized ibis command
// ---------------------------------------------------------------------------

struct IbisCommand {
    float   vision_global_pos[2];    // metres, SSL vision coordinate system
    float   target_global_theta;     // radians
    float   kick_power;              // 0..1 normalised
    float   dribble_power;           // 0..1 normalised
    bool    enable_chip;
    bool    stop_emergency;
    float   acceleration_limit;      // m/s^2 (0 means "use default")
    float   linear_velocity_limit;   // m/s   (0 means "no limit")
    float   angular_velocity_limit;  // rad/s
    float   polar_velocity_r;        // m/s
    float   polar_velocity_theta;    // radians (global direction)
    uint8_t check_counter;
};

// Cached SSL vision state for one robot (position in mm, orientation in rad).
struct IbisVisionState {
    float x_mm          = 0.0f;
    float y_mm          = 0.0f;
    float orientation_rad = 0.0f;
    bool  valid         = false;
};

// Pure deserialization (same logic as grSim ibis_command_receiver.cpp)

inline float ibisDecodeTwoByte(uint8_t high, uint8_t low, float range)
{
    uint16_t two_byte = (static_cast<uint16_t>(high) << 8) | low;
    return static_cast<float>(two_byte - 32767.f) / 32767.f * range;
}

inline IbisCommand ibisDeserialize(const uint8_t* d)
{
    IbisCommand cmd;
    cmd.check_counter        = d[CHECK_COUNTER];
    cmd.vision_global_pos[0] = ibisDecodeTwoByte(d[VISION_GLOBAL_X_H], d[VISION_GLOBAL_X_L], 32.767f);
    cmd.vision_global_pos[1] = ibisDecodeTwoByte(d[VISION_GLOBAL_Y_H], d[VISION_GLOBAL_Y_L], 32.767f);
    cmd.target_global_theta  = ibisDecodeTwoByte(d[TARGET_GLOBAL_TH_H], d[TARGET_GLOBAL_TH_L], static_cast<float>(M_PI));
    cmd.kick_power           = d[KICK_POWER] / 20.f;
    cmd.dribble_power        = d[DRIBBLE_POWER] / 20.f;
    cmd.acceleration_limit   = ibisDecodeTwoByte(d[ACCEL_LIMIT_H], d[ACCEL_LIMIT_L], 32.767f);
    cmd.linear_velocity_limit  = ibisDecodeTwoByte(d[LINEAR_VEL_LIMIT_H], d[LINEAR_VEL_LIMIT_L], 32.767f);
    cmd.angular_velocity_limit = ibisDecodeTwoByte(d[ANGULAR_VEL_LIMIT_H], d[ANGULAR_VEL_LIMIT_L], 32.767f);

    uint8_t flags = d[FLAGS];
    cmd.enable_chip    = (flags >> ENABLE_CHIP) & 0x01;
    cmd.stop_emergency = (flags >> STOP_EMERGENCY) & 0x01;

    // POLAR_VELOCITY_TARGET_MODE args at CONTROL_MODE_ARGS (offset 24)
    cmd.polar_velocity_r     = ibisDecodeTwoByte(d[CONTROL_MODE_ARGS + 0], d[CONTROL_MODE_ARGS + 1], 32.767f);
    cmd.polar_velocity_theta = ibisDecodeTwoByte(d[CONTROL_MODE_ARGS + 2], d[CONTROL_MODE_ARGS + 3], 32.767f);

    return cmd;
}

// ---------------------------------------------------------------------------
// 128-byte feedback packet builder
// Ported from grSim BinaryFeedbackSender::buildPacket, using scalar args
// instead of Robot* so it has no grSim/ODE dependency.
//
// Parameters:
//   buffer          - 128-byte output buffer (must be pre-allocated)
//   robotId         - robot identifier (0..15)
//   counter         - rolling counter (caller increments)
//   yaw_rad         - orientation in radians (SSL vision convention)
//   ball_detected   - is ball in contact with dribbler
//   kick_status     - 0=none, 1=flat, 2=chip
//   odom_x_m        - position x in metres (SSL vision coords)
//   odom_y_m        - position y in metres (SSL vision coords)
//   vel_x_ms        - global velocity x in m/s (SSL vision coords)
//   vel_y_ms        - global velocity y in m/s (SSL vision coords)
// ---------------------------------------------------------------------------

inline void ibisBuildFeedbackPacket(
    uint8_t* buffer,
    int      robotId,
    uint8_t  counter,
    float    yaw_rad,
    bool     ball_detected,
    uint8_t  kick_status,
    float    odom_x_m,
    float    odom_y_m,
    float    vel_x_ms,
    float    vel_y_ms)
{
    std::memset(buffer, 0, IBIS_FEEDBACK_SIZE);

    // Header (0-1)
    buffer[0] = 0xAB;
    buffer[1] = 0xEA;

    // Robot ID (2)
    buffer[2] = static_cast<uint8_t>(robotId);

    // Counter (3)
    buffer[3] = counter;

    // Yaw angle in radians (4-7)
    std::memcpy(&buffer[4], &yaw_rad, sizeof(float));

    // Battery voltage: fixed 24.0 V (8-11)
    float voltage = 24.0f;
    std::memcpy(&buffer[8], &voltage, sizeof(float));

    // Ball detection sensors 0-2 (12-14)
    buffer[12] = ball_detected ? 1 : 0;
    buffer[13] = ball_detected ? 1 : 0;
    buffer[14] = ball_detected ? 1 : 0;

    // Kick status (15): 0=none, 1=flat, 2=chip
    buffer[15] = kick_status;

    // Error info (16-23): 0 (no errors)
    // Motor current (24-27): 0
    // Ball detection 3 (28): 0
    // Already zeroed by memset.

    // Temperature: fixed 25 C (29-35)
    for (int i = 29; i <= 35; i++) {
        buffer[i] = 25;
    }

    // Angle diff (36-39): 0
    float angle_diff = 0.0f;
    std::memcpy(&buffer[36], &angle_diff, sizeof(float));

    // Capacitor voltage: fixed 200.0 V (40-43)
    float cap_voltage = 200.0f;
    std::memcpy(&buffer[40], &cap_voltage, sizeof(float));

    // Odometry position (44-51)
    std::memcpy(&buffer[44], &odom_x_m, sizeof(float));
    std::memcpy(&buffer[48], &odom_y_m, sizeof(float));

    // Global velocity (52-59)
    std::memcpy(&buffer[52], &vel_x_ms, sizeof(float));
    std::memcpy(&buffer[56], &vel_y_ms, sizeof(float));

    // Check version byte: 0x01 = simulator (60)
    buffer[60] = 0x01;

    // Extended data (61-127): 0 (already zeroed)
}
