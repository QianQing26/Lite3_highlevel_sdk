/**
 * @file network_codes.hpp
 * @brief Shared binary packet definitions — Lite3 lowlevel ↔ highlevel SDK
 *
 * Mirrors Lite3_lowlevel_sdk/communication/nx_network_codes.hpp.
 * Keep both copies in sync.
 */
#pragma once

#include <cstdint>
#include <cstddef>

// ============================================================================
// Port assignments
// ============================================================================

/// Default port: RK3588 sends sensor + gamepad data to NX
constexpr int DEFAULT_SENSOR_PORT  = 31001;

/// Default port: NX sends joint commands back to RK3588
constexpr int DEFAULT_COMMAND_PORT = 31002;

// ============================================================================
// Packet structs
// ============================================================================

struct SensorPacket {
    uint32_t seq;             ///< Monotonic sequence number (network byte order)
    uint32_t timestamp_ms;    ///< Millisecond timestamp from robot interface
    int32_t  current_state;   ///< RobotMotionState enum value
    float    rpy[3];          ///< Roll, pitch, yaw from IMU [rad]
    float    acc[3];          ///< Base linear acceleration [m/s²]
    float    omega[3];        ///< Body-frame angular velocity [rad/s]
    float    joint_pos[12];   ///< Joint positions [rad]
    float    joint_vel[12];   ///< Joint velocities [rad/s]
    float    joint_tau[12];   ///< Joint torques [Nm]
    float    cmd_vel[3];      ///< User command velocity (normalised)
};
static_assert(sizeof(SensorPacket) == 204, "SensorPacket size mismatch");

struct CommandPacket {
    uint32_t seq;               ///< Monotonic sequence number from NX
    uint32_t heartbeat;         ///< Application-level heartbeat counter
    float    joint_pos_des[12]; ///< Desired joint positions [rad]
    float    joint_vel_des[12]; ///< Desired joint velocities [rad/s] (usually 0)
    float    kp[12];            ///< PD proportional gains
    float    kd[12];            ///< PD derivative gains
    float    tau_ff[12];        ///< Feed-forward torque [Nm]
};
static_assert(sizeof(CommandPacket) == 248, "CommandPacket size mismatch");

// ============================================================================
// Special sentinel / heartbeat values
// ============================================================================

/// NX sends seq == SHUTDOWN_SEQUENCE to request immediate JointDamping on RK3588
constexpr uint32_t SHUTDOWN_SEQUENCE = 0xFFFFFFFF;

/// Heartbeat value that signals the RK3588 should ignore this packet
constexpr uint32_t HEARTBEAT_SHUTDOWN = 0;

/// Handshake: NX sets heartbeat==HEARTBEAT_READY to acknowledge readiness
constexpr uint32_t HEARTBEAT_READY = 0x0B0B0B0B;

// ============================================================================
// RobotMotionState enum (mirrors types/custom_types.h)
// ============================================================================
// Values of SensorPacket::current_state during normal operation.

namespace lite3 {
enum class RobotMotionState : int32_t {
    WaitingForStand  = 0,  // Idle — waiting for stand-up command
    StandingUp       = 1,  // Executing cubic-spline stand-up sequence
    JointDamping     = 2,  // Safety fallback — passive joint damping
    RLHandshakeMode  = 5,  // Waiting for NX to send HEARTBEAT_READY
    RLControlMode    = 6,  // Receiving commands from NX, normal operation
};
} // namespace lite3
