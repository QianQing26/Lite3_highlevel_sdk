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

constexpr int DEFAULT_SENSOR_PORT  = 31001;  // lowlevel → highlevel
constexpr int DEFAULT_COMMAND_PORT = 31002;  // highlevel → lowlevel
constexpr uint32_t SHUTDOWN_SEQUENCE = 0xFFFFFFFF;

struct SensorPacket {
    uint32_t seq;
    uint32_t timestamp_ms;
    int32_t  current_state;
    float    rpy[3];
    float    acc[3];
    float    omega[3];
    float    joint_pos[12];
    float    joint_vel[12];
    float    joint_tau[12];
    float    cmd_vel[3];
};
static_assert(sizeof(SensorPacket) == 204, "SensorPacket size mismatch");

struct CommandPacket {
    uint32_t seq;
    uint32_t heartbeat;
    float    joint_pos_des[12];
    float    joint_vel_des[12];
    float    kp[12];
    float    kd[12];
    float    tau_ff[12];
};
static_assert(sizeof(CommandPacket) == 248, "CommandPacket size mismatch");
