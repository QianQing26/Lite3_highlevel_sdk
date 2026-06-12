/**
 * @file types.hpp
 * @brief Core data types for Lite3 highlevel SDK (C++ API)
 */
#pragma once

#include <cstdint>
#include <vector>
#include <cmath>
#include <array>

namespace lite3 {

// ============================================================================
// Robot state (received from lowlevel SDK)
// ============================================================================

struct RobotState {
    uint32_t seq = 0;
    uint32_t timestamp_ms = 0;
    int32_t  current_state = 0;
    float    rpy[3] = {0, 0, 0};
    float    acc[3] = {0, 0, 0};
    float    omega[3] = {0, 0, 0};
    float    joint_pos[12] = {0};
    float    joint_vel[12] = {0};
    float    joint_tau[12] = {0};
    float    cmd_vel[3] = {0, 0, 0};

    /// Gravity vector projected into body frame (ZYX Euler convention).
    void getProjectedGravity(float g[3]) const {
        float cp = std::cos(rpy[1]);
        g[0] = std::sin(rpy[1]);
        g[1] = -cp * std::sin(rpy[0]);
        g[2] = -cp * std::cos(rpy[0]);
    }
};

// ============================================================================
// Joint command (sent to lowlevel SDK)
// ============================================================================

struct JointCommand {
    float joint_pos_des[12] = {0};
    float joint_vel_des[12] = {0};
    float kp[12]  = {30,30,30,30,30,30,30,30,30,30,30,30};
    float kd[12]  = {1,1,1,1,1,1,1,1,1,1,1,1};
    float tau_ff[12] = {0};
};

// ============================================================================
// Policy interface
// ============================================================================

class PolicyBase {
public:
    virtual ~PolicyBase() = default;

    /// Called once when entering RL control
    virtual void onEnter() {}

    /// Called every control tick (~50 Hz)
    virtual JointCommand step(const RobotState& state) = 0;

    /// Called once when exiting RL control
    virtual void onExit() {}
};

} // namespace lite3
