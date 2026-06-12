/**
 * @file observation.hpp
 * @brief Observation builder & command processor (C++ API)
 */
#pragma once

#include "types.hpp"
#include <vector>
#include <cstring>

namespace lite3 {

// ============================================================================
// ObsBuilder
// ============================================================================

struct ObsConfig {
    int   num_joints  = 12;
    float omega_scale = 0.25f;
    float dof_vel_scale = 0.05f;
    float default_joint_pos[12] = {0, -0.65f, 1.30f, 0, -0.65f, 1.30f,
                                   0, -0.65f, 1.30f, 0, -0.65f, 1.30f};
    float action_scale[12] = {0.125f, 0.25f, 0.25f, 0.125f, 0.25f, 0.25f,
                              0.125f, 0.25f, 0.25f, 0.125f, 0.25f, 0.25f};
    float max_cmd_vel[3] = {0.8f, 0.8f, 0.8f};
    int   obs_dim = 45;
};

class ObsBuilder {
public:
    ObsBuilder(const ObsConfig& cfg = ObsConfig{}) : cfg_(cfg) {
        std::fill(last_action_, last_action_ + 12, 0.0f);
    }

    int obsDim() const { return cfg_.obs_dim; }

    void build(const RobotState& state, float* out) {
        int idx = 0;
        float pg[3]; state.getProjectedGravity(pg);

        for (int i = 0; i < 3; ++i) out[idx++] = state.omega[i] * cfg_.omega_scale;
        for (int i = 0; i < 3; ++i) out[idx++] = pg[i];
        for (int i = 0; i < 3; ++i) out[idx++] = state.cmd_vel[i] * cfg_.max_cmd_vel[i];
        for (int i = 0; i < cfg_.num_joints; ++i)
            out[idx++] = state.joint_pos[i] - cfg_.default_joint_pos[i];
        for (int i = 0; i < cfg_.num_joints; ++i)
            out[idx++] = state.joint_vel[i] * cfg_.dof_vel_scale;
        for (int i = 0; i < cfg_.num_joints; ++i)
            out[idx++] = last_action_[i];
    }

    void setLastAction(const float* action) {
        std::memcpy(last_action_, action, cfg_.num_joints * sizeof(float));
    }

    const ObsConfig& config() const { return cfg_; }

private:
    ObsConfig cfg_;
    float  last_action_[12];
};

// ============================================================================
// CmdProcessor
// ============================================================================

struct CmdConfig {
    int   num_joints  = 12;
    float default_joint_pos[12] = {0, -0.65f, 1.30f, 0, -0.65f, 1.30f,
                                   0, -0.65f, 1.30f, 0, -0.65f, 1.30f};
    float action_scale[12] = {0.125f, 0.25f, 0.25f, 0.125f, 0.25f, 0.25f,
                              0.125f, 0.25f, 0.25f, 0.125f, 0.25f, 0.25f};
    float kp = 30.0f;
    float kd = 1.0f;
};

class CmdProcessor {
public:
    CmdProcessor(const CmdConfig& cfg = CmdConfig{}) : cfg_(cfg) {}

    JointCommand process(const float* raw_action) const {
        JointCommand cmd;
        for (int i = 0; i < cfg_.num_joints; ++i) {
            float action = raw_action[i] * cfg_.action_scale[i];
            cmd.joint_pos_des[i] = cfg_.default_joint_pos[i] + action;
            cmd.joint_vel_des[i] = 0.0f;
            cmd.kp[i]  = cfg_.kp;
            cmd.kd[i]  = cfg_.kd;
            cmd.tau_ff[i] = 0.0f;
        }
        return cmd;
    }

    const CmdConfig& config() const { return cfg_; }

private:
    CmdConfig cfg_;
};

} // namespace lite3
