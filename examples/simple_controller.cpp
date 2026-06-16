/**
 * @file simple_controller.cpp
 * @brief Minimal example: ONNX policy → Lite3 control via highlevel SDK (C++)
 *
 * Demonstrates the handshake protocol:
 *   1. Bridge waits for robot to enter RLHandshakeMode
 *   2. Sends HEARTBEAT_READY until robot enters RLControlMode
 *   3. Runs the policy control loop at 50 Hz
 *
 * Build:
 *   g++ -std=c++17 -I../cpp/include -I<onnxruntime>/include
 *       simple_controller.cpp ../cpp/src/onnx_runner.cpp
 *       -L<onnxruntime>/lib -lonnxruntime -lpthread
 */

#include "lite3_highlevel/lite3_highlevel.hpp"

#include <iostream>
#include <iomanip>
#include <cmath>
#include <csignal>
#include <chrono>
#include <thread>

static volatile bool g_running = true;

void signal_handler(int) { g_running = false; }

int main(int argc, char* argv[]) {
    std::string robot_ip   = (argc > 1) ? argv[1] : "192.168.1.2";
    std::string model_path = (argc > 2) ? argv[2] : "policy.onnx";

    std::signal(SIGINT, signal_handler);

    std::cout << "=== Lite3 Highlevel Controller (C++) ===" << std::endl;
    std::cout << "Robot IP: " << robot_ip << std::endl;
    std::cout << "Model:    " << model_path << std::endl;

    // --- Setup ---
    lite3::OnnxRunner runner(model_path);
    lite3::ObsBuilder builder;
    lite3::CmdProcessor processor;
    lite3::Bridge bridge(robot_ip);

    if (!bridge.open()) {
        std::cerr << "Failed to open bridge" << std::endl;
        return 1;
    }

    float obs[45];
    float raw_action[12];
    float last_action[12] = {0};
    int tick = 0;

    bool policy_initialised = false;  // set up once when handshake completes
    const double period_s = 1.0 / 50.0;  // 50 Hz

    std::cout << "Running at 50 Hz... Press Ctrl+C to stop." << std::endl;

    // --- Control loop with handshake ---
    while (g_running) {
        auto tick_start = std::chrono::steady_clock::now();

        // 1. Receive latest sensor data
        struct SensorPacket pkt;
        bool got = bridge.recv(pkt);

        lite3::RobotMotionState robot_state = lite3::RobotMotionState::WaitingForStand;
        if (got) {
            robot_state = static_cast<lite3::RobotMotionState>(pkt.current_state);
        }

        // 2. Update handshake state machine
        bridge.updateHandshake(robot_state);

        // 3. Act based on handshake state
        auto hs = bridge.getHandshakeState();

        if (hs == lite3::HandshakeState::ESTABLISHED) {
            // --- Normal policy loop ---
            if (!policy_initialised) {
                // Reset observation state on fresh handshake
                std::memset(last_action, 0, sizeof(last_action));
                policy_initialised = true;
            }

            if (got) {
                // Convert to RobotState
                lite3::RobotState state;
                state.seq = pkt.seq;
                state.timestamp_ms = pkt.timestamp_ms;
                state.current_state = pkt.current_state;
                for (int i = 0; i < 3; ++i) {
                    state.rpy[i]   = pkt.rpy[i];
                    state.acc[i]   = pkt.acc[i];
                    state.omega[i] = pkt.omega[i];
                    state.cmd_vel[i] = pkt.cmd_vel[i];
                }
                std::memcpy(state.joint_pos, pkt.joint_pos, sizeof(state.joint_pos));
                std::memcpy(state.joint_vel, pkt.joint_vel, sizeof(state.joint_vel));
                std::memcpy(state.joint_tau, pkt.joint_tau, sizeof(state.joint_tau));

                // Build observation
                builder.setLastAction(last_action);
                builder.build(state, obs);

                // Run ONNX inference
                runner.infer(obs, raw_action);
                std::memcpy(last_action, raw_action, sizeof(last_action));

                // Convert to joint command and send
                auto cmd = processor.process(raw_action);
                bridge.sendCommand(cmd);
            } else {
                // No sensor data — send zero hold (safe)
                bridge.sendZeroHold();
            }

            // Periodic status
            if (got && ++tick % 50 == 0) {
                std::cout << "[" << tick << "]"
                          << " rpy=("
                          << std::fixed << std::setprecision(1)
                          << pkt.rpy[0]*57.3 << ","
                          << pkt.rpy[1]*57.3 << ","
                          << pkt.rpy[2]*57.3 << ") deg"
                          << std::endl;
            }

        } else if (hs == lite3::HandshakeState::HANDSHAKING) {
            // Send HEARTBEAT_READY and wait for RLControlMode
            bridge.sendReady();

        } else {
            // DISCONNECTED — send zero hold (safe while robot is idle/standing up)
            if (got) {
                bridge.sendZeroHold();
            }
            policy_initialised = false;
        }

        // Maintain frequency
        auto elapsed = std::chrono::steady_clock::now() - tick_start;
        auto remaining = std::chrono::duration<double>(period_s) - elapsed;
        if (remaining.count() > 0)
            std::this_thread::sleep_for(remaining);
    }

    std::cout << "\nShutting down..." << std::endl;
    bridge.sendShutdown();
    bridge.close();
    return 0;
}
