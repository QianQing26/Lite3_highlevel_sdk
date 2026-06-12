/**
 * @file simple_controller.cpp
 * @brief Minimal example: ONNX policy → Lite3 control via highlevel SDK (C++)
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

    const double period_s = 1.0 / 50.0;  // 50 Hz

    std::cout << "Running at 50 Hz... Press Ctrl+C to stop." << std::endl;

    // --- Control loop ---
    while (g_running) {
        auto tick_start = std::chrono::steady_clock::now();

        // 1. Receive latest sensor data
        struct SensorPacket pkt;
        bool got = bridge.recv(pkt);

        if (got) {
            // Convert to RobotState
            lite3::RobotState state;
            state.seq = ntohl(pkt.seq);
            state.timestamp_ms = ntohl(pkt.timestamp_ms);
            state.current_state = static_cast<int32_t>(ntohl(static_cast<uint32_t>(pkt.current_state)));
            for (int i = 0; i < 3; ++i) {
                state.rpy[i]   = pkt.rpy[i];
                state.acc[i]   = pkt.acc[i];
                state.omega[i] = pkt.omega[i];
                state.cmd_vel[i] = pkt.cmd_vel[i];
            }
            std::memcpy(state.joint_pos, pkt.joint_pos, sizeof(state.joint_pos));
            std::memcpy(state.joint_vel, pkt.joint_vel, sizeof(state.joint_vel));
            std::memcpy(state.joint_tau, pkt.joint_tau, sizeof(state.joint_tau));

            // 2. Build observation
            builder.build(state, obs);
            builder.setLastAction(last_action);

            // 3. Run ONNX inference
            runner.infer(obs, raw_action);
            std::memcpy(last_action, raw_action, sizeof(last_action));

            // 4. Convert to joint command and send
            auto cmd = processor.process(raw_action);
            bridge.sendCommand(cmd);

            if (++tick % 50 == 0) {
                std::cout << "[" << tick << "] rpy=("
                          << std::fixed << std::setprecision(1)
                          << state.rpy[0]*57.3 << ","
                          << state.rpy[1]*57.3 << ","
                          << state.rpy[2]*57.3 << ") deg"
                          << std::endl;
            }
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
