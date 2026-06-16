/**
 * @file bridge.hpp
 * @brief UDP bridge to Lite3 lowlevel SDK with handshake support (C++ API)
 */
#pragma once

#include "network_codes.hpp"
#include "types.hpp"

#include <string>
#include <cstring>
#include <iostream>
#include <chrono>
#include <thread>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>

namespace lite3 {

// ============================================================================
// Handshake state
// ============================================================================

/// NX-side handshake state — mirrors the RK3588's state progression.
///
/// Transitions:
///   DISCONNECTED → HANDSHAKING when robot enters RLHandshakeMode
///   HANDSHAKING  → ESTABLISHED  when robot enters RLControlMode
///   ANY          → DISCONNECTED when robot drops to JointDamping
enum class HandshakeState {
    DISCONNECTED = 0,  // No contact with robot (Idle / StandUp / JointDamping)
    HANDSHAKING  = 1,  // Sending HEARTBEAT_READY, waiting for RLControlMode
    ESTABLISHED  = 2,  // Handshake complete — running policy
};

// ============================================================================
// Bridge
// ============================================================================

class Bridge {
public:
    Bridge(const std::string& robot_ip,
           int recv_port = DEFAULT_SENSOR_PORT,
           int send_port = DEFAULT_COMMAND_PORT)
        : robot_ip_(robot_ip), recv_port_(recv_port), send_port_(send_port)
    {
        send_addr_.sin_family = AF_INET;
        send_addr_.sin_port   = htons(send_port_);
        send_addr_.sin_addr.s_addr = inet_addr(robot_ip_.c_str());
    }

    ~Bridge() { close(); }

    // ========================================================================
    // Lifecycle
    // ========================================================================

    /// Open sockets. Returns false on failure.
    bool open() {
        sock_recv_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_recv_ < 0) { std::cerr << "[Bridge] recv socket failed\n"; return false; }

        int reuse = 1;
        setsockopt(sock_recv_, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

        struct timeval tv = {0, 2000}; // 2 ms timeout for drain
        setsockopt(sock_recv_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        struct sockaddr_in addr = {};
        addr.sin_family = AF_INET;
        addr.sin_port   = htons(recv_port_);
        addr.sin_addr.s_addr = INADDR_ANY;
        if (bind(sock_recv_, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
            std::cerr << "[Bridge] bind failed on port " << recv_port_ << "\n";
            return false;
        }

        sock_send_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_send_ < 0) { std::cerr << "[Bridge] send socket failed\n"; return false; }

        std::cout << "[Bridge] Opened — " << robot_ip_
                  << ":" << send_port_ << " / :" << recv_port_ << std::endl;
        return true;
    }

    void close() {
        if (sock_recv_ >= 0) { ::close(sock_recv_); sock_recv_ = -1; }
        if (sock_send_ >= 0) { ::close(sock_send_); sock_send_ = -1; }
    }

    // ========================================================================
    // Data I/O
    // ========================================================================

    /// Receive the latest SensorPacket (drains socket, returns newest).
    /// Returns true if a packet was received.
    bool recv(struct SensorPacket& pkt) {
        bool got = false;
        struct SensorPacket tmp;
        while (true) {
            ssize_t n = recvfrom(sock_recv_, &tmp, sizeof(tmp), 0, nullptr, nullptr);
            if (n == sizeof(tmp)) { pkt = tmp; got = true; }
            else break;
        }
        return got;
    }

    /// Send a raw CommandPacket.
    void send(const struct CommandPacket& pkt) {
        sendto(sock_send_, &pkt, sizeof(pkt), 0,
               (struct sockaddr*)&send_addr_, sizeof(send_addr_));
    }

    /// Send a joint command (auto-fills seq and heartbeat with seq).
    void sendCommand(const struct JointCommand& cmd) {
        sendCommandInternal(cmd, seq_);
        seq_++;
    }

    /// Send shutdown signal to lowlevel SDK.
    void sendShutdown() {
        struct CommandPacket pkt;
        std::memset(&pkt, 0, sizeof(pkt));
        pkt.seq = htonl(SHUTDOWN_SEQUENCE);
        send(pkt);
        std::cout << "[Bridge] Shutdown sent\n";
    }

    // ========================================================================
    // Handshake API
    // ========================================================================

    /// Current handshake state.
    HandshakeState getHandshakeState() const { return hstate_; }

    /// Set the handshake timeout (seconds, default 5.0).
    void setHandshakeTimeout(double seconds) { handshake_timeout_ = seconds; }

    /// Send HEARTBEAT_READY with zero joint commands (ZOH hold).
    /// Call this while HANDSHAKING until the robot enters RLControlMode.
    void sendReady() {
        struct CommandPacket pkt;
        std::memset(&pkt, 0, sizeof(pkt));
        pkt.seq       = htonl(seq_++);
        pkt.heartbeat = htonl(HEARTBEAT_READY);
        send(pkt);
    }

    /// Send a zero-command (ZOH hold). Called before handshake completes.
    void sendZeroHold() {
        struct CommandPacket pkt;
        std::memset(&pkt, 0, sizeof(pkt));
        pkt.seq       = htonl(seq_++);
        pkt.heartbeat = htonl(seq_); // normal heartbeat
        send(pkt);
    }

    /**
     * @brief Advance the handshake state machine based on the robot's current state.
     *
     * Call this every tick with the robot's current_state from the latest
     * SensorPacket. The internal FSM handles the transitions:
     *
     *   DISCONNECTED → HANDSHAKING when robot enters RLHandshakeMode
     *   HANDSHAKING  → ESTABLISHED  when robot enters RLControlMode
     *   ANY          → DISCONNECTED when robot drops to JointDamping
     *
     * @param robot_state  The current_state field from the most recent
     *                     SensorPacket (use RobotMotionState enum for comparison).
     */
    void updateHandshake(RobotMotionState robot_state) {
        auto now = std::chrono::steady_clock::now();

        switch (hstate_) {
        case HandshakeState::DISCONNECTED:
            if (robot_state == RobotMotionState::RLHandshakeMode) {
                hstate_ = HandshakeState::HANDSHAKING;
                handshake_start_ = now;
                std::cout << "[Bridge] Robot entered RLHandshakeMode"
                          << " — sending HEARTBEAT_READY..." << std::endl;
            }
            break;

        case HandshakeState::HANDSHAKING:
            if (robot_state == RobotMotionState::RLControlMode) {
                hstate_ = HandshakeState::ESTABLISHED;
                auto elapsed = std::chrono::duration<double>(now - handshake_start_);
                std::cout << "[Bridge] Handshake ESTABLISHED"
                          << " (took " << elapsed.count() << " s)"
                          << " — running policy" << std::endl;
            } else if (robot_state == RobotMotionState::JointDamping) {
                std::cout << "[Bridge] Robot entered JointDamping during handshake"
                          << " — reverting to DISCONNECTED" << std::endl;
                hstate_ = HandshakeState::DISCONNECTED;
            } else if (elapsedSeconds(now, handshake_start_) > handshake_timeout_) {
                std::cout << "[Bridge] Handshake timeout"
                          << " — robot did not enter RLControlMode within "
                          << handshake_timeout_ << " s" << std::endl;
                hstate_ = HandshakeState::DISCONNECTED;
            }
            break;

        case HandshakeState::ESTABLISHED:
            if (robot_state == RobotMotionState::JointDamping) {
                std::cout << "[Bridge] Robot entered JointDamping"
                          << " — reverting to DISCONNECTED" << std::endl;
                hstate_ = HandshakeState::DISCONNECTED;
            }
            break;
        }
    }

private:
    // ========================================================================
    // Internal
    // ========================================================================

    void sendCommandInternal(const struct JointCommand& cmd, uint32_t seq_val) {
        struct CommandPacket pkt;
        std::memset(&pkt, 0, sizeof(pkt));
        pkt.seq       = htonl(seq_val);
        pkt.heartbeat = htonl(seq_val);
        std::memcpy(pkt.joint_pos_des, cmd.joint_pos_des, sizeof(cmd.joint_pos_des));
        std::memcpy(pkt.joint_vel_des, cmd.joint_vel_des, sizeof(cmd.joint_vel_des));
        std::memcpy(pkt.kp,   cmd.kp,   sizeof(cmd.kp));
        std::memcpy(pkt.kd,   cmd.kd,   sizeof(cmd.kd));
        std::memcpy(pkt.tau_ff, cmd.tau_ff, sizeof(cmd.tau_ff));
        send(pkt);
    }

    static double elapsedSeconds(
        std::chrono::steady_clock::time_point now,
        std::chrono::steady_clock::time_point start) {
        return std::chrono::duration<double>(now - start).count();
    }

    // --- State ---
    std::string robot_ip_;
    int recv_port_, send_port_;
    int sock_recv_ = -1, sock_send_ = -1;
    struct sockaddr_in send_addr_ = {};
    uint32_t seq_ = 0;

    // Handshake FSM
    HandshakeState hstate_ = HandshakeState::DISCONNECTED;
    std::chrono::steady_clock::time_point handshake_start_;
    double handshake_timeout_ = 5.0;
};

} // namespace lite3
