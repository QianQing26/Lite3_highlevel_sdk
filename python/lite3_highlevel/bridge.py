"""UDP bridge to Lite3 Lowlevel SDK with handshake support."""

import socket
import struct
import threading
import time
import enum
from dataclasses import dataclass, field
from typing import Optional, Callable, Tuple

from .network_codes import (
    DEFAULT_SENSOR_PORT, DEFAULT_COMMAND_PORT,
    SENSOR_PACKET_SIZE, COMMAND_PACKET_SIZE,
    _SENSOR_FMT, _COMMAND_FMT,
    SHUTDOWN_SEQUENCE, HEARTBEAT_READY, HEARTBEAT_SHUTDOWN,
    RobotMotionState,
)


# ============================================================================
# Handshake state
# ============================================================================

class HandshakeState(enum.IntEnum):
    """NX-side handshake state — mirrors the RK3588's state progression.

    State transitions:
        DISCONNECTED ──(robot enters RLHandshakeMode)──→ HANDSHAKING
        HANDSHAKING  ──(robot enters RLControlMode)───→ ESTABLISHED
        ESTABLISHED  ──(robot drops to JointDamping)──→ DISCONNECTED
    """
    DISCONNECTED = 0  # No contact with robot (Idle/StandUp/JointDamping)
    HANDSHAKING  = 1  # Sending HEARTBEAT_READY, waiting for RLControlMode
    ESTABLISHED  = 2  # Handshake complete — running policy


# ============================================================================
# Data classes
# ============================================================================

@dataclass
class RobotState:
    """Sensor + gamepad data received from the robot."""
    seq: int = 0
    timestamp_ms: int = 0
    current_state: int = 0
    rpy: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    acc: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    omega: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    joint_pos: Tuple[float, ...] = field(default_factory=lambda: (0.0,) * 12)
    joint_vel: Tuple[float, ...] = field(default_factory=lambda: (0.0,) * 12)
    joint_tau: Tuple[float, ...] = field(default_factory=lambda: (0.0,) * 12)
    cmd_vel: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    @property
    def projected_gravity(self) -> Tuple[float, float, float]:
        """Gravity vector in body frame (ZYX Euler convention)."""
        import math
        cp = math.cos(self.rpy[1])
        return (math.sin(self.rpy[1]), -cp * math.sin(self.rpy[0]), -cp * math.cos(self.rpy[0]))

    @property
    def is_rl_control(self) -> bool:
        """True if the robot is in RLControlMode (normal operation)."""
        return self.current_state == RobotMotionState.RLControlMode

    @property
    def is_handshake(self) -> bool:
        """True if the robot is waiting for our HEARTBEAT_READY."""
        return self.current_state == RobotMotionState.RLHandshakeMode

    @property
    def is_damping(self) -> bool:
        """True if the robot has entered JointDamping (safety fallback)."""
        return self.current_state == RobotMotionState.JointDamping


@dataclass
class JointCommand:
    """Joint command to send back to the robot."""
    joint_pos_des: list = field(default_factory=lambda: [0.0] * 12)
    joint_vel_des: list = field(default_factory=lambda: [0.0] * 12)
    kp: list = field(default_factory=lambda: [30.0] * 12)
    kd: list = field(default_factory=lambda: [1.0] * 12)
    tau_ff: list = field(default_factory=lambda: [0.0] * 12)


# ============================================================================
# Bridge
# ============================================================================

PolicyCallback = Callable[[RobotState], JointCommand]


class Bridge:
    """UDP bridge to Lite3 lowlevel SDK with automatic handshake.

    The bridge waits for the robot to enter RLHandshakeMode, sends a
    HEARTBEAT_READY signal, and begins the policy control loop once
    RLControlMode is confirmed.

    Usage:
        def policy(state: RobotState) -> JointCommand:
            cmd = JointCommand()
            cmd.joint_pos_des = [...]  # your policy output
            return cmd

        bridge = Bridge(robot_ip="192.168.1.2")
        bridge.run(policy, frequency_hz=50)
    """

    def __init__(self,
                 robot_ip: str = "192.168.1.2",
                 recv_port: int = DEFAULT_SENSOR_PORT,
                 send_port: int = DEFAULT_COMMAND_PORT,
                 handshake_timeout: float = 5.0):
        self._robot_ip = robot_ip
        self._recv_port = recv_port
        self._send_port = send_port
        self._send_addr = (robot_ip, send_port)
        self._handshake_timeout = handshake_timeout

        self._sock_recv: Optional[socket.socket] = None
        self._sock_send: Optional[socket.socket] = None
        self._running = threading.Event()
        self._seq = 0

        # Handshake state machine
        self._hstate = HandshakeState.DISCONNECTED
        self._handshake_enter_time: float = 0.0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def run(self, policy: PolicyCallback, frequency_hz: float = 50.0):
        """Blocking main loop with automatic handshake.

        1. Waits for the robot to enter RLHandshakeMode (after StandUp).
        2. Sends HEARTBEAT_READY until the robot enters RLControlMode.
        3. Runs the policy at the given control frequency.

        Args:
            policy: Function (RobotState) -> JointCommand
            frequency_hz: Control frequency (default 50 Hz)
        """
        self._setup_sockets()
        self._running.set()

        period_s = 1.0 / frequency_hz
        print(f"[Bridge] Running at {frequency_hz:.0f} Hz "
              f"— listening :{self._recv_port}, sending {self._robot_ip}:{self._send_port}")
        print("[Bridge] Waiting for robot handshake...")

        try:
            while self._running.is_set():
                tick_start = time.monotonic()

                # 1. Receive latest sensor data (non-blocking drain)
                state = self._recv_latest()

                # 2. Update handshake state machine
                self._update_handshake(state)

                # 3. Act based on current handshake state
                if self._hstate == HandshakeState.ESTABLISHED:
                    cmd = policy(state) if state is not None else JointCommand()
                    self._send(cmd)
                elif self._hstate == HandshakeState.HANDSHAKING:
                    self._send_ready()
                else:  # DISCONNECTED
                    self._send_zoh()

                # 4. Maintain frequency
                elapsed = time.monotonic() - tick_start
                remaining = period_s - elapsed
                if remaining > 0:
                    time.sleep(remaining)

        except KeyboardInterrupt:
            print("\n[Bridge] Interrupted")
        finally:
            self._close()

    def shutdown(self):
        """Tell lowlevel SDK to enter JointDamping, then stop."""
        if self._sock_send:
            pkt = struct.pack(_COMMAND_FMT, SHUTDOWN_SEQUENCE, HEARTBEAT_SHUTDOWN,
                              *([0.0] * 60))
            self._sock_send.sendto(pkt, self._send_addr)
        self._running.clear()

    # ------------------------------------------------------------------
    # Handshake state machine
    # ------------------------------------------------------------------

    def _update_handshake(self, state: Optional[RobotState]):
        """Advance the handshake state machine based on inbound sensor data.

        Transitions:
            DISCONNECTED → HANDSHAKING:  Robot entered RLHandshakeMode.
            HANDSHAKING  → ESTABLISHED:  Robot entered RLControlMode.
            ANY          → DISCONNECTED: Robot fell to JointDamping, or handshake timeout.
        """
        if state is None:
            return

        if self._hstate == HandshakeState.DISCONNECTED:
            if state.is_handshake:
                self._hstate = HandshakeState.HANDSHAKING
                self._handshake_enter_time = time.monotonic()
                print(f"[Bridge] Robot entered RLHandshakeMode — sending HEARTBEAT_READY...")

        elif self._hstate == HandshakeState.HANDSHAKING:
            if state.is_rl_control:
                self._hstate = HandshakeState.ESTABLISHED
                elapsed = time.monotonic() - self._handshake_enter_time
                print(f"[Bridge] Handshake ESTABLISHED (took {elapsed:.2f} s) — running policy")

            elif time.monotonic() - self._handshake_enter_time > self._handshake_timeout:
                print(f"[Bridge] Handshake timeout — robot did not enter RLControlMode "
                      f"within {self._handshake_timeout:.0f} s")
                self._hstate = HandshakeState.DISCONNECTED

            elif state.is_damping:
                print("[Bridge] Robot entered JointDamping during handshake — "
                      "waiting for re-entry")
                self._hstate = HandshakeState.DISCONNECTED

        elif self._hstate == HandshakeState.ESTABLISHED:
            if state.is_damping:
                print("[Bridge] Robot entered JointDamping — reverting to DISCONNECTED")
                self._hstate = HandshakeState.DISCONNECTED

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _setup_sockets(self):
        self._sock_recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock_recv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._sock_recv.settimeout(0.002)  # 2 ms — non-blocking drain
        self._sock_recv.bind(("", self._recv_port))

        self._sock_send = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def _close(self):
        self._running.clear()
        for s in (self._sock_recv, self._sock_send):
            if s:
                try:
                    s.close()
                except Exception:
                    pass
        print("[Bridge] Stopped")

    def _recv_latest(self) -> Optional[RobotState]:
        """Drain socket, return most recent SensorPacket."""
        latest = None
        while True:
            try:
                data, _ = self._sock_recv.recvfrom(SENSOR_PACKET_SIZE)
                if len(data) == SENSOR_PACKET_SIZE:
                    latest = self._unpack(data)
            except socket.timeout:
                break
            except OSError:
                break
        return latest

    def _send(self, cmd: JointCommand,
              heartbeat_override: Optional[int] = None):
        """Send a CommandPacket.

        Args:
            cmd: Joint command to send.
            heartbeat_override: If set, use this heartbeat value instead of seq.
                Used for HEARTBEAT_READY during handshake.
        """
        heartbeat = heartbeat_override if heartbeat_override is not None else self._seq
        floats = (cmd.joint_pos_des + cmd.joint_vel_des +
                  cmd.kp + cmd.kd + cmd.tau_ff)
        pkt = struct.pack(_COMMAND_FMT, self._seq, heartbeat, *floats)
        self._seq += 1
        self._sock_send.sendto(pkt, self._send_addr)

    def _send_ready(self):
        """Send HEARTBEAT_READY with zero joint commands (ZOH hold)."""
        self._send(JointCommand(), heartbeat_override=HEARTBEAT_READY)

    def _send_zoh(self):
        """Send a zero-command — RK3588 holds current position safely."""
        self._send(JointCommand())

    @staticmethod
    def _unpack(data: bytes) -> RobotState:
        f = struct.unpack(_SENSOR_FMT, data)
        return RobotState(
            seq=f[0], timestamp_ms=f[1], current_state=f[2],
            rpy=f[3:6], acc=f[6:9], omega=f[9:12],
            joint_pos=f[12:24], joint_vel=f[24:36], joint_tau=f[36:48],
            cmd_vel=f[48:51],
        )
