"""UDP bridge to Lite3 Lowlevel SDK."""

import socket
import struct
import threading
import time
from dataclasses import dataclass, field
from typing import Optional, Callable, Tuple

from .network_codes import (
    DEFAULT_SENSOR_PORT, DEFAULT_COMMAND_PORT,
    SENSOR_PACKET_SIZE, COMMAND_PACKET_SIZE,
    _SENSOR_FMT, _COMMAND_FMT, SHUTDOWN_SEQUENCE,
)


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
    """UDP bridge to Lite3 lowlevel SDK.

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
                 send_port: int = DEFAULT_COMMAND_PORT):
        self._robot_ip = robot_ip
        self._recv_port = recv_port
        self._send_port = send_port
        self._send_addr = (robot_ip, send_port)

        self._sock_recv: Optional[socket.socket] = None
        self._sock_send: Optional[socket.socket] = None
        self._running = threading.Event()
        self._seq = 0

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def run(self, policy: PolicyCallback, frequency_hz: float = 50.0):
        """Blocking main loop. Calls policy at the given frequency.

        Args:
            policy: Function (RobotState) -> JointCommand
            frequency_hz: Control frequency (default 50 Hz)
        """
        self._setup_sockets()
        self._running.set()

        period_s = 1.0 / frequency_hz
        print(f"[Bridge] Running at {frequency_hz:.0f} Hz "
              f"— listening :{self._recv_port}, sending {self._robot_ip}:{self._send_port}")

        try:
            while self._running.is_set():
                tick_start = time.monotonic()

                # 1. Receive latest sensor data (non-blocking drain)
                state = self._recv_latest()

                # 2. Call policy
                if state is not None:
                    cmd = policy(state)
                else:
                    cmd = JointCommand()  # no sensor data yet — send zeros (safe)

                # 3. Send command
                self._send(cmd)

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
            pkt = struct.pack(_COMMAND_FMT, SHUTDOWN_SEQUENCE, 0, *([0.0] * 60))
            self._sock_send.sendto(pkt, self._send_addr)
        self._running.clear()

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
                try: s.close()
                except Exception: pass
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

    def _send(self, cmd: JointCommand):
        floats = (cmd.joint_pos_des + cmd.joint_vel_des +
                  cmd.kp + cmd.kd + cmd.tau_ff)
        pkt = struct.pack(_COMMAND_FMT, self._seq, self._seq, *floats)
        self._seq += 1
        self._sock_send.sendto(pkt, self._send_addr)

    @staticmethod
    def _unpack(data: bytes) -> RobotState:
        f = struct.unpack(_SENSOR_FMT, data)
        return RobotState(
            seq=f[0], timestamp_ms=f[1], current_state=f[2],
            rpy=f[3:6], acc=f[6:9], omega=f[9:12],
            joint_pos=f[12:24], joint_vel=f[24:36], joint_tau=f[36:48],
            cmd_vel=f[48:51],
        )
