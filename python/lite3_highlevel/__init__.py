"""
Lite3 Highlevel SDK — Python API for controlling the Lite3 quadruped robot.

Quick start (ONNX policy):
    from lite3_highlevel import Bridge, OnnxRunner, ObsBuilder, CmdProcessor

    runner = OnnxRunner("policy.onnx")
    builder = ObsBuilder()
    processor = CmdProcessor()

    def policy(state):
        obs = builder.build(state)
        raw_action = runner.infer(obs)
        return processor.process(raw_action)

    bridge = Bridge(robot_ip="192.168.1.120")
    bridge.run(policy, frequency_hz=50)

Quick start (custom policy):
    from lite3_highlevel import Bridge, JointCommand

    def my_policy(state):
        cmd = JointCommand()
        cmd.joint_pos_des = [0.0, -0.65, 1.30] * 4  # standing pose
        return cmd

    Bridge(robot_ip="192.168.1.120").run(my_policy)
"""

from .network_codes import (
    # Port defaults
    DEFAULT_SENSOR_PORT, DEFAULT_COMMAND_PORT,
    # Packet sizes
    SENSOR_PACKET_SIZE, COMMAND_PACKET_SIZE,
    # Sentinel / heartbeat
    SHUTDOWN_SEQUENCE, HEARTBEAT_READY, HEARTBEAT_SHUTDOWN,
    # Robot motion state enum (for inspecting state.current_state)
    RobotMotionState, StateName,
)
from .bridge import (
    Bridge, RobotState, JointCommand, PolicyCallback,
    HandshakeState,
)
from .observation import ObsBuilder, CmdProcessor
from .onnx_runner import OnnxRunner

__all__ = [
    # Bridge
    "Bridge", "RobotState", "JointCommand", "PolicyCallback",
    "HandshakeState",
    # Observation & processing
    "ObsBuilder", "CmdProcessor",
    # ONNX inference
    "OnnxRunner",
    # Network codes
    "DEFAULT_SENSOR_PORT", "DEFAULT_COMMAND_PORT",
    "SHUTDOWN_SEQUENCE", "HEARTBEAT_READY", "HEARTBEAT_SHUTDOWN",
    "RobotMotionState", "StateName",
]
