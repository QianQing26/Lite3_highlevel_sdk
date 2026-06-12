#!/usr/bin/env python3
"""
Minimal example: uses an ONNX policy to control Lite3 via the highlevel SDK.

Usage:
    python simple_controller.py [robot_ip] [model_path]

Defaults: robot_ip=192.168.1.2, model_path=policy.onnx
"""

import sys
import os
import time

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "python"))

from lite3_highlevel import Bridge, RobotState, JointCommand
from lite3_highlevel import OnnxRunner, ObsBuilder, CmdProcessor


def main():
    robot_ip   = sys.argv[1] if len(sys.argv) > 1 else "192.168.1.2"
    model_path = sys.argv[2] if len(sys.argv) > 2 else "policy.onnx"

    print(f"=== Lite3 Highlevel Controller ===")
    print(f"Robot IP: {robot_ip}")
    print(f"Model:    {model_path}")

    # Load ONNX policy
    runner = OnnxRunner(model_path)
    builder = ObsBuilder()
    processor = CmdProcessor()

    last_action = [0.0] * 12
    tick = 0

    def policy(state: RobotState) -> JointCommand:
        nonlocal last_action, tick
        tick += 1

        # Build observation from sensor data
        obs = builder.build(state, last_action=last_action)

        # Run ONNX inference
        raw_action = runner.infer(obs)
        last_action = raw_action

        # Convert to joint command
        cmd = processor.process(raw_action)

        if tick % 50 == 0:
            rpy_deg = tuple(v * 57.3 for v in state.rpy)
            print(f"[{tick:5d}] rpy=({rpy_deg[0]:5.1f},{rpy_deg[1]:5.1f},{rpy_deg[2]:5.1f}) deg"
                  f"  cmd_vel=({state.cmd_vel[0]:.2f},{state.cmd_vel[1]:.2f},{state.cmd_vel[2]:.2f})")

        return cmd

    # Run control loop
    bridge = Bridge(robot_ip=robot_ip)
    try:
        bridge.run(policy, frequency_hz=50)
    except KeyboardInterrupt:
        print("\nShutting down...")
        bridge.shutdown()


if __name__ == "__main__":
    main()
