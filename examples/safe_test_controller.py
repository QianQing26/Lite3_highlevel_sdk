#!/usr/bin/env python3
"""
Safe hardware test — no ONNX policy, just standing pose with tiny joint motion.

This script sends a fixed standing-pose command to the robot, with an
optional small sinusoidal oscillation on the knee joints to verify that
the communication bridge and PD control are working correctly.

Usage:
    python safe_test_controller.py [robot_ip] [--amplitude 0.05] [--frequency 0.3]

Defaults:
    robot_ip  = 192.168.1.120
    amplitude = 0.05 rad  (≈ 2.9 degrees) — safe knee perturbation
    frequency = 0.3 Hz     — slow oscillation

What to expect:
    1. Handshake completes (robot stands up via gamepad)
    2. Robot holds standing pose — all 12 joints at fixed positions
    3. Knee joints oscillate gently (±0.05 rad) — barely visible,
       just enough to see the robot is responding to commands
    4. Press Ctrl+C to exit — Bridge sends shutdown → robot does JointDamping
"""

import sys
import os
import time
import math

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "python"))

from lite3_highlevel import Bridge, RobotState, JointCommand, RobotMotionState


# ============================================================================
# Standing pose — Lite3 default (12 joints, 4 legs × 3 DOF each)
# ============================================================================
# Joint order per leg: HipX (abduction), HipY (pitch), Knee
# FR and HR legs mirror HipX (negative sign)

# fmt: off
STANDING_POSE = [
     0.0, -0.65, 1.30,   # FL: hip_x,   hip_y,   knee
     0.0, -0.65, 1.30,   # FR: hip_x,   hip_y,   knee (mirrored in code)
     0.0, -0.65, 1.30,   # HL: hip_x,   hip_y,   knee
     0.0, -0.65, 1.30,   # HR: hip_x,   hip_y,   knee (mirrored in code)
]
# fmt: on

# FR and HR have mirrored HipX
STANDING_POSE[3] = -STANDING_POSE[3]  # FR hip_x

# PD gains — moderate stiffness for safe testing
KP = 30.0
KD = 1.0


def make_standing_command(elapsed_s: float = 0.0,
                          amplitude: float = 0.0,
                          frequency: float = 0.0) -> JointCommand:
    """Build a JointCommand with standing pose + optional knee oscillation.

    Args:
        elapsed_s: Seconds since policy started (for sine phase).
        amplitude: Peak knee perturbation in radians (0 = no motion).
        frequency: Oscillation frequency in Hz (0 = no motion).

    Returns:
        JointCommand ready to send to the robot.
    """
    cmd = JointCommand()
    pose = list(STANDING_POSE)  # copy

    if amplitude > 0 and frequency > 0:
        # Apply sine perturbation to all 4 knee joints (indices 2, 5, 8, 11)
        offset = amplitude * math.sin(2.0 * math.pi * frequency * elapsed_s)
        for knee_idx in (2, 5, 8, 11):
            pose[knee_idx] += offset

    for i in range(12):
        cmd.joint_pos_des[i] = pose[i]
        cmd.joint_vel_des[i] = 0.0
        cmd.kp[i] = KP
        cmd.kd[i] = KD
        cmd.tau_ff[i] = 0.0

    return cmd


# ============================================================================
# Safe policy — called by Bridge at 50 Hz
# ============================================================================

class SafePolicy:
    """A policy that holds standing pose with optional gentle knee motion.

    The amplitude and frequency are intentionally small so you can
    confirm the robot is under control without risk of aggressive motion.
    """

    def __init__(self, amplitude: float = 0.05, frequency: float = 0.3):
        self.amplitude = amplitude
        self.frequency = frequency
        self.tick = 0
        self.start_time: float | None = None  # set when RL control begins

    def __call__(self, state: RobotState) -> JointCommand:
        self.tick += 1

        if self.start_time is None:
            self.start_time = time.monotonic()

        elapsed = time.monotonic() - self.start_time
        cmd = make_standing_command(elapsed, self.amplitude, self.frequency)

        # Print status every 50 ticks (~1 second)
        if self.tick % 50 == 0:
            rpy_deg = tuple(v * 57.3 for v in state.rpy)
            jp = state.joint_pos
            knee_offset = self.amplitude * math.sin(
                2.0 * math.pi * self.frequency * elapsed
            )

            print(
                f"[t={elapsed:5.1f}s] "
                f"rpy=({rpy_deg[0]:+5.1f}, {rpy_deg[1]:+5.1f}, {rpy_deg[2]:+5.1f}) deg | "
                f"FL_knee={jp[2]:+.3f} FR_knee={jp[5]:+.3f} "
                f"HL_knee={jp[8]:+.3f} HR_knee={jp[11]:+.3f} | "
                f"knee_cmd_offset={knee_offset:+.4f} rad | "
                f"state={state.current_state}"
            )

        return cmd


# ============================================================================
# Main
# ============================================================================

def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="Lite3 safe hardware test — standing pose + gentle knee motion"
    )
    parser.add_argument(
        "robot_ip", nargs="?", default="192.168.1.120",
        help="RK3588 IP address (default: 192.168.1.120)"
    )
    parser.add_argument(
        "--amplitude", "-a", type=float, default=0.05,
        help="Knee oscillation amplitude in radians (default: 0.05 ≈ 2.9°)"
    )
    parser.add_argument(
        "--frequency", "-f", type=float, default=0.3,
        help="Oscillation frequency in Hz (default: 0.3)"
    )
    parser.add_argument(
        "--no-motion", action="store_true",
        help="Disable knee oscillation entirely (pure standing hold)"
    )
    args = parser.parse_args()

    if args.no_motion:
        args.amplitude = 0.0
        args.frequency = 0.0

    print("=" * 60)
    print("Lite3 Safe Hardware Test")
    print("=" * 60)
    print(f"Robot IP : {args.robot_ip}")
    print(f"Amplitude: {args.amplitude:.3f} rad ({args.amplitude * 57.3:.1f}°)")
    print(f"Frequency: {args.frequency:.1f} Hz")
    print(f"Mode     : {'pure standing hold' if args.no_motion else 'knee oscillation'}")
    print()
    print("Instructions:")
    print("  1. Make sure ./rl_deploy is running on RK3588")
    print("  2. Press gamepad StandUp key to start standing sequence")
    print("  3. Wait for handshake to complete (automatic)")
    print("  4. Robot will hold standing pose with gentle knee motion")
    print("  5. Press Ctrl+C to safely shut down")
    print("=" * 60)

    policy = SafePolicy(amplitude=args.amplitude, frequency=args.frequency)
    bridge = Bridge(robot_ip=args.robot_ip)

    try:
        bridge.run(policy, frequency_hz=50)
    except KeyboardInterrupt:
        print("\nShutting down safely...")
        bridge.shutdown()


if __name__ == "__main__":
    main()
