"""
Observation builder — constructs policy input from raw sensor data.

Default format matches the original Lite3 ONNX policy (45-dim):
  [omega*0.25 (3), projected_gravity (3), cmd_vel*max_vel (3),
   joint_pos - default_pos (12), joint_vel*0.05 (12), last_action (12)]
"""

import math
from typing import List, Tuple, Optional
from .bridge import RobotState


class ObsBuilder:
    """Builds observation vectors for Lite3 RL policies.

    Usage:
        builder = ObsBuilder()  # uses defaults from the original ONNX policy
        obs = builder.build(state, last_action=None)  # -> list of 45 floats
    """

    def __init__(self,
                 num_joints: int = 12,
                 omega_scale: float = 0.25,
                 dof_vel_scale: float = 0.05,
                 max_cmd_vel: Tuple[float, float, float] = (0.8, 0.8, 0.8),
                 default_joint_pos: Optional[List[float]] = None,
                 action_scale: Optional[List[float]] = None,
                 obs_dim: int = 45):
        self.num_joints = num_joints

        # Scaling factors
        self.omega_scale = omega_scale
        self.dof_vel_scale = dof_vel_scale
        self.max_cmd_vel = max_cmd_vel

        # Default standing pose (joint positions when standing)
        if default_joint_pos is None:
            # Lite3 standing pose: hip_x=0, hip_y=-0.65, knee=1.30 for all 4 legs
            self.default_joint_pos = [0.0, -0.65, 1.30] * 4
        else:
            self.default_joint_pos = list(default_joint_pos)

        # Action scale (per-joint)
        if action_scale is None:
            self.action_scale = [0.125, 0.25, 0.25] * 4
        else:
            self.action_scale = list(action_scale)

        self.obs_dim = obs_dim
        self._last_action = [0.0] * num_joints

    # ------------------------------------------------------------------
    # Build
    # ------------------------------------------------------------------

    def build(self, state: RobotState, last_action: Optional[List[float]] = None) -> List[float]:
        """Build observation vector from robot state.

        Args:
            state: Raw sensor data from the robot
            last_action: Previous action output (12-dim). If None, uses zeros.

        Returns:
            Observation vector (default 45-dim)
        """
        if last_action is not None:
            self._last_action = list(last_action)

        obs = []
        obs.extend(self._scaled_omega(state))           # 3
        obs.extend(self._projected_gravity(state))      # 3
        obs.extend(self._scaled_cmd_vel(state))         # 3
        obs.extend(self._joint_pos_error(state))        # 12
        obs.extend(self._scaled_joint_vel(state))       # 12
        obs.extend(self._last_action)                    # 12
        return obs

    @property
    def obs_dim(self) -> int:
        return self._obs_dim

    @obs_dim.setter
    def obs_dim(self, value: int):
        self._obs_dim = value

    # ------------------------------------------------------------------
    # Observation components
    # ------------------------------------------------------------------

    def _scaled_omega(self, s: RobotState) -> List[float]:
        return [s.omega[i] * self.omega_scale for i in range(3)]

    def _projected_gravity(self, s: RobotState) -> List[float]:
        return list(s.projected_gravity)

    def _scaled_cmd_vel(self, s: RobotState) -> List[float]:
        return [s.cmd_vel[i] * self.max_cmd_vel[i] for i in range(3)]

    def _joint_pos_error(self, s: RobotState) -> List[float]:
        return [s.joint_pos[i] - self.default_joint_pos[i] for i in range(self.num_joints)]

    def _scaled_joint_vel(self, s: RobotState) -> List[float]:
        return [s.joint_vel[i] * self.dof_vel_scale for i in range(self.num_joints)]


class CmdProcessor:
    """Converts raw policy output into JointCommand.

    Processing pipeline:
      1. action[i] = action_scale[i] * raw_output[i]
      2. joint_pos_des[i] = default_joint_pos[i] + action[i]
      3. Fill kp, kd
    """

    def __init__(self,
                 num_joints: int = 12,
                 default_joint_pos: Optional[List[float]] = None,
                 action_scale: Optional[List[float]] = None,
                 kp: float = 30.0,
                 kd: float = 1.0):
        self.num_joints = num_joints

        if default_joint_pos is None:
            self.default_joint_pos = [0.0, -0.65, 1.30] * 4
        else:
            self.default_joint_pos = list(default_joint_pos)

        if action_scale is None:
            self.action_scale = [0.125, 0.25, 0.25] * 4
        else:
            self.action_scale = list(action_scale)

        self.kp = kp
        self.kd = kd

    def process(self, raw_action: List[float]) -> 'JointCommand':
        """Convert raw policy output to JointCommand."""
        from .bridge import JointCommand
        cmd = JointCommand()
        for i in range(self.num_joints):
            action = raw_action[i] * self.action_scale[i]
            cmd.joint_pos_des[i] = self.default_joint_pos[i] + action
            cmd.joint_vel_des[i] = 0.0
            cmd.kp[i] = self.kp
            cmd.kd[i] = self.kd
            cmd.tau_ff[i] = 0.0
        return cmd
