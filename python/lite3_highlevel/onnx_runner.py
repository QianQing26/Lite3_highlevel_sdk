"""
ONNX Runtime inference wrapper for Lite3 RL policies.
"""

import os
from typing import List, Optional
import numpy as np


class OnnxRunner:
    """Loads an ONNX policy model and runs inference.

    Usage:
        runner = OnnxRunner("policy.onnx")
        action = runner.infer(observation)  # observation: list[float] or np.ndarray
    """

    def __init__(self, model_path: str):
        """
        Args:
            model_path: Path to .onnx model file
        """
        import onnxruntime as ort
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"ONNX model not found: {model_path}")

        self._session = ort.InferenceSession(model_path)
        self._input_name = self._session.get_inputs()[0].name
        self._output_name = self._session.get_outputs()[0].name
        self._obs_dim = self._session.get_inputs()[0].shape[1]
        self._act_dim = self._session.get_outputs()[0].shape[1]

        print(f"[OnnxRunner] Loaded {model_path}")
        print(f"  Input : {self._input_name} [{1}, {self._obs_dim}]")
        print(f"  Output: {self._output_name} [{1}, {self._act_dim}]")

        # Warm-up
        dummy = np.zeros((1, self._obs_dim), dtype=np.float32)
        self._session.run([self._output_name], {self._input_name: dummy})

    @property
    def obs_dim(self) -> int:
        return self._obs_dim

    @property
    def act_dim(self) -> int:
        return self._act_dim

    def infer(self, obs: List[float]) -> List[float]:
        """Run inference on a single observation.

        Args:
            obs: Observation vector (list of length obs_dim)

        Returns:
            Raw action vector (list of length act_dim)
        """
        arr = np.array([obs], dtype=np.float32)
        outputs = self._session.run([self._output_name], {self._input_name: arr})
        return outputs[0][0].tolist()
