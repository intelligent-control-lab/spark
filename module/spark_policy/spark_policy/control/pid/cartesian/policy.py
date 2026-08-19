"""Cartesian PID building block.

The controller computes a task-space command; mapping it to robot controls is
provided by an injected callable so Jacobian-transpose, pseudoinverse, and
operational-space mappings remain robot/controller choices.
"""

from dataclasses import dataclass
from typing import Callable

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass
class CartesianPIDController:
    kp_position: np.ndarray | float
    kd_position: np.ndarray | float
    kp_orientation: np.ndarray | float
    kd_orientation: np.ndarray | float
    command_mapper: Callable[[np.ndarray], np.ndarray] | None = None

    def reset(self, context=None) -> None:
        pass

    def compute(
        self,
        target_frame: np.ndarray,
        current_frame: np.ndarray,
        current_twist: np.ndarray | None = None,
        target_twist: np.ndarray | None = None,
    ) -> np.ndarray:
        target_frame = np.asarray(target_frame, dtype=float)
        current_frame = np.asarray(current_frame, dtype=float)
        current_twist = (
            np.zeros(6) if current_twist is None else np.asarray(current_twist, dtype=float)
        )
        target_twist = (
            np.zeros(6) if target_twist is None else np.asarray(target_twist, dtype=float)
        )

        position_error = target_frame[:3, 3] - current_frame[:3, 3]
        relative_rotation = target_frame[:3, :3] @ current_frame[:3, :3].T
        orientation_error = Rotation.from_matrix(relative_rotation).as_rotvec()
        twist_error = target_twist - current_twist
        task_command = np.concatenate(
            [
                np.asarray(self.kp_position) * position_error
                + np.asarray(self.kd_position) * twist_error[:3],
                np.asarray(self.kp_orientation) * orientation_error
                + np.asarray(self.kd_orientation) * twist_error[3:],
            ]
        )
        if self.command_mapper is not None:
            return np.asarray(self.command_mapper(task_command), dtype=float)
        return task_command
