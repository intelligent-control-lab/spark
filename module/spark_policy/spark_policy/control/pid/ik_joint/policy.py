"""Cartesian-reference tracking through IK followed by joint PID."""

from dataclasses import dataclass

import numpy as np

from spark_policy.core import JointReference
from ..joint import JointPIDController


@dataclass
class IKJointPIDController:
    robot_kinematics: object
    joint_controller: JointPIDController

    def reset(self, context=None) -> None:
        self.joint_controller.reset(context)

    def compute(
        self,
        target_frames: list[np.ndarray],
        position: np.ndarray,
        velocity: np.ndarray,
        dt: float | None = None,
    ) -> tuple[np.ndarray, dict]:
        target, detail = self.robot_kinematics.inverse_kinematics(target_frames, position)
        if isinstance(detail, dict) and "success" in detail:
            success = bool(detail["success"])
        elif isinstance(detail, (bool, np.bool_)):
            success = bool(detail)
        else:
            success = target is not None
        if target is None:
            return np.zeros_like(position, dtype=float), {"ik_success": False, "ik_detail": detail}
        command = self.joint_controller.compute(
            JointReference(position=np.asarray(target, dtype=float)),
            position,
            velocity,
            dt,
        )
        return command, {
            "ik_success": success,
            "ik_detail": detail,
            "joint_target": np.asarray(target),
        }
