"""Backend-neutral actuator data for the Unitree G1 dexterous hands."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class UnitreeG1GripperSpec:
    """Joint topology and default servo parameters for one G1 hand."""

    side: str
    joint_names: tuple[str, ...]
    open_position: tuple[float, ...]
    closed_position: tuple[float, ...]
    lower_limit: tuple[float, ...]
    upper_limit: tuple[float, ...]
    kp: float = 10.0
    kd: float = 1.0

    def __post_init__(self) -> None:
        lengths = {
            len(self.joint_names),
            len(self.open_position),
            len(self.closed_position),
            len(self.lower_limit),
            len(self.upper_limit),
        }
        if lengths != {7}:
            raise ValueError("A Unitree G1 gripper specification must contain seven joints")

    def target(self, closed: bool) -> np.ndarray:
        values = self.closed_position if closed else self.open_position
        return np.asarray(values, dtype=float)

    def clamp(self, target) -> np.ndarray:
        return np.clip(
            np.asarray(target, dtype=float).reshape(7),
            np.asarray(self.lower_limit, dtype=float),
            np.asarray(self.upper_limit, dtype=float),
        )


UNITREE_G1_GRIPPER_SPECS = {
    "left": UnitreeG1GripperSpec(
        side="left",
        joint_names=(
            "left_hand_thumb_0_joint",
            "left_hand_thumb_1_joint",
            "left_hand_thumb_2_joint",
            "left_hand_index_0_joint",
            "left_hand_index_1_joint",
            "left_hand_middle_0_joint",
            "left_hand_middle_1_joint",
        ),
        open_position=(-0.4, 0.2, 0.6, -0.8, -0.6, -0.8, -0.6),
        closed_position=(-0.4, 0.2, 0.9, -0.8, -0.9, -0.8, -0.9),
        lower_limit=(-1.05, -0.724, 0.0, -1.57, -1.75, -1.57, -1.75),
        upper_limit=(1.05, 1.05, 1.75, 0.0, 0.0, 0.0, 0.0),
    ),
    "right": UnitreeG1GripperSpec(
        side="right",
        joint_names=(
            "right_hand_thumb_0_joint",
            "right_hand_thumb_1_joint",
            "right_hand_thumb_2_joint",
            "right_hand_index_0_joint",
            "right_hand_index_1_joint",
            "right_hand_middle_0_joint",
            "right_hand_middle_1_joint",
        ),
        open_position=(-0.4, -0.2, -0.6, 0.8, 0.6, 0.8, 0.6),
        closed_position=(-0.4, -0.2, -0.9, 0.8, 0.9, 0.8, 0.9),
        lower_limit=(-1.05, -1.05, -1.75, 0.0, 0.0, 0.0, 0.0),
        upper_limit=(1.05, 0.742, 0.0, 1.57, 1.75, 1.57, 1.75),
    ),
}


def unitree_g1_gripper_spec(side: str) -> UnitreeG1GripperSpec:
    try:
        return UNITREE_G1_GRIPPER_SPECS[str(side).lower()]
    except KeyError as exc:
        raise ValueError(f"Unknown Unitree G1 gripper side: {side!r}") from exc
