"""Shared articulated-gripper metadata for Kinova Gen3 Isaac assets."""

from spark_robot.base.backend_capability import ArticulatedGripperSpec


_FINGER_JOINTS = (
    "spark_robotiq_right_finger_joint",
    "spark_robotiq_left_finger_joint",
)
_OPEN_POSITIONS = (0.0, 0.0)
_CLOSED_POSITIONS = (-0.032, 0.032)


KINOVA_SINGLE_ARM_GRIPPERS = (
    ArticulatedGripperSpec(
        side="right",
        joint_names=_FINGER_JOINTS,
        open_positions=_OPEN_POSITIONS,
        closed_positions=_CLOSED_POSITIONS,
    ),
)

KINOVA_DUAL_ARM_GRIPPERS = tuple(
    ArticulatedGripperSpec(
        side=side,
        joint_names=tuple(f"{side}_{name}" for name in _FINGER_JOINTS),
        open_positions=_OPEN_POSITIONS,
        closed_positions=_CLOSED_POSITIONS,
    )
    for side in ("left", "right")
)
