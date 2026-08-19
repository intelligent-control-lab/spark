"""Articulated Robotiq 3F metadata for FANUC Isaac assets."""

from spark_robot.base.backend_capability import ArticulatedGripperSpec


_FINGER_JOINT_SUFFIXES = tuple(
    f"spark_robotiq_{finger}_joint_{joint}" for finger in ("A", "B", "C") for joint in (1, 2, 3)
)
_OPEN_POSITIONS = (0.0, 0.0, 0.0) * 3
# Match the binary close command used by the MuJoCo Robotiq register adapter:
# proximal=62 deg, middle=90 deg, distal=-48.7 deg for every finger.
_CLOSED_POSITIONS = (1.082104, 1.570796, -0.850324) * 3


FANUC_SINGLE_ARM_GRIPPERS = (
    ArticulatedGripperSpec(
        side="right",
        joint_names=_FINGER_JOINT_SUFFIXES,
        open_positions=_OPEN_POSITIONS,
        closed_positions=_CLOSED_POSITIONS,
    ),
)

FANUC_DUAL_ARM_GRIPPERS = tuple(
    ArticulatedGripperSpec(
        side=side,
        joint_names=tuple(f"{side}_{name}" for name in _FINGER_JOINT_SUFFIXES),
        open_positions=_OPEN_POSITIONS,
        closed_positions=_CLOSED_POSITIONS,
    )
    for side in ("left", "right")
)


# The FANUC arm needs a stiff industrial position drive, but applying the same
# 4000/250 gains to the much lighter gripper phalanxes lets a binary close
# command overpower self-contact.  Keep the hand responsive while limiting
# its contact force independently of the six arm joints.
FANUC_SINGLE_ARM_GRIPPER_GAIN_OVERRIDES = tuple(
    (name, 400.0, 25.0) for name in _FINGER_JOINT_SUFFIXES
)
FANUC_DUAL_ARM_GRIPPER_GAIN_OVERRIDES = tuple(
    (f"{side}_{name}", stiffness, damping)
    for side in ("left", "right")
    for name, stiffness, damping in FANUC_SINGLE_ARM_GRIPPER_GAIN_OVERRIDES
)
