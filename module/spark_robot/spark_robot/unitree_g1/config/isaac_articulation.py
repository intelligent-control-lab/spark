"""Declarative Isaac articulations for reduced Unitree G1 configurations."""

from spark_robot.base.backend_capability import IsaacArticulationSpec


LEFT_ARM_JOINTS = (
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
)
RIGHT_ARM_JOINTS = tuple(name.replace("left_", "right_") for name in LEFT_ARM_JOINTS)
WAIST_JOINTS = ("waist_yaw_joint", "waist_roll_joint", "waist_pitch_joint")
PLANAR_JOINTS = ("pelvis_x_joint", "pelvis_y_joint", "pelvis_yaw_joint")

_SPAWN_HEIGHT = (0.0, 0.0, 0.793)
_RIGHT_DEFAULTS = (("right_shoulder_roll_joint", -0.5),)
_DUAL_DEFAULTS = (
    ("left_shoulder_roll_joint", 0.5),
    ("left_elbow_joint", 1.57),
    ("right_shoulder_roll_joint", -0.5),
    ("right_elbow_joint", 1.57),
)
_FIXED_DEFAULTS = (
    ("left_shoulder_roll_joint", 0.5),
    ("right_shoulder_roll_joint", -0.5),
)


RIGHT_ARM_ISAAC_ARTICULATION = IsaacArticulationSpec(
    urdf_path="unitree_g1/urdf/g1_29dof_right_arm.urdf",
    joint_names=RIGHT_ARM_JOINTS,
    fixed_base=True,
    joint_position_defaults=_RIGHT_DEFAULTS,
    base_translation=_SPAWN_HEIGHT,
)

DUAL_ARM_ISAAC_ARTICULATION = IsaacArticulationSpec(
    urdf_path="unitree_g1/urdf/g1_29dof_dual_arm.urdf",
    joint_names=LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS,
    fixed_base=True,
    joint_position_defaults=_DUAL_DEFAULTS,
    base_translation=_SPAWN_HEIGHT,
)

FIXED_BASE_ISAAC_ARTICULATION = IsaacArticulationSpec(
    urdf_path="unitree_g1/urdf/g1_29dof_fixed_base.urdf",
    joint_names=WAIST_JOINTS + LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS,
    fixed_base=True,
    joint_position_defaults=_FIXED_DEFAULTS,
    base_translation=_SPAWN_HEIGHT,
)

MOBILE_BASE_ISAAC_ARTICULATION = IsaacArticulationSpec(
    urdf_path="unitree_g1/urdf/g1_29dof_mobile_base.urdf",
    joint_names=WAIST_JOINTS + LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS + PLANAR_JOINTS,
    fixed_base=True,
    joint_position_defaults=_FIXED_DEFAULTS,
    base_translation=_SPAWN_HEIGHT,
)
