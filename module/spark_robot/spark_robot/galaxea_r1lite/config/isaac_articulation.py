"""Shared declarative Isaac metadata for Galaxea R1 Lite configurations."""

from spark_robot.base.backend_capability import (
    ArticulatedGripperSpec,
    BackendCapability,
    IsaacArticulationSpec,
    IsaacPlanarBaseSpec,
)


R1_LITE_GRIPPERS = (
    ArticulatedGripperSpec(
        side="left",
        joint_names=("left_gripper_finger_joint1", "left_gripper_finger_joint2"),
        actuator_names=(
            "left_gripper_finger_joint1_ctrl",
            "left_gripper_finger_joint2_ctrl",
        ),
        open_positions=(0.05, -0.05),
        closed_positions=(0.0, 0.0),
    ),
    ArticulatedGripperSpec(
        side="right",
        joint_names=("right_gripper_finger_joint1", "right_gripper_finger_joint2"),
        actuator_names=(
            "right_gripper_finger_joint1_ctrl",
            "right_gripper_finger_joint2_ctrl",
        ),
        open_positions=(0.05, -0.05),
        closed_positions=(0.0, 0.0),
    ),
)


def _joint_names(prefix: str) -> tuple[str, ...]:
    return tuple(f"{prefix}_joint{index}" for index in range(1, 7))


RIGHT_ARM_ISAAC_ARTICULATION = IsaacArticulationSpec(
    urdf_path="galaxea_r1lite/r1lite.urdf",
    joint_names=_joint_names("right_arm"),
    stiffness=3000.0,
    damping=120.0,
    allow_self_collision=True,
    grippers=(R1_LITE_GRIPPERS[1],),
)

DUAL_ARM_ISAAC_ARTICULATION = IsaacArticulationSpec(
    urdf_path="galaxea_r1lite/r1lite.urdf",
    joint_names=_joint_names("left_arm") + _joint_names("right_arm"),
    stiffness=3000.0,
    damping=120.0,
    allow_self_collision=True,
    grippers=R1_LITE_GRIPPERS,
)

FIXED_BASE_ISAAC_ARTICULATION = IsaacArticulationSpec(
    urdf_path="galaxea_r1lite/r1lite.urdf",
    joint_names=(
        "torso_joint1",
        "torso_joint2",
        "torso_joint3",
        *_joint_names("left_arm"),
        *_joint_names("right_arm"),
    ),
    stiffness=3000.0,
    damping=120.0,
    allow_self_collision=True,
    grippers=R1_LITE_GRIPPERS,
)

MOBILE_BASE_ISAAC_ARTICULATION = IsaacArticulationSpec(
    urdf_path="galaxea_r1lite/r1lite.urdf",
    joint_names=(
        "spark_planar_x_joint",
        "spark_planar_y_joint",
        "spark_planar_yaw_joint",
        "torso_joint1",
        "torso_joint2",
        "torso_joint3",
        *_joint_names("left_arm"),
        *_joint_names("right_arm"),
    ),
    planar_base=IsaacPlanarBaseSpec(),
    stiffness=3000.0,
    damping=120.0,
    allow_self_collision=True,
    grippers=R1_LITE_GRIPPERS,
)

ISAAC_CAPABILITY = BackendCapability(
    "ConfiguredIsaacAgent",
    batched=True,
    rendering=True,
    tensor_io=True,
    validation="validated",
    notes=(
        "CPU single-environment and CUDA 16-environment hold/tracking validated "
        "for the adapted official Galaxea R1 Lite 2025 articulation and generated "
        "planar root."
    ),
)
