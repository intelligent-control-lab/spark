"""Public configuration for the Unitree G1 WBT policy."""

from typing import Any

import numpy as np

from .policy import WBTPolicyConfig


# WBT's learned action and gain vectors use this order.  Keeping the names
# beside the policy prevents a simulator adapter from silently assigning the
# right values to the wrong articulation joints.
WBT_JOINT_NAMES = (
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)

# The locomotion and squat experts use the same actuator gains.  Export the
# profile so another G1 executor can be tested against exactly the same plant
# contract instead of silently falling back to agent-specific defaults.
WBT_MOTOR_KPS = np.asarray(
    [
        100,
        100,
        100,
        150,
        40,
        40,
        100,
        100,
        100,
        150,
        40,
        40,
        300,
        300,
        300,
        100,
        100,
        50,
        50,
        20,
        20,
        20,
        100,
        100,
        50,
        50,
        20,
        20,
        20,
    ],
    dtype=float,
)
WBT_MOTOR_KDS = np.asarray(
    [
        2,
        2,
        2,
        4,
        2,
        2,
        2,
        2,
        2,
        4,
        2,
        2,
        3,
        3,
        3,
        2,
        2,
        2,
        2,
        1,
        1,
        1,
        2,
        2,
        2,
        2,
        1,
        1,
        1,
    ],
    dtype=float,
)


def wbt_isaac_actuation_config() -> dict[str, Any]:
    """Return WBT's shared one/parallel Isaac plant configuration.

    Reproduce OpenWBT's Isaac deployment on SPARK's bundled G1 model: native
    position drives at every physics step, the published gains, 5 ms physics
    with four-step policy decimation, lower-body/waist versus arm armatures,
    reduced wrist masses, and the same PhysX solver iteration counts.
    Environment count changes only tensor shape and scene layout.
    """

    lower_body_tokens = ("hip", "knee", "ankle", "waist")
    stiffness = dict(zip(WBT_JOINT_NAMES, WBT_MOTOR_KPS, strict=True))
    damping = dict(zip(WBT_JOINT_NAMES, WBT_MOTOR_KDS, strict=True))
    return {
        # Omitting usd_path deliberately selects the content-addressed USD
        # imported from the robot configuration's bundled URDF.
        "native_implicit_pd": True,
        "hybrid_implicit_upper_body": False,
        "joint_stiffness": stiffness,
        "joint_damping": damping,
        "joint_armature_map": {
            name: 0.01 if any(token in name for token in lower_body_tokens) else 0.001
            for name in WBT_JOINT_NAMES
        },
        "joint_armature": 0.001,
        "joint_friction": 0.1,
        "solver_position_iteration_count": 4,
        "solver_velocity_iteration_count": 0,
        # OpenWBT disables articulation self-contact, but SPARK's safe
        # teleoperation/benchmark contract requires the simulated hands and
        # distal arms to be physically blocked by the legs and torso, just as
        # they are in the MuJoCo plant. This remains robot-owned metadata; the
        # CLI --enable-self-collision flag separately controls safety-volume
        # constraints.
        "allow_self_collision": True,
        "link_mass_scales": {"wrist": 0.5},
        "sim_stiffness_scale": 1.0,
        "sim_damping_scale": 1.0,
    }


__all__ = [
    "WBTPolicyConfig",
    "WBT_JOINT_NAMES",
    "WBT_MOTOR_KPS",
    "WBT_MOTOR_KDS",
    "wbt_isaac_actuation_config",
]
