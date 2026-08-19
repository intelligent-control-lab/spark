"""Configuration for the Unitree G1 SONIC policy."""

from dataclasses import dataclass, field
from typing import Any

import numpy as np

from spark_policy.core import PolicyConfig


@dataclass
class UnitreeG1SonicPolicyConfig(PolicyConfig):
    client: dict[str, Any] = field(default_factory=dict)
    options: dict[str, Any] = field(default_factory=dict)


def sonic_isaac_actuation_config() -> dict[str, Any]:
    """Return the Isaac plant parameters required by the released policy.

    The simulator adapter consumes these values, but their source of truth is
    kept beside the policy/checkpoint that requires them.
    """
    armature = {
        "5020": 0.003609725,
        "7520_14": 0.010177520,
        "7520_22": 0.025101925,
        "4010": 0.00425,
    }
    omega = 10.0 * 2.0 * np.pi
    stiffness = {key: value * omega**2 for key, value in armature.items()}
    damping = {key: 4.0 * value * omega for key, value in armature.items()}
    motor_class: dict[str, str] = {}
    for side in ("left", "right"):
        for joint in ("hip_pitch", "hip_roll", "knee"):
            motor_class[f"{side}_{joint}_joint"] = "7520_22"
        motor_class[f"{side}_hip_yaw_joint"] = "7520_14"
        for joint in (
            "ankle_pitch",
            "ankle_roll",
            "shoulder_pitch",
            "shoulder_roll",
            "shoulder_yaw",
            "elbow",
            "wrist_roll",
        ):
            motor_class[f"{side}_{joint}_joint"] = "5020"
        for joint in ("wrist_pitch", "wrist_yaw"):
            motor_class[f"{side}_{joint}_joint"] = "4010"
    motor_class.update(
        waist_yaw_joint="7520_14",
        waist_roll_joint="5020",
        waist_pitch_joint="5020",
    )

    def scale(name: str) -> float:
        return 2.0 if "ankle" in name or name in {"waist_roll_joint", "waist_pitch_joint"} else 1.0

    defaults = {name: 0.0 for name in motor_class}
    for side in ("left", "right"):
        defaults.update(
            {
                f"{side}_hip_pitch_joint": -0.312,
                f"{side}_knee_joint": 0.669,
                f"{side}_ankle_pitch_joint": -0.363,
                f"{side}_shoulder_pitch_joint": 0.2,
                f"{side}_shoulder_roll_joint": 0.2 if side == "left" else -0.2,
                f"{side}_elbow_joint": 0.6,
            }
        )
    return {
        "spawn_position": (0.0, 0.0, 0.76),
        "spawn_height": 0.76,
        "native_implicit_pd": True,
        "sim_pd_backend": "implicit",
        "hybrid_implicit_upper_body": False,
        "default_joint_positions": defaults,
        "joint_stiffness": {
            name: stiffness[kind] * scale(name) for name, kind in motor_class.items()
        },
        "joint_damping": {name: damping[kind] * scale(name) for name, kind in motor_class.items()},
        "joint_armature_map": {
            name: armature[kind] * scale(name) for name, kind in motor_class.items()
        },
        "joint_armature": 0.01,
        "scale_action_gains": False,
        "sim_stiffness_scale": 1.0,
        "sim_damping_scale": 1.0,
    }
