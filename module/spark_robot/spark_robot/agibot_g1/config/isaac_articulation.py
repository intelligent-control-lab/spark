"""Shared declarative Isaac metadata for AgiBot G1 configurations."""

from spark_robot.base.backend_capability import (
    BackendCapability,
    IsaacArticulationSpec,
    IsaacPlanarBaseSpec,
)

from .joint_defaults import AGIBOT_G1_LOCKED_JOINT_DEFAULTS


# The source meshes contain one uniform gray material.  These neutral white,
# silver, and charcoal values follow AgiBot's published G1 product imagery;
# notably, the production robot does not use blue arm covers.
AGIBOT_G1_VISUAL_LINK_COLORS = (
    ("robot", (0.82, 0.84, 0.85, 1.0)),
    ("base_link", (0.82, 0.84, 0.85, 1.0)),
    ("link_up_down_body", (0.91, 0.92, 0.92, 1.0)),
    ("link_pitch_body", (0.92, 0.93, 0.93, 1.0)),
    ("link_yaw_head", (0.18, 0.19, 0.20, 1.0)),
    ("link_pitch_head", (0.055, 0.060, 0.065, 1.0)),
    ("base_link_[lr]", (0.22, 0.23, 0.24, 1.0)),
    ("link1_[lr]", (0.90, 0.91, 0.91, 1.0)),
    ("link2_[lr]", (0.74, 0.76, 0.77, 1.0)),
    ("link3_[lr]", (0.91, 0.92, 0.92, 1.0)),
    ("link4_[lr]", (0.25, 0.26, 0.27, 1.0)),
    ("link5_[lr]", (0.88, 0.89, 0.90, 1.0)),
    ("link6_[lr]", (0.73, 0.75, 0.76, 1.0)),
    ("link7_[lr]", (0.22, 0.23, 0.24, 1.0)),
    ("*_base_link", (0.16, 0.17, 0.18, 1.0)),
    ("*narrow*", (0.19, 0.20, 0.21, 1.0)),
    ("*wide*", (0.68, 0.70, 0.71, 1.0)),
)


def _joint_names(suffix: str) -> tuple[str, ...]:
    return tuple(f"Joint{index}_{suffix}" for index in range(1, 8))


AGIBOT_G1_ISAAC_JOINT_DEFAULTS = tuple(AGIBOT_G1_LOCKED_JOINT_DEFAULTS.items())
AGIBOT_G1_TORSO_GAIN_OVERRIDES = (
    ("joint_lift_body", 40000.0, 400.0),
    ("joint_body_pitch", 12000.0, 250.0),
)


RIGHT_ARM_ISAAC_ARTICULATION = IsaacArticulationSpec(
    urdf_path="agibot_g1/agibot_g1_mobile_base.urdf",
    joint_names=_joint_names("r"),
    stiffness=8000.0,
    damping=200.0,
    joint_position_defaults=AGIBOT_G1_ISAAC_JOINT_DEFAULTS,
    joint_gain_overrides=AGIBOT_G1_TORSO_GAIN_OVERRIDES,
    allow_self_collision=True,
)

DUAL_ARM_ISAAC_ARTICULATION = IsaacArticulationSpec(
    urdf_path="agibot_g1/agibot_g1_mobile_base.urdf",
    joint_names=_joint_names("l") + _joint_names("r"),
    stiffness=8000.0,
    damping=200.0,
    joint_position_defaults=AGIBOT_G1_ISAAC_JOINT_DEFAULTS,
    joint_gain_overrides=AGIBOT_G1_TORSO_GAIN_OVERRIDES,
    allow_self_collision=True,
)

FIXED_BASE_ISAAC_ARTICULATION = IsaacArticulationSpec(
    urdf_path="agibot_g1/agibot_g1_mobile_base.urdf",
    joint_names=("joint_lift_body", *_joint_names("l"), *_joint_names("r")),
    stiffness=8000.0,
    damping=200.0,
    joint_position_defaults=AGIBOT_G1_ISAAC_JOINT_DEFAULTS,
    joint_gain_overrides=AGIBOT_G1_TORSO_GAIN_OVERRIDES,
    allow_self_collision=True,
)

MOBILE_BASE_ISAAC_ARTICULATION = IsaacArticulationSpec(
    urdf_path="agibot_g1/agibot_g1_mobile_base.urdf",
    joint_names=(
        "joint_lift_body",
        "joint_body_pitch",
        *_joint_names("l"),
        *_joint_names("r"),
        "spark_planar_x_joint",
        "spark_planar_y_joint",
        "spark_planar_yaw_joint",
    ),
    planar_base=IsaacPlanarBaseSpec(),
    stiffness=8000.0,
    damping=200.0,
    joint_position_defaults=AGIBOT_G1_ISAAC_JOINT_DEFAULTS,
    joint_gain_overrides=AGIBOT_G1_TORSO_GAIN_OVERRIDES,
    allow_self_collision=True,
)


def agibot_g1_isaac_capability(agent_class_name: str) -> BackendCapability:
    """Build consistent capability metadata for an embodiment-specific agent."""

    return BackendCapability(
        agent_class_name,
        batched=True,
        rendering=True,
        tensor_io=True,
        validation="validated",
        notes=(
            "CUDA hold/tracking validated at 1 and 4 environments; generated planar roots "
            "and bundled AgiBot articulation retain the recorded provenance caveat."
        ),
    )
