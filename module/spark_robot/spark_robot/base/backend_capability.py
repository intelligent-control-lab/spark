"""Declarative backend and simulator-dynamics capability records."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Literal


ValidationStatus = Literal["declared", "experimental", "validated"]
StateOwner = Literal["model", "simulator", "hardware"]
IsaacControlMode = Literal["position", "velocity", "effort"]


@dataclass(frozen=True)
class ArticulatedGripperSpec:
    """Describe an auxiliary articulated gripper outside the robot DoF vector."""

    side: Literal["left", "right"]
    joint_names: tuple[str, ...]
    open_positions: tuple[float, ...]
    closed_positions: tuple[float, ...]
    actuator_names: tuple[str, ...] = ()

    def __post_init__(self) -> None:
        size = len(self.joint_names)
        if size < 1 or len(set(self.joint_names)) != size:
            raise ValueError("Gripper joint_names must be non-empty and unique")
        if len(self.open_positions) != size or len(self.closed_positions) != size:
            raise ValueError("Gripper targets must match joint_names")
        if self.actuator_names and len(self.actuator_names) != size:
            raise ValueError("Gripper actuator_names must match joint_names when supplied")
        if not all(
            math.isfinite(value) for value in (*self.open_positions, *self.closed_positions)
        ):
            raise ValueError("Gripper targets must be finite")


@dataclass(frozen=True)
class BackendCapability:
    """Describe what one robot/backend adapter publicly supports.

    The record is intentionally independent of simulator packages so
    ``spark_robot`` can expose capability metadata without importing MuJoCo,
    Isaac, ROS, or a hardware SDK.
    """

    agent_class_name: str
    single_environment: bool = True
    batched: bool = False
    rendering: bool = False
    tensor_io: bool = False
    real_hardware: bool = False
    validation: ValidationStatus = "declared"
    notes: str = ""


@dataclass(frozen=True)
class SimulatorDynamicsSpec:
    """Qualified execution settings for one simulator-dynamics adapter."""

    physics_dt: float = 0.002
    control_decimation: int = 10
    state_owner: StateOwner = "simulator"
    command_contract: str = "joint_control"
    control_mode: str = "position_target"
    allows_state_projection: bool = False
    hold_duration: float = 2.0
    max_position_drift: float = 0.1
    max_abs_velocity: float = 1.0

    def __post_init__(self) -> None:
        if self.physics_dt <= 0.0:
            raise ValueError("physics_dt must be positive")
        if self.control_decimation < 1:
            raise ValueError("control_decimation must be positive")
        if self.hold_duration <= 0.0:
            raise ValueError("hold_duration must be positive")
        if self.max_position_drift < 0.0 or self.max_abs_velocity < 0.0:
            raise ValueError("stability thresholds must be non-negative")

    @property
    def control_period(self) -> float:
        return self.physics_dt * self.control_decimation


@dataclass(frozen=True)
class IsaacArticulationInstanceSpec:
    """Place one prefixed URDF instance in a composite articulation."""

    prefix: str
    translation: tuple[float, float, float] = (0.0, 0.0, 0.0)
    rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        if not self.prefix or "/" in self.prefix:
            raise ValueError("Isaac articulation instance prefix must be non-empty and slash-free")
        if len(self.translation) != 3 or len(self.rpy) != 3:
            raise ValueError("Isaac articulation instance poses must have three components")
        if not all(math.isfinite(value) for value in (*self.translation, *self.rpy)):
            raise ValueError("Isaac articulation instance poses must be finite")


@dataclass(frozen=True)
class IsaacPlanarBaseSpec:
    """Add global x/y translation and yaw joints below a URDF root link."""

    joint_names: tuple[str, str, str] = (
        "spark_planar_x_joint",
        "spark_planar_y_joint",
        "spark_planar_yaw_joint",
    )
    translation_limit: float = 100.0
    effort_limit: float = 5000.0
    linear_velocity_limit: float = 5.0
    angular_velocity_limit: float = 10.0
    damping: float = 20.0

    def __post_init__(self) -> None:
        if len(set(self.joint_names)) != 3 or any(not name for name in self.joint_names):
            raise ValueError("Isaac planar-base joint names must be three unique names")
        values = (
            self.translation_limit,
            self.effort_limit,
            self.linear_velocity_limit,
            self.angular_velocity_limit,
            self.damping,
        )
        if not all(math.isfinite(value) and value > 0.0 for value in values):
            raise ValueError("Isaac planar-base limits and damping must be finite and positive")


@dataclass(frozen=True)
class IsaacArticulationSpec:
    """Simulator-independent description of an Isaac articulation asset."""

    urdf_path: str
    joint_names: tuple[str, ...]
    fixed_base: bool = True
    merge_fixed_joints: bool = False
    instances: tuple[IsaacArticulationInstanceSpec, ...] = ()
    planar_base: IsaacPlanarBaseSpec | None = None
    control_mode: IsaacControlMode = "position"
    stiffness: float = 400.0
    damping: float = 40.0
    joint_position_defaults: tuple[tuple[str, float], ...] = ()
    joint_gain_overrides: tuple[tuple[str, float, float], ...] = ()
    allow_self_collision: bool = False
    grippers: tuple[ArticulatedGripperSpec, ...] = ()
    base_translation: tuple[float, float, float] = (0.0, 0.0, 0.0)
    base_rpy: tuple[float, float, float] = (0.0, 0.0, 0.0)

    def __post_init__(self) -> None:
        if len(self.base_translation) != 3 or not all(
            math.isfinite(value) for value in self.base_translation
        ):
            raise ValueError("Isaac articulation base_translation must contain three finite values")
        if len(self.base_rpy) != 3 or not all(math.isfinite(value) for value in self.base_rpy):
            raise ValueError("Isaac articulation base_rpy must contain three finite values")
        if not self.urdf_path:
            raise ValueError("urdf_path must not be empty")
        if not self.joint_names:
            raise ValueError("joint_names must not be empty")
        if len(set(self.joint_names)) != len(self.joint_names):
            raise ValueError("joint_names must be unique")
        instance_prefixes = [instance.prefix for instance in self.instances]
        if len(set(instance_prefixes)) != len(instance_prefixes):
            raise ValueError("Isaac articulation instance prefixes must be unique")
        if len(self.instances) > 1 and not self.merge_fixed_joints:
            raise ValueError("Composite Isaac articulations must merge fixed joints")
        if self.planar_base is not None and not self.fixed_base:
            raise ValueError("Planar-base articulations require a fixed generated world root")
        if self.stiffness < 0.0 or self.damping < 0.0:
            raise ValueError("Isaac drive gains must be non-negative")
        default_names = []
        for joint_name, position in self.joint_position_defaults:
            if not joint_name or not math.isfinite(position):
                raise ValueError("Isaac joint defaults require a name and finite position")
            default_names.append(joint_name)
        if len(set(default_names)) != len(default_names):
            raise ValueError("Isaac joint default names must be unique")
        overridden_names = []
        for joint_name, stiffness, damping in self.joint_gain_overrides:
            if not joint_name:
                raise ValueError("Isaac gain override names must not be empty")
            if not all(math.isfinite(value) and value >= 0.0 for value in (stiffness, damping)):
                raise ValueError("Isaac joint gain overrides must be finite and non-negative")
            overridden_names.append(joint_name)
        if len(set(overridden_names)) != len(overridden_names):
            raise ValueError("Isaac joint gain override names must be unique")
        gripper_sides = [gripper.side for gripper in self.grippers]
        if len(set(gripper_sides)) != len(gripper_sides):
            raise ValueError("Isaac gripper sides must be unique")
        gripper_joint_names = [name for gripper in self.grippers for name in gripper.joint_names]
        if len(set(gripper_joint_names)) != len(gripper_joint_names):
            raise ValueError("Isaac gripper joint names must be unique")
