"""Configuration for the Unitree G1 learned Sport controller."""

from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class UnitreeG1SportPolicyConfig:
    num_threads: int = 1
    model_path: str = field(
        default_factory=lambda: str(Path(__file__).with_name("resources") / "motion.pt")
    )
    control_dt: float = 0.02
    period: float = 0.8
    command_scale: tuple[float, float, float] = (2.0, 2.0, 0.25)
    command_limit: tuple[float, float, float] = (0.15, 0.12, 1.0)
    dof_position_scale: float = 1.0
    dof_velocity_scale: float = 0.05
    angular_velocity_scale: float = 0.25
    action_scale: float = 0.25
    default_lower_body_position: tuple[float, ...] = (
        -0.1,
        0.0,
        0.0,
        0.3,
        -0.2,
        0.0,
        -0.1,
        0.0,
        0.0,
        0.3,
        -0.2,
        0.0,
    )
