"""Configuration for Cartesian PID control."""

from dataclasses import dataclass
from typing import Any

from spark_policy.core import PolicyConfig


@dataclass
class CartesianPIDConfig(PolicyConfig):
    kp_position: Any = 1.0
    kd_position: Any = 0.0
    kp_orientation: Any = 1.0
    kd_orientation: Any = 0.0
