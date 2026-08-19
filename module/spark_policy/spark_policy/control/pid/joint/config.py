"""Configuration for joint-space PID control."""

from dataclasses import dataclass
from typing import Any

from spark_policy.core import PolicyConfig


@dataclass
class JointPIDConfig(PolicyConfig):
    kp: Any = 1.0
    kd: Any = 0.0
    ki: Any = 0.0
    integral_limit: Any | None = None
