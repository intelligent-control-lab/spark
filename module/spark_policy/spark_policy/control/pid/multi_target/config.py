"""Configuration for multi-target PID control."""

from dataclasses import dataclass

from spark_policy.core import PolicyConfig


@dataclass
class MultiTargetPIDConfig(PolicyConfig):
    pass
