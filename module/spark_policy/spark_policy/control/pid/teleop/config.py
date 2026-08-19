"""Configuration for the teleoperation PID policy variant."""

from dataclasses import dataclass

from spark_policy.core import PolicyConfig


@dataclass
class TeleopPIDPolicyConfig(PolicyConfig):
    pass
