"""Configuration for the PID-based Unitree G1 WBC policy variant."""

from dataclasses import dataclass

from spark_policy.core import PolicyConfig


@dataclass
class UnitreeG1WBCPIDPolicyConfig(PolicyConfig):
    pass
