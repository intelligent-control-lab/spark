from dataclasses import dataclass

from spark_policy.core import PolicyConfig


@dataclass
class ILCPolicyConfig(PolicyConfig):
    horizon: int = 1
    dt: float = 1.0
    discretization: str = "ZOH"
