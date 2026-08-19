from dataclasses import dataclass

from spark_policy.core import PolicyConfig


@dataclass
class LinearQuadraticPolicyConfig(PolicyConfig):
    dt: float = 0.1
    horizon_steps: int = 20
    execution_mode: str = "feedback"
