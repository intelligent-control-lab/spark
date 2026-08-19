"""Configuration namespace for the grounded SONIC plus safety policy."""

from dataclasses import dataclass
from typing import Any

from spark_policy.core import PolicyConfig


@dataclass
class UnitreeG1SonicSafePolicyConfig(PolicyConfig):
    """Policy and deployment defaults shared by teleop and benchmarks."""

    nominal_controller: Any = None
    monitor: Any = None
    safety_filter: Any = None
    goal_tracking_type: str = "pid"
    auto_launch_server: bool = True
    policy_precision: str = "16"
    planner_precision: str = "16"
    server_startup_timeout: float = 600.0
