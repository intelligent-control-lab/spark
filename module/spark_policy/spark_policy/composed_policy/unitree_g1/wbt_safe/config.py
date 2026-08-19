"""Configuration namespace for the grounded WBT plus safety policy."""

from dataclasses import dataclass
from typing import Any

from spark_policy.core import PolicyConfig


@dataclass
class UnitreeG1WBTSafePolicyConfig(PolicyConfig):
    nominal_controller: Any = None
    monitor: Any = None
    safety_filter: Any = None
    goal_tracking_type: str = "pid"
