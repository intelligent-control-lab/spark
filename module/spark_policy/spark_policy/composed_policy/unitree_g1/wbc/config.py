from dataclasses import dataclass
from typing import Any

from spark_policy.core import PolicyConfig


@dataclass
class UnitreeG1WBCComposedPolicyConfig(PolicyConfig):
    goal_tracking_policy: Any = None
    wbc_policy: Any = None
