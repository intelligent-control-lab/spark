"""Configuration for a grounded nominal-plus-safety policy."""

from dataclasses import dataclass
from typing import Any

from spark_policy.core import PolicyConfig


@dataclass
class SafetyFilteredPolicyConfig(PolicyConfig):
    nominal_controller: Any = None
    monitor: Any = None
    safety_filter: Any = None
    clip_action_to_control_limits: bool = True


class TeleopSafetyFilteredPolicyConfig:
    """Default self-contained safe teleoperation policy composition."""

    class_name = "SafetyFilteredPolicy"
    clip_action_to_control_limits = True

    class nominal_controller:
        class_name = "TeleopPIDPolicy"

    class safe_controller:
        class_name = None

        class safety_index:
            class_name = "FirstOrderCollisionSafetyIndex"
            min_distance = {"environment": 0.05, "self": 0.01}
            enable_self_collision = False

        class safe_algo:
            class_name = "ByPassSafeControl"
