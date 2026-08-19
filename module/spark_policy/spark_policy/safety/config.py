"""Common configuration contract for safety policies."""

from dataclasses import dataclass, field
from typing import Any

from spark_policy.core import PolicyConfig


@dataclass
class SafetyPolicyConfig(PolicyConfig):
    options: dict[str, Any] = field(default_factory=dict)
