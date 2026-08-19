"""Configuration for iLQR trajectory policies."""

from dataclasses import dataclass

from spark_policy.core import PolicyConfig


@dataclass
class ILQRPolicyConfig(PolicyConfig):
    pass
