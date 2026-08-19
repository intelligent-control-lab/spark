"""Configuration namespace for Kalman-family estimation policies."""

from dataclasses import dataclass

from spark_policy.core import PolicyConfig


@dataclass
class KalmanPolicyConfig(PolicyConfig):
    pass
