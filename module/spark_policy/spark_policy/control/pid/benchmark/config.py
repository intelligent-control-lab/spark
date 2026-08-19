"""Configuration for the benchmark PID policy variant."""

from dataclasses import dataclass

from spark_policy.core import PolicyConfig


@dataclass
class BenchmarkPIDPolicyConfig(PolicyConfig):
    pass
