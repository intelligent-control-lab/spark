"""Configuration namespace for recursive parameter-estimation policies."""

from dataclasses import dataclass

from spark_policy.core import PolicyConfig


@dataclass
class RecursiveEstimatorPolicyConfig(PolicyConfig):
    pass
