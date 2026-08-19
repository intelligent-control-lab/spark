"""Configuration for trajectory-optimization policies."""

from spark_policy.core import PolicyConfig

from .policy import TrajOptParams


class TrajOptPolicyConfig(TrajOptParams, PolicyConfig):
    """Typed public alias for the existing trajectory-optimization parameters."""
