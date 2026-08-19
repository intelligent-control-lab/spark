"""Current base/arm multi-target PID implementation."""

from spark_policy.control.pid.base import BasePIDPolicy


class MultiTargetPIDPolicy(BasePIDPolicy):
    """Explicit name for the legacy PID policy's multi-target behavior."""


__all__ = ["MultiTargetPIDPolicy"]
