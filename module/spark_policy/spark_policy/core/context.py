"""Execution lifecycle context shared by policy components."""

from dataclasses import dataclass


@dataclass(frozen=True)
class ExecutionContext:
    timestamp: float | None = None
    dt: float | None = None
    episode_step: int | None = None


@dataclass(frozen=True)
class ResetContext:
    seed: int | None = None
    timestamp: float | None = None
