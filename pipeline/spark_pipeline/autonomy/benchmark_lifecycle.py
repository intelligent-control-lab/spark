"""Small backend-neutral helpers for parallel benchmark episode lifecycles."""

from __future__ import annotations


def reset_quota_reached(completed_episodes, requested_resets: int) -> bool:
    """Return whether every environment has completed the finite reset quota.

    Parallel environments are intentionally not disabled one row at a time.
    Faster rows keep resetting and sampling new episodes until the slowest row
    reaches the requested quota, at which point the benchmark ends together.
    ``-1`` retains the interactive continuous-reset mode.
    """

    requested_resets = int(requested_resets)
    if requested_resets < 0:
        return False
    if requested_resets == 0:
        raise ValueError("requested_resets must be positive or -1")
    reached = (completed_episodes >= requested_resets).all()
    if hasattr(reached, "item"):
        reached = reached.item()
    return bool(reached)
