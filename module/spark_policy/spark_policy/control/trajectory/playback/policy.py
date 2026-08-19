"""Stateful execution of a precomputed control trajectory."""

from __future__ import annotations

import numpy as np


class TrajectoryPlaybackController:
    def __init__(self, controls: np.ndarray | None = None):
        self.set_trajectory(controls)

    def set_trajectory(self, controls: np.ndarray | None) -> None:
        self.controls = None if controls is None else np.asarray(controls, dtype=float)
        self.reset()

    def reset(self, context=None) -> None:
        self.index = 0

    def next_control(self) -> np.ndarray:
        if self.controls is None:
            raise RuntimeError("No control trajectory has been configured")
        if self.index >= len(self.controls):
            return np.zeros_like(self.controls[-1])
        control = self.controls[self.index].copy()
        self.index += 1
        return control
