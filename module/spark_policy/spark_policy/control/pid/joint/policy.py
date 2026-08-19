"""Reusable joint-space PID feedback."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from spark_policy.core import JointReference


@dataclass
class JointPIDController:
    kp: np.ndarray | float
    kd: np.ndarray | float
    ki: np.ndarray | float = 0.0
    integral_limit: np.ndarray | float | None = None

    def __post_init__(self):
        self.reset()

    def reset(self, context=None) -> None:
        self._integral_error = None

    def compute(
        self,
        reference: JointReference,
        position: np.ndarray,
        velocity: np.ndarray,
        dt: float | None = None,
    ) -> np.ndarray:
        position = np.asarray(position, dtype=float)
        velocity = np.asarray(velocity, dtype=float)
        target_position = np.asarray(reference.position, dtype=float)
        target_velocity = (
            np.zeros_like(velocity)
            if reference.velocity is None
            else np.asarray(reference.velocity, dtype=float)
        )
        position_error = target_position - position
        velocity_error = target_velocity - velocity

        integral = np.zeros_like(position_error)
        if dt is not None and float(dt) > 0.0 and np.any(np.asarray(self.ki) != 0.0):
            if self._integral_error is None:
                self._integral_error = np.zeros_like(position_error)
            self._integral_error += position_error * float(dt)
            if self.integral_limit is not None:
                limit = np.asarray(self.integral_limit, dtype=float)
                self._integral_error = np.clip(self._integral_error, -limit, limit)
            integral = self._integral_error

        command = (
            np.asarray(self.kp) * position_error
            + np.asarray(self.kd) * velocity_error
            + np.asarray(self.ki) * integral
        )
        if reference.mask is not None:
            command = np.where(np.asarray(reference.mask, dtype=bool), command, 0.0)
        return np.asarray(command, dtype=float)
