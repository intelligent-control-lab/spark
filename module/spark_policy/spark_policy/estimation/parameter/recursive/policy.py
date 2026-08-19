"""Online scalar parameter estimators used by adaptive controllers."""

from __future__ import annotations

import numpy as np

from spark_policy.core import ParameterEstimate
from spark_policy.estimation.base import ParameterEstimator


def _sample_values(observation) -> tuple[float, float]:
    if isinstance(observation, dict):
        return float(observation["input"]), float(observation["measurement"])
    try:
        input_value, measurement = observation
    except (TypeError, ValueError) as exc:
        raise TypeError(
            "Parameter-estimator observations must be an (input, measurement) "
            "pair or a mapping with 'input' and 'measurement' entries."
        ) from exc
    return float(input_value), float(measurement)


class RecursiveLeastSquaresEstimator(ParameterEstimator):
    def __init__(self, p0hat: float, H0: float, lam: float):
        self._initial_parameter = float(p0hat)
        self._initial_information = float(H0)
        self.lam = float(lam)
        self.reset()

    def reset(self, context=None) -> None:
        del context
        self.phat = self._initial_parameter
        self.H = self._initial_information

    def step(self, x: float, y: float) -> tuple[float, float]:
        gradient = np.sin(x)
        error = y - self.phat * np.sin(x)
        denominator = self.lam * self.H + gradient * gradient
        self.phat = self.phat + gradient * error / denominator
        self.H = denominator
        return self.phat, self.H

    def estimate(self, observation, context=None) -> ParameterEstimate:
        input_value, measurement = _sample_values(observation)
        parameter, information = self.step(input_value, measurement)
        timestamp = None if context is None else getattr(context, "timestamp", None)
        return ParameterEstimate(
            parameters=np.array([parameter], dtype=float),
            information=np.array([[information]], dtype=float),
            timestamp=timestamp,
        )


class GradientParameterEstimator(ParameterEstimator):
    def __init__(self, p0hat: float, H0: float):
        self._initial_parameter = float(p0hat)
        self.H0 = float(H0)
        self.reset()

    def reset(self, context=None) -> None:
        del context
        self.phat = self._initial_parameter

    def step(self, x: float, y: float) -> float:
        gradient = np.sin(x)
        error = y - self.phat * np.sin(x)
        self.phat = self.phat + gradient * error / self.H0
        return self.phat

    def estimate(self, observation, context=None) -> ParameterEstimate:
        input_value, measurement = _sample_values(observation)
        parameter = self.step(input_value, measurement)
        timestamp = None if context is None else getattr(context, "timestamp", None)
        return ParameterEstimate(
            parameters=np.array([parameter], dtype=float),
            information=np.array([[self.H0]], dtype=float),
            timestamp=timestamp,
        )
