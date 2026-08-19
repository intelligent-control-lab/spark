"""Linear and nonlinear Kalman-family state estimators."""

from __future__ import annotations

from collections.abc import Mapping

import numpy as np
from scipy.linalg import solve_discrete_are, sqrtm

from spark_policy.core import StateEstimate
from spark_policy.estimation.base import StateEstimator


Array = np.ndarray


def _control_measurement(observation) -> tuple[Array, Array]:
    """Normalize the component API input while retaining the numerical API."""
    if isinstance(observation, Mapping):
        return observation["control"], observation["measurement"]
    try:
        control, measurement = observation
    except (TypeError, ValueError) as exc:
        raise TypeError(
            "Estimator observations must be a (control, measurement) pair or a "
            "mapping with 'control' and 'measurement' entries."
        ) from exc
    return control, measurement


class KalmanFilterEstimator(StateEstimator):
    """Discrete-time Kalman filter for a linear system."""

    def __init__(
        self,
        A: Array,
        B: Array,
        C: Array,
        Bw: Array,
        W: Array,
        V: Array,
        x0hat: Array,
        X0: Array,
    ) -> None:
        self.A = np.atleast_2d(np.asarray(A, dtype=float))
        self.B = np.atleast_2d(np.asarray(B, dtype=float))
        self.C = np.atleast_2d(np.asarray(C, dtype=float))
        self.Bw = np.atleast_2d(np.asarray(Bw, dtype=float))
        self.W = np.atleast_2d(np.asarray(W, dtype=float))
        self.V = np.atleast_2d(np.asarray(V, dtype=float))
        self._initial_xhat = np.asarray(x0hat, dtype=float).reshape(-1)
        self._initial_covariance = np.atleast_2d(np.asarray(X0, dtype=float))
        self.reset()

    def reset(self, context=None) -> None:
        del context
        self.xhat = self._initial_xhat.copy()
        self.Z = self._initial_covariance.copy()

    def predict_covariance(self, covariance: Array) -> Array:
        return self.A @ covariance @ self.A.T + self.Bw @ self.W @ self.Bw.T

    def step(self, control: Array, measurement: Array) -> tuple[Array, Array]:
        control = np.asarray(control, dtype=float).reshape(-1)
        measurement = np.asarray(measurement, dtype=float).reshape(-1)
        xprior = self.A @ self.xhat + self.B @ control
        prior_covariance = self.predict_covariance(self.Z)
        innovation_covariance = self.V + self.C @ prior_covariance @ self.C.T
        gain = prior_covariance @ self.C.T @ np.linalg.inv(innovation_covariance)
        self.xhat = xprior + gain @ (measurement - self.C @ xprior)
        self.Z = prior_covariance - gain @ self.C @ prior_covariance
        return self.xhat.copy(), self.Z.copy()

    def estimate(self, observation, context=None) -> StateEstimate:
        control, measurement = _control_measurement(observation)
        state, covariance = self.step(control, measurement)
        timestamp = None if context is None else getattr(context, "timestamp", None)
        return StateEstimate(state=state, covariance=covariance, timestamp=timestamp)


class SteadyStateKalmanFilterEstimator(KalmanFilterEstimator):
    """Linear Kalman filter using its steady-state covariance and gain."""

    def __init__(
        self,
        A: Array,
        B: Array,
        C: Array,
        Bw: Array,
        W: Array,
        V: Array,
        x0hat: Array,
        X0: Array,
    ) -> None:
        super().__init__(A, B, C, Bw, W, V, x0hat, X0)
        self.Ms = solve_discrete_are(
            self.A.T,
            self.C.T,
            self.Bw @ self.W @ self.Bw.T,
            self.V,
        )
        innovation_covariance = self.V + self.C @ self.Ms @ self.C.T
        self.gain = self.Ms @ self.C.T @ np.linalg.inv(innovation_covariance)
        self.Zss = self.Ms - self.gain @ self.C @ self.Ms

    def step(self, control: Array, measurement: Array) -> tuple[Array, Array]:
        control = np.asarray(control, dtype=float).reshape(-1)
        measurement = np.asarray(measurement, dtype=float).reshape(-1)
        xprior = self.A @ self.xhat + self.B @ control
        self.xhat = xprior + self.gain @ (measurement - self.C @ xprior)
        self.Z = self.Zss.copy()
        return self.xhat.copy(), self.Z.copy()


class ExtendedKalmanFilterEstimator(StateEstimator):
    """Extended Kalman filter with caller-provided models and Jacobians."""

    def __init__(self, f_nominal, h_nominal, A_func, C_func, Bw, W, V, x0hat, X0):
        self.f_nominal = f_nominal
        self.h_nominal = h_nominal
        self.A_func = A_func
        self.C_func = C_func
        self.Bw = np.atleast_2d(np.asarray(Bw, dtype=float))
        self.W = np.atleast_2d(np.asarray(W, dtype=float))
        self.V = np.atleast_2d(np.asarray(V, dtype=float))
        self._initial_xhat = np.asarray(x0hat, dtype=float).reshape(-1)
        self._initial_covariance = np.atleast_2d(np.asarray(X0, dtype=float))
        self.reset()

    def reset(self, context=None) -> None:
        del context
        self.xhat = self._initial_xhat.copy()
        self.Z = self._initial_covariance.copy()

    def step(self, control: Array, measurement: Array) -> tuple[Array, Array]:
        control = np.asarray(control, dtype=float).reshape(-1)
        measurement = np.asarray(measurement, dtype=float).reshape(-1)
        A = np.atleast_2d(np.asarray(self.A_func(self.xhat.copy(), control.copy()), dtype=float))
        xprior = np.asarray(self.f_nominal(self.xhat.copy(), control.copy()), dtype=float).reshape(
            -1
        )
        C = np.atleast_2d(np.asarray(self.C_func(xprior.copy()), dtype=float))
        prior_covariance = A @ self.Z @ A.T + self.Bw @ self.W @ self.Bw.T
        innovation_covariance = self.V + C @ prior_covariance @ C.T
        gain = prior_covariance @ C.T @ np.linalg.inv(innovation_covariance)
        predicted_measurement = np.asarray(self.h_nominal(xprior.copy()), dtype=float).reshape(-1)
        self.xhat = xprior + gain @ (measurement - predicted_measurement)
        identity = np.eye(prior_covariance.shape[0])
        correction = identity - gain @ C
        self.Z = correction @ prior_covariance @ correction.T + gain @ self.V @ gain.T
        self.Z = 0.5 * (self.Z + self.Z.T)
        return self.xhat.copy(), self.Z.copy()

    def estimate(self, observation, context=None) -> StateEstimate:
        control, measurement = _control_measurement(observation)
        state, covariance = self.step(control, measurement)
        timestamp = None if context is None else getattr(context, "timestamp", None)
        return StateEstimate(state=state, covariance=covariance, timestamp=timestamp)


class UnscentedKalmanFilterEstimator(StateEstimator):
    """Unscented Kalman filter using a symmetric sigma-point set."""

    def __init__(self, f_nominal, h_nominal, Bw, W, V, x0hat, X0, kappa: float = 2.0):
        self.f_nominal = f_nominal
        self.h_nominal = h_nominal
        self.Bw = np.atleast_2d(np.asarray(Bw, dtype=float))
        self.W = np.atleast_2d(np.asarray(W, dtype=float))
        self.V = np.atleast_2d(np.asarray(V, dtype=float))
        self._initial_xhat = np.asarray(x0hat, dtype=float).reshape(-1)
        self._initial_covariance = np.atleast_2d(np.asarray(X0, dtype=float))
        self.kappa = float(kappa)
        self.reset()

    def reset(self, context=None) -> None:
        del context
        self.xhat = self._initial_xhat.copy()
        self.Z = self._initial_covariance.copy()

    def step(self, control: Array, measurement: Array) -> tuple[Array, Array]:
        control = np.asarray(control, dtype=float).reshape(-1)
        measurement = np.asarray(measurement, dtype=float).reshape(-1)
        dynamic_points = self._sigma_points(self.xhat, self.Z)
        state_points = [
            (np.asarray(self.f_nominal(point, control), dtype=float).reshape(-1), weight)
            for point, weight in dynamic_points
        ]
        xprior = weighted_mean(state_points)
        prior_covariance = weighted_variance(state_points) + self.Bw @ self.W @ self.Bw.T

        measurement_points = self._sigma_points(xprior, prior_covariance)
        output_points = [
            (np.asarray(self.h_nominal(point), dtype=float).reshape(-1), weight)
            for point, weight in measurement_points
        ]
        yprior = weighted_mean(output_points)
        output_covariance = weighted_variance(output_points)
        cross_covariance = weighted_cross_covariance(measurement_points, output_points)
        innovation_covariance = self.V + output_covariance
        gain = cross_covariance @ np.linalg.inv(innovation_covariance)
        self.xhat = xprior + gain @ (measurement - yprior)
        self.Z = prior_covariance - gain @ innovation_covariance @ gain.T
        return self.xhat.copy(), self.Z.copy()

    def estimate(self, observation, context=None) -> StateEstimate:
        control, measurement = _control_measurement(observation)
        state, covariance = self.step(control, measurement)
        timestamp = None if context is None else getattr(context, "timestamp", None)
        return StateEstimate(state=state, covariance=covariance, timestamp=timestamp)

    def _sigma_points(self, mean: Array, covariance: Array):
        mean = np.asarray(mean, dtype=float).reshape(-1)
        covariance = np.atleast_2d(np.asarray(covariance, dtype=float))
        count = mean.size
        points = [(mean.copy(), self.kappa / (count + self.kappa))]
        root = sqrtm((count + self.kappa) * covariance).real
        for index in range(count):
            weight = 1.0 / (2.0 * (count + self.kappa))
            points.append((mean + root[:, index], weight))
            points.append((mean - root[:, index], weight))
        return points


def weighted_mean(points):
    total = sum(point * weight for point, weight in points)
    weight_total = sum(weight for _, weight in points)
    return total / weight_total


def weighted_variance(points):
    mean = weighted_mean(points)
    covariance = np.zeros((mean.size, mean.size), dtype=float)
    weight_total = 0.0
    for point, weight in points:
        difference = point - mean
        covariance += np.outer(difference, difference) * weight
        weight_total += weight
    return covariance / weight_total


def weighted_cross_covariance(x_points, y_points):
    x_mean = weighted_mean(x_points)
    y_mean = weighted_mean(y_points)
    covariance = np.zeros((x_mean.size, y_mean.size), dtype=float)
    weight_total = 0.0
    for (x_value, weight), (y_value, _) in zip(x_points, y_points):
        covariance += np.outer(x_value - x_mean, y_value - y_mean) * weight
        weight_total += weight
    return covariance / weight_total
