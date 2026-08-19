"""Frequency-domain and lifted time-domain iterative learning control."""

from __future__ import annotations

import numpy as np
from scipy.signal import lfilter

from spark_policy.core import ComponentResult, get_component_spec
from spark_policy.utils.control_math import lift_dynamics


Array = np.ndarray


def polynomial_add(a: Array, b: Array) -> Array:
    a = np.asarray(a, dtype=float).reshape(-1)
    b = np.asarray(b, dtype=float).reshape(-1)
    count = max(a.size, b.size)
    output = np.zeros(count, dtype=float)
    output[-a.size :] += a
    output[-b.size :] += b
    return output


class FrequencyILCPolicy:
    """Frequency-domain ILC update defined by learning and robustness filters."""

    def __init__(self, a: Array, b: Array, c: Array):
        self.L_b = polynomial_add(a, np.convolve(b, -np.asarray(c, dtype=float).reshape(-1)))
        self.L_a = np.asarray(b, dtype=float).reshape(-1)
        self.Q_b = np.array([1.0])
        self.Q_a = np.array([1.0])

    @property
    def spec(self):
        return get_component_spec(type(self).__name__)

    def reset(self, context=None) -> None:
        del context

    def _learning_filter(self, error: Array) -> Array:
        error = np.asarray(error, dtype=float).reshape(-1)
        output = lfilter(self.L_b, self.L_a, error)
        shift = self.L_b.size - self.L_a.size
        if shift > 0:
            output[:-shift] = output[shift:]
        return output

    def update_feedforward(self, error: Array, previous_feedforward: Array) -> Array:
        signal = np.asarray(previous_feedforward, dtype=float).reshape(-1) + self._learning_filter(
            error
        )
        return lfilter(self.Q_b, self.Q_a, signal)

    def update(self, error: Array, previous_feedforward: Array) -> Array:
        """Compatibility name for :meth:`update_feedforward`."""

        return self.update_feedforward(error, previous_feedforward)

    def step(self, inputs, context=None) -> ComponentResult[Array]:
        if not isinstance(inputs, dict):
            raise TypeError("ILC inputs must contain 'error' and 'previous_feedforward'.")
        value = self.update_feedforward(inputs["error"], inputs["previous_feedforward"])
        timestamp = None if context is None else getattr(context, "timestamp", None)
        return ComponentResult(
            value=value, diagnostics={"learning_filter": self.L_b.copy()}, timestamp=timestamp
        )


class TimeDomainILCPolicy:
    """Lifted-system time-domain ILC update."""

    def __init__(self, A: Array, B: Array, K: Array, C: Array, horizon: int):
        self.A = np.asarray(A, dtype=float)
        self.B = np.asarray(B, dtype=float)
        self.K = np.asarray(K, dtype=float)
        self.C = np.asarray(C, dtype=float)
        if self.C.ndim == 1:
            self.C = self.C.reshape(1, -1)
        self.horizon = int(horizon)
        closed_loop_A = self.A - self.B @ self.K
        _, lifted_B = lift_dynamics(closed_loop_A, self.B, self.horizon)
        lifted_C = np.kron(np.eye(self.horizon + 1), self.C)
        self.lifted_plant = lifted_C @ lifted_B
        self.L = np.linalg.pinv(self.lifted_plant)

    @classmethod
    def from_dynamics_model(
        cls,
        dynamics_model,
        K: Array,
        C: Array,
        horizon: int,
        *,
        dt: float = 1.0,
        discretization: str = "ZOH",
    ) -> "TimeDomainILCPolicy":
        """Build the lifted controller from a robot-config-backed model."""

        A, B = dynamics_model.discrete_matrices(dt, discretization)
        return cls(A, B, K, C, horizon)

    @property
    def spec(self):
        return get_component_spec(type(self).__name__)

    def reset(self, context=None) -> None:
        del context

    def update_feedforward(self, error: Array, previous_feedforward: Array) -> Array:
        error = np.asarray(error, dtype=float).reshape(-1)
        previous_feedforward = np.asarray(previous_feedforward, dtype=float).reshape(-1)
        expected_error_size = (self.horizon + 1) * self.C.shape[0]
        expected_feedforward_size = self.horizon * self.B.shape[1]
        if error.size != expected_error_size:
            raise ValueError(
                f"Expected lifted error of length {expected_error_size}, got {error.size}."
            )
        if previous_feedforward.size != expected_feedforward_size:
            raise ValueError(
                "Expected previous feedforward of length "
                f"{expected_feedforward_size}, got {previous_feedforward.size}."
            )
        return previous_feedforward + self.L @ error

    def update(self, error: Array, previous_feedforward: Array) -> Array:
        """Compatibility name for :meth:`update_feedforward`."""

        return self.update_feedforward(error, previous_feedforward)

    def step(self, inputs, context=None) -> ComponentResult[Array]:
        if not isinstance(inputs, dict):
            raise TypeError("ILC inputs must contain 'error' and 'previous_feedforward'.")
        value = self.update_feedforward(inputs["error"], inputs["previous_feedforward"])
        timestamp = None if context is None else getattr(context, "timestamp", None)
        return ComponentResult(
            value=value, diagnostics={"learning_matrix": self.L.copy()}, timestamp=timestamp
        )
