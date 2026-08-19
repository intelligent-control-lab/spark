"""Model-reference adaptive control."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Mapping

import numpy as np

from spark_policy.core import ComponentResult, ControlCommand, get_component_spec


Array = np.ndarray


@dataclass(frozen=True)
class MRACSnapshot:
    """One immutable snapshot of the adaptive-controller state."""

    state_matrix: Array
    control_matrix: Array
    information: Array
    gain: Array
    tracking_error: Array | None
    regressor: Array


class MRACPolicy:
    """Discrete model-reference adaptive controller with online RLS updates.

    ``step`` is the component API and receives the current state and reference
    explicitly.  The legacy ``control(state, time, step)`` API remains as a
    compatibility adapter for older experiments that supplied ``x0``,
    ``horizon``, and ``dt`` to construct a reference rollout internally.
    """

    def __init__(
        self,
        Astar: Array,
        Ahat: Array,
        Bhat: Array,
        F: Array,
        x0: Array | None = None,
        horizon: int | None = None,
        dt: float = 1.0,
        *,
        record_history: bool = True,
    ) -> None:
        self.Astar = np.asarray(Astar, dtype=float)
        self._initial_Ahat = np.asarray(Ahat, dtype=float)
        self._initial_Bhat = np.asarray(Bhat, dtype=float)
        self._initial_F = np.asarray(F, dtype=float)
        self.x0 = None if x0 is None else np.asarray(x0, dtype=float).reshape(-1)
        self.horizon = None if horizon is None else int(horizon)
        self.dt = float(dt)
        self.record_history = bool(record_history)
        self.n = self._initial_Ahat.shape[0]
        self._validate_dimensions()
        self.ref = (
            self._reference_rollout() if self.x0 is not None and self.horizon is not None else None
        )
        self.reset()

    @property
    def spec(self):
        return get_component_spec(type(self).__name__)

    def _validate_dimensions(self) -> None:
        if self.Astar.shape != self._initial_Ahat.shape:
            raise ValueError("Astar and Ahat must have the same shape.")
        if self.Astar.ndim != 2 or self.Astar.shape[0] != self.Astar.shape[1]:
            raise ValueError("Astar and Ahat must be square matrices.")
        if self._initial_Bhat.ndim != 2 or self._initial_Bhat.shape[0] != self.n:
            raise ValueError("Bhat must have the same state dimension as Ahat.")
        parameter_dim = self.n + self._initial_Bhat.shape[1]
        if self._initial_F.shape != (parameter_dim, parameter_dim):
            raise ValueError("F must be square with dimension state_dim + control_dim.")
        if self.x0 is not None and self.x0.size != self.n:
            raise ValueError("x0 must match the reference-model state dimension.")

    def reset(self, context=None) -> None:
        del context
        self.Ahat = self._initial_Ahat.copy()
        self.Bhat = self._initial_Bhat.copy()
        self.F = self._initial_F.copy()
        self.phi = None
        self.Ahat_history = [self.Ahat.copy()] if self.record_history else []
        self.Bhat_history = [self.Bhat.copy()] if self.record_history else []
        self.F_history = [self.F.copy()] if self.record_history else []
        self.error_history = [self.x0.copy()] if self.record_history and self.x0 is not None else []
        self.phi_history = []
        self.last_error = None
        self.last_gain = np.linalg.pinv(self.Bhat) @ (self.Astar - self.Ahat)

    def _reference_rollout(self) -> Array:
        if self.x0 is None or self.horizon is None:
            raise RuntimeError("Legacy reference rollout requires x0 and horizon.")
        states = np.zeros((self.horizon + 1, self.x0.size), dtype=float)
        states[0] = self.x0
        for step in range(self.horizon):
            states[step + 1] = self.Astar @ states[step]
        return states

    def compute_control(self, state: Array, reference_state: Array) -> Array:
        """Update the adaptive model and return the current control command."""

        state = np.asarray(state, dtype=float).reshape(-1)
        reference_state = np.asarray(reference_state, dtype=float).reshape(-1)
        if state.size != self.n or reference_state.size != self.n:
            raise ValueError("state and reference_state must match the MRAC state dimension.")

        if self.phi is not None:
            error = state - reference_state
            self.F = np.linalg.inv(np.linalg.inv(self.F) + np.outer(self.phi, self.phi))
            self.Ahat = self.Ahat + np.outer(error, self.phi @ self.F[:, : self.n])
            self.Bhat = self.Bhat + np.outer(error, self.phi @ self.F[:, self.n :])
            self.last_error = error.copy()
            if self.record_history:
                self.Ahat_history.append(self.Ahat.copy())
                self.Bhat_history.append(self.Bhat.copy())
                self.F_history.append(self.F.copy())
                self.error_history.append(error.copy())
                self.phi_history.append(self.phi.copy())

        gain = np.linalg.pinv(self.Bhat) @ (self.Astar - self.Ahat)
        control = gain @ state
        self.phi = np.concatenate([state, control])
        self.last_gain = gain.copy()
        return control

    def snapshot(self) -> MRACSnapshot:
        if self.phi is None:
            raise RuntimeError("MRAC has not produced a control command yet.")
        return MRACSnapshot(
            state_matrix=self.Ahat.copy(),
            control_matrix=self.Bhat.copy(),
            information=self.F.copy(),
            gain=self.last_gain.copy(),
            tracking_error=None if self.last_error is None else self.last_error.copy(),
            regressor=self.phi.copy(),
        )

    def step(self, inputs: Mapping[str, Array], context=None) -> ComponentResult[ControlCommand]:
        """Execute one adaptive-control update through the component contract."""

        if not isinstance(inputs, Mapping):
            raise TypeError("MRAC inputs must be a mapping with 'state' and 'reference'.")
        control = self.compute_control(inputs["state"], inputs["reference"])
        snapshot = self.snapshot()
        timestamp = None if context is None else getattr(context, "timestamp", None)
        return ComponentResult(
            value=ControlCommand(values=control.copy()),
            diagnostics={
                "Ahat": snapshot.state_matrix,
                "Bhat": snapshot.control_matrix,
                "F": snapshot.information,
                "gain": snapshot.gain,
                "tracking_error": snapshot.tracking_error,
                "regressor": snapshot.regressor,
            },
            timestamp=timestamp,
        )

    def control(self, state: Array, time: float, _step: int) -> Array:
        """Compatibility adapter using the legacy internally generated reference."""

        if self.ref is None:
            raise RuntimeError(
                "control(state, time, step) requires x0 and horizon; use "
                "compute_control(state, reference_state) or step(...) instead."
            )
        reference_index = max(
            0,
            min(self.ref.shape[0] - 1, int(round(float(time) / self.dt))),
        )
        # The legacy implementation did not adapt at t=0.  ``phi`` is initially
        # None, so the explicit-reference implementation preserves that behavior.
        return self.compute_control(state, self.ref[reference_index])
