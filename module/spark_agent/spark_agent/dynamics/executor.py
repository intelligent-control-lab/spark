from __future__ import annotations

from typing import Any, Mapping

import numpy as np

from spark_robot import DynamicsStepContext, RobotDynamicsModel


class DynamicsExecutor:
    """Stateful runtime for a config-selected dynamics model.

    The executor owns episode state, time, random state and numerical
    integration settings.  It never chooses a dynamics equation; that model is
    created by the selected robot configuration and injected here.
    """

    def __init__(
        self,
        dynamics_model: RobotDynamicsModel,
        *,
        dt: float,
        integrator: str | None = None,
        substeps: int = 1,
        seed: int | None = None,
    ) -> None:
        if dt <= 0.0:
            raise ValueError("dt must be positive.")
        if int(substeps) <= 0:
            raise ValueError("substeps must be positive.")
        self.model = dynamics_model
        self.dt = float(dt)
        self.integrator = integrator or getattr(dynamics_model, "default_integrator", "Euler")
        self.substeps = int(substeps)
        self.rng = np.random.default_rng(seed)
        self.state: np.ndarray | None = None
        self.time = 0.0
        self.step_index = 0
        self.episode_index = -1

    def reset(
        self,
        state,
        *,
        time: float = 0.0,
        seed: int | None = None,
        episode_index: int | None = None,
    ) -> np.ndarray:
        self.state = np.asarray(state, dtype=float).reshape(self.model.state_dim).copy()
        self.time = float(time)
        self.step_index = 0
        self.episode_index = self.episode_index + 1 if episode_index is None else int(episode_index)
        if seed is not None:
            self.rng = np.random.default_rng(seed)
        return self.state.copy()

    def clear(self) -> None:
        self.state = None
        self.time = 0.0
        self.step_index = 0

    def update_parameters(self, **parameters: Any) -> None:
        self.model.update_parameters(**parameters)

    def step(
        self,
        control,
        *,
        state=None,
        dt: float | None = None,
        parameters: Mapping[str, Any] | None = None,
        exogenous: Mapping[str, Any] | None = None,
    ) -> np.ndarray:
        if state is not None:
            state = np.asarray(state, dtype=float).reshape(self.model.state_dim)
            if self.state is None:
                self.reset(state)
            else:
                self.state = state.copy()
        if self.state is None:
            raise RuntimeError("DynamicsExecutor.reset(state) must be called before step().")
        if parameters:
            self.update_parameters(**dict(parameters))

        control = np.asarray(control, dtype=float).reshape(self.model.control_dim)
        step_dt = self.dt if dt is None else float(dt)
        if step_dt <= 0.0:
            raise ValueError("dt must be positive.")

        native_discrete = getattr(self.model, "time_domain", "continuous") == "discrete"
        if native_discrete and self.substeps != 1:
            raise ValueError("Native discrete dynamics cannot be subdivided by the executor.")
        local_dt = step_dt / self.substeps
        for substep in range(self.substeps):
            context = DynamicsStepContext(
                time=self.time + substep * local_dt,
                step_index=self.step_index,
                episode_index=self.episode_index,
                rng=self.rng,
                exogenous=exogenous,
            )
            self.state = self.model.step(
                self.state,
                control,
                local_dt,
                self.integrator,
                context=context,
            )

        self.time += step_dt
        self.step_index += 1
        return self.state.copy()
