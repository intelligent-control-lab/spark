from __future__ import annotations

from dataclasses import dataclass, replace
from typing import Any, Mapping, Sequence, Tuple

import numpy as np


Array = np.ndarray


def _enum_indices(enum_cls, names: Sequence[str | int]) -> Tuple[int, ...]:
    members = enum_cls.__members__
    indices = []
    for name in names:
        if isinstance(name, str):
            if name not in members:
                raise KeyError(f'{enum_cls.__name__} has no member "{name}".')
            indices.append(int(members[name]))
        else:
            indices.append(int(name))
    return tuple(indices)


@dataclass(frozen=True)
class DynamicsDimensions:
    state: int
    control: int


@dataclass(frozen=True)
class DynamicsStepContext:
    """Runtime information supplied to one model evaluation.

    Configuration selects the equation; the context carries values that vary
    during an episode (time, random generator, disturbances, and online model
    parameters).  Keeping those separate lets adaptive experiments update a
    model without reconstructing the robot configuration.
    """

    time: float = 0.0
    step_index: int = 0
    episode_index: int = 0
    rng: np.random.Generator | None = None
    exogenous: Mapping[str, Any] | None = None

    def at_time(self, time: float) -> "DynamicsStepContext":
        return replace(self, time=float(time))


class RobotDynamicsModel:
    """Reduced dynamics view backed by one :class:`RobotConfig`.

    The selected robot configuration remains the source of the derivative via
    ``robot_cfg.dynamics_xdot``. Policies may select coordinates, but they do
    not select or construct a different dynamics equation.
    """

    def __init__(
        self,
        robot_cfg,
        state_dof_names: Sequence[str | int] | None = None,
        control_names: Sequence[str | int] | None = None,
    ) -> None:
        self.robot_cfg = robot_cfg
        self.parameters = dict(getattr(robot_cfg, "dynamics_parameters", {}))
        self.variant = str(getattr(robot_cfg, "dynamics_variant", robot_cfg.__class__.__name__))
        configured_order = getattr(robot_cfg, "dynamics_order", None)
        self.order = int(self._infer_order() if configured_order is None else configured_order)
        self.full_state_only = bool(getattr(robot_cfg, "dynamics_full_state_only", False))

        if state_dof_names is None:
            state_dof_names = getattr(
                robot_cfg, "default_dynamics_state_dof_names", tuple(d.name for d in robot_cfg.DoFs)
            )
        if control_names is None:
            control_names = getattr(
                robot_cfg,
                "default_dynamics_control_names",
                tuple(c.name for c in robot_cfg.Control),
            )

        self.state_dof_names = tuple(state_dof_names)
        self.control_names = tuple(control_names)
        if self.full_state_only and len(self.state_dof_names) != len(robot_cfg.DoFs):
            raise ValueError(f"{self.variant} only supports its full non-Euclidean state layout.")
        self.dof_indices = _enum_indices(robot_cfg.DoFs, self.state_dof_names)
        self.control_indices = _enum_indices(robot_cfg.Control, self.control_names)
        self.position_dim = len(self.dof_indices)
        self.state_dim = (
            int(robot_cfg.num_state) if self.full_state_only else self.position_dim * self.order
        )
        self.control_dim = len(self.control_indices)
        self.dimensions = DynamicsDimensions(self.state_dim, self.control_dim)

        if self.order not in (1, 2):
            raise ValueError(
                f"Unsupported dynamics order {self.order} for {type(robot_cfg).__name__}."
            )

    def _infer_order(self) -> int:
        if self.robot_cfg.num_state == self.robot_cfg.num_dof:
            return 1
        if self.robot_cfg.num_state == 2 * self.robot_cfg.num_dof:
            return 2
        raise ValueError(
            f"Cannot infer dynamics order for {type(self.robot_cfg).__name__}: "
            f"num_state={self.robot_cfg.num_state}, num_dof={self.robot_cfg.num_dof}."
        )

    @property
    def state_names(self) -> tuple[str, ...]:
        position_names = tuple(str(name) for name in self.state_dof_names)
        if self.order == 1:
            return position_names
        if self.full_state_only:
            velocity_dim = self.state_dim - self.position_dim
            velocity_names = tuple(
                getattr(
                    self.robot_cfg,
                    "dynamics_velocity_names",
                    (f"v{i}" for i in range(velocity_dim)),
                )
            )
            return position_names + velocity_names
        return position_names + tuple(f"v{name}" for name in position_names)

    def extract_state(self, dof_pos: Array, dof_vel: Array | None = None) -> Array:
        dof_pos = np.asarray(dof_pos, dtype=float).reshape(-1)
        if self.full_state_only:
            if dof_vel is None:
                raise ValueError("This dynamics layout requires velocity feedback.")
            return np.asarray(
                self.robot_cfg.compose_state_from_dof(dof_pos, dof_vel), dtype=float
            ).reshape(self.state_dim)
        position = dof_pos[list(self.dof_indices)]
        if self.order == 1:
            return position.astype(float)
        if dof_vel is None:
            raise ValueError("Second-order dynamics require dof_vel feedback.")
        dof_vel = np.asarray(dof_vel, dtype=float).reshape(-1)
        return np.concatenate([position, dof_vel[list(self.dof_indices)]]).astype(float)

    def project_state(self, state: Array) -> Array:
        """Project a robot-config state into this model's coordinates.

        Agents generally expose the complete state described by ``robot_cfg``
        while a dynamics variant may operate on a reduced view (for example,
        the AgiBot unicycle model only uses ``x``, ``y``, and yaw).  Keeping the
        projection here prevents simulator agents and control modes from
        duplicating knowledge of the selected dynamics coordinates.
        """

        state = np.asarray(state, dtype=float).reshape(-1)
        if state.size == self.state_dim:
            return state.copy()
        if state.size != self.robot_cfg.num_state:
            raise ValueError(
                f"Cannot project state of size {state.size} into {self.variant} "
                f"(expected model size {self.state_dim} or robot-config size "
                f"{self.robot_cfg.num_state})."
            )

        dof_pos = self.robot_cfg.decompose_state_to_dof_pos(state)
        if self.order == 1:
            return self.extract_state(dof_pos)
        dof_vel = self.robot_cfg.decompose_state_to_dof_vel(state)
        return self.extract_state(dof_pos, dof_vel)

    def project_control(self, control: Array) -> Array:
        """Project a robot-config command into this model's control axes."""

        control = np.asarray(control, dtype=float).reshape(-1)
        if control.size == self.control_dim:
            return control.copy()
        full_control_dim = len(self.robot_cfg.Control)
        if control.size != full_control_dim:
            raise ValueError(
                f"Cannot project control of size {control.size} into {self.variant} "
                f"(expected model size {self.control_dim} or robot-config size "
                f"{full_control_dim})."
            )
        return control[list(self.control_indices)].copy()

    def merge_state(self, state: Array, into: Array | None = None) -> Array:
        """Merge a model state into an existing robot-config state.

        Unmodelled coordinates retain their values from ``into``.  With no
        target state this behaves like :meth:`expand_state` and embeds the
        model state into the robot configuration's nominal state.
        """

        state = np.asarray(state, dtype=float).reshape(self.state_dim)
        if into is None:
            return self.expand_state(state)

        into = np.asarray(into, dtype=float).reshape(-1)
        if into.size == self.state_dim:
            return state.copy()
        if into.size != self.robot_cfg.num_state:
            raise ValueError(
                f"Cannot merge {self.variant} into state of size {into.size} "
                f"(expected model size {self.state_dim} or robot-config size "
                f"{self.robot_cfg.num_state})."
            )

        dof_pos = (
            np.asarray(self.robot_cfg.decompose_state_to_dof_pos(into), dtype=float)
            .reshape(-1)
            .copy()
        )
        dof_vel = (
            np.asarray(self.robot_cfg.decompose_state_to_dof_vel(into), dtype=float)
            .reshape(-1)
            .copy()
        )
        dof_pos[list(self.dof_indices)] = state[: self.position_dim]
        if self.order == 2:
            dof_vel[list(self.dof_indices)] = state[self.position_dim :]
        return np.asarray(
            self.robot_cfg.compose_state_from_dof(dof_pos, dof_vel), dtype=float
        ).reshape(self.robot_cfg.num_state)

    def expand_state(self, state: Array) -> Array:
        """Embed a reduced model state in the configuration's full state."""

        return self._embed_state(state)

    def update_parameters(self, **parameters: Any) -> None:
        """Update mutable runtime parameters on this model instance."""

        self.parameters.update(parameters)

    def derivative(
        self,
        state: Array,
        control: Array,
        context: DynamicsStepContext | None = None,
    ) -> Array:
        full_state = self._embed_state(state)
        full_control = np.zeros(len(self.robot_cfg.Control), dtype=float)
        full_control[list(self.control_indices)] = np.asarray(control, dtype=float).reshape(
            self.control_dim
        )
        dynamics = getattr(self.robot_cfg, "dynamics_derivative", None)
        if dynamics is None:
            full_derivative = self.robot_cfg.dynamics_xdot(full_state, full_control)
        else:
            full_derivative = dynamics(
                full_state,
                full_control,
                context=context,
                parameters=self.parameters,
            )
        full_derivative = np.asarray(full_derivative, dtype=float).reshape(-1)
        return self._reduce_state(full_derivative)

    def step(
        self,
        state: Array,
        control: Array,
        dt: float,
        integrator: str = "Euler",
        context: DynamicsStepContext | None = None,
    ) -> Array:
        state = np.asarray(state, dtype=float).reshape(self.state_dim)
        control = np.asarray(control, dtype=float).reshape(self.control_dim)
        mode = str(integrator).lower()
        if mode in ("euler", "forwardeuler", "fe"):
            return state + float(dt) * self.derivative(state, control, context)
        if mode == "rk4":
            context = context or DynamicsStepContext()
            k1 = self.derivative(state, control, context)
            k2 = self.derivative(
                state + 0.5 * dt * k1, control, context.at_time(context.time + 0.5 * dt)
            )
            k3 = self.derivative(
                state + 0.5 * dt * k2, control, context.at_time(context.time + 0.5 * dt)
            )
            k4 = self.derivative(state + dt * k3, control, context.at_time(context.time + dt))
            return state + (float(dt) / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
        if mode in ("zoh", "direct"):
            if self.is_linear:
                a, b = self.discrete_matrices(dt, "ZOH")
                return a @ state + b @ control
            from scipy.integrate import solve_ivp

            sol = solve_ivp(
                lambda local_t, value: self.derivative(
                    value,
                    control,
                    (context or DynamicsStepContext()).at_time(
                        (context.time if context is not None else 0.0) + local_t
                    ),
                ),
                (0.0, float(dt)),
                state,
                rtol=1e-6,
                atol=1e-8,
            )
            return sol.y[:, -1]
        raise ValueError(f'Unknown integrator "{integrator}".')

    @property
    def is_linear(self) -> bool:
        return bool(getattr(self.robot_cfg, "dynamics_is_linear", False))

    def continuous_matrices(self) -> tuple[Array, Array]:
        if not self.is_linear:
            raise ValueError(f"{self.variant} is nonlinear; use linearize(state, control).")
        zero_x = np.zeros(self.state_dim, dtype=float)
        zero_u = np.zeros(self.control_dim, dtype=float)
        return self._continuous_linearize(zero_x, zero_u)

    def discrete_matrices(self, dt: float, discretization: str = "ZOH") -> tuple[Array, Array]:
        a_ct, b_ct = self.continuous_matrices()
        mode = str(discretization).lower()
        if mode in ("euler", "forwardeuler", "fe"):
            return np.eye(self.state_dim) + float(dt) * a_ct, float(dt) * b_ct
        if mode not in ("zoh", ""):
            raise ValueError(f'Unknown discretization "{discretization}".')

        from scipy.linalg import expm

        augmented = np.zeros(
            (self.state_dim + self.control_dim, self.state_dim + self.control_dim), dtype=float
        )
        augmented[: self.state_dim, : self.state_dim] = a_ct
        augmented[: self.state_dim, self.state_dim :] = b_ct
        discrete = expm(float(dt) * augmented)
        return discrete[: self.state_dim, : self.state_dim], discrete[
            : self.state_dim, self.state_dim :
        ]

    def linearize(
        self, state: Array, control: Array, dt: float | None = None
    ) -> tuple[Array, Array]:
        a_ct, b_ct = self._continuous_linearize(state, control)
        if dt is None:
            return a_ct, b_ct
        return np.eye(self.state_dim) + float(dt) * a_ct, float(dt) * b_ct

    def _continuous_linearize(self, state: Array, control: Array) -> tuple[Array, Array]:
        state = np.asarray(state, dtype=float).reshape(self.state_dim)
        control = np.asarray(control, dtype=float).reshape(self.control_dim)
        eps = 1e-6
        a = np.zeros((self.state_dim, self.state_dim), dtype=float)
        b = np.zeros((self.state_dim, self.control_dim), dtype=float)
        for idx in range(self.state_dim):
            delta = np.zeros(self.state_dim, dtype=float)
            delta[idx] = eps
            a[:, idx] = (
                self.derivative(state + delta, control) - self.derivative(state - delta, control)
            ) / (2.0 * eps)
        for idx in range(self.control_dim):
            delta = np.zeros(self.control_dim, dtype=float)
            delta[idx] = eps
            b[:, idx] = (
                self.derivative(state, control + delta) - self.derivative(state, control - delta)
            ) / (2.0 * eps)
        return a, b

    def _nominal_full_state(self) -> Array:
        position = np.array(
            [self.robot_cfg.DefaultDoFVal[dof] for dof in self.robot_cfg.DoFs], dtype=float
        )
        if self.order == 1:
            return position
        if self.full_state_only:
            velocity_dim = self.state_dim - self.robot_cfg.num_dof
            return self.robot_cfg.compose_state_from_dof(
                position, np.zeros(velocity_dim, dtype=float)
            )
        return np.concatenate([position, np.zeros(self.robot_cfg.num_dof, dtype=float)])

    def _embed_state(self, reduced_state: Array) -> Array:
        reduced_state = np.asarray(reduced_state, dtype=float).reshape(self.state_dim)
        if self.full_state_only:
            return reduced_state.copy()
        full_state = self._nominal_full_state()
        full_state[list(self.dof_indices)] = reduced_state[: self.position_dim]
        if self.order == 2:
            velocity_indices = [self.robot_cfg.num_dof + idx for idx in self.dof_indices]
            full_state[velocity_indices] = reduced_state[self.position_dim :]
        return full_state

    def _reduce_state(self, full_state: Array) -> Array:
        full_state = np.asarray(full_state, dtype=float).reshape(-1)
        if self.full_state_only:
            return full_state.reshape(self.state_dim).copy()
        position = full_state[list(self.dof_indices)]
        if self.order == 1:
            return position.astype(float)
        velocity_indices = [self.robot_cfg.num_dof + idx for idx in self.dof_indices]
        return np.concatenate([position, full_state[velocity_indices]]).astype(float)
