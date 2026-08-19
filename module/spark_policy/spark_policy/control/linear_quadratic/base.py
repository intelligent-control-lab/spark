from __future__ import annotations

from typing import Optional, Tuple

import numpy as np

from spark_policy.core.policy import BasePolicy
from spark_policy.utils.control_math import (
    finite_horizon_lqr,
    infinite_lqr,
    lift_dynamics,
    matrix_arg,
    vector_arg,
)
from spark_robot import RobotConfig, RobotKinematics


class _LinearQuadraticPolicy(BasePolicy):
    """Shared implementation for linear-quadratic feedback policies."""

    algorithm_name = "linear_quadratic"

    def __init__(self, robot_cfg: RobotConfig, robot_kinematics: RobotKinematics, **kwargs) -> None:
        super().__init__(robot_cfg, robot_kinematics)

        self.dt = float(kwargs.get("dt", 0.1))
        self.horizon_steps = int(kwargs.get("horizon_steps", kwargs.get("N", 20)))
        self.execution_mode = str(kwargs.get("execution_mode", "feedback"))
        self.state_feedback_mode = str(kwargs.get("state_feedback_mode", "agent"))
        self.command_mode = str(kwargs.get("command_mode", "direct"))
        self.discretization = str(kwargs.get("discretization", "ZOH"))
        self.integrator = str(kwargs.get("integrator", "direct"))
        self.clip_robot_action = bool(kwargs.get("clip_robot_action", True))

        state_dim_hint = kwargs.get("state_dim", kwargs.get("dim", None))
        full_state_only = bool(getattr(self.robot_cfg, "dynamics_full_state_only", False))
        if full_state_only:
            self.dynamics_model = self.robot_cfg.create_dynamics_model()
            self.position_dof_names = tuple(self.dynamics_model.state_dof_names)
            self.velocity_dof_names = self.position_dof_names
            self.dynamics_control_names = tuple(self.dynamics_model.control_names)
            requested_state_names = kwargs.get("position_dof_names", None)
            requested_control_names = kwargs.get(
                "dynamics_control_names", kwargs.get("control_names", None)
            )
            if (
                requested_state_names is not None
                and tuple(requested_state_names) != self.position_dof_names
            ):
                raise ValueError("This dynamics configuration requires its complete state layout.")
            if (
                requested_control_names is not None
                and tuple(requested_control_names) != self.dynamics_control_names
            ):
                raise ValueError(
                    "This dynamics configuration requires its complete control layout."
                )
        else:
            position_count = 2 if state_dim_hint in (None, 4) else max(1, int(state_dim_hint) // 2)
            default_position_names = ("LinearX", "LinearY")[:position_count]
            self.position_dof_names = tuple(
                kwargs.get("position_dof_names", default_position_names)
            )
            self.velocity_dof_names = tuple(
                kwargs.get("velocity_dof_names", self.position_dof_names)
            )
            if self.velocity_dof_names != self.position_dof_names:
                raise ValueError(
                    "The selected robot dynamics requires matching position_dof_names and velocity_dof_names."
                )

            default_prefix = "a" if int(getattr(self.robot_cfg, "dynamics_order", 1)) == 2 else "v"
            default_control_names = tuple(
                f"{default_prefix}{name}" for name in self.position_dof_names
            )
            self.dynamics_control_names = tuple(
                kwargs.get(
                    "dynamics_control_names", kwargs.get("control_names", default_control_names)
                )
            )
            self.dynamics_model = self.robot_cfg.create_dynamics_model(
                state_dof_names=self.position_dof_names,
                control_names=self.dynamics_control_names,
            )
        self.state_dim = self.dynamics_model.state_dim
        self.control_dim = self.dynamics_model.control_dim
        self._validate_legacy_dimensions(kwargs)
        self._validate_legacy_model(kwargs.get("model", None))

        self.state_names = tuple(kwargs.get("state_names", self.dynamics_model.state_names))
        self.policy_control_names = tuple(
            kwargs.get("policy_control_names", self.dynamics_control_names)
        )
        self.command_control_names = tuple(kwargs.get("control_names", self.dynamics_control_names))
        self.zero_control_names = tuple(kwargs.get("zero_control_names", ()))

        self._position_dof_indices = self._resolve_enum_indices(
            self.robot_cfg.DoFs, self.position_dof_names
        )
        self._velocity_dof_indices = self._resolve_enum_indices(
            self.robot_cfg.DoFs, self.velocity_dof_names
        )
        self._command_control_indices = self._resolve_enum_indices(
            self.robot_cfg.Control, self.command_control_names
        )
        self._zero_control_indices = self._resolve_enum_indices(
            self.robot_cfg.Control, self.zero_control_names
        )

        self.A, self.B = self._resolve_dynamics_matrices(kwargs)
        if self.B.shape[1] != self.control_dim:
            self.control_dim = int(self.B.shape[1])

        self.Q = matrix_arg(kwargs.get("Q", None), np.eye(self.state_dim))
        self.R = matrix_arg(kwargs.get("R", None), np.eye(self.control_dim))
        self.S = matrix_arg(kwargs.get("S", None), self.Q)
        self.goal_state = vector_arg(kwargs.get("goal_state", None), self.state_dim, default=0.0)
        self.reference_trajectory = self._normalize_reference_trajectory(
            kwargs.get("reference_trajectory", None)
        )
        self.initial_state = self._optional_vector(
            kwargs.get("initial_state", kwargs.get("x0", None)), self.state_dim
        )
        self.u_ref = vector_arg(kwargs.get("u_ref", None), self.control_dim, default=0.0)
        self.u_min = self._optional_vector(
            kwargs.get("u_min", kwargs.get("umin", None)), self.control_dim
        )
        self.u_max = self._optional_vector(
            kwargs.get("u_max", kwargs.get("umax", None)), self.control_dim
        )

        self.K, self.P = self._compute_gains()

        self._planned = False
        self._plan_step = 0
        self._x_plan: Optional[np.ndarray] = None
        self._u_plan: Optional[np.ndarray] = None
        self._reference_plan: Optional[np.ndarray] = None
        self._internal_state: Optional[np.ndarray] = None

    def act(self, agent_feedback: dict, task_info: dict):
        reference_states = self._reference_horizon(task_info, self._plan_step)
        if self.execution_mode == "plan_once":
            if not self._planned:
                x0 = self._initial_or_feedback_state(agent_feedback)
                self._x_plan, self._u_plan = self._rollout_plan(x0, reference_states)
                self._reference_plan = reference_states.copy()
                self._planned = True
                self._plan_step = 0
            assert self._x_plan is not None
            assert self._u_plan is not None
            state_k = min(self._plan_step, self._x_plan.shape[0] - 1)
            control_k = min(self._plan_step, self._u_plan.shape[0] - 1)
            x = self._x_plan[state_k].copy()
            if self._plan_step < self._u_plan.shape[0]:
                u = self._u_plan[control_k].copy()
                command = self._command_from_policy_control(u, x, control_k, self._x_plan)
            else:
                u = np.zeros(self.control_dim, dtype=float)
                command = np.zeros(len(self.robot_cfg.Control), dtype=float)
        else:
            x = self._feedback_state(agent_feedback)
            self._x_plan, self._u_plan = self._rollout_plan(x, reference_states)
            self._reference_plan = reference_states.copy()
            k = 0
            u = self._u_plan[0].copy()
            command = self._command_from_policy_control(u, x, k, self._x_plan)

        if self.clip_robot_action:
            command = self._clip_robot_command(command)

        assert self._reference_plan is not None
        info = {
            "policy_name": self.algorithm_name,
            "policy_plan_step": int(self._plan_step),
            "policy_state": x.copy(),
            "policy_control": u.copy(),
            "policy_plan": {
                "states": self._x_plan.copy(),
                "controls": self._u_plan.copy(),
                "references": self._reference_plan.copy(),
                "state_t": np.arange(self._x_plan.shape[0], dtype=float) * self.dt,
                "control_t": np.arange(self._u_plan.shape[0], dtype=float) * self.dt,
                "state_names": self.state_names,
                "control_names": self.policy_control_names,
                "solver_info": self._solver_info(),
            },
        }
        if self.execution_mode != "plan_once" and self.state_feedback_mode.lower() in (
            "internal",
            "internal_model",
        ):
            self._internal_state = self.dynamics_model.step(x, u, self.dt, self.integrator)
        self._plan_step += 1
        return command, info

    def _compute_gains(self):
        raise NotImplementedError

    def _gain_at_step(self, k: int) -> np.ndarray:
        raise NotImplementedError

    def _feedback_control(
        self,
        x: np.ndarray,
        k: int,
        reference_state: np.ndarray | None = None,
    ) -> np.ndarray:
        reference_state = self.goal_state if reference_state is None else reference_state
        u = self.u_ref - self._gain_at_step(k) @ (x - reference_state)
        if self.u_min is not None:
            u = np.maximum(u, self.u_min)
        if self.u_max is not None:
            u = np.minimum(u, self.u_max)
        return u

    def _rollout_plan(
        self,
        x0: np.ndarray,
        reference_states: np.ndarray | None = None,
    ) -> Tuple[np.ndarray, np.ndarray]:
        if reference_states is None:
            reference_states = np.tile(self.goal_state, (self.horizon_steps + 1, 1))
        reference_states = np.asarray(reference_states, dtype=float).reshape(
            self.horizon_steps + 1, self.state_dim
        )
        states = np.zeros((self.horizon_steps + 1, self.state_dim), dtype=float)
        controls = np.zeros((self.horizon_steps, self.control_dim), dtype=float)
        states[0] = np.asarray(x0, dtype=float).reshape(self.state_dim)
        for k in range(self.horizon_steps):
            controls[k] = self._feedback_control(states[k], k, reference_states[k])
            states[k + 1] = self.dynamics_model.step(
                states[k], controls[k], self.dt, self.integrator
            )
        return states, controls

    def _command_from_policy_control(
        self,
        policy_control: np.ndarray,
        policy_state: np.ndarray,
        step_index: int,
        planned_states: Optional[np.ndarray],
    ) -> np.ndarray:
        command = np.zeros(len(self.robot_cfg.Control), dtype=float)
        mode = self.command_mode.lower()

        if mode in ("direct", "velocity"):
            command_values = policy_control
        elif mode in ("average_velocity_from_plan", "average_velocity"):
            if planned_states is not None and step_index + 1 < planned_states.shape[0]:
                half = self.state_dim // 2
                command_values = (
                    planned_states[step_index + 1, :half] - planned_states[step_index, :half]
                ) / self.dt
            else:
                command_values = self._average_velocity(policy_state, policy_control)
        elif mode in ("velocity_from_acceleration", "next_velocity"):
            half = self.state_dim // 2
            command_values = policy_state[half:] + self.dt * policy_control
        else:
            raise ValueError(f'Unknown command_mode "{self.command_mode}".')

        command_values = np.asarray(command_values, dtype=float).reshape(-1)
        if command_values.size != len(self._command_control_indices):
            raise ValueError(
                "Command/control dimension mismatch: "
                f"{command_values.size} values for {len(self._command_control_indices)} robot controls."
            )

        for i, control_idx in enumerate(self._command_control_indices):
            command[int(control_idx)] = command_values[i]
        for control_idx in self._zero_control_indices:
            command[int(control_idx)] = 0.0
        return command

    def _average_velocity(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        half = self.state_dim // 2
        return x[half:] + 0.5 * self.dt * u

    def _extract_state(self, agent_feedback: dict) -> np.ndarray:
        dof_pos = np.asarray(agent_feedback["dof_pos_fbk"], dtype=float).reshape(-1)
        dof_vel = np.asarray(agent_feedback["dof_vel_fbk"], dtype=float).reshape(-1)
        return self.dynamics_model.extract_state(dof_pos, dof_vel)

    def _initial_or_feedback_state(self, agent_feedback: dict) -> np.ndarray:
        if self.initial_state is not None:
            return self.initial_state.copy()
        return self._extract_state(agent_feedback)

    def _feedback_state(self, agent_feedback: dict) -> np.ndarray:
        mode = self.state_feedback_mode.lower()
        if mode in ("agent", "feedback"):
            return self._extract_state(agent_feedback)
        if mode in ("internal", "internal_model"):
            if self._internal_state is None:
                self._internal_state = self._initial_or_feedback_state(agent_feedback)
            return self._internal_state.copy()
        raise ValueError(f'Unknown state_feedback_mode "{self.state_feedback_mode}".')

    def _clip_robot_command(self, command: np.ndarray) -> np.ndarray:
        clipped = np.asarray(command, dtype=float).copy()
        for control_id in self.robot_cfg.Control:
            limit = float(self.robot_cfg.ControlLimit[control_id])
            clipped[int(control_id)] = np.clip(clipped[int(control_id)], -limit, limit)
        return clipped

    def _resolve_dynamics_matrices(self, kwargs) -> Tuple[np.ndarray, np.ndarray]:
        if kwargs.get("A", None) is not None or kwargs.get("B", None) is not None:
            raise ValueError(
                "A/B overrides are no longer supported; select a robot dynamics configuration."
            )
        return self.dynamics_model.discrete_matrices(self.dt, self.discretization)

    def _solver_info(self):
        return {
            "algorithm": self.algorithm_name,
            "dynamics_variant": self.dynamics_model.variant,
            "robot_config": type(self.robot_cfg).__name__,
            "dt": self.dt,
            "horizon_steps": self.horizon_steps,
            "execution_mode": self.execution_mode,
            "state_feedback_mode": self.state_feedback_mode,
            "command_mode": self.command_mode,
        }

    def _normalize_reference_trajectory(self, value) -> Optional[np.ndarray]:
        if value is None:
            return None
        reference = np.asarray(value, dtype=float)
        if reference.ndim == 1:
            if reference.size != self.state_dim:
                raise ValueError(
                    f"Expected reference state of length {self.state_dim}, got {reference.size}."
                )
            reference = reference.reshape(1, self.state_dim)
        if reference.ndim != 2 or reference.shape[1] != self.state_dim:
            raise ValueError(
                f"reference_trajectory must have shape (steps, state_dim); got {reference.shape}."
            )
        if reference.shape[0] == 0:
            raise ValueError("reference_trajectory must contain at least one state.")
        return reference.copy()

    def _reference_horizon(self, task_info: dict, start_step: int) -> np.ndarray:
        task_reference = None
        if task_info:
            task_reference = task_info.get("reference_trajectory")
        reference = (
            self._normalize_reference_trajectory(task_reference)
            if task_reference is not None
            else self.reference_trajectory
        )
        if reference is None:
            return np.tile(self.goal_state, (self.horizon_steps + 1, 1))
        indices = np.minimum(
            np.arange(int(start_step), int(start_step) + self.horizon_steps + 1),
            reference.shape[0] - 1,
        )
        return reference[indices].copy()

    def _validate_legacy_dimensions(self, kwargs) -> None:
        state_dim = kwargs.get("state_dim", kwargs.get("dim", None))
        control_dim = kwargs.get("control_dim", None)
        if state_dim is not None and int(state_dim) != self.state_dim:
            raise ValueError(
                f"state_dim={state_dim} conflicts with {type(self.robot_cfg).__name__} dynamics "
                f"dimension {self.state_dim}."
            )
        if control_dim is not None and int(control_dim) != self.control_dim:
            raise ValueError(
                f"control_dim={control_dim} conflicts with {type(self.robot_cfg).__name__} dynamics "
                f"dimension {self.control_dim}."
            )

    def _validate_legacy_model(self, model) -> None:
        if model is None:
            return
        normalized = (
            str(model).lower().replace("-", "_").replace("doubleintegrator", "double_integrator")
        )
        if normalized != self.dynamics_model.variant:
            raise ValueError(
                f'model="{model}" conflicts with robot-config dynamics "{self.dynamics_model.variant}". '
                "Select the appropriate robot configuration instead."
            )

    def _default_state_names(self):
        if self.state_dim == 4:
            return ("x", "y", "vx", "vy")
        return tuple(f"x{i + 1}" for i in range(self.state_dim))

    def _default_policy_control_names(self):
        if self.control_dim == 2:
            return ("ux", "uy")
        return tuple(f"u{i + 1}" for i in range(self.control_dim))

    def _default_position_dof_names(self):
        if self.state_dim >= 2:
            return ("LinearX", "LinearY")
        return ("LinearX",)

    def _default_command_control_names(self):
        if self.control_dim >= 2:
            return ("vLinearX", "vLinearY")
        return ("vLinearX",)

    @staticmethod
    def _optional_vector(value, length: int) -> Optional[np.ndarray]:
        if value is None:
            return None
        arr = np.asarray(value, dtype=float).reshape(-1)
        if arr.size != length:
            raise ValueError(f"Expected vector of length {length}, got {arr.size}.")
        return arr

    @staticmethod
    def _resolve_enum_indices(enum_cls, names):
        indices = []
        members = enum_cls.__members__
        for name in names:
            if isinstance(name, str):
                if name not in members:
                    raise KeyError(f'{enum_cls.__name__} has no member "{name}".')
                indices.append(int(members[name]))
            else:
                indices.append(int(name))
        return tuple(indices)


class LQRPolicy(_LinearQuadraticPolicy):
    """Infinite-horizon linear-quadratic feedback policy."""

    algorithm_name = "lqr"

    def _compute_gains(self):
        return infinite_lqr(self.A, self.B, self.Q, self.R, discrete=True)

    def _gain_at_step(self, k: int) -> np.ndarray:
        return self.K


class FiniteHorizonLQRPolicy(_LinearQuadraticPolicy):
    """Finite-horizon linear-quadratic feedback policy."""

    algorithm_name = "finite_horizon_lqr"

    def _compute_gains(self):
        return finite_horizon_lqr(self.A, self.B, self.Q, self.R, self.S, self.horizon_steps)

    def _gain_at_step(self, k: int) -> np.ndarray:
        return self.K[min(int(k), self.K.shape[0] - 1)]


class LinearMPCPolicy(FiniteHorizonLQRPolicy):
    """Unconstrained linear MPC using the first finite-horizon gain at each step."""

    algorithm_name = "linear_mpc"

    def _gain_at_step(self, k: int) -> np.ndarray:
        return self.K[0]


class InputConstrainedMPCPolicy(LinearMPCPolicy):
    """Linear MPC with elementwise input clipping."""

    algorithm_name = "input_constrained_mpc"


class ConstrainedMPCPolicy(_LinearQuadraticPolicy):
    """Linear MPC with lifted state and input inequality constraints."""

    algorithm_name = "constrained_mpc"

    def __init__(self, robot_cfg: RobotConfig, robot_kinematics: RobotKinematics, **kwargs) -> None:
        super().__init__(robot_cfg, robot_kinematics, **kwargs)
        self.x_min = self._optional_vector(
            kwargs.get("x_min", kwargs.get("xmin", None)), self.state_dim
        )
        self.x_max = self._optional_vector(
            kwargs.get("x_max", kwargs.get("xmax", None)), self.state_dim
        )
        self.qp_max_iter = int(kwargs.get("qp_max_iter", 4000))
        self.qp_ftol = float(kwargs.get("qp_ftol", 1e-10))
        self._last_solution: Optional[np.ndarray] = None
        self._build_qp_matrices()

    def _compute_gains(self):
        return (
            np.zeros((self.control_dim, self.state_dim), dtype=float),
            np.zeros((self.state_dim, self.state_dim), dtype=float),
        )

    def _gain_at_step(self, k: int) -> np.ndarray:
        return self.K

    def _rollout_plan(
        self,
        x0: np.ndarray,
        reference_states: np.ndarray | None = None,
    ) -> Tuple[np.ndarray, np.ndarray]:
        if reference_states is None:
            reference_states = np.tile(self.goal_state, (self.horizon_steps + 1, 1))
        reference_states = np.asarray(reference_states, dtype=float).reshape(
            self.horizon_steps + 1, self.state_dim
        )
        u_seq, success, message = self._solve_qp(x0, reference_states)
        if not success:
            raise RuntimeError(f"Constrained MPC QP failed: {message}")
        stacked_states = self._bar_A @ x0 + self._bar_B @ u_seq
        states = stacked_states.reshape(self.horizon_steps + 1, self.state_dim)
        controls = u_seq.reshape(self.horizon_steps, self.control_dim)
        return states, controls

    def _build_qp_matrices(self) -> None:
        self._bar_A, self._bar_B = lift_dynamics(self.A, self.B, self.horizon_steps)
        bar_q = np.kron(np.eye(self.horizon_steps + 1), self.Q)
        terminal = slice(
            self.horizon_steps * self.state_dim, (self.horizon_steps + 1) * self.state_dim
        )
        bar_q[terminal, terminal] = self.S
        bar_r = np.kron(np.eye(self.horizon_steps), self.R)
        self._qp_H = self._bar_B.T @ bar_q @ self._bar_B + bar_r
        self._qp_H = 0.5 * (self._qp_H + self._qp_H.T)
        self._qp_reference_map = self._bar_B.T @ bar_q
        self._bar_r = bar_r

        n_decision = self.horizon_steps * self.control_dim
        identity_u = np.eye(n_decision)
        self._constraint_A = np.vstack([self._bar_B, -self._bar_B, identity_u, -identity_u])

    def _constraint_b(self, x: np.ndarray) -> np.ndarray:
        if self.x_max is None:
            x_max = np.full(self.state_dim, np.inf, dtype=float)
        else:
            x_max = self.x_max
        if self.x_min is None:
            x_min = np.full(self.state_dim, -np.inf, dtype=float)
        else:
            x_min = self.x_min
        if self.u_max is None:
            u_max = np.full(self.control_dim, np.inf, dtype=float)
        else:
            u_max = self.u_max
        if self.u_min is None:
            u_min = np.full(self.control_dim, -np.inf, dtype=float)
        else:
            u_min = self.u_min

        max_x = np.tile(x_max, self.horizon_steps + 1)
        min_x = np.tile(x_min, self.horizon_steps + 1)
        max_u = np.tile(u_max, self.horizon_steps)
        min_u = np.tile(u_min, self.horizon_steps)
        nominal = self._bar_A @ x
        return np.concatenate([max_x - nominal, -min_x + nominal, max_u, -min_u])

    def _initial_qp_guess(self) -> np.ndarray:
        n_decision = self.horizon_steps * self.control_dim
        if self._last_solution is None:
            return np.zeros(n_decision, dtype=float)
        shifted = np.zeros_like(self._last_solution)
        shifted[: -self.control_dim] = self._last_solution[self.control_dim :]
        return shifted

    def _solve_qp(self, x: np.ndarray, reference_states: np.ndarray):
        x = np.asarray(x, dtype=float).reshape(self.state_dim)
        reference = np.asarray(reference_states, dtype=float).reshape(-1)
        control_reference = np.tile(self.u_ref, self.horizon_steps)
        f = self._qp_reference_map @ (self._bar_A @ x - reference) - self._bar_r @ control_reference
        a_ineq = self._constraint_A
        b_ineq = self._constraint_b(x)
        z0 = self._initial_qp_guess()

        try:
            return self._solve_qp_osqp(f, a_ineq, b_ineq, z0)
        except ImportError:
            return self._solve_qp_slsqp(f, a_ineq, b_ineq, z0)

    def _solve_qp_osqp(self, f: np.ndarray, a_ineq: np.ndarray, b_ineq: np.ndarray, z0: np.ndarray):
        import osqp
        from scipy import sparse

        problem = osqp.OSQP()
        lower = np.full(a_ineq.shape[0], -np.inf, dtype=float)
        problem.setup(
            P=sparse.csc_matrix(self._qp_H),
            q=np.asarray(f, dtype=float),
            A=sparse.csc_matrix(a_ineq),
            l=lower,
            u=np.asarray(b_ineq, dtype=float),
            verbose=False,
            polish=True,
            eps_abs=self.qp_ftol,
            eps_rel=self.qp_ftol,
            max_iter=self.qp_max_iter,
        )
        problem.warm_start(x=z0)
        result = problem.solve()
        success = result.info.status in ("solved", "solved inaccurate")
        if success:
            self._last_solution = np.asarray(result.x, dtype=float).reshape(-1)
        return np.asarray(result.x, dtype=float).reshape(-1), bool(success), str(result.info.status)

    def _solve_qp_slsqp(
        self, f: np.ndarray, a_ineq: np.ndarray, b_ineq: np.ndarray, z0: np.ndarray
    ):
        from scipy.optimize import minimize

        def objective(z):
            return 0.5 * z @ self._qp_H @ z + f @ z

        def gradient(z):
            return self._qp_H @ z + f

        constraints = {
            "type": "ineq",
            "fun": lambda z: b_ineq - a_ineq @ z,
            "jac": lambda z: -a_ineq,
        }
        result = minimize(
            objective,
            z0,
            jac=gradient,
            constraints=constraints,
            method="SLSQP",
            options={"ftol": self.qp_ftol, "maxiter": self.qp_max_iter, "disp": False},
        )
        if result.success:
            self._last_solution = np.asarray(result.x, dtype=float).reshape(-1)
        return (
            np.asarray(result.x, dtype=float).reshape(-1),
            bool(result.success),
            str(result.message),
        )

    def _solver_info(self):
        info = super()._solver_info()
        info.update(
            {
                "qp_max_iter": self.qp_max_iter,
                "qp_ftol": self.qp_ftol,
                "has_state_bounds": self.x_min is not None or self.x_max is not None,
                "has_input_bounds": self.u_min is not None or self.u_max is not None,
            }
        )
        return info
