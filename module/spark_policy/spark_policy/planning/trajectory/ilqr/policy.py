from __future__ import annotations

from typing import Callable, Dict, Optional, Tuple

import numpy as np

from spark_policy.core.policy import BasePolicy
from spark_robot import RobotConfig, RobotKinematics


def _wrap_to_pi(angle: float) -> float:
    return (float(angle) + np.pi) % (2.0 * np.pi) - np.pi


class ILQRPolicy(BasePolicy):
    """Plan-once iLQR policy for the AgiBot mobile base.

    It solves once, stores the full plan, and replays one reduced base command
    per ``act`` call. The policy is self-contained: prediction rollout,
    linearization, cost evaluation, and line search live in this file.
    """

    def __init__(self, robot_cfg: RobotConfig, robot_kinematics: RobotKinematics, **kwargs) -> None:
        super().__init__(robot_cfg, robot_kinematics)

        self.N = int(kwargs.get("N", kwargs.get("horizon_steps", 150)))
        self.dt = float(kwargs.get("dt", 0.1))
        self.integrator = str(kwargs.get("integrator", "Euler"))
        self.initial_nominal = str(kwargs.get("initial_nominal", "rollout"))
        self.solve_mode = str(kwargs.get("solve_mode", "plan_once"))

        self.Q = self._matrix_arg(kwargs.get("Q", None), np.diag([1.0, 1.0, 0.1]))
        self.R = self._matrix_arg(kwargs.get("R", None), np.diag([2.0, 2.5]))
        self.S = self._matrix_arg(kwargs.get("S", None), np.diag([100.0, 100.0, 50.0]))
        self.u_ref = np.asarray(kwargs.get("u_ref", [0.0, 0.0]), dtype=float).reshape(2)
        self.alphas = np.asarray(
            kwargs.get("alphas", [1.0, 0.5, 0.25, 0.1, 0.05]), dtype=float
        ).reshape(-1)
        self.max_iter = int(kwargs.get("maxIter", kwargs.get("max_iter", 120)))
        self.tol_j = float(kwargs.get("tolJ", kwargs.get("tol_j", 1e-6)))
        self.lambda0 = float(kwargs.get("lambda0", 1e-8))
        self.lambda_max = float(kwargs.get("lambdaMax", kwargs.get("lambda_max", 1e8)))
        self.improve_tol = float(kwargs.get("improve_tol", 1e-12))

        self.verbose = bool(kwargs.get("verbose", False))
        self.goal_state = self._optional_vector(kwargs.get("goal_state", None), 3)

        self.state_names = tuple(kwargs.get("state_names", ["LinearX", "LinearY", "RotYaw"]))
        self.control_names = tuple(kwargs.get("control_names", ["vLinearX", "vRotYaw"]))
        self.zero_control_names = tuple(kwargs.get("zero_control_names", ["vLinearY"]))

        self.dynamics_model = self.robot_cfg.create_dynamics_model(
            state_dof_names=self.state_names,
            control_names=self.control_names,
        )
        if self.dynamics_model.variant != "unicycle":
            raise ValueError(
                "ILQRPolicy requires a unicycle robot configuration; "
                f"got {type(self.robot_cfg).__name__} ({self.dynamics_model.variant})."
            )
        if self.dynamics_model.state_dim != 3 or self.dynamics_model.control_dim != 2:
            raise ValueError("ILQRPolicy requires a 3-state, 2-control robot dynamics model.")

        self._state_indices = self._resolve_enum_indices(self.robot_cfg.DoFs, self.state_names)
        self._control_indices = self._resolve_enum_indices(
            self.robot_cfg.Control, self.control_names
        )
        self._zero_control_indices = self._resolve_enum_indices(
            self.robot_cfg.Control, self.zero_control_names
        )

        if len(self._state_indices) != 3:
            raise ValueError("ILQRPolicy currently expects exactly 3 reduced state DoFs.")
        if len(self._control_indices) != 2:
            raise ValueError("ILQRPolicy currently expects exactly 2 reduced controls.")

        self._planned = False
        self._plan_step = 0
        self._x_plan: Optional[np.ndarray] = None
        self._u_plan: Optional[np.ndarray] = None
        self._solve_info: Dict[str, object] = {}

    # ------------------------------------------------------------------
    # Public policy entrypoint
    # ------------------------------------------------------------------

    def act(self, agent_feedback: dict, task_info: dict):
        if self.solve_mode != "plan_once":
            raise NotImplementedError(
                'This first ILQRPolicy version supports solve_mode="plan_once" only.'
            )

        if not self._planned:
            x0 = self._extract_state(agent_feedback)
            xg = self._extract_goal(task_info)
            x_plan, u_plan, solve_info = self._solve_ilqr(x0, xg)
            self._x_plan = x_plan
            self._u_plan = u_plan
            self._solve_info = solve_info
            self._planned = True
            self._plan_step = 0

        assert self._x_plan is not None
        assert self._u_plan is not None

        control = np.zeros(len(self.robot_cfg.Control), dtype=float)
        if self._plan_step < self._u_plan.shape[1]:
            u_reduced = self._u_plan[:, self._plan_step]
        else:
            u_reduced = np.zeros(2, dtype=float)

        for reduced_id, control_id in enumerate(self._control_indices):
            control[control_id] = u_reduced[reduced_id]
        for control_id in self._zero_control_indices:
            control[control_id] = 0.0

        for control_id in self.robot_cfg.Control:
            limit = float(self.robot_cfg.ControlLimit[control_id])
            control[int(control_id)] = np.clip(control[int(control_id)], -limit, limit)

        info = {
            "ilqr_plan_step": int(self._plan_step),
            "ilqr_reduced_control": u_reduced.copy(),
            "policy_plan": {
                "states": self._x_plan.T.copy(),
                "controls": self._u_plan.T.copy(),
                "state_t": np.arange(self._x_plan.shape[1], dtype=float) * self.dt,
                "control_t": np.arange(self._u_plan.shape[1], dtype=float) * self.dt,
                "state_names": self.state_names,
                "control_names": self.control_names,
                "solver_info": self._solve_info,
            },
        }
        self._plan_step += 1
        return control, info

    # ------------------------------------------------------------------
    # Robot adapters
    # ------------------------------------------------------------------

    def _extract_state(self, agent_feedback: dict) -> np.ndarray:
        dof_pos = np.asarray(agent_feedback["dof_pos_fbk"], dtype=float).reshape(-1)
        x = dof_pos[list(self._state_indices)].astype(float)
        x[2] = _wrap_to_pi(x[2])
        return x

    def _extract_goal(self, task_info: dict) -> np.ndarray:
        if self.goal_state is not None:
            xg = self.goal_state.copy()
            xg[2] = _wrap_to_pi(xg[2])
            return xg

        goal_teleop = task_info.get("goal_teleop", {})
        if "base" not in goal_teleop:
            raise KeyError('ILQRPolicy needs goal_state or task_info["goal_teleop"]["base"].')

        base_goal = np.asarray(goal_teleop["base"], dtype=float)
        if base_goal.shape == (1, 4, 4):
            base_goal = base_goal[0]
        elif base_goal.shape != (4, 4):
            base_goal = base_goal.reshape(-1, 4, 4)[0]

        yaw = np.arctan2(base_goal[1, 0], base_goal[0, 0])
        return np.array([base_goal[0, 3], base_goal[1, 3], _wrap_to_pi(yaw)], dtype=float)

    # ------------------------------------------------------------------
    # iLQR implementation
    # ------------------------------------------------------------------

    def _solve_ilqr(
        self, x0: np.ndarray, xg: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray, Dict[str, object]]:
        nx = 3
        nu = 2
        ubar = np.zeros((nu, self.N), dtype=float)
        xbar = np.zeros((nx, self.N + 1), dtype=float)

        if self.initial_nominal.lower() in ("linear",):
            for k in range(self.N + 1):
                tau = k / float(self.N)
                xbar[:, k] = (1.0 - tau) * x0 + tau * xg
        elif self.initial_nominal.lower() in ("rollout", "dynamic"):
            xbar, ubar = self._rollout(x0, lambda _x, k: ubar[:, k])
        else:
            raise ValueError(f'Unknown initial_nominal "{self.initial_nominal}".')

        cost = self._total_cost(xbar, ubar, xg)
        prev_cost = cost
        lambda_reg = self.lambda0

        cost_history = [float(cost)]
        cost_iteration = [0]
        alpha_history = []
        lambda_history = [float(lambda_reg)]
        accepted_iterations = 0
        backward_failures = 0
        line_search_rejections = 0
        status = "max_iter"
        last_iteration = 0

        if self.verbose:
            print(f"Iter {0:2d}: J = {cost:.6f}")

        for iteration in range(1, self.max_iter + 1):
            last_iteration = iteration

            A = []
            B = []
            defect = []
            q = []
            r = []
            for k in range(self.N):
                xk = xbar[:, k]
                uk = ubar[:, k]
                Ak, Bk = self._linearize(xk, uk)
                A.append(Ak)
                B.append(Bk)
                defect.append(self._step(xk, uk) - xbar[:, k + 1])
                q.append(self.Q @ (xk - xg))
                r.append(self.R @ (uk - self.u_ref))

            P = [None] * (self.N + 1)
            s = [None] * (self.N + 1)
            K = [None] * self.N
            kfeed = [None] * self.N
            P[self.N] = self.S.copy()
            s[self.N] = np.zeros(nx, dtype=float)
            diverged = False

            for k in range(self.N - 1, -1, -1):
                Ak = A[k]
                Bk = B[k]
                dk = defect[k]
                Pn = P[k + 1]
                sn = s[k + 1]
                qk = q[k]
                rk = r[k]

                Quu = self.R + Bk.T @ Pn @ Bk + lambda_reg * np.eye(nu)
                Qux = Bk.T @ Pn @ Ak
                gu = rk + Bk.T @ (Pn @ dk + sn)

                try:
                    chol = np.linalg.cholesky(Quu)
                    inv_quu = np.linalg.solve(chol.T, np.linalg.solve(chol, np.eye(nu)))
                except np.linalg.LinAlgError:
                    diverged = True
                    break

                K[k] = -inv_quu @ Qux
                kfeed[k] = -inv_quu @ gu
                P[k] = self.Q + Ak.T @ Pn @ Ak - Qux.T @ inv_quu @ Qux
                s[k] = qk + Ak.T @ (Pn @ dk + sn) - Qux.T @ inv_quu @ gu
                P[k] = 0.5 * (P[k] + P[k].T)

            if diverged:
                backward_failures += 1
                lambda_reg = min(lambda_reg * 10.0, self.lambda_max)
                if lambda_reg >= self.lambda_max:
                    status = "backward_pass_failed"
                    break
                continue

            best_cost = np.inf
            best_alpha = np.nan
            best_x = None
            best_u = None
            for alpha in self.alphas:
                xnew, unew = self._rollout(
                    x0,
                    lambda x, k, alpha=alpha: (
                        ubar[:, k] + alpha * kfeed[k] + K[k] @ (x - xbar[:, k])
                    ),
                )
                candidate_cost = self._total_cost(xnew, unew, xg)
                if candidate_cost < best_cost:
                    best_cost = candidate_cost
                    best_alpha = float(alpha)
                    best_x = xnew
                    best_u = unew

            if (cost - best_cost) > max(self.improve_tol, 1e-12 * abs(cost)):
                xbar = best_x
                ubar = best_u
                cost = float(best_cost)
                lambda_reg = max(lambda_reg / 5.0, 1e-12)
                accepted_iterations += 1
                cost_history.append(float(cost))
                cost_iteration.append(int(iteration))
                alpha_history.append(float(best_alpha))
                lambda_history.append(float(lambda_reg))
            else:
                line_search_rejections += 1
                lambda_reg = min(lambda_reg * 10.0, self.lambda_max)
                if lambda_reg >= self.lambda_max:
                    status = "line_search_failed"
                    break
                continue

            if self.verbose:
                print(
                    f"Iter {iteration:2d}: J = {cost:.6f}  (alpha={best_alpha:.2f}, lambda={lambda_reg:.1e})"
                )
            if abs(prev_cost - cost) < self.tol_j:
                status = "tolJ"
                break
            prev_cost = cost

        info = {
            "ilqr_status": status,
            "ilqr_iterations": int(last_iteration),
            "ilqr_accepted_iterations": int(accepted_iterations),
            "ilqr_backward_failures": int(backward_failures),
            "ilqr_line_search_rejections": int(line_search_rejections),
            "ilqr_cost": float(cost),
            "ilqr_cost_history": np.asarray(cost_history, dtype=float),
            "ilqr_cost_iteration": np.asarray(cost_iteration, dtype=int),
            "ilqr_alpha_history": np.asarray(alpha_history, dtype=float),
            "ilqr_lambda_history": np.asarray(lambda_history, dtype=float),
            "ilqr_goal_state": xg.copy(),
            "ilqr_final_state": xbar[:, -1].copy(),
            "ilqr_goal_error": xbar[:, -1].copy() - xg,
        }
        return xbar, ubar, info

    def _rollout(
        self, x0: np.ndarray, controller: Callable[[np.ndarray, int], np.ndarray]
    ) -> Tuple[np.ndarray, np.ndarray]:
        xlist = np.zeros((3, self.N + 1), dtype=float)
        ulist = np.zeros((2, self.N), dtype=float)
        xlist[:, 0] = np.asarray(x0, dtype=float).reshape(3)

        for k in range(self.N):
            xk = xlist[:, k]
            uk = np.asarray(controller(xk, self._control_index(k)), dtype=float).reshape(2)
            ulist[:, k] = uk
            xlist[:, k + 1] = self._step(xk, uk)

        return xlist, ulist

    def _control_index(self, k: int) -> int:
        idx = int(np.floor((k * self.dt) / self.dt))
        return max(0, min(self.N - 1, idx))

    def _step(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        return self.dynamics_model.step(x, u, self.dt, self.integrator)

    def _dynamics(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        return self.dynamics_model.derivative(x, u)

    def _linearize(self, x: np.ndarray, u: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        return self.dynamics_model.linearize(x, u, dt=self.dt)

    def _total_cost(self, x: np.ndarray, u: np.ndarray, xg: np.ndarray) -> float:
        total = 0.0
        for k in range(u.shape[1]):
            dx = x[:, k] - xg
            du = u[:, k] - self.u_ref
            total += float(dx.T @ self.Q @ dx + du.T @ self.R @ du)
        terminal = x[:, -1] - xg
        total += float(terminal.T @ self.S @ terminal)
        return total

    # ------------------------------------------------------------------
    # Config helpers
    # ------------------------------------------------------------------

    @staticmethod
    def _matrix_arg(value, default: np.ndarray) -> np.ndarray:
        if value is None:
            return np.asarray(default, dtype=float)
        arr = np.asarray(value, dtype=float)
        if arr.ndim == 1:
            return np.diag(arr)
        return arr

    @staticmethod
    def _optional_vector(value, length: int) -> Optional[np.ndarray]:
        if value is None:
            return None
        arr = np.asarray(value, dtype=float).reshape(-1)
        if arr.size != length:
            raise ValueError(f"Expected vector of length {length}, got {arr.size}.")
        return arr

    @staticmethod
    def _resolve_enum_indices(enum_obj, names) -> Tuple[int, ...]:
        indices = []
        members = enum_obj.__members__
        for name in names:
            if name not in members:
                raise ValueError(f"{enum_obj.__class__.__name__} has no member {name}.")
            indices.append(int(members[name]))
        return tuple(indices)
