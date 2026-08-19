from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import numpy as np
import casadi as ca

from spark_policy.core.policy import BasePolicy
from spark_robot import RobotKinematics, RobotConfig


# =========================================================
# Robot-specific (best-effort) limits for pseudo v/a bounds
# =========================================================


def _deg2rad(x: float) -> float:
    return x * np.pi / 180.0


def default_joint_speed_limits_rad_s(robot_cfg: RobotConfig, nq: int) -> np.ndarray:
    name = robot_cfg.__class__.__name__.lower()

    if "iiwa" in name and nq >= 7:
        v = np.array([1.48, 1.48, 1.75, 1.31, 2.27, 2.36, 2.36], dtype=float)
        if nq > 7:
            v = np.pad(v, (0, nq - 7), constant_values=v[-1])
        return v[:nq]

    if "gen3" in name and nq >= 7:
        v = np.array([1.39, 1.39, 1.39, 1.39, 1.22, 1.22, 1.22], dtype=float)
        if nq > 7:
            v = np.pad(v, (0, nq - 7), constant_values=v[-1])
        return v[:nq]

    if ("lrmate" in name or "lr_mate" in name or "fanuc" in name) and nq >= 6:
        v_deg = np.array([450.0, 380.0, 520.0, 550.0, 545.0, 1000.0], dtype=float)
        v = _deg2rad(v_deg)
        if nq > 6:
            v = np.pad(v, (0, nq - 6), constant_values=v[-1])
        return v[:nq]

    return np.full(nq, 1.0, dtype=float)


def default_joint_acc_limits_rad_s2(v_max: np.ndarray, dt: float) -> np.ndarray:
    a = 4.0 * v_max / max(dt, 1e-3)
    return np.minimum(a, 50.0)


def sanitize_bounds(
    q_lower: np.ndarray, q_upper: np.ndarray, big: float = 1000.0
) -> Tuple[np.ndarray, np.ndarray]:
    ql = np.array(q_lower, dtype=float).copy()
    qu = np.array(q_upper, dtype=float).copy()

    ql[np.isnan(ql)] = -big
    qu[np.isnan(qu)] = big
    ql[np.isneginf(ql)] = -big
    qu[np.isposinf(qu)] = big

    swap = ql > qu
    if np.any(swap):
        tmp = ql[swap].copy()
        ql[swap] = qu[swap]
        qu[swap] = tmp

    return ql, qu


def compute_safe_nominal(q_lower: np.ndarray, q_upper: np.ndarray) -> np.ndarray:
    q_nom = np.zeros_like(q_lower, dtype=float)
    finite = np.isfinite(q_lower) & np.isfinite(q_upper)
    q_nom[finite] = 0.5 * (q_lower[finite] + q_upper[finite])
    return q_nom


def _as_horizon_frames(value: np.ndarray, *, name: str) -> np.ndarray:
    frames = np.array(value, dtype=float)
    if frames.shape == (4, 4):
        frames = frames.reshape(1, 4, 4)
    if frames.ndim != 3 or frames.shape[1:] != (4, 4):
        raise ValueError(
            f"{name} must have shape (4, 4), (1, 4, 4), or (H, 4, 4); got {frames.shape}."
        )
    return frames


def _broadcast_horizon_frames(frames: np.ndarray, H: int, *, name: str) -> np.ndarray:
    frames_arr = _as_horizon_frames(frames, name=name)
    if frames_arr.shape[0] == H:
        return frames_arr
    if frames_arr.shape[0] == 1:
        return np.repeat(frames_arr, H, axis=0)
    raise ValueError(
        f"{name} horizon mismatch: expected 1 or {H} frames, got {frames_arr.shape[0]}."
    )


def _kinematics_target_count(kin: RobotKinematics) -> int:
    pos_n_in = int(kin.translational_error.n_in())
    rot_n_in = int(kin.rotational_error.n_in())
    if pos_n_in != rot_n_in:
        raise RuntimeError(
            f"Kinematics target signatures do not match: translational_error expects {pos_n_in} inputs "
            f"but rotational_error expects {rot_n_in}."
        )

    target_count = pos_n_in - 1
    if target_count < 1:
        raise RuntimeError(
            "Kinematics error functions must accept q plus at least one SE(3) target."
        )
    if target_count > 3:
        raise RuntimeError(
            f"Unsupported kinematics target signature with {target_count} SE(3) targets."
        )

    return target_count


# =========================================================
# Cached Opti horizon solver (but policy will not cache solutions)
# =========================================================


class _HorizonOptiSolver:
    """
    Whole-horizon joint trajectory optimizer using CasADi Opti + IPOPT.

    Determinism notes:
      - IPOPT itself is deterministic given identical problem + identical initial guess.
      - This solver object is cached ONLY to avoid rebuild overhead.
      - The policy will still solve fresh every act() call (no Q reuse).
    """

    def __init__(
        self,
        kin: RobotKinematics,
        robot_cfg: RobotConfig,
        H: int,
        dt: float,
        q_lower: np.ndarray,
        q_upper: np.ndarray,
        q_nom: np.ndarray,
        *,
        w_pos: float,
        w_ori: float,
        lam_v: float,
        lam_a: float,
        lam_nom: float,
        lam_q0: float,
        ipopt_max_iter: int,
        ipopt_tol: float,
        ipopt_print_level: int,
        ipopt_linear_solver: str,
    ) -> None:
        if not hasattr(kin, "translational_error") or not hasattr(kin, "rotational_error"):
            raise RuntimeError(
                "RobotKinematics must expose translational_error and rotational_error CasADi Functions."
            )

        if hasattr(kin, "reduced_fixed_base_model"):
            nq = int(kin.reduced_fixed_base_model.nq)
        elif hasattr(kin, "model"):
            nq = int(kin.model.nq)
        else:
            raise RuntimeError("RobotKinematics must expose reduced_fixed_base_model or model.")

        self.kin = kin
        self.robot_cfg = robot_cfg
        self.H = int(H)
        self.dt = float(dt)
        self.nq = nq

        self.q_lower = np.array(q_lower, dtype=float).reshape(self.nq, 1)
        self.q_upper = np.array(q_upper, dtype=float).reshape(self.nq, 1)
        self.q_nom = np.array(q_nom, dtype=float).reshape(self.nq, 1)

        v_max = default_joint_speed_limits_rad_s(robot_cfg, self.nq)
        a_max = default_joint_acc_limits_rad_s2(v_max, dt)
        self.v_max = v_max.reshape(self.nq, 1)
        self.a_max = a_max.reshape(self.nq, 1)
        self.target_count = _kinematics_target_count(kin)

        opti = ca.Opti()

        # Decision variables: (nq, H)
        Q = opti.variable(self.nq, self.H)

        # Parameters: targets + last q
        T_list = [[opti.parameter(4, 4) for _ in range(self.target_count)] for _ in range(self.H)]
        q_last = opti.parameter(self.nq, 1)

        # bounds
        opti.subject_to(opti.bounded(self.q_lower, Q, self.q_upper))

        # NOTE:
        # Velocity / acceleration regularization remains in the objective via lam_v / lam_a,
        # but we intentionally do not impose hard dq / ddq constraints here.

        # objective
        J = 0
        for k in range(self.H):
            qk = Q[:, k]
            pos_err = kin.translational_error(qk, *T_list[k])
            rot_err = kin.rotational_error(qk, *T_list[k])
            J += w_pos * ca.sumsqr(pos_err) + w_ori * ca.sumsqr(rot_err)
            J += lam_nom * ca.sumsqr(qk - self.q_nom)

        # smoothness
        if self.H >= 2:
            for k in range(self.H - 1):
                J += lam_v * ca.sumsqr(Q[:, k + 1] - Q[:, k])
        if self.H >= 3:
            for k in range(self.H - 2):
                J += lam_a * ca.sumsqr(Q[:, k + 2] - 2 * Q[:, k + 1] + Q[:, k])

        # soft q0 anchor
        J += lam_q0 * ca.sumsqr(Q[:, 0] - q_last)

        opti.minimize(J)

        p_opts = {
            "expand": True,
            "print_time": False,  # suppress CasADi solver timing output
        }
        s_opts = {
            "max_iter": int(ipopt_max_iter),
            "tol": float(ipopt_tol),
            "print_level": 0,  # suppress IPOPT iteration logs
            "sb": "yes",  # silent banner
            "linear_solver": str(ipopt_linear_solver),
        }
        opti.solver("ipopt", p_opts, s_opts)

        self.opti = opti
        self.Q = Q
        self.T_list = T_list
        self.q_last = q_last

    def solve(
        self, T_targets: List[np.ndarray], Q_init: np.ndarray, q_last: np.ndarray
    ) -> Tuple[np.ndarray, Dict]:
        if len(T_targets) != self.target_count:
            raise ValueError(f"Expected {self.target_count} target streams, got {len(T_targets)}.")

        target_streams = [np.array(target, dtype=float) for target in T_targets]
        for idx, target in enumerate(target_streams):
            if target.shape != (self.H, 4, 4):
                raise ValueError(
                    f"Target stream {idx} must have shape {(self.H, 4, 4)}, got {target.shape}."
                )

        for k in range(self.H):
            for target_idx, target in enumerate(target_streams):
                self.opti.set_value(self.T_list[k][target_idx], target[k])
        self.opti.set_value(self.q_last, q_last.reshape(self.nq, 1))

        # Deterministic initial guess
        self.opti.set_initial(self.Q, Q_init)

        info: Dict = {}

        try:
            sol = self.opti.solve()
            Q_val = sol.value(self.Q)
            info["success"] = True
        except Exception as e:
            # Deterministic fallback: use debug values
            Q_val = self.opti.debug.value(self.Q)
            info["success"] = False
            info["error"] = str(e)

        return Q_val, info


# =========================================================
# Policy parameters
# =========================================================


@dataclass
class TrajOptParams:
    dt: float = 0.1

    # weights
    w_pos: float = 10.0
    w_ori: float = 20.0
    lam_v: float = 1e-2
    lam_a: float = 1e-1
    lam_nom: float = 1e-3
    lam_q0: float = 1e-2

    # IPOPT
    ipopt_max_iter: int = 80
    ipopt_tol: float = 1e-4
    ipopt_print_level: int = 0
    ipopt_linear_solver: str = "mumps"

    # determinism: make policy a pure function of (T_world, fbk, base_frame)
    # Always solve, no warmstart, always execute first waypoint
    deterministic: bool = True
    execute_index: int = 0  # which waypoint to execute when deterministic

    # bounds sanitize
    big_bound: float = 1000.0


# =========================================================
# SPARK TrajOptPolicy (deterministic)
# =========================================================


class TrajOptPolicy(BasePolicy):
    """
    Deterministic whole-horizon TO policy (no collision):
      - Always solve the full horizon on every act() call
      - No caching of targets
      - No warm-start from previous solution
      - No trajectory counter advancement
      - Execute a fixed waypoint index (default 0)
    """

    def __init__(
        self,
        robot_cfg: RobotConfig,
        robot_kinematics: RobotKinematics,
        params: Optional[TrajOptParams] = None,
    ) -> None:
        super().__init__(robot_cfg, robot_kinematics)
        self.params = params or TrajOptParams()

        if hasattr(robot_kinematics, "reduced_fixed_base_model"):
            model = robot_kinematics.reduced_fixed_base_model
        elif hasattr(robot_kinematics, "model"):
            model = robot_kinematics.model
        else:
            raise RuntimeError("RobotKinematics must expose reduced_fixed_base_model or model.")

        self.nq = int(model.nq)

        q_lower_raw = np.array(model.lowerPositionLimit, dtype=float)[: self.nq]
        q_upper_raw = np.array(model.upperPositionLimit, dtype=float)[: self.nq]

        # if bounds are infinite or NaN, sanitize them to large finite values and ensure q_lower <= q_upper

        self.q_nom = compute_safe_nominal(q_lower_raw, q_upper_raw)
        self.q_lower, self.q_upper = sanitize_bounds(
            q_lower_raw, q_upper_raw, big=self.params.big_bound
        )

        self._solver_cache: Dict[Tuple[int, float], _HorizonOptiSolver] = {}
        self._target_count = _kinematics_target_count(robot_kinematics)

        # NOTE: these are kept for API compatibility but NOT used for determinism
        self._prev_Q: Optional[np.ndarray] = None
        self._cached_targets_hash: Optional[int] = None
        self.traj_cnt = 0
        self._last_to_info: Dict[str, object] = {}

    # ---------------------------
    # Low-level tracking
    # ---------------------------

    def tracking_pos_with_vel(
        self, dof_pos_target, dof_vel_target, dof_pos_current, dof_vel_current
    ):
        K_p = 1.0 * np.ones(len(self.robot_cfg.Control))
        K_d = 0.0 * np.ones(len(self.robot_cfg.Control))
        dof_vel_nominal = K_p * (dof_pos_target - dof_pos_current) + K_d * (
            dof_vel_target - dof_vel_current
        )
        drift = self.robot_cfg.dynamics_f(dof_pos_current).reshape(-1)
        nominal_control = np.linalg.pinv(self.robot_cfg.dynamics_g(dof_pos_current)) @ (
            dof_vel_nominal - drift
        )
        return nominal_control

    def tracking_pos_with_acc(
        self, dof_pos_target, dof_vel_target, dof_pos_current, dof_vel_current
    ):
        K_p = 1.0 * np.ones(len(self.robot_cfg.Control))
        K_d = 0.0 * np.ones(len(self.robot_cfg.Control))
        nominal_dof_vel = K_p * (dof_pos_target - dof_pos_current) + K_d * dof_vel_current
        nominal_dof_acc = nominal_dof_vel - dof_vel_current
        nominal_control = np.linalg.pinv(
            self.robot_cfg.dynamics_g(np.concatenate((dof_pos_current, dof_vel_current)))
        ) @ np.concatenate((nominal_dof_vel, nominal_dof_acc))
        return nominal_control

    # ---------------------------
    # Utilities
    # ---------------------------

    def _ensure_Q_shape(self, Q_val: np.ndarray, H: int) -> np.ndarray:
        """
        Ensure Q is (nq, H). Accepts:
          - (nq,H), (H,nq)
          - (nq,1) or (1,nq) -> tiled
          - flat (nq*H,)
          - flat (nq,) -> tiled
        """
        Q_arr = np.array(Q_val, dtype=float)

        if Q_arr.ndim == 2:
            if Q_arr.shape == (self.nq, H):
                return Q_arr
            if Q_arr.shape == (H, self.nq):
                return Q_arr.T
            if Q_arr.shape == (self.nq, 1):
                return np.tile(Q_arr, (1, H))
            if Q_arr.shape == (1, self.nq):
                return np.tile(Q_arr.T, (1, H))

            flat = Q_arr.reshape((-1,), order="F")
            if flat.size == self.nq * H:
                return flat.reshape((self.nq, H), order="F")
            if flat.size == self.nq:
                return np.tile(flat.reshape(self.nq, 1), (1, H))

        if Q_arr.ndim == 1:
            if Q_arr.size == self.nq * H:
                return Q_arr.reshape((self.nq, H), order="F")
            if Q_arr.size == self.nq:
                return np.tile(Q_arr.reshape(self.nq, 1), (1, H))

        raise ValueError(
            f"Unexpected Q shape: ndim={Q_arr.ndim}, shape={Q_arr.shape}, size={Q_arr.size}"
        )

    def _get_solver(self, H: int, dt: float) -> _HorizonOptiSolver:
        key = (int(H), float(dt))
        if key in self._solver_cache:
            return self._solver_cache[key]

        solver = _HorizonOptiSolver(
            kin=self.robot_kinematics,
            robot_cfg=self.robot_cfg,
            H=H,
            dt=dt,
            q_lower=self.q_lower,
            q_upper=self.q_upper,
            q_nom=self.q_nom,
            w_pos=self.params.w_pos,
            w_ori=self.params.w_ori,
            lam_v=self.params.lam_v,
            lam_a=self.params.lam_a,
            lam_nom=self.params.lam_nom,
            lam_q0=self.params.lam_q0,
            ipopt_max_iter=self.params.ipopt_max_iter,
            ipopt_tol=self.params.ipopt_tol,
            ipopt_print_level=self.params.ipopt_print_level,
            ipopt_linear_solver=self.params.ipopt_linear_solver,
        )
        self._solver_cache[key] = solver
        return solver

    def _deterministic_Q_init(self, q_seed: np.ndarray, H: int) -> np.ndarray:
        """Deterministic initial guess: tile q_seed."""
        return np.tile(q_seed[: self.nq].reshape(self.nq, 1), (1, H))

    def _default_dof_pos(self) -> np.ndarray:
        return np.array([self.robot_cfg.DefaultDoFVal[d] for d in self.robot_cfg.DoFs], dtype=float)

    def _current_frame_sequence(self, q_seed: np.ndarray, frame_name: str, H: int) -> np.ndarray:
        frame_enum = getattr(self.robot_cfg.Frames, frame_name, None)
        if frame_enum is None:
            raise RuntimeError(
                f"Dual-arm trajectory optimization needs {frame_name}, but {self.robot_cfg.__class__.__name__}.Frames "
                f"does not define it."
            )
        frames = np.array(self.robot_kinematics.forward_kinematics(q_seed), dtype=float)
        return np.repeat(frames[int(frame_enum)][None, :, :], H, axis=0)

    def _infer_goal_horizon(self, goal_teleop: Dict[str, np.ndarray]) -> int:
        horizons: List[int] = []
        for key in ("right", "left", "base"):
            if key in goal_teleop:
                horizons.append(
                    _as_horizon_frames(goal_teleop[key], name=f'goal_teleop["{key}"]').shape[0]
                )
        if not horizons:
            raise KeyError('goal_teleop must include at least one of "right", "left", or "base".')
        return max(horizons)

    def _build_target_streams(
        self,
        goal_teleop: Dict[str, np.ndarray],
        robot_base_frame: np.ndarray,
        q_seed: np.ndarray,
    ) -> Tuple[List[np.ndarray], Dict[str, str]]:
        H = self._infer_goal_horizon(goal_teleop)
        base_inv = np.linalg.inv(robot_base_frame)
        target_streams: List[np.ndarray] = []
        source_info: Dict[str, str] = {}

        right_world = _broadcast_horizon_frames(
            goal_teleop["right"], H=H, name='goal_teleop["right"]'
        )
        right_base = base_inv @ right_world

        if self._target_count == 1:
            target_streams.append(right_base)
            source_info["right"] = "task"
            return target_streams, source_info

        if "left" in goal_teleop:
            left_world = _broadcast_horizon_frames(
                goal_teleop["left"], H=H, name='goal_teleop["left"]'
            )
            left_base = base_inv @ left_world
            source_info["left"] = "task"
        else:
            left_base = self._current_frame_sequence(q_seed=q_seed, frame_name="L_ee", H=H)
            source_info["left"] = "current_fk"

        target_streams.extend([left_base, right_base])
        source_info["right"] = "task"

        if self._target_count == 3:
            if "base" in goal_teleop:
                base_world = _broadcast_horizon_frames(
                    goal_teleop["base"], H=H, name='goal_teleop["base"]'
                )
                platform_base = base_inv @ base_world
                source_info["base"] = "task"
            else:
                platform_base = np.repeat(np.eye(4, dtype=float)[None, :, :], H, axis=0)
                source_info["base"] = "identity"
            target_streams.append(platform_base)

        return target_streams, source_info

    def _build_solution_debug_frames(self, Q_sol: np.ndarray) -> Dict[str, np.ndarray]:
        right_sol: List[np.ndarray] = []
        left_sol: List[np.ndarray] = []
        has_left = getattr(self.robot_cfg.Frames, "L_ee", None) is not None

        for q in Q_sol.T:
            T_fk = np.array(self.robot_kinematics.forward_kinematics(q), dtype=float)
            right_sol.append(T_fk[int(self.robot_cfg.Frames.R_ee)])
            if has_left:
                left_sol.append(T_fk[int(self.robot_cfg.Frames.L_ee)])

        debug_frames = {
            "right": np.array(right_sol, dtype=float),
        }
        if has_left:
            debug_frames["left"] = np.array(left_sol, dtype=float)
        return debug_frames

    # ---------------------------
    # act()
    # ---------------------------

    def act(self, agent_feedback: dict, task_info: dict):
        info: Dict = {}
        dof_pos_cmd = agent_feedback["dof_pos_cmd"]
        dof_vel_cmd = agent_feedback["dof_vel_cmd"]
        dof_pos_fbk = np.array(
            agent_feedback.get("dof_pos_fbk", self._default_dof_pos()), dtype=float
        )
        dof_vel_fbk = agent_feedback["dof_vel_fbk"]
        robot_base_frame = np.array(agent_feedback["robot_base_frame"], dtype=float)
        goal_teleop = task_info["goal_teleop"]

        q_last = dof_pos_fbk[: self.nq].copy() if dof_pos_fbk is not None else self.q_nom.copy()
        target_streams, target_sources = self._build_target_streams(
            goal_teleop=goal_teleop,
            robot_base_frame=robot_base_frame,
            q_seed=q_last,
        )
        H = int(target_streams[0].shape[0])
        # dt
        dt = float(task_info.get("dt", self.params.dt))

        # Always solve (deterministic)
        solver = self._get_solver(H=H, dt=dt)

        Q_init = self._deterministic_Q_init(q_seed=q_last, H=H)

        Q_raw, to_info = solver.solve(T_targets=target_streams, Q_init=Q_init, q_last=q_last)
        Q_sol = self._ensure_Q_shape(Q_raw, H)
        debug_frames = self._build_solution_debug_frames(Q_sol)
        # Store for debugging
        self._prev_Q = Q_sol.copy()
        self._last_to_info = dict(to_info)
        # Choose waypoint index deterministically
        idx = int(np.clip(self.params.execute_index, 0, H - 1))
        self.params.execute_index += 1
        if self.params.execute_index >= H:
            self.params.execute_index = 0
        # idx = 0
        q_target = Q_sol[:, idx].copy()

        # Build full dof target (pad if dof vector includes non-arm dofs)
        dof_pos_target = np.zeros_like(dof_pos_fbk)
        dof_pos_target[: self.nq] = q_target
        info["dof_pos_target"] = dof_pos_target.copy()

        # Convert to control
        cfg_name = self.robot_cfg.__class__.__name__.lower()
        if "dynamic1" in cfg_name:
            dof_vel_target = np.zeros_like(dof_vel_fbk)
            control = self.tracking_pos_with_vel(
                dof_pos_target, dof_vel_target, dof_pos_fbk, dof_vel_fbk
            )
        elif "dynamic2" in cfg_name:
            dof_vel_target = np.zeros_like(dof_vel_fbk)
            control = self.tracking_pos_with_acc(
                dof_pos_target, dof_vel_target, dof_pos_cmd, dof_vel_cmd
            )
        else:
            dof_vel_target = np.zeros_like(dof_vel_fbk)
            control = self.tracking_pos_with_vel(
                dof_pos_target, dof_vel_target, dof_pos_fbk, dof_vel_fbk
            )

        # Clip control
        for control_id in self.robot_cfg.Control:
            control[control_id] = np.clip(
                control[control_id],
                -self.robot_cfg.ControlLimit[control_id],
                self.robot_cfg.ControlLimit[control_id],
            )

        # Info
        info["to_success"] = bool(to_info.get("success", True))
        info["to_error"] = str(to_info.get("error", ""))
        info["robot_name"] = self.robot_cfg.__class__.__name__

        # Determinism debug
        info["q_last_used"] = q_last.copy()
        info["robot_base_frame_used"] = robot_base_frame.copy()

        # Keep Q for dataset/debug
        info["Q_sol"] = Q_sol.copy()
        info["T_sol"] = debug_frames["right"].copy()
        if "left" in debug_frames:
            info["T_sol_left"] = debug_frames["left"].copy()
        info["target_sources"] = dict(target_sources)
        if "left_gripper_goal" in goal_teleop:
            info["left_gripper_goal"] = goal_teleop["left_gripper_goal"]
        if "right_gripper_goal" in goal_teleop:
            info["right_gripper_goal"] = goal_teleop["right_gripper_goal"]

        return control, info
