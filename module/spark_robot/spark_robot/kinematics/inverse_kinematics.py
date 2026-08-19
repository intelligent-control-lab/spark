"""Common inverse-kinematics API and the default CPU implementation.

The CPU solver deliberately uses only numeric Pinocchio and SciPy.  It therefore
works with current PyPI Pinocchio wheels and does not require ``pinocchio.casadi``
or IPOPT.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from time import perf_counter
from typing import Sequence

import numpy as np
import pinocchio as pin
from scipy.optimize import least_squares


@dataclass(frozen=True)
class IKTarget:
    """A desired pose for one Pinocchio frame."""

    frame_id: int
    pose: np.ndarray
    position_weight: float | np.ndarray = 50.0
    orientation_weight: float | np.ndarray = 20.0
    position_mask: tuple[bool, bool, bool] = (True, True, True)
    orientation_mask: tuple[bool, bool, bool] = (True, True, True)


@dataclass(frozen=True)
class IKSolverConfig:
    """Objective and convergence settings shared by robot-specific IK calls."""

    regularization_weight: float = 0.0
    smoothness_weight: float = 0.1
    regularization_indices: tuple[int, ...] | None = None
    max_evaluations: int = 40
    function_tolerance: float = 1e-8
    step_tolerance: float = 1e-8
    gradient_tolerance: float = 1e-8
    position_tolerance: float = 2e-3
    orientation_tolerance: float = 2e-2


@dataclass(frozen=True)
class IKProblem:
    targets: Sequence[IKTarget]
    initial_configuration: np.ndarray
    previous_configuration: np.ndarray | None = None
    nominal_configuration: np.ndarray | None = None
    fixed_configuration_indices: tuple[int, ...] = ()


@dataclass(frozen=True)
class IKResult:
    configuration: np.ndarray
    success: bool
    position_error: float
    orientation_error: float
    objective: float
    iterations: int
    solve_time: float
    message: str = ""
    backend: str = "cpu_bounded_least_squares"
    diagnostics: dict = field(default_factory=dict)


def _weight_vector(value: float | np.ndarray) -> np.ndarray:
    weights = np.broadcast_to(np.asarray(value, dtype=float), (3,))
    if np.any(weights < 0.0):
        raise ValueError("IK weights must be non-negative")
    return np.sqrt(weights)


class BoundedLeastSquaresIK:
    """Bound-constrained nonlinear least-squares IK for one or more frames."""

    def __init__(self, model: pin.Model, config: IKSolverConfig | None = None):
        self.model = model
        self.data = model.createData()
        self.config = config or IKSolverConfig()

    def _pose_errors(self, q: np.ndarray, targets: Sequence[IKTarget]):
        pin.framesForwardKinematics(self.model, self.data, q)
        position_errors = []
        orientation_errors = []
        for target in targets:
            current = self.data.oMf[int(target.frame_id)]
            pose = np.asarray(target.pose, dtype=float)
            position_errors.append(current.translation - pose[:3, 3])
            orientation_errors.append(pin.log3(current.rotation @ pose[:3, :3].T))
        return position_errors, orientation_errors

    def solve(self, problem: IKProblem) -> IKResult:
        cfg = self.config
        q0 = np.asarray(problem.initial_configuration, dtype=float).copy()
        if q0.shape != (self.model.nq,):
            raise ValueError(
                f"Expected initial configuration shape {(self.model.nq,)}, got {q0.shape}"
            )
        previous = (
            q0
            if problem.previous_configuration is None
            else np.asarray(problem.previous_configuration, dtype=float)
        )
        nominal = (
            np.zeros_like(q0)
            if problem.nominal_configuration is None
            else np.asarray(problem.nominal_configuration, dtype=float)
        )
        regularization_indices = (
            np.arange(self.model.nq)
            if cfg.regularization_indices is None
            else np.asarray(cfg.regularization_indices, dtype=int)
        )

        lower = np.asarray(self.model.lowerPositionLimit, dtype=float).copy()
        upper = np.asarray(self.model.upperPositionLimit, dtype=float).copy()
        lower[~np.isfinite(lower)] = -1e6
        upper[~np.isfinite(upper)] = 1e6
        q0 = np.clip(q0, lower + 1e-10, upper - 1e-10)
        fixed_indices = np.asarray(problem.fixed_configuration_indices, dtype=int)

        def residual(q):
            position, orientation = self._pose_errors(q, problem.targets)
            values = []
            for target, p_error, r_error in zip(problem.targets, position, orientation):
                position_mask = np.asarray(target.position_mask, dtype=bool)
                orientation_mask = np.asarray(target.orientation_mask, dtype=bool)
                values.extend((_weight_vector(target.position_weight) * p_error)[position_mask])
                values.extend(
                    (_weight_vector(target.orientation_weight) * r_error)[orientation_mask]
                )
            if cfg.regularization_weight > 0.0 and regularization_indices.size:
                values.extend(
                    np.sqrt(cfg.regularization_weight)
                    * (q[regularization_indices] - nominal[regularization_indices])
                )
            if cfg.smoothness_weight > 0.0:
                values.extend(np.sqrt(cfg.smoothness_weight) * (q - previous))
            if fixed_indices.size:
                values.extend(1e6 * (q[fixed_indices] - previous[fixed_indices]))
            return np.asarray(values)

        def jacobian(q):
            # SciPy otherwise finite-differences every joint, even though
            # Pinocchio already computes the exact frame Jacobians in one
            # forward pass.  LOCAL_WORLD_ALIGNED uses [linear, angular] rows.
            # For r = log(R_current R_target.T), Jlog3(R_error.T) maps the
            # world angular frame Jacobian to the derivative of that residual.
            pin.computeJointJacobians(self.model, self.data, q)
            pin.updateFramePlacements(self.model, self.data)
            rows = []
            for target in problem.targets:
                frame_id = int(target.frame_id)
                current = self.data.oMf[frame_id]
                frame_jacobian = pin.getFrameJacobian(
                    self.model,
                    self.data,
                    frame_id,
                    pin.LOCAL_WORLD_ALIGNED,
                )
                position_mask = np.asarray(target.position_mask, dtype=bool)
                orientation_mask = np.asarray(target.orientation_mask, dtype=bool)
                position_jacobian = (
                    _weight_vector(target.position_weight)[:, None] * frame_jacobian[:3]
                )
                rotation_error = current.rotation @ np.asarray(target.pose, dtype=float)[:3, :3].T
                orientation_jacobian = (
                    _weight_vector(target.orientation_weight)[:, None]
                    * pin.Jlog3(rotation_error.T)
                    @ frame_jacobian[3:]
                )
                rows.extend(position_jacobian[position_mask])
                rows.extend(orientation_jacobian[orientation_mask])
            if cfg.regularization_weight > 0.0 and regularization_indices.size:
                regularization_jacobian = np.zeros(
                    (regularization_indices.size, self.model.nv), dtype=float
                )
                regularization_jacobian[
                    np.arange(regularization_indices.size), regularization_indices
                ] = np.sqrt(cfg.regularization_weight)
                rows.extend(regularization_jacobian)
            if cfg.smoothness_weight > 0.0:
                rows.extend(np.sqrt(cfg.smoothness_weight) * np.eye(self.model.nv))
            if fixed_indices.size:
                fixed_jacobian = np.zeros((fixed_indices.size, self.model.nv), dtype=float)
                fixed_jacobian[np.arange(fixed_indices.size), fixed_indices] = 1e6
                rows.extend(fixed_jacobian)
            return np.asarray(rows, dtype=float)

        started = perf_counter()
        solution = least_squares(
            residual,
            q0,
            bounds=(lower, upper),
            method="trf",
            jac=jacobian if self.model.nq == self.model.nv else "2-point",
            max_nfev=cfg.max_evaluations,
            ftol=cfg.function_tolerance,
            xtol=cfg.step_tolerance,
            gtol=cfg.gradient_tolerance,
        )
        elapsed = perf_counter() - started
        position, orientation = self._pose_errors(solution.x, problem.targets)
        max_position = max(
            (
                np.linalg.norm(error[np.asarray(target.position_mask, dtype=bool)])
                for target, error in zip(problem.targets, position)
            ),
            default=0.0,
        )
        max_orientation = max(
            (
                np.linalg.norm(error[np.asarray(target.orientation_mask, dtype=bool)])
                for target, error in zip(problem.targets, orientation)
            ),
            default=0.0,
        )
        converged = bool(
            solution.success
            and max_position <= cfg.position_tolerance
            and max_orientation <= cfg.orientation_tolerance
        )
        return IKResult(
            configuration=solution.x.copy(),
            success=converged,
            position_error=float(max_position),
            orientation_error=float(max_orientation),
            objective=float(2.0 * solution.cost),
            iterations=int(solution.nfev),
            solve_time=elapsed,
            message=str(solution.message),
            diagnostics={
                "optimizer_success": bool(solution.success),
                "status": int(solution.status),
            },
        )
