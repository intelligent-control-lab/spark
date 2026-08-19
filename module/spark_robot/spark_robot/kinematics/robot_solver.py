"""Helpers for connecting the common IK solver to SPARK robot classes."""

from __future__ import annotations

import numpy as np
import pinocchio as pin

from .inverse_kinematics import BoundedLeastSquaresIK, IKProblem, IKSolverConfig, IKTarget


def configure_robot_ik(
    robot,
    *,
    position_weight: float = 50.0,
    orientation_weight: float = 20.0,
    regularization_weight: float = 0.0,
    smoothness_weight: float = 0.1,
    max_evaluations: int = 40,
    regularization_indices: tuple[int, ...] | None = None,
    fixed_indices: tuple[int, ...] = (),
    position_tolerance: float = 2e-3,
    orientation_tolerance: float = 2e-2,
    use_robot_config_position_limits: bool = False,
):
    """Configure the numeric solver while retaining robot-specific objective weights."""
    if use_robot_config_position_limits:
        _apply_robot_config_position_limits(robot)
    robot._ik_position_weight = position_weight
    robot._ik_orientation_weight = orientation_weight
    robot._ik_fixed_indices = fixed_indices
    robot.ik_solver = BoundedLeastSquaresIK(
        robot.reduced_fixed_base_model,
        IKSolverConfig(
            regularization_weight=regularization_weight,
            smoothness_weight=smoothness_weight,
            max_evaluations=max_evaluations,
            regularization_indices=regularization_indices,
            position_tolerance=position_tolerance,
            orientation_tolerance=orientation_tolerance,
        ),
    )


def _apply_robot_config_position_limits(robot):
    """Apply name-aligned hardware limits to a reduced fixed-base IK model."""

    model = robot.reduced_fixed_base_model
    robot_cfg = robot.robot_cfg
    dofs = tuple(robot_cfg.DoFs)
    if model.nq != len(dofs):
        raise ValueError(
            "Robot-config position limits require one IK coordinate per configured DoF"
        )

    motor_members = robot_cfg.RealMotors.__members__
    limits = robot_cfg.RealMotorPosLimit
    for dof in dofs:
        motor = motor_members.get(dof.name)
        if motor is None or motor not in limits:
            raise ValueError(f"No hardware position limit is configured for DoF {dof.name!r}")
        lower, upper = limits[motor]
        model.lowerPositionLimit[int(dof)] = float(lower)
        model.upperPositionLimit[int(dof)] = float(upper)


def solve_robot_ik(
    robot,
    frame_targets,
    current_q=None,
    target_options=None,
    fixed_indices=None,
):
    """Solve frame targets and construct the legacy-compatible result dictionary."""
    if current_q is not None:
        robot.init_data = np.asarray(current_q, dtype=float).copy()
    options = target_options or [{} for _ in frame_targets]
    targets = [
        IKTarget(
            frame_id=frame_id,
            pose=pose,
            position_weight=option.get("position_weight", robot._ik_position_weight),
            orientation_weight=option.get("orientation_weight", robot._ik_orientation_weight),
            position_mask=option.get("position_mask", (True, True, True)),
            orientation_mask=option.get("orientation_mask", (True, True, True)),
        )
        for (frame_id, pose), option in zip(frame_targets, options)
    ]
    configured_fixed_indices = tuple(getattr(robot, "_ik_fixed_indices", ()))
    if fixed_indices is None:
        resolved_fixed_indices = configured_fixed_indices
    else:
        resolved_fixed_indices = tuple(
            dict.fromkeys(configured_fixed_indices + tuple(int(i) for i in fixed_indices))
        )
    result = robot.ik_solver.solve(
        IKProblem(
            targets=targets,
            initial_configuration=robot.init_data,
            previous_configuration=robot.init_data,
            fixed_configuration_indices=resolved_fixed_indices,
        )
    )
    robot.init_data = result.configuration.copy()
    velocity = np.zeros(robot.reduced_fixed_base_model.nv)
    torque = pin.rnea(
        robot.reduced_fixed_base_model,
        robot.reduced_fixed_base_data,
        result.configuration,
        velocity,
        np.zeros(robot.reduced_fixed_base_model.nv),
    )
    if torque.size < robot.num_dof:
        torque = np.pad(torque, (0, robot.num_dof - torque.size))
    info = {
        "sol_tauff": torque,
        "success": result.success,
        "ik_result": result,
        "ik_backend": result.backend,
    }
    return result.configuration.copy(), info
