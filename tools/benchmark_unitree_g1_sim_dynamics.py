#!/usr/bin/env python3
"""Compare Unitree G1 joint tracking through MuJoCo or Isaac dynamics.

Run this script once in a MuJoCo-capable environment with ``--backend mujoco``
and once in an Isaac-enabled environment with ``--backend isaac``. Both runs
execute the same bounded velocity-command profile through SPARK's normal agent
interface and write a small JSON report suitable for backend comparison.
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import time
import traceback

import numpy as np


def _command_profile(
    step: int,
    move_steps: int,
    num_control: int,
    profile: str,
) -> np.ndarray:
    command = np.zeros(num_control, dtype=float)
    active_command = np.array([0.45, -0.35, 0.20, -0.45, 0.35, -0.20], dtype=float)
    if step < move_steps:
        command[[0, 3, 5, 7, 10, 12]] = active_command
    elif profile == "reversal" and 2 * move_steps <= step < 3 * move_steps:
        command[[0, 3, 5, 7, 10, 12]] = -active_command
    return command


def _build_agent(args):
    import spark_agent
    import spark_robot
    from spark_robot import get_agent_class_name

    robot_cfg_class = getattr(spark_robot, args.robot_config)
    robot_cfg = robot_cfg_class()
    common = dict(
        dt=args.physics_dt,
        control_decimation=args.control_decimation,
        dynamics_backend="simulator",
        use_sim_dynamics=True,
        enable_viewer=False,
        enable_keyboard_control=False,
        enable_camera=False,
        real_time=False,
    )
    agent_class_name = get_agent_class_name(args.robot_config, backend=args.backend)
    agent_class = getattr(spark_agent, agent_class_name)
    if args.backend == "mujoco":
        return robot_cfg, agent_class(robot_cfg, **common)

    return robot_cfg, agent_class(
        robot_cfg,
        device=args.isaac_device,
        render=False,
        render_on_step=False,
        sim_damping_scale=args.isaac_damping_scale,
        sim_stiffness_scale=args.isaac_stiffness_scale,
        sim_position_error_limit=args.position_error_limit,
        gravity_compensation_scale=args.gravity_compensation_scale,
        coriolis_compensation_scale=args.coriolis_compensation_scale,
        enable_actuator_diagnostics=True,
        **common,
    )


def _run(args) -> dict:
    simulation_app = None
    if args.backend == "isaac":
        from isaacsim import SimulationApp

        simulation_app = SimulationApp(
            {
                "headless": True,
                "hide_ui": True,
                "disable_viewport_updates": True,
                "limit_cpu_threads": args.isaac_cpu_threads,
            }
        )

    agent = None
    try:
        robot_cfg, agent = _build_agent(args)
        agent.reset({})
        feedback = agent.get_feedback()
        initial_position = np.asarray(feedback["dof_pos_fbk"], dtype=float)
        whole_body_target = None
        if "WholeBody" in args.robot_config:
            whole_body_target = (
                np.asarray(feedback.get("body_joint_pos", initial_position[-29:]), dtype=float)
                .reshape(-1)[-29:]
                .copy()
            )
        control_period = args.physics_dt * args.control_decimation
        total_steps = int(round(args.duration / control_period))
        move_steps = int(round(args.move_duration / control_period))

        position_feedback = []
        position_command = []
        velocity_feedback = []
        step_times = []
        requested_efforts = []
        applied_efforts = []
        for step in range(total_steps):
            command = _command_profile(
                step,
                move_steps,
                len(robot_cfg.Control),
                args.profile,
            )
            start = time.perf_counter()
            action_info = (
                {"target_actuated_pos": whole_body_target} if whole_body_target is not None else {}
            )
            agent.step(command, action_info=action_info)
            step_times.append(time.perf_counter() - start)
            feedback = agent.get_feedback()
            position_feedback.append(np.asarray(feedback["dof_pos_fbk"], dtype=float))
            position_command.append(np.asarray(feedback["dof_pos_cmd"], dtype=float))
            velocity_feedback.append(np.asarray(feedback["dof_vel_fbk"], dtype=float))
            if args.backend == "isaac":
                requested_efforts.append(agent._last_requested_effort_tensor.cpu().numpy().copy())
                applied_efforts.append(agent._last_applied_effort_tensor.cpu().numpy().copy())

        q = np.asarray(position_feedback)
        q_cmd = np.asarray(position_command)
        dq = np.asarray(velocity_feedback)
        error = q_cmd - q
        active = np.array([0, 3, 5, 7, 10, 12], dtype=int)
        final_target = q_cmd[-1]
        final_error = final_target - q[-1]
        requested_efforts = np.asarray(requested_efforts)
        applied_efforts = np.asarray(applied_efforts)
        report = {
            "backend": args.backend,
            "robot_config": args.robot_config,
            "physics_dt": args.physics_dt,
            "control_decimation": args.control_decimation,
            "control_period": control_period,
            "duration": args.duration,
            "move_duration": args.move_duration,
            "profile": args.profile,
            "isaac_stiffness_scale": args.isaac_stiffness_scale,
            "isaac_damping_scale": args.isaac_damping_scale,
            "gravity_compensation_scale": args.gravity_compensation_scale,
            "coriolis_compensation_scale": args.coriolis_compensation_scale,
            "num_steps": total_steps,
            "active_dofs": active.tolist(),
            "initial_position": initial_position.tolist(),
            "final_target": final_target.tolist(),
            "final_position": q[-1].tolist(),
            "active_rmse": float(np.sqrt(np.mean(np.square(error[:, active])))),
            "active_peak_abs_error": float(np.max(np.abs(error[:, active]))),
            "active_final_abs_error_mean": float(np.mean(np.abs(final_error[active]))),
            "active_peak_abs_velocity": float(np.max(np.abs(dq[:, active]))),
            "mean_agent_step_ms": float(np.mean(step_times) * 1e3),
            "p95_agent_step_ms": float(np.percentile(step_times, 95) * 1e3),
            "samples": {
                "time": (np.arange(1, total_steps + 1) * control_period).tolist(),
                "position_command": q_cmd[:, active].tolist(),
                "position_feedback": q[:, active].tolist(),
                "velocity_feedback": dq[:, active].tolist(),
            },
        }
        if args.backend == "isaac":
            saturation = np.abs(requested_efforts) > (
                np.asarray(agent._sim_effort_limits)[None, :] + 1e-6
            )
            report["effort_saturation_fraction"] = float(np.mean(saturation))
            report["peak_requested_effort"] = float(np.max(np.abs(requested_efforts)))
            report["peak_applied_effort"] = float(np.max(np.abs(applied_efforts)))
        # Isaac Kit may terminate native services during ``SimulationApp.close``
        # before control returns to Python, so persist the report first.
        if args.output is not None:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        return report
    except Exception:
        # Isaac's native shutdown can terminate logging services before Python
        # prints an exception propagated out of this function. Emit it while
        # Kit is still alive so unsupported config/backend pairs are diagnosable.
        traceback.print_exc()
        raise
    finally:
        if agent is not None and hasattr(agent, "close"):
            agent.close()
        if simulation_app is not None:
            simulation_app.close()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("mujoco", "isaac"), required=True)
    parser.add_argument(
        "--robot-config",
        default="UnitreeG1DualArmDynamic1Config",
        help="Exported spark_robot Unitree G1 config class.",
    )
    parser.add_argument("--output", type=Path, default=None)
    parser.add_argument("--physics-dt", type=float, default=0.005)
    parser.add_argument("--control-decimation", type=int, default=4)
    parser.add_argument("--duration", type=float, default=1.0)
    parser.add_argument("--move-duration", type=float, default=0.4)
    parser.add_argument(
        "--profile",
        choices=("hold", "reversal"),
        default="hold",
        help="Use one motion segment or a motion/stop/reverse/settle transient.",
    )
    parser.add_argument("--position-error-limit", type=float, default=0.35)
    parser.add_argument("--isaac-stiffness-scale", type=float, default=4.0)
    parser.add_argument("--isaac-damping-scale", type=float, default=1.0)
    parser.add_argument("--gravity-compensation-scale", type=float, default=0.0)
    parser.add_argument("--coriolis-compensation-scale", type=float, default=0.0)
    parser.add_argument("--isaac-cpu-threads", type=int, default=4)
    parser.add_argument(
        "--isaac-device",
        default="cpu",
        help="Isaac tensor/PhysX device; CPU is fastest for this single-environment case.",
    )
    args = parser.parse_args()
    report = _run(args)
    encoded = json.dumps(report, indent=2)
    if args.output is not None and not args.output.is_file():
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(encoded + "\n", encoding="utf-8")
    summary = {key: value for key, value in report.items() if key != "samples"}
    print(json.dumps(summary, indent=2))


if __name__ == "__main__":
    main()
