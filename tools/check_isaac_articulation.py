"""Run the release hold gate for one declarative Isaac articulation."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import subprocess
import sys
import traceback


def _parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--robot-config",
        nargs="+",
        default="KinovaGen3SingleArmDynamic1Config",
        help="One or more exported RobotConfig classes using ConfiguredIsaacAgent",
    )
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--num-envs", type=int, default=1)
    parser.add_argument("--dynamics-backend", choices=("simulator", "model"), default="simulator")
    parser.add_argument("--hold-seconds", type=float, default=None)
    parser.add_argument(
        "--traceback",
        action="store_true",
        help="Print a full traceback when a configuration cannot be checked",
    )
    parser.add_argument(
        "--print-layout",
        action="store_true",
        help="Print the first cloned USD and physics root positions.",
    )
    return parser.parse_args()


def _check_config(args, config_name: str) -> bool:
    agent = None
    try:
        import numpy as np
        import spark_agent
        import spark_robot
        from spark_agent import ConfiguredIsaacAgent

        config_type = getattr(spark_robot, config_name, None)
        if config_type is None:
            raise ValueError(f"Unknown robot configuration: {config_name}")
        config = config_type()
        agent_name = config.agent_class_name("isaac")
        agent_type = getattr(spark_agent, agent_name, None)
        if agent_type is None or not issubclass(agent_type, ConfiguredIsaacAgent):
            raise ValueError(
                f"{config_name} does not use the declarative single-environment Isaac adapter"
            )

        spec = config.simulator_dynamics
        hold_seconds = spec.hold_duration if args.hold_seconds is None else args.hold_seconds
        if hold_seconds <= 0.0:
            raise ValueError("hold-seconds must be positive")
        if args.num_envs < 1:
            raise ValueError("num-envs must be positive")
        # Exercise the robot-specific adapter selected by RobotConfig. These
        # lightweight subclasses own any plant-specific initialization that a
        # direct ConfiguredIsaacAgent construction would silently bypass.
        agent = agent_type(
            config,
            device=args.device,
            render=False,
            num_envs=args.num_envs,
            dynamics_backend=args.dynamics_backend,
        )
        if args.print_layout and hasattr(agent, "env_positions"):
            import omni.usd
            from pxr import Usd, UsdGeom

            count = min(args.num_envs, 8)
            declared = agent.env_positions[:count].detach().cpu().numpy()
            stage = omni.usd.get_context().get_stage()
            authored = []
            for index in range(count):
                prim = stage.GetPrimAtPath(f"/World/envs/env_{index}")
                matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
                    Usd.TimeCode.Default()
                )
                authored.append(tuple(float(value) for value in matrix.ExtractTranslation()))
            physical = agent.articulation.get_world_poses()[0][:count].detach().cpu().numpy()
            print(
                f"{config_name}: clone layout declared={declared.tolist()} "
                f"USD={authored} physics={physical.tolist()}",
                flush=True,
            )
        initial_value = agent.get_feedback()["dof_pos_fbk"]
        initial = (
            initial_value.detach().clone()
            if hasattr(initial_value, "detach")
            else initial_value.copy()
        )
        steps = math.ceil(hold_seconds / spec.control_period)
        zero_control = np.zeros(
            (args.num_envs, len(config.Control)) if args.num_envs > 1 else len(config.Control),
            dtype=float,
        )
        for _ in range(steps):
            agent.step(zero_control)
        feedback = agent.get_feedback()
        position = feedback["dof_pos_fbk"]
        velocity = feedback["dof_vel_fbk"]
        if hasattr(position, "detach"):
            absolute_drift = (position - initial).abs()
            absolute_velocity = velocity.abs()
            flat_drift_index = int(absolute_drift.argmax().item())
            flat_velocity_index = int(absolute_velocity.argmax().item())
            drift_env_index = flat_drift_index // len(config.DoFs)
            velocity_env_index = flat_velocity_index // len(config.DoFs)
            drift_index = flat_drift_index % len(config.DoFs)
            velocity_index = flat_velocity_index % len(config.DoFs)
            position_drift = float(absolute_drift.max().item())
            max_velocity = float(absolute_velocity.max().item())
            finite = bool(position.isfinite().all().item() and velocity.isfinite().all().item())
        else:
            absolute_drift = np.abs(position - initial)
            absolute_velocity = np.abs(velocity)
            flat_drift_index = int(np.argmax(absolute_drift))
            flat_velocity_index = int(np.argmax(absolute_velocity))
            drift_env_index = flat_drift_index // len(config.DoFs)
            velocity_env_index = flat_velocity_index // len(config.DoFs)
            drift_index = flat_drift_index % len(config.DoFs)
            velocity_index = flat_velocity_index % len(config.DoFs)
            position_drift = float(np.max(absolute_drift))
            max_velocity = float(np.max(absolute_velocity))
            finite = bool(np.isfinite(position).all() and np.isfinite(velocity).all())
        dof_names = [dof.name for dof in config.DoFs]
        passed = bool(
            finite
            and position_drift <= spec.max_position_drift
            and max_velocity <= spec.max_abs_velocity
        )
        print(
            f"{config_name}: {'PASS' if passed else 'FAIL'} "
            f"({args.num_envs} envs, {args.dynamics_backend} dynamics, "
            f"{steps} controls, {feedback['time']:.3f} s, "
            f"max drift={position_drift:.6g} "
            f"[env {drift_env_index}, {dof_names[drift_index]}], "
            f"max velocity={max_velocity:.6g} "
            f"[env {velocity_env_index}, {dof_names[velocity_index]}])",
            flush=True,
        )
        return passed
    except Exception as exc:
        print(f"{config_name}: ERROR: {type(exc).__name__}: {exc}", flush=True)
        if args.traceback:
            traceback.print_exc()
        return False
    finally:
        if agent is not None:
            agent.close()


def main() -> int:
    args = _parse_args()
    config_names = (
        [args.robot_config] if isinstance(args.robot_config, str) else list(args.robot_config)
    )
    if len(config_names) > 1:
        statuses = []
        for config_name in config_names:
            command = [
                sys.executable,
                str(Path(__file__).resolve()),
                "--robot-config",
                config_name,
                "--device",
                args.device,
                "--num-envs",
                str(args.num_envs),
                "--dynamics-backend",
                args.dynamics_backend,
            ]
            if args.hold_seconds is not None:
                command.extend(("--hold-seconds", str(args.hold_seconds)))
            if args.traceback:
                command.append("--traceback")
            if args.print_layout:
                command.append("--print-layout")
            statuses.append(subprocess.run(command, check=False).returncode)
        return 0 if not any(statuses) else 1

    # Kit must own process initialization before spark_agent lazily imports
    # any Isaac modules.
    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": True, "multi_gpu": False})
    results = []
    exit_code = 1
    try:
        for config_name in config_names:
            results.append(_check_config(args, config_name))
        exit_code = 0 if results and all(results) else 1
    finally:
        # Isaac Sim's fast shutdown may terminate the interpreter directly.
        # Forward the result explicitly so an ERROR cannot be replaced by a
        # successful process exit during Kit teardown.
        simulation_app.close(exit_code=exit_code)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
