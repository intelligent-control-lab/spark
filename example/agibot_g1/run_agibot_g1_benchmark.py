#!/usr/bin/env python3
"""Run AgiBot G1 on the shared whole-body benchmark task.

The task, obstacle distribution, policy pipeline, simulator agent, and renderer
are shared SPARK components. AgiBot-specific task grounding stays local to this
runner and does not enter the robot configuration.
"""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys

from spark_pipeline import (
    AgiBotG1BenchmarkPipelineConfig as PipelineConfig,
    TeleopPipeline as Pipeline,
    apply_benchmark_test_case,
    resolve_benchmark_test_case,
    run_simulation_pipeline,
)
from spark_robot import get_agent_class_name
from spark_pipeline.autonomy.manipulator_benchmark_launcher import run_tensor_benchmark


ROBOT_CONFIGS = (
    "AgiBotG1RightArmDynamic1Config",
    "AgiBotG1MobileBaseDynamic1Config",
    "AgiBotG1MobileBaseDynamic2Config",
    "AgiBotG1DualArmDynamic1Config",
    "AgiBotG1FixedBaseDynamic1Config",
)
TEST_CASE_ALIASES = {
    "v0": "whole_goal_static_v0",
    "v1": "whole_goal_static_v1",
    "joint_goal_reaching_v0": "whole_goal_static_v0",
    "joint_goal_reaching_v1": "whole_goal_static_v1",
}
TEST_CASE_CHOICES = (
    "arm_goal_static_v0",
    "arm_goal_static_v1",
    "whole_goal_static_v0",
    "whole_goal_static_v1",
    *TEST_CASE_ALIASES,
)
BENCHMARK_GOAL_OFFSETS = {
    "left_arm_goal": (0.25, 0.0, 0.5),
    "right_arm_goal": (0.25, 0.0, 0.5),
}


def _canonical_test_case(name: str) -> str:
    return TEST_CASE_ALIASES.get(name, name)


def _simulator_timing(robot_config: str, dt, control_decimation) -> tuple[float, int]:
    """Resolve timing from the selected robot unless explicitly overridden."""
    import spark_robot

    dynamics = getattr(spark_robot, robot_config)().simulator_dynamics
    return (
        float(dynamics.physics_dt if dt is None else dt),
        int(dynamics.control_decimation if control_decimation is None else control_decimation),
    )


def _apply_benchmark_goal_offsets(cfg):
    """Apply AGI-local arm-goal offsets without changing the shared case."""
    field_by_role = {
        "left_arm_goal": "left_arm_goal_range",
        "right_arm_goal": "right_arm_goal_range",
    }
    for role, field_name in field_by_role.items():
        bounds = [
            tuple(float(value) for value in axis) for axis in getattr(cfg.env.task, field_name)
        ]
        if len(bounds) != 3 or any(len(axis) != 2 for axis in bounds):
            raise ValueError(f"Benchmark field {field_name!r} must contain three axis bounds")
        offset = BENCHMARK_GOAL_OFFSETS[role]
        bounds = [(low + delta, high + delta) for (low, high), delta in zip(bounds, offset)]
        setattr(cfg.env.task, field_name, bounds)
    return cfg


def _configure_safety(cfg, *, robot_config: str, test_case: str, safe_algo, minimum_distance):
    import spark_robot

    task = resolve_benchmark_test_case(test_case).task
    if safe_algo is None:
        safe_algo = "bypass" if int(task["num_obstacle_task"]) == 0 else "rssa"

    safe_cfg = cfg.policy.safe_controller.safe_algo
    if safe_algo == "bypass":
        safe_cfg.class_name = "ByPassSafeControl"
    elif safe_algo == "ssa":
        safe_cfg.class_name = "BasicSafeSetAlgorithm"
        safe_cfg.eta_ssa = 0.1
    elif safe_algo == "rssa":
        safe_cfg.class_name = "RelaxedSafeSetAlgorithm"
        safe_cfg.eta_ssa = 0.1
        safe_cfg.slack_weight = 1.0e3
    else:
        raise ValueError(f"Unknown safe algorithm: {safe_algo!r}")

    if safe_algo != "bypass":
        robot_cfg = getattr(spark_robot, robot_config)()
        safe_cfg.control_weight = [1.0] * len(robot_cfg.Control)

    index_cfg = cfg.policy.safe_controller.safety_index
    if "Dynamic2" in robot_config:
        index_cfg.class_name = "SecondOrderCollisionSafetyIndex"
        index_cfg.phi_n = 1.0
        index_cfg.phi_k = 1.0
    else:
        index_cfg.class_name = "FirstOrderCollisionSafetyIndex"
    index_cfg.enable_self_collision = False
    distance = task.get("minimum_distance", 0.05)
    if minimum_distance is not None:
        distance = minimum_distance
    index_cfg.min_distance["environment"] = float(distance)
    index_cfg.min_distance["self"] = 0.0
    return safe_algo


def build_config(**kwargs) -> PipelineConfig:
    """Build the fully grounded AGI benchmark configuration."""
    test_case = _canonical_test_case(kwargs.get("test_case", "whole_goal_static_v0"))
    resolve_benchmark_test_case(test_case)
    robot_config = kwargs.get("robot_config")
    if robot_config is None:
        robot_config = ROBOT_CONFIGS[0] if test_case.startswith("arm_goal_") else ROBOT_CONFIGS[1]
    if robot_config not in ROBOT_CONFIGS:
        raise ValueError(f"Unknown AgiBot benchmark configuration: {robot_config}")
    single_arm = "RightArm" in robot_config
    mobile_base = "MobileBase" in robot_config
    if test_case.startswith("whole_goal_") and not mobile_base:
        raise ValueError("Whole-body benchmarks require a mobile-base configuration")
    if test_case.startswith("arm_goal_") and mobile_base:
        raise ValueError("Arm-goal benchmarks require a right-, dual-, or fixed-base configuration")

    backend = kwargs.get("backend", "mujoco")
    cfg = PipelineConfig()
    apply_benchmark_test_case(cfg, test_case)
    _apply_benchmark_goal_offsets(cfg)
    cfg.robot.cfg.class_name = robot_config
    cfg.env.task.use_dual_arm = not single_arm
    cfg.env.task.arm_goal_position_only = True

    cfg.env.agent.class_name = get_agent_class_name(robot_config, backend=backend)
    cfg.env.agent.dynamics_backend = kwargs.get("dynamics_backend", "simulator")
    cfg.env.agent.use_sim_dynamics = cfg.env.agent.dynamics_backend == "simulator"
    # The viewer is the normal interactive default. ``--headless`` is the
    # single CLI switch that disables both the window and visual overlays.
    cfg.env.agent.enable_viewer = bool(kwargs.get("viewer", True))
    cfg.env.agent.viewer_show_simulation_info = bool(
        kwargs.get("viewer_show_simulation_info", False)
    )
    cfg.env.agent.enable_keyboard_control = bool(kwargs.get("num_debug_obstacles", 0))
    cfg.env.agent.real_time = bool(kwargs.get("real_time", False))
    requested_device = kwargs.get("device")
    cfg.env.agent.device = requested_device or (
        "cpu" if int(kwargs.get("num_envs", 1)) == 1 else "cuda:0"
    )
    physics_dt, control_decimation = _simulator_timing(
        robot_config,
        kwargs.get("dt"),
        kwargs.get("control_decimation"),
    )
    cfg.env.agent.dt = physics_dt
    cfg.env.agent.control_decimation = control_decimation
    cfg.env.agent.obstacle_debug = {
        **cfg.env.agent.obstacle_debug,
        "num_obstacle": int(kwargs.get("num_debug_obstacles", 0)),
    }
    cfg.env.task.dt = cfg.env.agent.dt * cfg.env.agent.control_decimation
    # AgiBot's planar base frame is at floor height, so a floating-base fall
    # height threshold does not apply.
    cfg.env.task.fall_height_threshold = None

    cfg.env.task.max_episode_length = int(kwargs.get("max_episode_length", 1000))
    cfg.max_num_reset = int(kwargs.get("num_resets", 10))
    max_num_steps = kwargs.get("max_num_steps")
    cfg.max_num_steps = (
        int(max_num_steps)
        if max_num_steps is not None
        else int(cfg.env.task.max_episode_length) * cfg.max_num_reset
    )
    render_every = kwargs.get("render_every")
    cfg.render_every = max(1, int(1 if render_every is None else render_every))
    cfg.render_robot_collision_volumes = bool(kwargs.get("render_robot_collision_volumes", True))
    cfg.enable_logger = False
    cfg.enable_plotter = False
    cfg.enable_safe_zone_render = False
    cfg.metric_selection.dist_goal_arm = True
    cfg.metric_selection.dist_goal_base = mobile_base
    cfg.metric_selection.dist_self = False
    cfg.metric_selection.dist_robot_to_env = bool(cfg.env.task.num_obstacle_task)
    cfg.metric_selection.seed = True
    cfg.metric_selection.done = True
    cfg.metric_selection.trigger_safe_controller = bool(cfg.env.task.num_obstacle_task)

    _configure_safety(
        cfg,
        robot_config=robot_config,
        test_case=test_case,
        safe_algo=kwargs.get("safe_algo"),
        minimum_distance=kwargs.get("minimum_distance"),
    )
    for name in (
        "record_video_path",
        "record_gif_path",
        "record_fps",
        "record_duration",
        "record_width",
        "record_height",
    ):
        value = kwargs.get(name)
        if value is not None:
            setattr(cfg.env.agent, name, value)
    return cfg


def _run_isaac_tensor_benchmark(kwargs):
    """Launch the Agi whole-body tensor runtime after Kit initialization."""

    if kwargs.get("dynamics_backend", "simulator") != "simulator":
        raise ValueError(
            "The batched Agi Isaac benchmark requires --dynamics-backend simulator. "
            "Model-owned execution remains available through the single-environment pipeline."
        )
    test_case = _canonical_test_case(kwargs.get("test_case", "whole_goal_static_v0"))
    robot_config = str(kwargs.get("robot_config") or ROBOT_CONFIGS[1])
    if "MobileBase" not in robot_config:
        configured = build_config(**kwargs)
        grounding = {
            "shared_case": test_case,
            "robot_config": robot_config,
            "use_dual_arm": "RightArm" not in robot_config,
            "mobile_base": False,
            "right_ee_body": "right_base_link",
            "left_ee_body": "left_base_link",
            "ee_local_offset": [0.0, 0.0, 0.10],
            "arm_goal_pair_keepout": 0.25,
        }
        return run_tensor_benchmark(
            kwargs,
            configured=configured,
            grounding=grounding,
            goal_rotation=(
                (0.0, -1.0, 0.0),
                (-1.0, 0.0, 0.0),
                (0.0, 0.0, -1.0),
            ),
            display_name="AgiBot G1",
        )
    num_envs = int(kwargs.get("num_envs", 1))
    num_resets = int(kwargs.get("num_resets", 10))
    max_episode_length = int(kwargs.get("max_episode_length", 1000))
    max_num_steps = kwargs.get("max_num_steps")
    if max_num_steps is None:
        max_num_steps = -1 if num_resets < 0 else max_episode_length * num_resets
    runtime = (
        Path(__file__).resolve().parents[2]
        / "pipeline"
        / "spark_pipeline"
        / "autonomy"
        / "agibot_g1_isaac_tensor_benchmark.py"
    )
    physics_dt, control_decimation = _simulator_timing(
        robot_config,
        kwargs.get("dt"),
        kwargs.get("control_decimation"),
    )
    render_every = kwargs.get("render_every")
    if render_every is None:
        # Keep the default viewport near 20 Hz without altering Agi's own
        # physics/control schedule. Explicit --render-every remains exact.
        render_every = max(
            1,
            int(math.ceil(1.0 / (20.0 * physics_dt * control_decimation))),
        )
    command = [
        sys.executable,
        str(runtime),
        "--robot-config",
        robot_config,
        "--test-case",
        test_case,
        "--num-envs",
        str(num_envs),
        "--num-resets",
        str(num_resets),
        "--max-episode-length",
        str(max_episode_length),
        "--seed",
        str(kwargs.get("seed", 0)),
        "--num-steps",
        str(max_num_steps),
        "--dt",
        str(physics_dt),
        "--control-decimation",
        str(control_decimation),
        "--render-every",
        str(render_every),
        "--max-visualized-collision-envs",
        str(kwargs.get("max_visualized_collision_envs", 1)),
    ]
    if not kwargs.get("viewer", True):
        command.append("--headless")
    if kwargs.get("viewer_show_simulation_info", False):
        command.append("--show-simulation-info")
    if kwargs.get("real_time", False):
        command.append("--real-time")
    if kwargs.get("device"):
        command += ["--device", str(kwargs["device"])]
    if kwargs.get("safe_algo"):
        command += ["--safe-algo", str(kwargs["safe_algo"])]
    if kwargs.get("minimum_distance") is not None:
        command += ["--minimum-distance", str(kwargs["minimum_distance"])]
    command.append(
        "--render-robot-collision-volumes"
        if kwargs.get("render_robot_collision_volumes", True)
        else "--no-render-robot-collision-volumes"
    )
    command.append(
        "--render-safety-trigger-constraints"
        if kwargs.get("render_safety_trigger_constraints", True)
        else "--no-render-safety-trigger-constraints"
    )
    command.append(
        "--render-safety-violations"
        if kwargs.get("render_safety_violations", True)
        else "--no-render-safety-violations"
    )
    if kwargs.get("profile_frequency", False):
        command.append("--profile-frequency")
    for key in (
        "record_video_path",
        "record_gif_path",
        "record_fps",
        "record_width",
        "record_height",
    ):
        if kwargs.get(key) is not None:
            command += ["--" + key.replace("_", "-"), str(kwargs[key])]
    print(
        f"Launching {num_envs}-environment AgiBot Isaac whole-body benchmark:",
        " ".join(command),
        flush=True,
    )
    environment = os.environ.copy()
    environment["SPARK_AGIBOT_BENCHMARK_GOAL_OFFSETS"] = json.dumps(BENCHMARK_GOAL_OFFSETS)
    environment["SPARK_AGIBOT_BENCHMARK_VIEWER_CONFIG"] = json.dumps(
        kwargs.get("viewer_config", {})
    )
    return subprocess.run(command, check=True, env=environment)


def _run_isaac_arm_tensor_benchmark(kwargs):
    """Launch fixed-base arm benchmarks through the neutral tensor runtime."""

    if kwargs.get("dynamics_backend", "simulator") != "simulator":
        raise ValueError("The Isaac tensor benchmark requires simulator-owned dynamics")
    configured = build_config(**kwargs)
    robot_config = configured.robot.cfg.class_name
    test_case = _canonical_test_case(kwargs.get("test_case", "arm_goal_static_v0"))
    return run_tensor_benchmark(
        kwargs,
        configured=configured,
        grounding={
            "shared_case": test_case,
            "robot_config": robot_config,
            "use_dual_arm": "RightArm" not in robot_config,
            "mobile_base": False,
            "right_ee_body": "right_base_link",
            "left_ee_body": "left_base_link",
            "ee_local_offset": [0.0, 0.0, 0.10],
            "arm_goal_pair_keepout": 0.25,
        },
        goal_rotation=(
            (0.0, -1.0, 0.0),
            (-1.0, 0.0, 0.0),
            (0.0, 0.0, -1.0),
        ),
        display_name="AgiBot G1",
    )


def run(**kwargs):
    num_envs = int(kwargs.get("num_envs", 1))
    backend = kwargs.get("backend", "mujoco")
    if backend == "isaac" and kwargs.get("dynamics_backend", "simulator") == "simulator":
        test_case = _canonical_test_case(kwargs.get("test_case", "whole_goal_static_v0"))
        if test_case.startswith("arm_goal_"):
            return _run_isaac_arm_tensor_benchmark(kwargs)
        return _run_isaac_tensor_benchmark(kwargs)
    if num_envs != 1:
        raise ValueError(
            "Multiple Agi benchmark environments require the Isaac simulator-owned tensor runtime."
        )
    cfg = build_config(**kwargs)
    return run_simulation_pipeline(
        cfg,
        backend=backend,
        pipeline_class=Pipeline,
        save_path=kwargs.get("save_path"),
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("mujoco", "isaac"), default="mujoco")
    parser.add_argument(
        "--robot-config",
        "--robot-cfg",
        dest="robot_config",
        choices=ROBOT_CONFIGS,
        default=None,
    )
    parser.add_argument("--test-case", choices=TEST_CASE_CHOICES, default="whole_goal_static_v0")
    parser.add_argument("--num-envs", type=int, default=1)
    parser.add_argument("--num-resets", type=int, default=10)
    parser.add_argument(
        "--max-episode-length",
        type=int,
        default=1000,
        help="Maximum control steps per episode (default: 1000).",
    )
    parser.add_argument("--max-num-steps", type=int, default=None)
    parser.add_argument("--dynamics-backend", choices=("simulator", "model"), default="simulator")
    parser.add_argument(
        "--device",
        default=None,
        help="Isaac device; defaults to cpu for one environment and cuda:0 for batches.",
    )
    parser.add_argument(
        "--headless",
        dest="viewer",
        action="store_false",
        default=True,
        help="Disable the viewer window and benchmark visualization.",
    )
    parser.add_argument(
        "--show-simulation-info",
        dest="viewer_show_simulation_info",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Show the dynamics information panel in the MuJoCo or Isaac viewer.",
    )
    parser.add_argument("--real-time", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument(
        "--render-every",
        type=int,
        default=None,
        help="Present every Nth control step (default: about 20 Hz for Isaac).",
    )
    parser.add_argument(
        "--dt",
        type=float,
        default=None,
        help="Override the selected robot configuration's physics step.",
    )
    parser.add_argument(
        "--control-decimation",
        type=int,
        default=None,
        help="Override the selected robot configuration's physics substeps per control step.",
    )
    parser.add_argument("--safe-algo", choices=("bypass", "ssa", "rssa"), default=None)
    parser.add_argument("--minimum-distance", type=float, default=None)
    parser.add_argument("--num-debug-obstacles", type=int, default=0)
    parser.add_argument(
        "--render-robot-collision-volumes",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument(
        "--render-safety-trigger-constraints",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Render blue closest-pair lines when safety correction is required.",
    )
    parser.add_argument(
        "--render-safety-violations",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Render purple closest-pair lines for residual post-filter violations.",
    )
    parser.add_argument("--max-visualized-collision-envs", type=int, default=1)
    parser.add_argument("--record-video-path", default=None)
    parser.add_argument("--record-gif-path", default=None)
    parser.add_argument("--record-fps", type=float, default=None)
    parser.add_argument("--record-duration", type=float, default=None)
    parser.add_argument("--record-width", type=int, default=None)
    parser.add_argument("--record-height", type=int, default=None)
    parser.add_argument("--save-path", default=None)
    parser.add_argument(
        "--profile-frequency",
        action="store_true",
        help="Synchronize and report Isaac control-loop stage timings.",
    )
    return parser


def main(argv=None) -> int:
    args = vars(_parser().parse_args(argv))
    if args["num_resets"] == 0 or args["num_resets"] < -1:
        raise SystemExit("--num-resets must be positive or -1 for continuous reset")
    if args["max_episode_length"] < 1:
        raise SystemExit("--max-episode-length must be positive")
    if args["max_num_steps"] is not None and (
        args["max_num_steps"] == 0 or args["max_num_steps"] < -1
    ):
        raise SystemExit("--max-num-steps must be positive or -1")
    run(**args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
