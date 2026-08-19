"""Run Unitree G1 PID, Sport, and WBT goal-tracking benchmarks.

The default mode is intentionally simple: a fixed-base G1 with articulated
hands tracks independently sampled Cartesian goals for both arms.  It has no
task/debug obstacles and applies no safety modification, which makes it a
useful nominal-control check before collision and safety components are added.
"""

from __future__ import annotations

import argparse
from copy import deepcopy
import math
from pathlib import Path
import subprocess
import multiprocessing as mp
import numpy as np
import sys
import traceback

from spark_agent.simulation.viewer_config import DEFAULT_VIEWER_CONFIG
from spark_robot import get_agent_class_name
from spark_pipeline import TeleopPipeline as Pipeline
from spark_pipeline import UnitreeG1BenchmarkPipelineConfig as PipelineConfig
from spark_pipeline import (
    LEGACY_TASK_CASE_LIST,
    apply_benchmark_test_case,
    list_benchmark_test_cases,
    resolve_benchmark_test_case,
    run_simulation_pipeline,
)
from spark_pipeline.autonomy.manipulator_benchmark_launcher import run_tensor_benchmark
from spark_task.autonomy.benchmark_goals import BENCHMARK_ENVIRONMENT_SEED_STRIDE


TASK_CASE_LIST = list_benchmark_test_cases()

# Keep the scalar benchmark aligned with the parallel Isaac benchmark.  These
# bounds are wide enough to exercise goal-conditioned locomotion but remain
# reachable within the default 500-step (10 s simulated-time) episode, leaving
# time to verify the locomotion-to-standing handoff after arrival.
DEFAULT_WBT_BASE_GOAL_RANGE = [(0.2, 0.5), (-0.2, 0.2), (0.793, 0.793)]
DEFAULT_WBT_BASE_GOAL_ROT_RANGE = (-0.35, 0.35)


POLICY_RUNTIME = {
    "TeleopPIDPolicy": "pid",
    "UnitreeG1SportPolicy": "sport",
    "UnitreeG1SportSafePolicy": "sport",
    "UnitreeG1WBTPolicy": "wbt",
    "UnitreeG1WBTSafePolicy": "wbt",
    "UnitreeG1SonicPolicy": "sonic",
    "UnitreeG1SonicSafePolicy": "sonic",
}


def _runtime_policy(robot_config, policy_config=None):
    """Validate a registered policy class against the robot state contract."""
    policy_config = policy_config or "TeleopPIDPolicy"
    control_policy = POLICY_RUNTIME[policy_config]
    if "WholeBody" in robot_config:
        inferred_mode = "whole_body"
    elif "MobileBase" in robot_config:
        inferred_mode = "mobile_base"
    elif any(name in robot_config for name in ("FixedBase", "DualArm", "RightArm")):
        inferred_mode = "fixed_base"
    else:
        raise ValueError(f"Cannot infer robot mode from {robot_config!r}")
    robot_mode = inferred_mode
    if control_policy in ("wbt", "sport", "sonic") and robot_mode == "fixed_base":
        raise ValueError(f"--policy-config {policy_config} requires a mobile/whole-body robot")
    if control_policy == "pid" and robot_mode == "whole_body":
        raise ValueError("PID benchmark policy does not produce whole-body joint commands")
    # `mobile_base` remains an internal ideal-plant runtime, never a policy.
    return (
        "mobile_base" if control_policy == "pid" and robot_mode == "mobile_base" else control_policy
    )


def _axis_ranges(value):
    if value is None:
        return None
    if len(value) == 3 and all(len(axis) == 2 for axis in value):
        return value
    if len(value) != 6:
        raise ValueError("A 3D range requires xmin xmax ymin ymax zmin zmax")
    return [tuple(value[index : index + 2]) for index in range(0, 6, 2)]


def _fixed_point_range(value):
    if value is None:
        return None
    return [(float(axis), float(axis)) for axis in value]


def _arm_goal_mode(kwargs) -> str:
    mode = kwargs.get("arm_goal_mode") or "random"
    enabled = kwargs.get("arm_goal_enable", None)
    if enabled is False:
        return "disabled"
    if enabled is True and mode == "disabled":
        return "random"
    return mode


def _resolve_case_inputs(kwargs):
    """Fill task runner inputs from the selected case, preserving CLI overrides."""
    resolved = dict(kwargs)
    viewer_config = resolved.get("viewer_config") or {}
    for argument, field in (
        ("viewer_lookat", "camera_lookat"),
        ("viewer_distance", "camera_distance"),
        ("viewer_azimuth", "camera_azimuth"),
        ("viewer_elevation", "camera_elevation"),
        ("viewer_vertical_fov", "camera_vertical_fov"),
    ):
        if resolved.get(argument) is None and viewer_config.get(field) is not None:
            resolved[argument] = viewer_config[field]
    case_name = resolved.get("test_case_name") or resolved.get("test_case") or "random_arm_nominal"
    case = resolve_benchmark_test_case(case_name)
    task = case.parameters()
    resolved["test_case"] = case.name
    # Compatibility only: old names bundled a robot choice into the scenario
    # string. New canonical case names never do this.
    legacy_name = resolved.get("test_case_name")
    if legacy_name and not resolved.get("robot_config"):
        dynamics = "Dynamic2" if "_D2_" in legacy_name else "Dynamic1"
        if "FixedBase" in legacy_name:
            resolved["robot_config"] = f"UnitreeG1FixedBase{dynamics}Config"
        elif "MobileBase" in legacy_name:
            resolved["robot_config"] = f"UnitreeG1MobileBase{dynamics}Config"
        elif "WholeBody" in legacy_name:
            resolved["robot_config"] = f"UnitreeG1WholeBody{dynamics}Config"
        elif "SportMode" in legacy_name:
            resolved["robot_config"] = f"UnitreeG1MobileBase{dynamics}Config"
    defaults = {
        "arm_goal_mode": (
            "disabled"
            if not task.get("arm_goal_enable", True)
            else "fixed"
            if task.get("arm_goal_mode") == "Velocity"
            else "random"
        ),
        "base_goal_mode": (
            "disabled"
            if not task.get("base_goal_enable", True)
            else "fixed"
            if task.get("base_goal_mode") == "Velocity"
            else "random"
        ),
        "obstacle_mode": (
            "disabled"
            if task.get("num_obstacle_task", 0) == 0
            else "fixed"
            if task.get("obstacle_mode") == "Velocity"
            else "random"
        ),
        "num_obstacles": task.get("num_obstacle_task", 0),
        "obstacle_range": task.get("obstacle_range"),
        "obstacle_size": task.get("obstacle_size"),
        "obstacle_velocity": task.get("obstacle_velocity"),
        "left_arm_goal_range": task.get("left_arm_goal_range"),
        "right_arm_goal_range": task.get("right_arm_goal_range"),
        "base_goal_range": task.get("base_goal_range"),
        "base_goal_velocity": task.get("base_goal_velocity", 0.0),
        "base_goal_minimum_distance": task.get("base_goal_minimum_distance", 0.0),
        "base_goal_relative_to_current": task.get("base_goal_relative_to_current", False),
        "base_goal_workspace_range": task.get("base_goal_workspace_range"),
        "base_goal_rot_range": task.get("base_goal_rot_range"),
        "arm_goal_size": task.get("arm_goal_size", 0.05),
        "base_goal_size": task.get("base_goal_size", 0.1),
        "base_goal_yaw_size": task.get("base_goal_yaw_size", float("inf")),
        "max_episode_length": task.get("max_episode_length", 1000),
        "max_num_steps": PipelineConfig.max_num_steps,
        "max_num_reset": PipelineConfig.max_num_reset,
        "fall_height_threshold": task.get("fall_height_threshold", 0.45),
        "reset_on_success": task.get("reset_on_success", True),
        "reset_on_timeout": task.get("reset_on_timeout", True),
        "seed": task.get("seed", 0),
        "minimum_distance": task.get("minimum_distance", 0.05),
        "safety_activation_distance": task.get("safety_activation_distance", 0.15),
        "obstacle_keepout": task.get("obstacle_keepout", 0.05),
        "robot_keepout": task.get("robot_keepout", 0.05),
        "obstacle_robot_keepaway": task.get("robot_keepout", 0.05),
        "obstacle_goal_keepaway": task.get("obstacle_goal_keepaway", 0.15),
        "arm_goal_minimum_distance": task.get("arm_goal_minimum_distance", 0.0),
        "arm_goal_pair_keepout": task.get("arm_goal_pair_keepout", 0.0),
    }
    for key, value in defaults.items():
        if resolved.get(key) is None:
            resolved[key] = deepcopy(value) if isinstance(value, (list, dict)) else value
    return resolved


def _mujoco_parallel_worker(worker_id, worker_kwargs, result_queue):
    """Run one isolated MuJoCo world for an N-environment benchmark."""
    try:
        print(f"[SPARK] MuJoCo env {worker_id} starting", flush=True)
        run(**worker_kwargs)
        result_queue.put((worker_id, True, None))
    except BaseException:
        result_queue.put((worker_id, False, traceback.format_exc()))


def _run_parallel_mujoco(kwargs, num_envs):
    """Execute independent MuJoCo worlds concurrently.

    MuJoCo does not expose the cloned tensor-world abstraction used by Isaac,
    so each environment owns a simulator, task, WBT recurrent state, and seed
    in a separate spawned process.  This preserves the benchmark's independent
    reset semantics and avoids sharing non-thread-safe simulator state.
    """
    if kwargs.get("viewer", False):
        print(
            "[SPARK] MuJoCo --num-envs > 1 runs headless independent worlds; "
            "use --num-envs 1 for an interactive viewer.",
            flush=True,
        )
    context = mp.get_context("spawn")
    result_queue = context.Queue()
    processes = []
    base_seed = int(kwargs.get("seed", 0))
    for worker_id in range(num_envs):
        worker_kwargs = dict(kwargs)
        worker_kwargs.update(
            num_envs=1,
            viewer=False,
            real_time=False,
            seed=base_seed + BENCHMARK_ENVIRONMENT_SEED_STRIDE * worker_id,
        )
        for key in ("record_video_path", "record_gif_path"):
            if worker_kwargs.get(key):
                output = Path(worker_kwargs[key])
                worker_kwargs[key] = str(
                    output.with_name(f"{output.stem}_env{worker_id}{output.suffix}")
                )
        process = context.Process(
            target=_mujoco_parallel_worker,
            args=(worker_id, worker_kwargs, result_queue),
            name=f"spark-mujoco-env-{worker_id}",
        )
        process.start()
        processes.append(process)
    results = [result_queue.get() for _ in processes]
    for process in processes:
        process.join()
    failures = [(env_id, detail) for env_id, ok, detail in results if not ok]
    if failures:
        details = "\n".join(f"MuJoCo env {env_id} failed:\n{detail}" for env_id, detail in failures)
        raise RuntimeError(details)
    print(
        f"[SPARK] MuJoCo parallel benchmark completed: {num_envs}/{num_envs} "
        "independent environments exited successfully.",
        flush=True,
    )
    return results


def _is_nominal_random_arm_mode(kwargs) -> bool:
    return _runtime_policy(kwargs.get("robot_config"), kwargs.get("policy_config")) == "pid" and (
        kwargs.get("test_case") == "random_arm_nominal"
        or kwargs.get("test_case") == "arm_goal_static_v0"
        or kwargs.get("mode") == "random-arm-no-collision"
    )


def _with_hand_robot_config(class_name: str) -> str:
    """Map a Unitree G1 body config to its one-to-one hand-equipped variant."""
    if "WithHand" in class_name:
        return class_name
    if not class_name.startswith("UnitreeG1") or "Dynamic" not in class_name:
        raise ValueError(f"No articulated-hand variant is known for {class_name!r}")
    return class_name.replace("Dynamic", "WithHandDynamic", 1)


def _without_hand_robot_config(class_name: str) -> str:
    """Return the matching handless robot contract."""
    return class_name.replace("WithHand", "", 1)


def _robot_uses_hands(kwargs) -> bool:
    """Resolve hand presence from an explicit override or the config name."""
    override = kwargs.get("with_hand")
    if override is not None:
        return bool(override)
    return "WithHand" in str(kwargs.get("robot_config", ""))


def config_task_module(cfg: PipelineConfig, **kwargs):
    """Configure episode resampling and hand goals."""
    if kwargs.get("max_episode_length") is not None:
        cfg.env.task.max_episode_length = int(kwargs["max_episode_length"])
    if kwargs.get("seed") is not None:
        cfg.env.task.seed = int(kwargs["seed"])
    grippers_closed = bool(kwargs.get("grippers_closed", False))
    cfg.env.task.left_gripper_goal = grippers_closed
    cfg.env.task.right_gripper_goal = grippers_closed
    arm_mode = kwargs.get("arm_goal_mode")
    if arm_mode is None:
        arm_mode = (
            "disabled"
            if not cfg.env.task.arm_goal_enable
            else ("fixed" if cfg.env.task.arm_goal_mode == "Velocity" else "random")
        )
    base_mode = kwargs.get("base_goal_mode")
    if base_mode is None:
        base_mode = (
            "disabled"
            if not cfg.env.task.base_goal_enable
            else ("fixed" if cfg.env.task.base_goal_mode == "Velocity" else "random")
        )
    obstacle_mode = kwargs.get("obstacle_mode")
    if obstacle_mode is None:
        obstacle_mode = (
            "disabled"
            if cfg.env.task.num_obstacle_task == 0
            else "fixed"
            if cfg.env.task.obstacle_mode == "Velocity"
            else "random"
        )
    cfg.env.task.arm_goal_enable = arm_mode != "disabled"
    cfg.env.task.base_goal_enable = base_mode != "disabled"
    cfg.env.task.arm_goal_reach_done = arm_mode != "disabled"
    cfg.env.task.base_goal_reach_done = base_mode != "disabled"
    cfg.env.task.arm_goal_size = float(kwargs.get("arm_goal_size", 0.05))
    cfg.env.task.base_goal_size = float(kwargs.get("base_goal_size", 0.15))
    cfg.env.task.base_goal_yaw_size = float(kwargs.get("base_goal_yaw_size", float("inf")))
    cfg.env.task.arm_goal_mode = "Velocity" if arm_mode == "fixed" else "Brownian"
    cfg.env.task.base_goal_mode = "Velocity" if base_mode == "fixed" else "Brownian"
    if obstacle_mode is not None:
        cfg.env.task.obstacle_mode = (
            "Velocity" if obstacle_mode in ("disabled", "fixed") else "Brownian"
        )
        cfg.env.task.num_obstacle_task = (
            0
            if obstacle_mode == "disabled"
            else int(
                cfg.env.task.num_obstacle_task
                if kwargs.get("num_obstacles") is None
                else kwargs["num_obstacles"]
            )
        )
    fixed_ranges = {
        "left_arm_goal_range": _fixed_point_range(kwargs.get("goal_left_init"))
        if arm_mode == "fixed"
        else None,
        "right_arm_goal_range": _fixed_point_range(kwargs.get("goal_right_init"))
        if arm_mode == "fixed"
        else None,
        "base_goal_range": _fixed_point_range(kwargs.get("base_goal_init"))
        if base_mode == "fixed"
        else None,
        "obstacle_range": _fixed_point_range(kwargs.get("obstacle_init"))
        if obstacle_mode == "fixed"
        else None,
    }
    for name in (
        "goal_left_init",
        "goal_right_init",
        "base_goal_init",
        "left_arm_goal_range",
        "right_arm_goal_range",
        "base_goal_range",
        "base_goal_rot_range",
        "obstacle_range",
        "obstacle_init",
        "obstacle_size",
        "obstacle_velocity",
    ):
        value = kwargs.get(name)
        if value is None:
            value = fixed_ranges.get(name)
        if value is not None:
            if name.endswith("_range") and name != "base_goal_rot_range":
                value = _axis_ranges(value)
            setattr(cfg.env.task, name, value)
    if (
        _runtime_policy(
            kwargs.get("robot_config"),
            kwargs.get("policy_config"),
        )
        == "sport"
        and base_mode == "random"
        and kwargs.get("base_goal_range") is None
    ):
        cfg.env.task.base_goal_range = DEFAULT_WBT_BASE_GOAL_RANGE
        cfg.env.task.base_goal_rot_range = DEFAULT_WBT_BASE_GOAL_ROT_RANGE
    if obstacle_mode == "random" and kwargs.get("obstacle_velocity") is None:
        cfg.env.task.obstacle_velocity = 0.005
    cfg.env.task.environment_representation = kwargs.get("environment_representation", "sphere")
    cfg.env.task.points_per_obstacle = int(kwargs.get("points_per_obstacle", 64))
    cfg.env.task.minimum_points_per_obstacle = int(kwargs.get("minimum_points_per_obstacle", 4))
    cfg.env.task.dynamic_point_count = bool(kwargs.get("dynamic_point_count", True))
    cfg.env.task.regenerate_point_cloud_every_step = bool(
        kwargs.get("regenerate_point_cloud_every_step", False)
    )
    cfg.env.task.point_radius = float(
        0.004 if kwargs.get("point_radius") is None else kwargs["point_radius"]
    )
    cfg.env.task.object_mesh_path = kwargs.get("object_mesh_path", None)
    cfg.env.task.object_mesh_scale = float(kwargs.get("object_mesh_scale", 1.0))
    cfg.env.task.max_visualized_points = int(kwargs.get("max_visualized_points", 200))
    cfg.env.task.obstacle_keepout = float(kwargs.get("obstacle_keepout", 0.05))
    cfg.env.task.robot_keepout = float(kwargs.get("robot_keepout", 0.05))
    cfg.env.task.obstacle_goal_keepaway = float(kwargs.get("obstacle_goal_keepaway", 0.15))
    cfg.env.task.arm_goal_minimum_distance = float(kwargs.get("arm_goal_minimum_distance", 0.0))
    cfg.env.task.arm_goal_pair_keepout = float(kwargs.get("arm_goal_pair_keepout", 0.0))
    cfg.env.task.deterministic_scenario_sampling = _runtime_policy(
        kwargs.get("robot_config"), kwargs.get("policy_config")
    ) in {"wbt", "sport", "sonic"} and not kwargs.get("shared_obstacle_environment", False)
    return cfg


def config_robot_module(cfg: PipelineConfig, **kwargs):
    """Select the robot contract; its name is authoritative for hand presence."""
    if kwargs.get("robot_config"):
        cfg.robot.cfg.class_name = kwargs["robot_config"]
    if kwargs.get("with_hand") is True:
        cfg.robot.cfg.class_name = _with_hand_robot_config(cfg.robot.cfg.class_name)
    elif kwargs.get("with_hand") is False:
        cfg.robot.cfg.class_name = _without_hand_robot_config(cfg.robot.cfg.class_name)
    return cfg


def config_agent_module(cfg: PipelineConfig, **kwargs):
    """Resolve the agent from the selected robot contract and backend."""
    import spark_robot

    backend = kwargs.get("backend", "mujoco")
    cfg.env.agent.class_name = get_agent_class_name(cfg.robot.cfg.class_name, backend=backend)
    cfg.env.agent.enable_viewer = bool(kwargs.get("viewer", True))
    cfg.env.agent.viewer_show_simulation_info = bool(
        kwargs.get("viewer_show_simulation_info", False)
    )
    cfg.env.agent.use_sim_dynamics = bool(kwargs.get("use_sim_dynamics", False))
    cfg.env.agent.real_time = bool(kwargs.get("real_time", True))
    timing = getattr(spark_robot, cfg.robot.cfg.class_name)().simulator_dynamics
    cfg.env.agent.dt = float(timing.physics_dt)
    cfg.env.agent.control_decimation = int(timing.control_decimation)
    if "SportMode" in cfg.robot.cfg.class_name:
        cfg.env.agent.sport_policy_num_threads = int(kwargs.get("sport_policy_threads", 1))
    # Task objects are advanced once per agent/control step.  Express their
    # velocities in simulated seconds, independent of the controller's physics
    # decimation, so a fixed test case produces the same obstacle motion for
    # mobile-base, WBT, and Sport Mode robots.
    cfg.env.task.dt = float(cfg.env.agent.dt) * int(cfg.env.agent.control_decimation)
    cfg.env.agent.viewer_config = dict(DEFAULT_VIEWER_CONFIG)
    cfg.env.agent.viewer_config.update(kwargs.get("viewer_config") or {})
    viewer_overrides = {
        "camera_lookat": kwargs.get("viewer_lookat"),
        "camera_distance": kwargs.get("viewer_distance"),
        "camera_azimuth": kwargs.get("viewer_azimuth"),
        "camera_elevation": kwargs.get("viewer_elevation"),
        "camera_vertical_fov": kwargs.get("viewer_vertical_fov"),
    }
    cfg.env.agent.viewer_config.update(
        {name: value for name, value in viewer_overrides.items() if value is not None}
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
    cfg.env.agent.obstacle_debug = {
        **cfg.env.agent.obstacle_debug,
        "num_obstacle": int(kwargs.get("num_debug_obstacles", 0)),
    }
    return cfg


def config_policy_module(cfg: PipelineConfig, **kwargs):
    """Use the benchmark Cartesian IK + joint-space PID policy."""
    del kwargs
    return cfg


def config_safety_module(cfg: PipelineConfig, **kwargs):
    """Configure optional legacy safety filtering."""
    safe_algo = (
        "bypass" if _is_nominal_random_arm_mode(kwargs) else (kwargs.get("safe_algo") or "rssa")
    )
    if kwargs.get("environment_representation", "sphere") != "sphere":
        cfg.policy.safe_controller.safety_index.nearest_points_per_link = int(
            kwargs.get("safety_nearest_points_per_link", 2)
        )
    match safe_algo:
        case "bypass":
            cfg.policy.safe_controller.safe_algo.class_name = "ByPassSafeControl"
        case "ssa":
            cfg.policy.safe_controller.safe_algo.class_name = "BasicSafeSetAlgorithm"
            cfg.policy.safe_controller.safe_algo.eta_ssa = 0.1
        case "rssa":
            cfg.policy.safe_controller.safe_algo.class_name = "RelaxedSafeSetAlgorithm"
            cfg.policy.safe_controller.safe_algo.eta_ssa = 0.1
            cfg.policy.safe_controller.safe_algo.slack_weight = 1e3
        case "sss":
            cfg.policy.safe_controller.safe_algo.class_name = "BasicSublevelSafeSetAlgorithm"
            cfg.policy.safe_controller.safe_algo.lambda_sss = 10.0
        case "rsss":
            cfg.policy.safe_controller.safe_algo.class_name = "RelaxedSublevelSafeSetAlgorithm"
            cfg.policy.safe_controller.safe_algo.lambda_sss = 10.0
            cfg.policy.safe_controller.safe_algo.slack_weight = 1e3
        case "cbf":
            cfg.policy.safe_controller.safe_algo.class_name = "BasicControlBarrierFunction"
            cfg.policy.safe_controller.safe_algo.lambda_cbf = 10.0
        case "rcbf":
            cfg.policy.safe_controller.safe_algo.class_name = "RelaxedControlBarrierFunction"
            cfg.policy.safe_controller.safe_algo.lambda_cbf = 10.0
            cfg.policy.safe_controller.safe_algo.slack_weight = 1e3
        case "pfm":
            cfg.policy.safe_controller.safe_algo.class_name = "BasicPotentialFieldMethod"
            cfg.policy.safe_controller.safe_algo.c_pfm = 1.0
        case "sma":
            cfg.policy.safe_controller.safe_algo.class_name = "BasicSlidingModeAlgorithm"
            cfg.policy.safe_controller.safe_algo.c_sma = 1.0
        case _:
            raise ValueError(f"Unknown safe algorithm: {safe_algo}")

    weighted_algorithms = {"ssa", "rssa", "sss", "rsss", "cbf", "rcbf"}
    if safe_algo in weighted_algorithms:
        control_weight = [1.0] * 20
        if "FixedBase" in cfg.robot.cfg.class_name:
            control_weight = control_weight[:-3]
        elif "RightArm" in cfg.robot.cfg.class_name:
            control_weight = control_weight[3:10]
        elif "DualArm" in cfg.robot.cfg.class_name:
            control_weight = control_weight[3:17]
        cfg.policy.safe_controller.safe_algo.control_weight = control_weight

    safety_index = kwargs.get("safety_index") or (
        "si2" if "Dynamic2" in cfg.robot.cfg.class_name else "si1"
    )
    match safety_index:
        case "si1":
            cfg.policy.safe_controller.safety_index.class_name = "FirstOrderCollisionSafetyIndex"
        case "si2":
            cfg.policy.safe_controller.safety_index.class_name = "SecondOrderCollisionSafetyIndex"
            cfg.policy.safe_controller.safety_index.phi_n = 1.0
            cfg.policy.safe_controller.safety_index.phi_k = 1.0
        case "si2nn":
            cfg.policy.safe_controller.safety_index.class_name = "SecondOrderNNCollisionSafetyIndex"
            cfg.policy.safe_controller.safety_index.phi_n = 2
            cfg.policy.safe_controller.safety_index.phi_k = 1
            cfg.policy.safe_controller.safety_index.phi_nn_path = "n_2_scalar.onnx"
        case _:
            raise ValueError(f"Unknown safety index: {safety_index}")
    return cfg


def config_pipeline(cfg: PipelineConfig, **kwargs):
    """Apply a task-only case before grounding it with a robot/backend."""
    controller = _runtime_policy(kwargs.get("robot_config"), kwargs.get("policy_config"))
    test_case_name = kwargs.get("test_case_name") or kwargs.get("test_case") or "random_arm_nominal"
    cfg = apply_benchmark_test_case(cfg, test_case_name)
    # The canonical arm-goal cases describe the environment, not the number of
    # manipulators.  A right-arm robot therefore consumes and renders only the
    # right goal while dual-arm configurations consume both goals.
    cfg.env.task.use_dual_arm = "RightArm" not in str(kwargs.get("robot_config", ""))
    cfg.max_num_steps = int(kwargs.get("max_num_steps", PipelineConfig.max_num_steps))
    cfg.max_num_reset = int(kwargs.get("max_num_reset", PipelineConfig.max_num_reset))
    cfg.enable_logger = bool(kwargs.get("enable_logger", False))
    cfg.enable_safe_zone_render = False
    cfg.profile_frequency = bool(kwargs.get("profile_frequency", False))
    configured_render_every = kwargs.get("render_every")
    cfg.render_every = max(
        1, int(1 if configured_render_every is None else configured_render_every)
    )

    if _is_nominal_random_arm_mode(kwargs):
        # Draw a new pair of Cartesian goals after each reach or episode timeout.
        cfg.env.task.task_name = "arm_goal_static_v0"
        cfg.env.task.num_obstacle_task = 0
        cfg.env.task.arm_goal_enable = True
        cfg.env.task.use_dual_arm = "RightArm" not in str(kwargs.get("robot_config", ""))
        cfg.env.task.arm_goal_velocity = float(kwargs.get("arm_goal_velocity", 0.0))
        cfg.env.task.arm_goal_reach_done = True
        cfg.env.task.base_goal_enable = False
        cfg.env.agent.obstacle_debug = {
            **cfg.env.agent.obstacle_debug,
            "num_obstacle": 0,
        }
        cfg.policy.safe_controller.safe_algo.class_name = "ByPassSafeControl"
        cfg.metric_selection.dist_self = False
        cfg.metric_selection.dist_robot_to_env = False
        cfg.metric_selection.dist_goal_base = False
        cfg.metric_selection.violation = False
        cfg.metric_selection.trigger_safe_controller = False
        cfg.render_robot_collision_volumes = False
    else:
        cfg.metric_selection.dist_self = True
        cfg.metric_selection.dist_robot_to_env = True
        cfg.metric_selection.dist_goal_base = "FixedBase" not in cfg.env.task.task_name
        cfg.metric_selection.violation = True
        cfg.render_robot_collision_volumes = bool(
            kwargs.get("render_robot_collision_volumes", True)
        )

    cfg.metric_selection.dist_goal_arm = _arm_goal_mode(kwargs) != "disabled"
    cfg.metric_selection.dist_goal_base = kwargs.get("base_goal_mode", "disabled") != "disabled"
    cfg.metric_selection.trigger_safe_controller = (
        not _is_nominal_random_arm_mode(kwargs) and kwargs.get("safe_algo", "bypass") != "bypass"
    )
    cfg.metric_selection.seed = True
    cfg.metric_selection.done = True
    return cfg


def build_config(**kwargs) -> PipelineConfig:
    """Build the fully grounded config; kept separate for smoke tests and reuse."""
    kwargs = _resolve_case_inputs(kwargs)
    cfg = PipelineConfig()
    cfg = config_pipeline(cfg, **kwargs)
    cfg = config_task_module(cfg, **kwargs)
    cfg = config_robot_module(cfg, **kwargs)
    cfg = config_agent_module(cfg, **kwargs)
    cfg = config_policy_module(cfg, **kwargs)
    cfg = config_safety_module(cfg, **kwargs)
    return cfg


def _run_pid_isaac_tensor_benchmark(kwargs, *, num_envs: int):
    """Run the requested PID robot contract through the generic tensor plant."""

    configured = build_config(**kwargs)
    robot_config = str(kwargs["robot_config"])
    use_dual_arm = "RightArm" not in robot_config
    mobile_base = "MobileBase" in robot_config
    configured.env.task.use_dual_arm = use_dual_arm
    configured.env.task.arm_goal_position_only = False
    configured.env.task.arm_goal_orientation_size = float(
        getattr(configured.env.task, "arm_goal_orientation_size", 0.1)
    )
    configured.env.task.arm_goal_minimum_distance = float(
        getattr(configured.env.task, "arm_goal_minimum_distance", 0.0)
    )
    configured.env.task.arm_goal_pair_keepout = float(
        getattr(configured.env.task, "arm_goal_pair_keepout", 0.0)
    )
    configured.policy.nominal_controller.position_kp = 1.0
    configured.policy.nominal_controller.velocity_kd = 0.1
    grounding = {
        "shared_case": kwargs["test_case"],
        "robot_config": robot_config,
        "use_dual_arm": use_dual_arm,
        "mobile_base": mobile_base,
        "right_ee_body": "right_wrist_yaw_link",
        "left_ee_body": "left_wrist_yaw_link",
        "ee_local_offset": [0.1, 0.0, 0.0],
        "right_ee_local_rotation": (
            (1.0, 0.0, 0.0),
            (0.0, 0.0, -1.0),
            (0.0, 1.0, 0.0),
        ),
        "left_ee_local_rotation": (
            (1.0, 0.0, 0.0),
            (0.0, 0.0, 1.0),
            (0.0, -1.0, 0.0),
        ),
        "arm_goal_pair_keepout": max(
            0.25, float(getattr(configured.env.task, "arm_goal_pair_keepout", 0.0))
        ),
    }
    return run_tensor_benchmark(
        {**kwargs, "num_envs": num_envs},
        configured=configured,
        grounding=grounding,
        goal_rotation=(
            (1.0, 0.0, 0.0),
            (0.0, 1.0, 0.0),
            (0.0, 0.0, 1.0),
        ),
        display_name="Unitree G1",
    )


def _run_isaac_tensor_benchmark(kwargs, *, controller: str, num_envs: int):
    """Delegate Isaac learned control to the tensor benchmark implementation."""
    import spark_robot

    timing = getattr(spark_robot, kwargs["robot_config"])().simulator_dynamics
    requested_dt = kwargs.get("isaac_physics_dt")
    physics_dt = float(timing.physics_dt if requested_dt is None else requested_dt)
    requested_decimation = kwargs.get("isaac_control_decimation")
    control_decimation = int(
        timing.control_decimation if requested_decimation is None else requested_decimation
    )
    if physics_dt <= 0.0 or control_decimation < 1:
        raise ValueError("Isaac physics dt must be positive and decimation at least one")
    if not math.isclose(physics_dt * control_decimation, 0.02, rel_tol=0.0, abs_tol=1.0e-9):
        raise ValueError(
            "WBT expects a 20 ms policy period: --isaac-physics-dt times "
            "--isaac-control-decimation must equal 0.02"
        )
    render_every = kwargs.get("render_every")
    if render_every is None:
        # Keep the interactive viewport near 10 Hz while physics, policy, and
        # reset checks remain at 50 Hz. Rendering every control step dominates
        # multi-environment Sport/WBT runs without improving control fidelity.
        render_every = max(
            1,
            int(math.ceil(1.0 / (10.0 * physics_dt * control_decimation))),
        )
    tensor_runtime = (
        Path(__file__).resolve().parents[2]
        / "pipeline"
        / "spark_pipeline"
        / "autonomy"
        / "unitree_g1_isaac_tensor_benchmark.py"
    )
    base_mode = kwargs.get("base_goal_mode", "disabled")
    arm_mode = _arm_goal_mode(kwargs)
    max_episode_length = kwargs.get("max_episode_length")
    if max_episode_length is None:
        max_episode_length = 1000
    command = [
        sys.executable,
        str(tensor_runtime),
        "--num-envs",
        str(num_envs),
        "--policy",
        controller,
        "--dt",
        str(physics_dt),
        "--control-decimation",
        str(control_decimation),
        "--num-steps",
        str(kwargs.get("max_num_steps", PipelineConfig.max_num_steps)),
        "--warmup-steps",
        str(kwargs.get("wbt_warmup_steps", 20)),
        "--max-episode-length",
        str(max_episode_length),
        "--fall-height-threshold",
        str(kwargs.get("fall_height_threshold", 0.45)),
        "--render-every",
        str(render_every),
        "--isaac-render-quality",
        str(kwargs.get("isaac_render_quality", "performance")),
        "--base-goal-tolerance",
        str(kwargs.get("base_goal_size", 0.15)),
        "--base-yaw-tolerance",
        str(kwargs.get("base_goal_yaw_size", 0.10)),
        "--base-goal-minimum-distance",
        str(kwargs.get("base_goal_minimum_distance", 0.0)),
        "--arm-goal-minimum-distance",
        str(kwargs.get("arm_goal_minimum_distance", 0.0)),
        "--arm-goal-pair-keepout",
        str(kwargs.get("arm_goal_pair_keepout", 0.0)),
        "--base-goal-velocity",
        str(kwargs.get("base_goal_velocity", 0.0)),
        "--arm-goal-tolerance",
        str(kwargs.get("arm_goal_size", 0.05)),
        "--wbt-max-forward-speed",
        str(kwargs.get("wbt_max_forward_speed", 0.12)),
        "--wbt-max-lateral-speed",
        str(kwargs.get("wbt_max_lateral_speed", 0.10)),
        "--wbt-max-yaw-rate",
        str(kwargs.get("wbt_max_yaw_rate", 0.30)),
        "--wbt-min-translation-speed",
        str(kwargs.get("wbt_min_translation_speed", 0.05)),
        "--wbt-min-yaw-rate",
        str(kwargs.get("wbt_min_yaw_rate", 0.08)),
        "--wbt-command-acceleration",
        str(kwargs.get("wbt_command_acceleration", 0.30)),
        "--wbt-yaw-acceleration",
        str(kwargs.get("wbt_yaw_acceleration", 0.75)),
        "--wbt-base-settle-steps",
        str(kwargs.get("wbt_base_settle_steps", 50)),
        "--base-velocity-kd",
        str(kwargs.get("wbt_base_velocity_kd", 0.4)),
        "--base-yaw-velocity-kd",
        str(kwargs.get("wbt_base_yaw_velocity_kd", 0.25)),
        "--dynamics-order",
        "2" if "Dynamic2" in kwargs.get("robot_config", "") else "1",
        "--second-order-position-gain",
        str(kwargs.get("second_order_position_gain", 1.0)),
        "--second-order-velocity-gain",
        str(kwargs.get("second_order_velocity_gain", 1.0)),
        "--second-order-upper-acceleration-limit",
        str(kwargs.get("second_order_upper_acceleration_limit", 100.0)),
        "--second-order-jacobian-rate-filter",
        str(kwargs.get("second_order_jacobian_rate_filter", 0.2)),
        "--second-order-jacobian-rate-limit",
        str(kwargs.get("second_order_jacobian_rate_limit", 10.0)),
        "--second-order-curvature-gain",
        str(kwargs.get("second_order_curvature_gain", 1.0)),
        "--wbt-safety-base-blend",
        str(kwargs.get("wbt_safety_base_blend", 0.0)),
        "--arm-goal-mode",
        {
            "disabled": "default",
            "fixed": "fixed",
            "random": "random",
        }[arm_mode],
        "--num-obstacles",
        str(
            0
            if kwargs.get("obstacle_mode", "disabled") == "disabled"
            else kwargs.get("num_obstacles", 0)
        ),
        "--obstacle-query-range",
        str(kwargs.get("obstacle_query_range", 2.0)),
        "--max-nearby-obstacles",
        str(kwargs.get("max_nearby_obstacles", 16)),
        "--shared-robot-yaw-range",
        *[
            str(value)
            for value in kwargs.get(
                "shared_robot_yaw_range", (-3.141592653589793, 3.141592653589793)
            )
        ],
        "--shared-robot-keepaway",
        str(kwargs.get("shared_robot_keepaway", 0.8)),
        "--obstacle-velocity",
        str(0.005 if kwargs.get("obstacle_velocity") is None else kwargs["obstacle_velocity"]),
        "--environment-representation",
        str(kwargs.get("environment_representation", "sphere")),
        "--points-per-obstacle",
        str(kwargs.get("points_per_obstacle", 64)),
        "--minimum-points-per-obstacle",
        str(kwargs.get("minimum_points_per_obstacle", 4)),
        "--point-visual-size",
        str(kwargs.get("point_visual_size", 0.008)),
        "--mesh-latitude-segments",
        str(kwargs.get("mesh_latitude_segments", 8)),
        "--mesh-longitude-segments",
        str(kwargs.get("mesh_longitude_segments", 12)),
        "--seed",
        str(kwargs.get("seed", 20)),
        "--safe-algo",
        str(kwargs.get("safe_algo", "bypass")),
        "--minimum-distance",
        str(kwargs.get("minimum_distance", 0.05)),
        "--self-collision-minimum-distance",
        str(kwargs.get("self_collision_minimum_distance", 0.0)),
        "--safety-activation-distance",
        str(kwargs.get("safety_activation_distance", 0.15)),
        "--safety-alpha",
        str(kwargs.get("safety_alpha", 4.0)),
        "--safety-qp-iterations",
        str(kwargs.get("safety_qp_iterations", 20)),
        "--safety-slack-weight",
        str(kwargs.get("safety_slack_weight", 1000.0)),
        "--safety-reactive-gain",
        str(kwargs.get("safety_reactive_gain", 0.5)),
        "--safety-nearest-points-per-link",
        str(kwargs.get("safety_nearest_points_per_link", 2)),
        "--safety-point-chunk-size",
        str(kwargs.get("safety_point_chunk_size", 2048)),
    ]
    command.append(
        "--base-goal-relative-to-current"
        if kwargs.get("base_goal_relative_to_current", True)
        else "--no-base-goal-relative-to-current"
    )
    for option, key in (
        ("--left-arm-goal-range", "left_arm_goal_range"),
        ("--right-arm-goal-range", "right_arm_goal_range"),
    ):
        if kwargs.get(key) is not None:
            ranges = _axis_ranges(kwargs[key])
            command += [option, *(str(value) for axis in ranges for value in axis)]
    hold_arm_goals = kwargs.get("hold_arm_goals_during_locomotion")
    if hold_arm_goals is None:
        hold_arm_goals = False
    command.append(
        "--hold-arm-goals-during-locomotion"
        if hold_arm_goals
        else "--no-hold-arm-goals-during-locomotion"
    )
    if kwargs.get("base_goal_workspace_range") is not None:
        workspace = np.asarray(kwargs["base_goal_workspace_range"], dtype=float).reshape(2, 2)
        command += [
            "--base-goal-workspace-range",
            *(str(value) for axis in workspace for value in axis),
        ]
    if controller == "sonic":
        endpoints = kwargs.get("sonic_endpoints")
        if endpoints is None or len(endpoints) not in (1, num_envs):
            raise ValueError(
                "Parallel SONIC requires one shared endpoint or one endpoint per environment"
            )
        command += ["--sonic-endpoints", *map(str, endpoints)]
        timeout_ms = kwargs.get("sonic_timeout_ms")
        if timeout_ms is None:
            timeout_ms = max(100, 4 * num_envs) if len(endpoints) == 1 else 100
        command += ["--sonic-timeout-ms", str(timeout_ms)]
        locomotion_mode = kwargs.get("sonic_locomotion_mode")
        if locomotion_mode is None:
            locomotion_mode = "walk" if kwargs.get("goal_tracking_type") == "pid" else "slow"
        command += ["--sonic-locomotion-mode", str(locomotion_mode)]
    if kwargs.get("shared_workspace_range") is not None:
        command.extend(
            ["--shared-workspace-range"]
            + [str(value) for value in kwargs["shared_workspace_range"]]
        )
    if kwargs.get("object_mesh_path"):
        command.extend(
            [
                "--object-mesh-path",
                str(kwargs["object_mesh_path"]),
                "--object-mesh-scale",
                str(kwargs.get("object_mesh_scale", 1.0)),
            ]
        )
    command.append(
        "--dynamic-point-count"
        if kwargs.get("dynamic_point_count", True)
        else "--no-dynamic-point-count"
    )
    command.append(
        "--enable-self-collision"
        if kwargs.get("enable_self_collision", True)
        else "--no-enable-self-collision"
    )
    if kwargs.get("profile_frequency", False):
        command.append("--profile-safety-stages")
    if kwargs.get("render_robot_collision_volumes", False):
        command.append("--render-robot-collision-volumes")
    if kwargs.get("render_safety_trigger_constraints", False):
        command.append("--render-safety-trigger-constraints")
    if kwargs.get("render_safety_violations", False):
        command.append("--render-safety-violations")
    obstacle_range = kwargs.get("obstacle_range")
    if obstacle_range is not None:
        obstacle_range = _axis_ranges(obstacle_range)
        command += [
            "--obstacle-range",
            *(str(value) for axis in obstacle_range for value in axis),
        ]
    if kwargs.get("obstacle_size") is not None:
        command += ["--obstacle-radius", str(kwargs["obstacle_size"])]
    if kwargs.get("point_radius") is not None:
        command += ["--point-radius", str(kwargs["point_radius"])]
    command += [
        "--obstacle-goal-keepaway",
        str(kwargs.get("obstacle_goal_keepaway", 0.15)),
        "--obstacle-keepout",
        str(kwargs.get("obstacle_keepout", 0.05)),
        "--obstacle-robot-keepaway",
        str(kwargs.get("obstacle_robot_keepaway", 0.05)),
    ]
    if kwargs.get("shared_obstacle_environment", False):
        command.append("--shared-obstacle-environment")
    if not kwargs.get("reset_on_success", True):
        command.append("--no-reset-on-success")
    if not kwargs.get("reset_on_timeout", True):
        command.append("--no-reset-on-timeout")
    if _robot_uses_hands(kwargs):
        command.append("--with-hand")
    else:
        command.append("--no-with-hand")
    if base_mode == "random":
        command += ["--goal-mode", "random"]
        configured_range = kwargs.get("base_goal_range")
        yaw_range = kwargs.get("base_goal_rot_range")
        if configured_range is not None:
            configured_range = _axis_ranges(configured_range)
            yaw_range = yaw_range or DEFAULT_WBT_BASE_GOAL_ROT_RANGE
            # BenchmarkTask's third positional range is nominal marker/root
            # height. The tensor goal state is SE(2), so forward only x/y and
            # take yaw from the dedicated rotation range.
            command += [
                "--base-goal-range",
                *(str(value) for axis in configured_range[:2] for value in axis),
                str(yaw_range[0]),
                str(yaw_range[1]),
                "--base-goal-height-range",
                str(configured_range[2][0]),
                str(configured_range[2][1]),
            ]
    elif base_mode == "fixed":
        base = kwargs.get("base_goal_init") or (0.35, 0.0, 0.0)
        command += [
            "--goal-mode",
            "fixed",
            "--base-goal",
            str(base[0]),
            str(base[1]),
            "0.0",
        ]
    else:
        command += ["--goal-mode", "velocity", "--velocity-command", "0", "0", "0"]
    if kwargs.get("viewer", True):
        command += ["--visualizer", "kit", "--max_visible_envs", str(num_envs)]
    if kwargs.get("record_video_path"):
        command += [
            "--visualizer",
            "kit",
            "--max_visible_envs",
            str(num_envs),
            "--record-video-path",
            str(kwargs["record_video_path"]),
            "--record-fps",
            str(kwargs.get("record_fps") or 15.0),
            "--record-width",
            str(kwargs.get("record_width") or 1280),
            "--record-height",
            str(kwargs.get("record_height") or 720),
        ]
    if kwargs.get("isaac_device"):
        command += ["--device", str(kwargs["isaac_device"])]
    if kwargs.get("viewer_show_simulation_info", False):
        command.append("--show-simulation-info")
    viewer_config = kwargs.get("viewer_config")
    if viewer_config:
        if viewer_config.get("camera_lookat") is not None:
            command += [
                "--viewer-lookat",
                *(str(value) for value in viewer_config["camera_lookat"]),
            ]
        for option, field in (
            ("--viewer-distance", "camera_distance"),
            ("--viewer-azimuth", "camera_azimuth"),
            ("--viewer-elevation", "camera_elevation"),
        ):
            if viewer_config.get(field) is not None:
                command += [option, str(viewer_config[field])]
    print(
        f"Launching {num_envs}-environment Isaac {controller.upper()} benchmark:",
        " ".join(command),
        flush=True,
    )
    return subprocess.run(command, check=True)


def run(**kwargs):
    kwargs = _resolve_case_inputs(kwargs)
    controller = _runtime_policy(
        kwargs.get("robot_config"),
        kwargs.get("policy_config"),
    )
    kwargs = dict(kwargs)
    kwargs["runtime_policy"] = controller
    if kwargs.get("goal_tracking_type") is None and controller in ("wbt", "sport", "sonic"):
        if controller == "sonic":
            from spark_policy.composed_policy.unitree_g1.sonic_safe import (
                UnitreeG1SonicSafePolicyConfig,
            )

            policy_defaults = UnitreeG1SonicSafePolicyConfig()
        elif controller == "sport":
            from spark_policy.composed_policy.unitree_g1.sport_safe import (
                UnitreeG1SportSafePolicyConfig,
            )

            policy_defaults = UnitreeG1SportSafePolicyConfig()
        else:
            from spark_policy.composed_policy.unitree_g1.wbt_safe import (
                UnitreeG1WBTSafePolicyConfig,
            )

            policy_defaults = UnitreeG1WBTSafePolicyConfig()
        kwargs["goal_tracking_type"] = policy_defaults.goal_tracking_type
    backend = kwargs.get("backend", "mujoco")
    num_envs = int(kwargs.get("num_envs", 1))
    if num_envs < 1:
        raise ValueError("--num-envs must be at least 1")
    if backend == "isaac" and kwargs.get("isaac_device") is None:
        # Learned-policy benchmarks use one tensor execution path and device
        # for N=1 and N>1 so tuning at one environment scales without changing
        # simulator or policy numerics.
        kwargs = dict(kwargs)
        kwargs["isaac_device"] = "cuda:0" if controller in ("wbt", "sport", "sonic") else "cpu"
    if backend == "isaac" and controller in ("pid", "mobile_base"):
        forwarded = dict(kwargs)
        forwarded["device"] = forwarded.pop("isaac_device", None)
        return _run_pid_isaac_tensor_benchmark(forwarded, num_envs=num_envs)
    if backend == "mujoco" and num_envs > 1:
        if controller != "wbt":
            raise ValueError(
                "Parallel MuJoCo currently supports WBT; each environment "
                "runs in an isolated simulator process."
            )
        return _run_parallel_mujoco(kwargs, num_envs)
    if controller == "sonic":
        try:
            from example.unitree_g1.sonic_support import (
                _build_sonic_backend_command,
                _launch_sonic_backend,
                _with_sonic_runtime_defaults,
                run as run_sonic,
            )
        except ModuleNotFoundError as exc:
            if exc.name != "example":
                raise
            # Direct execution puts example/unitree_g1 on sys.path.
            from sonic_support import (
                _build_sonic_backend_command,
                _launch_sonic_backend,
                _with_sonic_runtime_defaults,
                run as run_sonic,
            )
        kwargs = _with_sonic_runtime_defaults(kwargs)
        if backend == "isaac":
            forwarded = dict(kwargs)
            endpoints = forwarded.get("sonic_endpoints")
            if forwarded["auto_launch_sonic_server"]:
                base_endpoint = str(forwarded.get("sonic_endpoint", "tcp://127.0.0.1:5560"))
                endpoints = [base_endpoint]
                server_kwargs = {**forwarded, "sonic_endpoint": base_endpoint}
                if num_envs > 1:
                    server_kwargs["sonic_server_batch_size"] = num_envs
                command, cwd = _build_sonic_backend_command(**server_kwargs)
                process = None
                try:
                    process = _launch_sonic_backend(
                        command,
                        endpoint=base_endpoint,
                        startup_timeout_s=forwarded["sonic_server_startup_timeout"],
                        cwd=cwd,
                    )
                    forwarded["sonic_endpoints"] = endpoints
                    return _run_isaac_tensor_benchmark(
                        forwarded, controller="sonic", num_envs=num_envs
                    )
                finally:
                    if process is not None:
                        process.terminate()
                        try:
                            process.wait(timeout=5.0)
                        except subprocess.TimeoutExpired:
                            process.kill()
            if endpoints is None or len(endpoints) not in (1, num_envs):
                raise ValueError(
                    "Disable auto-launch only when providing one shared Sonic "
                    "endpoint or one endpoint per environment."
                )
            forwarded["sonic_endpoints"] = endpoints
            return _run_isaac_tensor_benchmark(forwarded, controller="sonic", num_envs=num_envs)
        forwarded = {
            **resolve_benchmark_test_case(kwargs["test_case"]).parameters(),
            **kwargs,
        }
        if "Dynamic2" in forwarded["robot_config"] and forwarded.get("safety_index", "si1") in (
            None,
            "si1",
        ):
            # The monitor is still instantiated for bypass mode, so its
            # dynamics order must match the robot even when no correction is
            # requested.
            forwarded["safety_index"] = "si2"
        # The shared SONIC adapter uses the WBT configuration vocabulary.
        # Ground the generic benchmark fields instead of silently retaining
        # its TeleopTask and test-case obstacle count.
        forwarded["task_class_name"] = forwarded.pop("class_name", "BenchmarkTask")
        forwarded["task_name"] = "UnitreeG1SonicBenchmark"
        forwarded["num_obstacle_task"] = (
            0
            if forwarded.get("obstacle_mode") == "disabled"
            else int(forwarded.get("num_obstacles") or 0)
        )
        # The Sonic teleoperation config includes one keyboard-controlled
        # debug obstacle. Benchmarks must contain only obstacles declared by
        # their self-contained test-case dictionary.
        forwarded["num_obstacle_debug"] = 0
        forwarded["arm_goal_enable"] = forwarded.get("arm_goal_mode") != "disabled"
        forwarded["base_goal_enable"] = forwarded.get("base_goal_mode") != "disabled"
        forwarded["arm_goal_reach_done"] = forwarded["arm_goal_enable"]
        forwarded["base_goal_reach_done"] = forwarded["base_goal_enable"]
        forwarded["arm_goal_mode"] = (
            "Velocity" if forwarded.get("arm_goal_mode") in ("disabled", "fixed") else "Brownian"
        )
        forwarded["base_goal_mode"] = (
            "Velocity" if forwarded.get("base_goal_mode") in ("disabled", "fixed") else "Brownian"
        )
        forwarded["obstacle_mode"] = (
            "Velocity" if forwarded.get("obstacle_mode") in ("disabled", "fixed") else "Brownian"
        )
        if forwarded.get("arm_goal_mode") == "Velocity":
            if forwarded.get("goal_left_init") is not None:
                forwarded["left_arm_goal_range"] = _fixed_point_range(forwarded["goal_left_init"])
            if forwarded.get("goal_right_init") is not None:
                forwarded["right_arm_goal_range"] = _fixed_point_range(forwarded["goal_right_init"])
        if (
            forwarded.get("base_goal_mode") == "Velocity"
            and forwarded.get("base_goal_init") is not None
        ):
            forwarded["base_goal_range"] = _fixed_point_range(forwarded["base_goal_init"])
        if (
            forwarded.get("obstacle_mode") == "Velocity"
            and forwarded.get("obstacle_init") is not None
        ):
            forwarded["obstacle_range"] = _fixed_point_range(forwarded["obstacle_init"])
        for name in (
            "left_arm_goal_range",
            "right_arm_goal_range",
            "base_goal_range",
            "obstacle_range",
        ):
            if forwarded.get(name) is not None:
                forwarded[name] = _axis_ranges(forwarded[name])
        forwarded["robot_cfg"] = forwarded.pop("robot_config")
        forwarded.pop("policy_config", None)
        # The benchmark CLI uses --headless while the shared
        # MuJoCo/Isaac plant configuration consumes enable_viewer.  Keep the
        # scalar SONIC benchmark on the same lifecycle as teleop and prevent
        # benchmark runs from installing an unused keyboard controller.
        forwarded["enable_viewer"] = bool(forwarded.get("viewer", True))
        forwarded["enable_keyboard_control"] = False
        forwarded["auto_launch_sonic_server"] = bool(
            forwarded.get("auto_launch_sonic_server", False)
        )
        return run_sonic(**forwarded)
    if backend == "isaac" and controller in ("wbt", "sport"):
        if kwargs.get("safe_algo", "bypass") not in (
            "bypass",
            "ssa",
            "rssa",
            "sss",
            "rsss",
            "cbf",
            "rcbf",
            "pfm",
            "sma",
        ):
            raise ValueError(
                "Isaac tensor benchmarks support the standard SPARK safety algorithms."
            )
        return _run_isaac_tensor_benchmark(kwargs, controller=controller, num_envs=num_envs)
    if controller == "mobile_base":
        if backend != "mujoco" or num_envs != 1:
            raise ValueError("MuJoCo mobile-base currently supports one environment")
    if num_envs > 1:
        raise ValueError(
            "Parallel execution is available through the Isaac tensor backend; "
            "MuJoCo learned-policy verification uses one environment."
        )

    if controller in ("wbt", "sport"):
        from spark_pipeline.teleop.unitree_g1_whole_body_config import run as run_unitree

        arm_mode = _arm_goal_mode(kwargs)
        base_mode = kwargs.get("base_goal_mode", "disabled")
        obstacle_mode = kwargs.get("obstacle_mode", "disabled")
        forwarded = {
            **resolve_benchmark_test_case(kwargs["test_case"]).parameters(),
            **kwargs,
        }
        if "Dynamic2" in forwarded["robot_config"] and forwarded.get("safety_index", "si1") in (
            None,
            "si1",
        ):
            forwarded["safety_index"] = "si2"
            forwarded.setdefault("wbt_arm_robot_cfg", "UnitreeG1DualArmDynamic2Config")
        if arm_mode == "fixed":
            if forwarded.get("left_arm_goal_range") is None:
                forwarded["left_arm_goal_range"] = _fixed_point_range(
                    forwarded.get("goal_left_init")
                )
            if forwarded.get("right_arm_goal_range") is None:
                forwarded["right_arm_goal_range"] = _fixed_point_range(
                    forwarded.get("goal_right_init")
                )
        if base_mode == "fixed" and forwarded.get("base_goal_range") is None:
            forwarded["base_goal_range"] = _fixed_point_range(forwarded.get("base_goal_init"))
        if obstacle_mode == "fixed" and forwarded.get("obstacle_range") is None:
            forwarded["obstacle_range"] = _fixed_point_range(forwarded.get("obstacle_init"))
        if base_mode == "random" and forwarded.get("base_goal_range") is None:
            forwarded["base_goal_range"] = DEFAULT_WBT_BASE_GOAL_RANGE
        if base_mode == "random" and forwarded.get("base_goal_rot_range") is None:
            forwarded["base_goal_rot_range"] = DEFAULT_WBT_BASE_GOAL_ROT_RANGE
        for name in (
            "left_arm_goal_range",
            "right_arm_goal_range",
            "base_goal_range",
            "obstacle_range",
        ):
            if forwarded.get(name) is not None:
                forwarded[name] = _axis_ranges(forwarded[name])
        forwarded.update(
            task_class_name="BenchmarkTask",
            task_name="UnitreeG1WBTBenchmark",
            # Preserve the selected dynamics order.  The old benchmark
            # silently grounded every WBT run to Dynamic1, so a nominal D2
            # command was never actually validating the D2 robot contract.
            robot_cfg=kwargs["robot_config"],
            enable_viewer=kwargs.get("viewer", True),
            viewer_show_simulation_info=kwargs.get("viewer_show_simulation_info", False),
            enable_keyboard_control=False,
            arm_goal_enable=arm_mode != "disabled",
            base_goal_enable=base_mode != "disabled",
            arm_goal_reach_done=arm_mode != "disabled",
            base_goal_reach_done=base_mode != "disabled",
            arm_goal_size=float(kwargs.get("arm_goal_size", 0.05)),
            base_goal_size=float(kwargs.get("base_goal_size", 0.15)),
            hold_upper_body_during_locomotion=bool(
                kwargs.get("hold_arm_goals_during_locomotion", False)
            ),
            arm_goal_mode="Velocity" if arm_mode == "fixed" else "Brownian",
            base_goal_mode="Velocity" if base_mode == "fixed" else "Brownian",
            obstacle_mode=("Velocity" if obstacle_mode in ("disabled", "fixed") else "Brownian"),
            num_obstacle_task=(
                0 if obstacle_mode == "disabled" else int(kwargs.get("num_obstacles", 0))
            ),
            obstacle_velocity=(
                float(
                    0.005
                    if kwargs.get("obstacle_velocity") is None
                    else kwargs["obstacle_velocity"]
                )
                if obstacle_mode == "random"
                else float(kwargs.get("obstacle_velocity") or 0.0)
            ),
            num_obstacle_debug=int(kwargs.get("num_debug_obstacles", 0)),
            max_num_reset=kwargs.get("max_num_reset", -1),
            reset_on_success=bool(kwargs.get("reset_on_success", True)),
            continuous_goal_transition=True,
            completion_mode=forwarded.get("completion_mode", "all_enabled_goals"),
            resample_arm_goals_on_reset=forwarded.get("resample_arm_goals_on_reset", True),
            resample_base_goal_on_reset=forwarded.get("resample_base_goal_on_reset", True),
            resample_obstacles_on_reset=forwarded.get("resample_obstacles_on_reset", True),
            deterministic_scenario_sampling=True,
        )
        if controller == "sport":
            forwarded.update(
                composed_policy_class="UnitreeG1SportSafePolicy",
                executor_policy_class="UnitreeG1SportExecutorAdapter",
                # The Sport gait realizes a substantially larger physical
                # stride than WBT for the same normalized input.  Keep its
                # tracker and post-safety envelope conservative so a local
                # correction cannot request a destabilizing reversal.
                max_planar_speed=0.150,
                min_planar_speed=0.030,
                max_yaw_rate=1.00,
                max_acceleration=0.20,
                max_yaw_acceleration=2.00,
                pre_safe_loco_command_limit=[0.150, 0.120, 1.00],
                pre_safe_loco_command_rate_limit=[0.010, 0.010, 0.080],
            )
            if backend == "isaac":
                forwarded["use_wbt_motor_gains"] = True
        return run_unitree(**forwarded)

    if backend != "mujoco":
        raise ValueError(
            "The scalar PID benchmark currently uses a MuJoCo agent; use "
            "--policy-config UnitreeG1WBTSafePolicy or "
            "UnitreeG1SportSafePolicy with Isaac."
        )
    pipeline = Pipeline(build_config(**kwargs))
    pipeline.run(save_path=kwargs.get("save_path"))
    return pipeline


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--robot-config",
        choices=(
            "UnitreeG1RightArmDynamic1Config",
            "UnitreeG1RightArmDynamic2Config",
            "UnitreeG1RightArmWithHandDynamic1Config",
            "UnitreeG1RightArmWithHandDynamic2Config",
            "UnitreeG1DualArmDynamic1Config",
            "UnitreeG1DualArmDynamic2Config",
            "UnitreeG1DualArmWithHandDynamic1Config",
            "UnitreeG1DualArmWithHandDynamic2Config",
            "UnitreeG1FixedBaseDynamic1Config",
            "UnitreeG1FixedBaseDynamic2Config",
            "UnitreeG1FixedBaseWithHandDynamic1Config",
            "UnitreeG1FixedBaseWithHandDynamic2Config",
            "UnitreeG1MobileBaseDynamic1Config",
            "UnitreeG1MobileBaseDynamic2Config",
            "UnitreeG1MobileBaseWithHandDynamic1Config",
            "UnitreeG1MobileBaseWithHandDynamic2Config",
            "UnitreeG1WholeBodyDynamic1Config",
            "UnitreeG1WholeBodyDynamic2Config",
            "UnitreeG1WholeBodyWithHandDynamic1Config",
            "UnitreeG1WholeBodyWithHandDynamic2Config",
        ),
        default="UnitreeG1FixedBaseWithHandDynamic1Config",
        help="Robot state/model contract; benchmark composition is inferred from it.",
    )
    parser.add_argument(
        "--policy-config",
        choices=tuple(POLICY_RUNTIME),
        default="TeleopPIDPolicy",
        help="Registered spark_policy class used by the benchmark.",
    )
    parser.add_argument("--sonic-endpoint", default="tcp://127.0.0.1:5560")
    parser.add_argument("--sonic-endpoints", nargs="+", default=None)
    parser.add_argument(
        "--auto-launch-sonic-server",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="Override the Sonic policy config; auto-launch is enabled by default.",
    )
    parser.add_argument(
        "--sonic-deploy-root",
        default=None,
        help=("Override SPARK_SONIC_DEPLOY_ROOT or the standard sibling SONIC checkout."),
    )
    parser.add_argument("--sonic-timeout-ms", type=int, default=None)
    parser.add_argument("--sonic-policy-precision", choices=("16", "32"), default=None)
    parser.add_argument("--sonic-planner-precision", choices=("16", "32"), default=None)
    parser.add_argument(
        "--sonic-server-startup-timeout",
        type=float,
        default=None,
        help=(
            "Seconds to wait for an auto-launched Sonic server. Increase this "
            "when TensorRT must build a new batch-size-specific engine cache."
        ),
    )
    parser.add_argument("--sonic-fallback-mode", choices=("hold", "default", "raise"), default=None)
    parser.add_argument("--sonic-planar-speed-scale", type=float, default=None)
    parser.add_argument(
        "--sonic-locomotion-mode",
        choices=("slow", "walk", "run", "hybrid"),
        default=None,
        help=(
            "SONIC gait used away from the goal. Near-goal slowdown may still "
            "select slow walk for accurate stopping."
        ),
    )
    parser.add_argument(
        "--goal-tracking-type",
        choices=("legged", "pid"),
        default=None,
        help="Select the policy-level base-goal tracker for Sonic and WBT.",
    )
    parser.add_argument("--base-position-kp", type=float, nargs=3, default=None)
    parser.add_argument("--base-velocity-kd", type=float, nargs=2, default=None)
    parser.add_argument("--base-planar-velocity-limit", type=float, default=None)
    parser.add_argument(
        "--override-sonic-upper-body-motor-gains",
        action=argparse.BooleanOptionalAction,
        default=None,
    )
    parser.add_argument("--upper-body-target-filter-gain", type=float, default=None)
    parser.add_argument("--locomotion-control-weight", type=float, default=None)
    parser.add_argument("--upper-body-control-weight", type=float, default=None)
    parser.add_argument("--waist-control-weight", type=float, default=None)
    parser.add_argument("--height-control-weight", type=float, default=None)
    parser.add_argument("--backend", choices=("mujoco", "isaac"), default="mujoco")
    parser.add_argument(
        "--test-case",
        choices=TASK_CASE_LIST,
        default="random_arm_nominal",
        help="Robot/backend-independent task and episode-reset specification.",
    )
    parser.add_argument(
        "--num-envs",
        type=int,
        default=1,
        help=(
            "Number of environments. Isaac learned policies use the same tensor "
            "adapter for one or many environments."
        ),
    )
    parser.add_argument(
        "--render-every",
        type=int,
        default=None,
        help="Present near 10 Hz by default; control and physics remain at 50 Hz.",
    )
    parser.add_argument("--record-video-path", default=None, help=argparse.SUPPRESS)
    parser.add_argument("--record-gif-path", default=None, help=argparse.SUPPRESS)
    parser.add_argument("--record-duration", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--record-fps", type=float, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--record-width", type=int, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--record-height", type=int, default=None, help=argparse.SUPPRESS)
    parser.add_argument("--viewer-lookat", type=float, nargs=3, default=None)
    parser.add_argument("--viewer-distance", type=float, default=None)
    parser.add_argument("--viewer-azimuth", type=float, default=None)
    parser.add_argument("--viewer-elevation", type=float, default=None)
    parser.add_argument("--viewer-vertical-fov", type=float, default=None)
    parser.add_argument(
        "--isaac-render-quality",
        choices=("performance", "quality", "cinematic"),
        default="performance",
        help="Isaac rendering preset; it does not change simulation or control settings.",
    )
    parser.add_argument(
        "--wbt-warmup-steps",
        type=int,
        default=20,
        help="Cold-start WBT stabilization; not repeated when only goals change.",
    )
    parser.add_argument("--wbt-min-translation-speed", type=float, default=0.05)
    parser.add_argument("--wbt-min-yaw-rate", type=float, default=0.08)
    parser.add_argument(
        "--mode",
        choices=("random-arm-no-collision", "test-case"),
        default=None,
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--test-case-name",
        choices=LEGACY_TASK_CASE_LIST,
        default=None,
        help="Deprecated robot-prefixed alias; prefer --test-case.",
    )
    parser.add_argument(
        "--with-hand",
        action=argparse.BooleanOptionalAction,
        default=None,
        help=(
            "Compatibility override. By default, hand presence is determined "
            "only by whether --robot-config contains 'WithHand'."
        ),
    )
    parser.add_argument(
        "--headless",
        dest="viewer",
        action="store_false",
        default=True,
        help="Disable the viewer; the window is enabled by default.",
    )
    parser.add_argument(
        "--show-simulation-info",
        dest="viewer_show_simulation_info",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Show the dynamics information panel in the MuJoCo or Isaac viewer.",
    )
    parser.add_argument("--use-sim-dynamics", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--real-time", action=argparse.BooleanOptionalAction, default=True)
    parser.add_argument("--grippers-closed", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--arm-goal-velocity", type=float, default=0.0)
    parser.add_argument("--arm-goal-mode", choices=("disabled", "fixed", "random"), default=None)
    parser.add_argument(
        "--arm-goal-enable",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="Enable arm goals; --no-arm-goal-enable overrides arm-goal-mode.",
    )
    parser.add_argument("--base-goal-mode", choices=("disabled", "fixed", "random"), default=None)
    parser.add_argument("--obstacle-mode", choices=("disabled", "fixed", "random"), default=None)
    parser.add_argument("--goal-left-init", type=float, nargs=3, default=None)
    parser.add_argument("--goal-right-init", type=float, nargs=3, default=None)
    parser.add_argument("--base-goal-init", type=float, nargs=3, default=None)
    parser.add_argument("--left-arm-goal-range", type=float, nargs=6, default=None)
    parser.add_argument("--right-arm-goal-range", type=float, nargs=6, default=None)
    parser.add_argument("--base-goal-range", type=float, nargs=6, default=None)
    parser.add_argument(
        "--base-goal-minimum-distance",
        type=float,
        default=None,
        help="Override the test case's minimum planar goal displacement.",
    )
    parser.add_argument(
        "--base-goal-workspace-range",
        type=float,
        nargs=4,
        default=None,
        metavar=("XMIN", "XMAX", "YMIN", "YMAX"),
        help="Override the test case's absolute XY base-goal workspace.",
    )
    parser.add_argument("--base-goal-rot-range", type=float, nargs=2, default=None)
    parser.add_argument(
        "--base-goal-velocity",
        type=float,
        default=None,
        help="Override the test case's planar base-goal speed in m/s.",
    )
    parser.add_argument(
        "--shared-obstacle-environment",
        action=argparse.BooleanOptionalAction,
        default=False,
    )
    parser.add_argument("--obstacle-query-range", type=float, default=2.0)
    parser.add_argument("--max-nearby-obstacles", type=int, default=16)
    parser.add_argument(
        "--shared-robot-yaw-range",
        type=float,
        nargs=2,
        default=(-3.141592653589793, 3.141592653589793),
    )
    parser.add_argument(
        "--shared-workspace-range",
        type=float,
        nargs=6,
        default=None,
        help=(
            "Optional absolute shared-scene bounds. By default, the test-case "
            "workspace X-Y area is automatically multiplied by num-envs."
        ),
    )
    parser.add_argument("--shared-robot-keepaway", type=float, default=0.8)
    parser.add_argument("--num-debug-obstacles", type=int, default=0)
    parser.add_argument(
        "--num-obstacles",
        type=int,
        default=None,
        help="Override the task case's number of safety obstacles.",
    )
    parser.add_argument("--obstacle-init", type=float, nargs=3, default=None)
    parser.add_argument("--obstacle-range", type=float, nargs=6, default=None)
    parser.add_argument("--obstacle-size", type=float, default=None)
    parser.add_argument("--obstacle-velocity", type=float, default=None)
    parser.add_argument(
        "--environment-representation",
        choices=("sphere", "point_cloud", "mesh"),
        default="sphere",
    )
    parser.add_argument("--points-per-obstacle", type=int, default=64)
    parser.add_argument(
        "--dynamic-point-count",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument(
        "--regenerate-point-cloud-every-step",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Resample point locations and optional valid count every task step.",
    )
    parser.add_argument("--minimum-points-per-obstacle", type=int, default=4)
    parser.add_argument("--point-radius", type=float, default=None)
    parser.add_argument("--point-visual-size", type=float, default=0.008)
    parser.add_argument(
        "--max-visualized-points",
        type=int,
        default=200,
        help="Viewer-only point budget; safety still consumes the full cloud.",
    )
    parser.add_argument("--mesh-latitude-segments", type=int, default=8)
    parser.add_argument("--mesh-longitude-segments", type=int, default=12)
    parser.add_argument("--object-mesh-path", default=None)
    parser.add_argument("--object-mesh-scale", type=float, default=1.0)
    parser.add_argument("--arm-goal-size", type=float, default=None)
    # Accepted during migration from the former joint-target benchmark. They
    # no longer affect Cartesian task generation or completion.
    parser.add_argument("--arm-goal-offset", type=float, default=0.25, help=argparse.SUPPRESS)
    parser.add_argument("--arm-joint-tolerance", type=float, default=0.08, help=argparse.SUPPRESS)
    parser.add_argument(
        "--reset-on-success",
        action=argparse.BooleanOptionalAction,
        default=None,
        help="In batched Isaac runs, reset and resample each successful environment independently.",
    )
    parser.add_argument(
        "--reset-on-timeout",
        action=argparse.BooleanOptionalAction,
        default=None,
    )
    parser.add_argument("--fall-height-threshold", type=float, default=None)
    parser.add_argument("--base-goal-size", type=float, default=None)
    parser.add_argument(
        "--base-goal-yaw-size",
        type=float,
        default=None,
        help="Required absolute base yaw error in radians before success.",
    )
    parser.add_argument("--wbt-max-forward-speed", type=float, default=0.12)
    parser.add_argument("--wbt-max-lateral-speed", type=float, default=0.10)
    parser.add_argument("--wbt-max-yaw-rate", type=float, default=0.30)
    parser.add_argument("--wbt-command-acceleration", type=float, default=0.30)
    parser.add_argument("--wbt-yaw-acceleration", type=float, default=0.75)
    parser.add_argument("--wbt-base-settle-steps", type=int, default=50)
    parser.add_argument("--second-order-position-gain", type=float, default=1.0)
    parser.add_argument("--second-order-velocity-gain", type=float, default=1.0)
    parser.add_argument("--second-order-upper-acceleration-limit", type=float, default=100.0)
    parser.add_argument("--second-order-jacobian-rate-filter", type=float, default=0.2)
    parser.add_argument("--second-order-jacobian-rate-limit", type=float, default=10.0)
    parser.add_argument("--second-order-curvature-gain", type=float, default=1.0)
    parser.add_argument("--wbt-safety-base-blend", type=float, default=0.0)
    parser.add_argument(
        "--sport-policy-threads",
        type=int,
        default=1,
        help="CPU threads used by the small Sport Mode TorchScript policy.",
    )
    parser.add_argument(
        "--hold-arm-goals-during-locomotion",
        action=argparse.BooleanOptionalAction,
        default=None,
        help=(
            "Hold Cartesian arm goals until base locomotion finishes. This is "
            "disabled by default so MuJoCo and Isaac track arms while walking."
        ),
    )
    parser.add_argument("--seed", type=int, default=None)
    parser.add_argument("--max-episode-length", type=int, default=None)
    parser.add_argument("--max-num-steps", type=int, default=None)
    parser.add_argument("--max-num-reset", type=int, default=None)
    parser.add_argument("--save-path", default=None)
    parser.add_argument("--enable-logger", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--profile-frequency", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument(
        "--viewer-hz",
        type=float,
        default=None,
        help="WBT presentation rate; defaults to its simulated control rate.",
    )
    parser.add_argument(
        "--isaac-device",
        default=None,
        help="Isaac tensor physics/policy device; learned policies default to cuda:0.",
    )
    parser.add_argument(
        "--isaac-physics-dt",
        type=float,
        default=None,
        help="Tensor Isaac physics step; defaults to the selected robot contract.",
    )
    parser.add_argument(
        "--isaac-control-decimation",
        type=int,
        default=None,
        help="Tensor Isaac physics steps per action; defaults to the robot contract.",
    )
    parser.add_argument(
        "--safe-algo",
        choices=("bypass", "ssa", "rssa", "sss", "rsss", "cbf", "rcbf", "pfm", "sma"),
        default="rssa",
        help=(
            "Safety filter for obstacle benchmarks. The nominal no-obstacle "
            "case always uses bypass."
        ),
    )
    parser.add_argument(
        "--safety-index",
        choices=("si1", "si2", "si2nn"),
        default=None,
        help="Collision safety-index order; defaults to si1/si2 from the robot dynamics order.",
    )
    parser.add_argument("--minimum-distance", type=float, default=None)
    parser.add_argument(
        "--self-collision-minimum-distance",
        type=float,
        default=0.0,
    )
    parser.add_argument("--safety-activation-distance", type=float, default=None)
    parser.add_argument("--safety-alpha", type=float, default=4.0)
    parser.add_argument("--safety-qp-iterations", type=int, default=20)
    parser.add_argument("--safety-slack-weight", type=float, default=1000.0)
    parser.add_argument("--safety-reactive-gain", type=float, default=0.5)
    parser.add_argument("--safety-nearest-points-per-link", type=int, default=2)
    parser.add_argument("--safety-point-chunk-size", type=int, default=2048)
    parser.add_argument("--obstacle-goal-keepaway", type=float, default=0.15)
    parser.add_argument("--obstacle-robot-keepaway", type=float, default=None)
    parser.add_argument(
        "--enable-self-collision",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Apply self-collision checking in both dual-arm IK and safety filtering.",
    )
    parser.add_argument(
        "--render-robot-collision-volumes",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument(
        "--render-safety-trigger-constraints",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument(
        "--render-safety-violations",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    return parser


if __name__ == "__main__":
    run(**vars(build_parser().parse_args()))
