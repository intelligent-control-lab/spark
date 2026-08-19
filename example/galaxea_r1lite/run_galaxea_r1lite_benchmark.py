#!/usr/bin/env python3
"""Run R1 Lite arm and mobile-base safety benchmarks."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys

from spark_pipeline import (
    GalaxeaR1LiteFixedBaseBenchmarkPipelineConfig as PipelineConfig,
    TeleopPipeline as Pipeline,
    apply_benchmark_test_case,
    run_simulation_pipeline,
)
from spark_robot import get_agent_class_name


ROBOT_CONFIGS = (
    "GalaxeaR1LiteRightArmDynamic1CollisionConfig",
    "GalaxeaR1LiteDualArmDynamic1CollisionConfig",
    "GalaxeaR1LiteMobileBaseDynamic1CollisionConfig",
    "GalaxeaR1LiteFixedBaseDynamic1CollisionConfig",
)
TEST_CASES = (
    "arm_goal_static_v0",
    "arm_goal_static_v1",
    "whole_goal_static_v0",
    "whole_goal_static_v1",
)
TEST_CASE_ALIASES = {
    "v0": "arm_goal_static_v0",
    "v1": "arm_goal_static_v1",
    "right_arm_goal_static_v0": "arm_goal_static_v0",
    "right_arm_goal_static_v1": "arm_goal_static_v1",
}
ROBOT_GROUNDING = {
    ROBOT_CONFIGS[0]: {"use_dual_arm": False, "mobile_base": False},
    ROBOT_CONFIGS[1]: {"use_dual_arm": True, "mobile_base": False},
    ROBOT_CONFIGS[2]: {"use_dual_arm": True, "mobile_base": True},
    ROBOT_CONFIGS[3]: {"use_dual_arm": True, "mobile_base": False},
}
# The shared benchmark ranges are expressed in a common robot-local frame.
# R1's shoulder/gripper height requires this embodiment-local translation.
BENCHMARK_GOAL_OFFSET = (0.25, 0.0, 0.90)
BENCHMARK_GOAL_ROTATION = (
    (0.0, 0.0, -1.0),
    (-1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
)
_GROUNDING_ENV = "SPARK_MANIPULATOR_BENCHMARK_GROUNDING"


def _canonical_test_case(name: str) -> str:
    return TEST_CASE_ALIASES.get(name, name)


def _grounding(test_case: str, robot_config: str | None):
    name = _canonical_test_case(test_case)
    if name not in TEST_CASES:
        raise ValueError(f"Unknown benchmark test case: {name!r}")
    mobile_case = name.startswith("whole_goal_")
    selected = robot_config or (ROBOT_CONFIGS[2] if mobile_case else ROBOT_CONFIGS[0])
    grounding = ROBOT_GROUNDING[selected]
    if mobile_case != grounding["mobile_base"]:
        raise ValueError(
            f"{name} requires the mobile-base configuration"
            if mobile_case
            else f"{name} requires a right-arm or dual-arm configuration"
        )
    return name, selected, {"shared_case": name, "robot_config": selected, **grounding}


def _offset_arm_goal_ranges(task):
    for field in ("right_arm_goal_range", "left_arm_goal_range"):
        setattr(
            task,
            field,
            [
                (
                    float(low) + BENCHMARK_GOAL_OFFSET[axis],
                    float(high) + BENCHMARK_GOAL_OFFSET[axis],
                )
                for axis, (low, high) in enumerate(getattr(task, field))
            ],
        )


def _configure_safety(cfg, *, robot_config: str, safe_algo, minimum_distance):
    import spark_robot

    obstacle_count = int(cfg.env.task.num_obstacle_task)
    safe_algo = safe_algo or ("bypass" if obstacle_count == 0 else "rssa")
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
        safe_cfg.control_weight = [1.0] * len(getattr(spark_robot, robot_config)().Control)

    index_cfg = cfg.policy.safe_controller.safety_index
    index_cfg.class_name = "FirstOrderCollisionSafetyIndex"
    index_cfg.enable_self_collision = False
    index_cfg.min_distance["environment"] = float(
        getattr(cfg.env.task, "minimum_distance", 0.05)
        if minimum_distance is None
        else minimum_distance
    )
    index_cfg.min_distance["self"] = 0.0
    return safe_algo


def build_config(**kwargs):
    test_case, robot_config, grounding = _grounding(
        kwargs.get("test_case", "arm_goal_static_v0"), kwargs.get("robot_config")
    )
    backend = kwargs.get("backend", "mujoco")
    cfg = PipelineConfig()
    apply_benchmark_test_case(cfg, grounding["shared_case"])
    cfg.robot.cfg.class_name = robot_config
    cfg.env.task.task_name = test_case
    cfg.env.task.use_dual_arm = bool(grounding["use_dual_arm"])
    _offset_arm_goal_ranges(cfg.env.task)
    cfg.env.task.right_arm_goal_rotation = BENCHMARK_GOAL_ROTATION
    cfg.env.task.left_arm_goal_rotation = BENCHMARK_GOAL_ROTATION
    cfg.env.task.arm_goal_position_only = True
    if grounding["mobile_base"]:
        # This is a display height only for a planar base; keep its marker at
        # the R1 platform instead of the legged-robot pelvis height.
        cfg.env.task.base_goal_range = [*cfg.env.task.base_goal_range[:2], (0.15, 0.15)]
    cfg.env.task.fall_height_threshold = None
    cfg.env.task.max_episode_length = int(kwargs.get("max_episode_length", 1000))

    cfg.env.agent.class_name = get_agent_class_name(robot_config, backend=backend)
    cfg.env.agent.dynamics_backend = kwargs.get("dynamics_backend", "simulator")
    cfg.env.agent.use_sim_dynamics = cfg.env.agent.dynamics_backend == "simulator"
    cfg.env.agent.enable_viewer = bool(kwargs.get("viewer", True))
    cfg.env.agent.viewer_show_simulation_info = bool(
        kwargs.get("viewer_show_simulation_info", False)
    )
    cfg.env.agent.enable_keyboard_control = False
    cfg.env.agent.enable_hand_control = True
    cfg.env.agent.real_time = bool(kwargs.get("real_time", False))
    cfg.env.agent.device = kwargs.get("device") or "cpu"
    cfg.env.agent.obstacle_debug = {**cfg.env.agent.obstacle_debug, "num_obstacle": 0}
    import spark_robot

    timing = getattr(spark_robot, robot_config)().simulator_dynamics
    cfg.env.agent.dt = float(timing.physics_dt if kwargs.get("dt") is None else kwargs["dt"])
    cfg.env.agent.control_decimation = int(
        timing.control_decimation
        if kwargs.get("control_decimation") is None
        else kwargs["control_decimation"]
    )
    cfg.env.task.dt = cfg.env.agent.dt * cfg.env.agent.control_decimation
    cfg.policy.nominal_controller.position_kp = 1.0
    cfg.policy.nominal_controller.velocity_kd = 0.1
    cfg.env.agent.sim_use_bias_compensation = True

    cfg.max_num_reset = int(kwargs.get("num_resets", 10))
    max_num_steps = kwargs.get("max_num_steps")
    cfg.max_num_steps = (
        int(max_num_steps)
        if max_num_steps is not None
        else cfg.env.task.max_episode_length * cfg.max_num_reset
    )
    cfg.render_every = max(1, int(kwargs.get("render_every") or 1))
    cfg.render_robot_collision_volumes = bool(kwargs.get("render_robot_collision_volumes", True))
    cfg.enable_logger = False
    cfg.enable_plotter = False
    cfg.enable_safe_zone_render = False
    cfg.metric_selection.dist_goal_arm = True
    cfg.metric_selection.dist_goal_base = bool(grounding["mobile_base"])
    cfg.metric_selection.dist_robot_to_env = bool(cfg.env.task.num_obstacle_task)
    cfg.metric_selection.seed = True
    cfg.metric_selection.done = True
    cfg.metric_selection.trigger_safe_controller = bool(cfg.env.task.num_obstacle_task)
    _configure_safety(
        cfg,
        robot_config=robot_config,
        safe_algo=kwargs.get("safe_algo"),
        minimum_distance=kwargs.get("minimum_distance"),
    )
    return cfg


def _run_tensor_benchmark(kwargs):
    test_case, robot_config, grounding = _grounding(
        kwargs.get("test_case", "arm_goal_static_v0"), kwargs.get("robot_config")
    )
    configured = build_config(**kwargs)
    if kwargs.get("dynamics_backend", "simulator") != "simulator":
        raise ValueError("The batched Isaac benchmark requires simulator-owned dynamics")
    runtime = (
        Path(__file__).resolve().parents[2]
        / "pipeline"
        / "spark_pipeline"
        / "autonomy"
        / "isaac_tensor_manipulator_benchmark.py"
    )
    num_envs = int(kwargs.get("num_envs", 1))
    num_resets = int(kwargs.get("num_resets", 10))
    max_episode_length = int(kwargs.get("max_episode_length", 1000))
    num_steps = kwargs.get("max_num_steps")
    if num_steps is None:
        num_steps = -1 if num_resets < 0 else num_resets * max_episode_length
    import spark_robot

    timing = getattr(spark_robot, robot_config)().simulator_dynamics
    dt = float(timing.physics_dt if kwargs.get("dt") is None else kwargs["dt"])
    decimation = int(
        timing.control_decimation
        if kwargs.get("control_decimation") is None
        else kwargs["control_decimation"]
    )
    render_every = kwargs.get("render_every")
    if render_every is None:
        render_every = max(1, int(math.ceil(1.0 / (20.0 * dt * decimation))))
    command = [
        sys.executable,
        str(runtime),
        "--robot-config",
        robot_config,
        "--test-case",
        grounding["shared_case"],
        "--display-test-case",
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
        str(num_steps),
        "--dt",
        str(dt),
        "--control-decimation",
        str(decimation),
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
    for key in ("device", "safe_algo", "minimum_distance", "activation_distance"):
        if kwargs.get(key) is not None:
            command += ["--" + key.replace("_", "-"), str(kwargs[key])]
    command.append(
        "--render-robot-collision-volumes"
        if kwargs.get("render_robot_collision_volumes", True)
        else "--no-render-robot-collision-volumes"
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
    environment = os.environ.copy()
    environment[_GROUNDING_ENV] = json.dumps(
        {
            **grounding,
            "right_arm_goal_range": configured.env.task.right_arm_goal_range,
            "left_arm_goal_range": configured.env.task.left_arm_goal_range,
            "obstacle_range": configured.env.task.obstacle_range,
            "position_kp": configured.policy.nominal_controller.position_kp,
            "velocity_kd": configured.policy.nominal_controller.velocity_kd,
            "arm_goal_rotation": BENCHMARK_GOAL_ROTATION,
            "arm_goal_position_only": configured.env.task.arm_goal_position_only,
            "right_ee_body": "right_gripper_link",
            "left_ee_body": "left_gripper_link",
            # The gripper body origin is already 0.08165 m beyond joint 6.
            "ee_local_offset": [0.06835, 0.0, 0.0],
            # Right/dual-arm agents leave the torso outside the configured
            # control vector, while the mobile benchmark deliberately holds
            # it at the stable nominal height. The fixed-base configuration
            # owns these joints and must retain them for whole-upper-body IK.
            "hold_joint_names": (
                []
                if "FixedBase" in robot_config
                else ["torso_joint1", "torso_joint2", "torso_joint3"]
            ),
            "viewer_config": kwargs.get("viewer_config")
            or dict(getattr(configured.env.agent, "viewer_config", {})),
        }
    )
    print(f"Launching {num_envs}-environment R1 Lite Isaac benchmark:", " ".join(command))
    return subprocess.run(command, check=True, env=environment)


def run(**kwargs):
    backend = kwargs.get("backend", "mujoco")
    num_envs = int(kwargs.get("num_envs", 1))
    if backend == "isaac" and kwargs.get("dynamics_backend", "simulator") == "simulator":
        return _run_tensor_benchmark(kwargs)
    if num_envs != 1:
        raise ValueError("Multiple environments require the Isaac simulator tensor runtime")
    return run_simulation_pipeline(
        build_config(**kwargs),
        backend=backend,
        pipeline_class=Pipeline,
        save_path=kwargs.get("save_path"),
    )


def _parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--backend", choices=("mujoco", "isaac"), default="mujoco")
    parser.add_argument("--robot-config", "--robot-cfg", dest="robot_config", choices=ROBOT_CONFIGS)
    parser.add_argument(
        "--test-case",
        choices=(*TEST_CASES, *TEST_CASE_ALIASES),
        default="arm_goal_static_v0",
    )
    parser.add_argument("--num-envs", type=int, default=1)
    parser.add_argument("--num-resets", type=int, default=10)
    parser.add_argument("--max-episode-length", type=int, default=1000)
    parser.add_argument("--max-num-steps", type=int)
    parser.add_argument("--dynamics-backend", choices=("simulator", "model"), default="simulator")
    parser.add_argument("--device")
    parser.add_argument("--headless", dest="viewer", action="store_false", default=True)
    parser.add_argument(
        "--show-simulation-info",
        dest="viewer_show_simulation_info",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Show the dynamics information panel in the MuJoCo or Isaac viewer.",
    )
    parser.add_argument("--real-time", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--render-every", type=int)
    parser.add_argument("--dt", type=float)
    parser.add_argument("--control-decimation", type=int)
    parser.add_argument("--safe-algo", choices=("bypass", "ssa", "rssa"))
    parser.add_argument("--minimum-distance", type=float)
    parser.add_argument(
        "--activation-distance",
        type=float,
        help="Override the shared test case's Isaac safety activation distance.",
    )
    parser.add_argument(
        "--render-robot-collision-volumes",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--max-visualized-collision-envs", type=int, default=1)
    parser.add_argument("--profile-frequency", action="store_true")
    parser.add_argument("--save-path")
    return parser


def main(argv=None):
    args = vars(_parser().parse_args(argv))
    if args["num_envs"] < 1:
        raise SystemExit("--num-envs must be positive")
    if args["num_resets"] == 0 or args["num_resets"] < -1:
        raise SystemExit("--num-resets must be positive or -1")
    if args["max_episode_length"] < 1:
        raise SystemExit("--max-episode-length must be positive")
    run(**args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
