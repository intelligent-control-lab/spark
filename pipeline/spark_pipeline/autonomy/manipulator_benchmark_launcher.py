"""Shared launcher utilities for fixed-base manipulator benchmarks."""

from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import subprocess
import sys

from spark_pipeline import apply_benchmark_test_case
from spark_robot import get_agent_class_name


TEST_CASES = ("arm_goal_static_v0", "arm_goal_static_v1")
TEST_CASE_ALIASES = {
    "v0": "arm_goal_static_v0",
    "v1": "arm_goal_static_v1",
    "right_arm_goal_static_v0": "arm_goal_static_v0",
    "right_arm_goal_static_v1": "arm_goal_static_v1",
}
_GROUNDING_ENV = "SPARK_MANIPULATOR_BENCHMARK_GROUNDING"


def canonical_test_case(name: str) -> str:
    """Resolve concise and single-arm-compatible case aliases."""

    return TEST_CASE_ALIASES.get(name, name)


def select_grounding(test_case, robot_config, robot_grounding, default_robot_config):
    """Validate a robot-local fixed-base benchmark selection."""

    name = canonical_test_case(test_case)
    if name not in TEST_CASES:
        raise ValueError(f"Unknown fixed-base benchmark test case: {name!r}")
    selected = robot_config or default_robot_config
    if selected not in robot_grounding:
        raise ValueError(f"Unsupported robot configuration: {selected!r}")
    return (
        name,
        selected,
        {
            "shared_case": name,
            "robot_config": selected,
            "mobile_base": False,
            **robot_grounding[selected],
        },
    )


def offset_arm_goal_ranges(task, offset):
    """Translate shared task ranges into an embodiment-local workspace."""

    for field in ("right_arm_goal_range", "left_arm_goal_range"):
        setattr(
            task,
            field,
            [
                (float(low) + offset[axis], float(high) + offset[axis])
                for axis, (low, high) in enumerate(getattr(task, field))
            ],
        )


def configured_physical_self_collision(robot_config):
    """Read the robot-owned PhysX link-mesh self-contact setting."""

    import spark_robot

    robot_cfg = getattr(spark_robot, robot_config)()
    isaac_spec = getattr(robot_cfg, "isaac_articulation", None)
    return bool(getattr(isaac_spec, "allow_self_collision", False))


def configure_safety(
    cfg,
    *,
    robot_config,
    safe_algo,
    minimum_distance,
    enable_self_collision=False,
):
    """Apply the shared first-order scalar safety contract."""

    import spark_robot

    obstacle_count = int(cfg.env.task.num_obstacle_task)
    self_collision_safety = bool(enable_self_collision)
    safe_algo = safe_algo or ("rssa" if obstacle_count or self_collision_safety else "bypass")
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
    index_cfg.enable_self_collision = self_collision_safety
    index_cfg.min_distance["environment"] = float(
        getattr(cfg.env.task, "minimum_distance", 0.05)
        if minimum_distance is None
        else minimum_distance
    )
    index_cfg.min_distance["self"] = 0.0


def build_fixed_base_config(
    kwargs,
    *,
    pipeline_config_type,
    robot_grounding,
    default_robot_config,
    goal_offset,
    goal_rotation,
    arm_goal_position_only=True,
    arm_goal_orientation_size=0.1,
    arm_goal_minimum_distance=0.0,
    right_arm_goal_range=None,
    left_arm_goal_range=None,
):
    """Build a scalar MuJoCo/Isaac config with shared benchmark semantics."""

    test_case, robot_config, grounding = select_grounding(
        kwargs.get("test_case", "arm_goal_static_v0"),
        kwargs.get("robot_config"),
        robot_grounding,
        default_robot_config,
    )
    backend = kwargs.get("backend", "mujoco")
    cfg = pipeline_config_type()
    apply_benchmark_test_case(cfg, grounding["shared_case"])
    cfg.robot.cfg.class_name = robot_config
    cfg.env.task.task_name = test_case
    cfg.env.task.use_dual_arm = bool(grounding["use_dual_arm"])
    offset_arm_goal_ranges(cfg.env.task, goal_offset)
    if right_arm_goal_range is not None:
        cfg.env.task.right_arm_goal_range = [list(bounds) for bounds in right_arm_goal_range]
    if left_arm_goal_range is not None:
        cfg.env.task.left_arm_goal_range = [list(bounds) for bounds in left_arm_goal_range]
    cfg.env.task.right_arm_goal_rotation = goal_rotation
    cfg.env.task.left_arm_goal_rotation = goal_rotation
    cfg.env.task.arm_goal_position_only = bool(arm_goal_position_only)
    cfg.env.task.arm_goal_orientation_size = float(arm_goal_orientation_size)
    cfg.env.task.arm_goal_minimum_distance = float(arm_goal_minimum_distance)
    cfg.env.task.arm_goal_pair_keepout = float(grounding.get("arm_goal_pair_keepout", 0.0))
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
    cfg.env.agent.allow_self_collision = configured_physical_self_collision(robot_config)
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
    cfg.policy.nominal_controller.position_kp = float(grounding.get("position_kp", 1.0))
    cfg.policy.nominal_controller.velocity_kd = float(grounding.get("velocity_kd", 0.1))
    cfg.env.agent.sim_use_bias_compensation = True

    cfg.max_num_reset = int(kwargs.get("num_resets", 10))
    max_num_steps = kwargs.get("max_num_steps")
    cfg.max_num_steps = (
        int(max_num_steps)
        if max_num_steps is not None
        else cfg.env.task.max_episode_length * cfg.max_num_reset
    )
    cfg.render_every = max(1, int(kwargs.get("render_every") or 1))
    cfg.profile_frequency = bool(kwargs.get("profile_frequency", False))
    cfg.render_robot_collision_volumes = bool(kwargs.get("render_robot_collision_volumes", True))
    cfg.enable_logger = False
    cfg.enable_plotter = False
    cfg.enable_safe_zone_render = False
    cfg.metric_selection.dist_goal_arm = True
    cfg.metric_selection.dist_goal_base = False
    cfg.metric_selection.dist_robot_to_env = bool(cfg.env.task.num_obstacle_task)
    cfg.metric_selection.seed = True
    cfg.metric_selection.done = True
    cfg.metric_selection.trigger_safe_controller = bool(
        cfg.env.task.num_obstacle_task or kwargs.get("enable_self_collision", False)
    )
    configure_safety(
        cfg,
        robot_config=robot_config,
        safe_algo=kwargs.get("safe_algo"),
        minimum_distance=kwargs.get("minimum_distance"),
        enable_self_collision=kwargs.get("enable_self_collision", False),
    )
    return cfg, grounding


def run_tensor_benchmark(
    kwargs,
    *,
    configured,
    grounding,
    goal_rotation,
    display_name,
):
    """Launch the shared Isaac tensor runtime with robot-local grounding."""

    if kwargs.get("dynamics_backend", "simulator") != "simulator":
        raise ValueError("The batched Isaac benchmark requires simulator-owned dynamics")
    runtime = Path(__file__).with_name("isaac_tensor_manipulator_benchmark.py")
    num_envs = int(kwargs.get("num_envs", 1))
    num_resets = int(kwargs.get("num_resets", 10))
    max_episode_length = int(kwargs.get("max_episode_length", 1000))
    num_steps = kwargs.get("max_num_steps")
    if num_steps is None:
        num_steps = -1 if num_resets < 0 else num_resets * max_episode_length

    import spark_robot

    robot_config = grounding["robot_config"]
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
        configured.env.task.task_name,
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
        "--enable-self-collision"
        if kwargs.get("enable_self_collision", False)
        else "--no-enable-self-collision"
    )
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

    nominal_controller = configured.policy.nominal_controller
    environment = os.environ.copy()
    environment[_GROUNDING_ENV] = json.dumps(
        {
            **grounding,
            "right_arm_goal_range": configured.env.task.right_arm_goal_range,
            "left_arm_goal_range": configured.env.task.left_arm_goal_range,
            "obstacle_range": configured.env.task.obstacle_range,
            "robot_keepout": configured.env.task.robot_keepout,
            "position_kp": getattr(nominal_controller, "position_kp", 1.0),
            "velocity_kd": getattr(nominal_controller, "velocity_kd", 0.1),
            "arm_goal_rotation": goal_rotation,
            "arm_goal_position_only": getattr(configured.env.task, "arm_goal_position_only", False),
            "arm_goal_orientation_size": getattr(
                configured.env.task, "arm_goal_orientation_size", 0.1
            ),
            "arm_goal_minimum_distance": getattr(
                configured.env.task, "arm_goal_minimum_distance", 0.15
            ),
            "arm_goal_pair_keepout": getattr(
                configured.env.task,
                "arm_goal_pair_keepout",
                grounding.get("arm_goal_pair_keepout", 0.0),
            ),
            "viewer_config": kwargs.get("viewer_config")
            or dict(getattr(configured.env.agent, "viewer_config", {})),
        }
    )
    print(f"Launching {num_envs}-environment {display_name} Isaac benchmark:", *command)
    return subprocess.run(command, check=True, env=environment)


def add_fixed_base_benchmark_arguments(parser: argparse.ArgumentParser, robot_configs):
    """Add the uniform benchmark CLI shared by fixed-base manipulators."""

    parser.add_argument("--backend", choices=("mujoco", "isaac"), default="mujoco")
    parser.add_argument("--robot-config", "--robot-cfg", dest="robot_config", choices=robot_configs)
    parser.add_argument(
        "--test-case",
        choices=(*TEST_CASES, *TEST_CASE_ALIASES),
        default="arm_goal_static_v0",
        help=(
            "Canonical cases are arm_goal_static_v0 and arm_goal_static_v1; "
            "short names are compatibility aliases."
        ),
    )
    parser.add_argument("--num-envs", type=int, default=1)
    parser.add_argument(
        "--num-resets",
        type=int,
        default=10,
        help=(
            "Minimum completed episodes per environment before the batch ends; "
            "use -1 to keep resetting continuously."
        ),
    )
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
    parser.add_argument("--activation-distance", type=float)
    parser.add_argument(
        "--enable-self-collision",
        action=argparse.BooleanOptionalAction,
        default=False,
        help="Enable SPARK collision-volume self-pairs as controller constraints.",
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


def validate_fixed_base_benchmark_arguments(args):
    """Validate common CLI bounds before simulator startup."""

    if args["num_envs"] < 1:
        raise SystemExit("--num-envs must be positive")
    if args["num_resets"] == 0 or args["num_resets"] < -1:
        raise SystemExit("--num-resets must be positive or -1")
    if args["max_episode_length"] < 1:
        raise SystemExit("--max-episode-length must be positive")
