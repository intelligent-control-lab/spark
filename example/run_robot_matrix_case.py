#!/usr/bin/env python3
"""Run one backend-neutral robot joint-control conformance case.

This runner intentionally tests the robot configuration, dynamics owner, and
simulator plant without embedding a robot-specific Cartesian task or policy.
Use the family benchmark entry points for scalar Cartesian safety tasks and
this runner for cloned Isaac throughput and command-tracking verification.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np


TEST_CASE_ALIASES = {
    "joint_goal_reaching_v0": "arm_goal_static_v0",
    "joint_goal_reaching_v1": "arm_goal_static_v1",
}
TEST_CASE_CHOICES = (
    "arm_goal_static_v0",
    "arm_goal_static_v1",
    *TEST_CASE_ALIASES,
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-config", "--robot-cfg", dest="robot_config", required=True)
    parser.add_argument("--backend", choices=("mujoco", "isaac"), default="mujoco")
    parser.add_argument("--mode", choices=("teleop", "benchmark"), default="benchmark")
    parser.add_argument("--test-case", choices=TEST_CASE_CHOICES, default="arm_goal_static_v0")
    parser.add_argument("--num-envs", type=int, default=1)
    parser.add_argument(
        "--num-resets",
        type=int,
        default=10,
        help="Number of reset episodes; use -1 to continue until stopped (default: 10).",
    )
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument(
        "--dynamics-backend",
        choices=("simulator", "model"),
        default="simulator",
    )
    parser.add_argument(
        "--use-sim-dynamics",
        action=argparse.BooleanOptionalAction,
        default=None,
        help=argparse.SUPPRESS,
    )
    parser.add_argument(
        "--device",
        default=None,
        help="Isaac device; defaults to cpu for one environment and cuda:0 for batches.",
    )
    parser.add_argument(
        "--headless",
        action="store_true",
        help="Disable the viewer; the window and benchmark overlays are enabled by default.",
    )
    parser.add_argument("--real-time", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--duration", type=float, default=2.0)
    parser.add_argument(
        "--max-episode-steps",
        type=int,
        default=1000,
        help="Maximum control steps in each reset episode (default: 1000).",
    )
    parser.add_argument("--amplitude", type=float, default=0.08)
    parser.add_argument("--control-name", default=None)
    parser.add_argument(
        "--position-kp",
        type=float,
        default=None,
        help=(
            "Override the robot-qualified conformance gain "
            "(fallback defaults: order 1 = 2, order 2 = 4)."
        ),
    )
    parser.add_argument(
        "--velocity-kd",
        type=float,
        default=None,
        help=(
            "Override the robot-qualified conformance damping "
            "(fallback defaults: order 1 = 0, order 2 = 4)."
        ),
    )
    parser.add_argument("--goal-tolerance", type=float, default=0.03)
    parser.add_argument("--safe-algo", choices=("control_limit",), default="control_limit")
    parser.add_argument("--report", type=Path, default=None)
    parser.add_argument("--record-video-path", default=None)
    parser.add_argument("--record-gif-path", default=None)
    parser.add_argument("--record-fps", type=float, default=10.0)
    parser.add_argument("--record-width", type=int, default=640)
    parser.add_argument("--record-height", type=int, default=360)
    parser.add_argument(
        "--render-robot-collision-volumes",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--camera-lookat", type=float, nargs=3, default=None)
    parser.add_argument("--camera-distance", type=float, default=None)
    parser.add_argument("--camera-azimuth", type=float, default=None)
    parser.add_argument("--camera-elevation", type=float, default=None)
    parser.add_argument("--camera-vertical-fov", type=float, default=None)
    parser.add_argument("--env-spacing", type=float, default=2.5)
    parser.add_argument(
        "--max-visualized-envs",
        type=int,
        default=1,
        help=(
            "Show expensive robot collision-volume overlays on the first N clones "
            "(default: 1); goals and obstacles remain visible for every clone."
        ),
    )
    parser.add_argument("--render-every", type=int, default=None)
    return parser


def _canonical_test_case(name: str) -> str:
    return TEST_CASE_ALIASES.get(name, name)


def _control_mapping(robot_cfg, control_name: str | None = None):
    """Return the active dynamics view and one control-to-DoF mapping."""
    dynamics = robot_cfg.create_dynamics_model()
    prefix = "v" if dynamics.order == 1 else "a"
    dof_names = {dof.name for dof in robot_cfg.DoFs}
    candidates = []
    for name in dynamics.control_names:
        name = str(name)
        dof_name = name.removeprefix(prefix)
        if name.startswith(prefix) and dof_name in dof_names:
            candidates.append(
                (
                    int(robot_cfg.Control.__members__[name]),
                    int(robot_cfg.DoFs.__members__[dof_name]),
                    name,
                    dof_name,
                )
            )
    if control_name is not None:
        candidates = [item for item in candidates if item[2] == control_name]
        if not candidates:
            raise ValueError(f"Control {control_name!r} is not an active {dynamics.variant} axis")
    elif candidates:
        preferred = f"{prefix}RightJoint1"
        candidates.sort(key=lambda item: item[2] != preferred)
    if not candidates:
        raise ValueError(f"No direct control-to-DoF mapping for {type(robot_cfg).__name__}")
    return dynamics, candidates[0]


def _mujoco_control_stiffness(robot_cfg, control_index: int) -> float:
    control = robot_cfg.Control(int(control_index))
    for motor, mapped_control in robot_cfg.MujocoMotor_to_Control.items():
        if int(mapped_control) == int(control):
            return float(robot_cfg.MujocoMotorKps[motor])
    return 0.0


def _geometry_bounding_radius(geometry) -> float:
    if geometry.type == "sphere":
        return float(geometry.attributes["radius"])
    if geometry.type == "box":
        return 0.5 * float(
            np.linalg.norm(
                [
                    geometry.attributes["length"],
                    geometry.attributes["width"],
                    geometry.attributes["height"],
                ]
            )
        )
    raise ValueError(f"Unsupported collision geometry: {geometry.type!r}")


def _deterministic_obstacle_ring(
    robot_frames,
    collision_items,
    count: int,
    obstacle_radius: float,
    *,
    clearance: float = 0.08,
    angle_offset: float = 0.0,
    vertical_phase: float = 0.0,
):
    """Place a deterministic ring with exact clearance from sparse geometry."""
    count = int(count)
    if count <= 0:
        return np.empty((0, 3), dtype=float)
    centers = np.asarray(
        [robot_frames[int(frame_id), :3, 3] for frame_id, _ in collision_items],
        dtype=float,
    )
    radii = np.asarray(
        [_geometry_bounding_radius(geometry) for _, geometry in collision_items],
        dtype=float,
    )
    if not len(centers):
        return np.empty((0, 3), dtype=float)
    origin = np.mean(centers, axis=0)

    def minimum_clearance(point):
        return float(np.min(np.linalg.norm(centers - point, axis=1) - radii - obstacle_radius))

    result = []
    for index in range(count):
        angle = float(angle_offset) + 2.0 * math.pi * index / count
        direction = np.array(
            [
                math.cos(angle),
                math.sin(angle),
                0.15 * math.sin(2.0 * angle + float(vertical_phase)),
            ]
        )
        direction /= np.linalg.norm(direction)
        # Begin from the furthest collision-volume support in this direction.
        support_index = int(np.argmax((centers - origin) @ direction + radii))
        anchor = centers[support_index]
        low = 0.0
        high = radii[support_index] + obstacle_radius + clearance
        while minimum_clearance(anchor + high * direction) < clearance:
            high *= 1.5
        # Select the outermost crossing so the obstacle cannot sit in a gap
        # between disconnected collision regions.
        low = high / 1.5
        while minimum_clearance(anchor + low * direction) >= clearance and low > 1.0e-9:
            high = low
            low /= 1.5
        for _ in range(80):
            middle = 0.5 * (low + high)
            if minimum_clearance(anchor + middle * direction) < clearance:
                low = middle
            else:
                high = middle
        result.append(anchor + high * direction)
    return np.asarray(result, dtype=float)


def _to_numpy(value):
    if hasattr(value, "detach"):
        value = value.detach().cpu().numpy()
    return np.asarray(value, dtype=float)


def _batch(value, width: int) -> np.ndarray:
    value = _to_numpy(value)
    if value.ndim == 1:
        value = value.reshape(1, width)
    return value.reshape(-1, width)


def _sample_goal_offsets(num_envs: int, amplitude: float, seed: int, reset_index: int):
    """Sample reproducible, visibly nonzero row-local scalar goals."""
    rng = np.random.default_rng(np.random.SeedSequence([int(seed), int(reset_index)]))
    magnitudes = float(amplitude) * (0.35 + 0.65 * rng.random(int(num_envs)))
    signs = np.where((np.arange(int(num_envs)) + int(reset_index)) % 2, -1.0, 1.0)
    rng.shuffle(signs)
    return magnitudes * signs


def _viewer_config(args) -> dict | None:
    names = (
        "camera_lookat",
        "camera_distance",
        "camera_azimuth",
        "camera_elevation",
        "camera_vertical_fov",
    )
    result = {name: getattr(args, name) for name in names if getattr(args, name, None) is not None}
    return result or None


class _BenchmarkVisualizer:
    """Small simulator-neutral overlay for the joint conformance benchmark."""

    def __init__(
        self,
        robot_cfg,
        agent,
        backend,
        position,
        target,
        test_case,
        args=None,
        *,
        reset_index=0,
    ):
        import spark_robot
        from spark_utils import Geometry, VizColor

        self.robot_cfg = robot_cfg
        self.backend = backend
        self.position = _batch(position, len(robot_cfg.DoFs))
        self.target = _batch(target, len(robot_cfg.DoFs))
        self.test_case = test_case
        self.collision_items = list(robot_cfg.CollisionVol.items())
        self.kinematics = getattr(spark_robot, robot_cfg.kinematics_class_name)(robot_cfg)
        env_positions = _to_numpy(getattr(agent, "env_positions", np.zeros((len(position), 3))))
        if env_positions.ndim == 1:
            env_positions = env_positions.reshape(1, 3)
        self.env_positions = env_positions
        self.visualized_env_ids = tuple(range(len(self.position)))
        maximum = int(getattr(args, "max_visualized_envs", 1))
        self.collision_volume_env_ids = tuple(range(min(len(self.position), maximum)))
        self.render_robot_collision_volumes = bool(
            getattr(args, "render_robot_collision_volumes", True)
        )
        count = int(test_case.task["num_obstacle_task"])
        radius = float(test_case.task["obstacle_size"])
        seed = int(getattr(args, "seed", 0))
        obstacle_positions = []
        for env_id, position_row in enumerate(self.position):
            rng = np.random.default_rng(
                np.random.SeedSequence([seed, int(reset_index), int(env_id), 1])
            )
            frames = self.kinematics.forward_kinematics(position_row)
            obstacle_positions.append(
                _deterministic_obstacle_ring(
                    frames,
                    self.collision_items,
                    count,
                    radius,
                    angle_offset=rng.uniform(0.0, 2.0 * math.pi),
                    vertical_phase=rng.uniform(0.0, 2.0 * math.pi),
                )
            )
        self.obstacle_positions = np.asarray(obstacle_positions, dtype=float)
        self.obstacle_geometries = [
            Geometry(type="sphere", radius=radius, color=VizColor.obstacle_task)
            for _ in range(count)
        ]

    def render(self, agent, positions, targets) -> None:
        from spark_utils import VizColor, collision_volume_distance_color

        positions = _batch(positions, len(self.robot_cfg.DoFs))
        targets = _batch(targets, len(self.robot_cfg.DoFs))
        opacity_scale = float(getattr(agent, "collision_volume_opacity_scale", 1.0))
        opacity_floor = float(getattr(agent, "collision_volume_opacity_floor", 0.0))
        color = collision_volume_distance_color(
            np.inf,
            opacity_scale=opacity_scale,
            opacity_floor=opacity_floor,
        )
        for env_id in self.visualized_env_ids:
            origin = self.env_positions[min(env_id, len(self.env_positions) - 1)]
            frames = self.kinematics.forward_kinematics(positions[env_id])
            target_frames = self.kinematics.forward_kinematics(targets[env_id])
            if self.render_robot_collision_volumes and env_id in self.collision_volume_env_ids:
                for frame_id, geometry in self.collision_items:
                    frame = frames[int(frame_id)].copy()
                    frame[:3, 3] += origin
                    if geometry.type == "sphere":
                        size = np.full(3, geometry.attributes["radius"], dtype=float)
                        agent.render_sphere(frame[:3, 3], frame[:3, :3], size, color)
                    elif geometry.type == "box":
                        size = np.array(
                            [
                                geometry.attributes["length"],
                                geometry.attributes["width"],
                                geometry.attributes["height"],
                            ],
                            dtype=float,
                        )
                        agent.render_box(frame[:3, 3], frame[:3, :3], size, color)
            goal_frame_id = int(self.collision_items[-1][0]) if self.collision_items else 0
            goal = target_frames[goal_frame_id].copy()
            goal[:3, 3] += origin
            agent.render_sphere(
                goal[:3, 3],
                goal[:3, :3],
                np.full(3, 0.045),
                VizColor.goal,
            )
            for obstacle, geometry in zip(
                self.obstacle_positions[env_id],
                self.obstacle_geometries,
            ):
                position = origin + obstacle
                agent.render_sphere(
                    position,
                    np.eye(3),
                    np.full(3, geometry.attributes["radius"]),
                    geometry.color,
                )


def _make_agents(args, robot_cfg, simulation_app=None):
    import spark_agent

    device = args.device or ("cpu" if int(args.num_envs) == 1 else "cuda:0")
    render_requested = bool(not args.headless or args.record_video_path or args.record_gif_path)
    class_name = robot_cfg.agent_class_name(args.backend)
    agent_type = getattr(spark_agent, class_name)
    common = dict(
        dynamics_backend=args.dynamics_backend,
        enable_viewer=render_requested,
        enable_keyboard_control=False,
        real_time=bool(args.real_time),
        render_robot_collision_volumes=bool(args.render_robot_collision_volumes),
        viewer_config=_viewer_config(args),
    )
    if args.backend == "isaac":
        agent = agent_type(
            robot_cfg,
            **common,
            num_envs=int(args.num_envs),
            env_spacing=float(args.env_spacing),
            device=device,
            render=render_requested,
            render_on_step=False,
        )
        if simulation_app is not None and hasattr(agent, "attach_simulation_app"):
            agent.attach_simulation_app(simulation_app)
        return SimpleNamespace(
            primary=agent,
            agents=(agent,),
            sim=agent.sim,
            backend=args.backend,
            num_envs=int(args.num_envs),
            device=device,
        )

    agents = []
    for env_id in range(int(args.num_envs)):
        kwargs = dict(common)
        kwargs["enable_viewer"] = render_requested and env_id == 0
        if env_id == 0:
            kwargs.update(
                record_video_path=args.record_video_path,
                record_gif_path=args.record_gif_path,
                record_fps=args.record_fps,
                record_width=args.record_width,
                record_height=args.record_height,
            )
        agents.append(agent_type(robot_cfg, **kwargs))
    return SimpleNamespace(
        primary=agents[0],
        agents=tuple(agents),
        sim=None,
        backend=args.backend,
        num_envs=int(args.num_envs),
        device="cpu",
    )


def _feedback(bundle) -> dict:
    records = [agent.get_feedback() for agent in bundle.agents]
    if bundle.backend == "isaac":
        return records[0]
    # The conformance controller only consumes state and command feedback.
    # MuJoCo's scalar agents intentionally do not expose their last control in
    # the common feedback contract, so do not make an unused diagnostic field
    # a backend requirement here.
    keys = ("dof_pos_fbk", "dof_vel_fbk", "dof_pos_cmd", "dof_vel_cmd")
    return {
        key: np.stack([np.asarray(record[key], dtype=float) for record in records]) for key in keys
    }


def _step(bundle, control) -> None:
    control = np.asarray(control, dtype=float)
    if bundle.backend == "isaac":
        bundle.primary.step(control[0] if bundle.num_envs == 1 else control)
        return
    for agent, command in zip(bundle.agents, control):
        agent.step(command)


def _reset(bundle) -> None:
    for agent in bundle.agents:
        agent.reset()


def _close_agents(bundle, backend=None) -> None:
    del backend
    if bundle is None:
        return
    for agent in bundle.agents:
        close_viewer = getattr(agent, "close_viewer", None)
        if callable(close_viewer):
            close_viewer()
        close = getattr(agent, "close", None)
        if callable(close):
            close()


def _aggregate_reset_reports(reports: list[dict]) -> dict:
    if not reports:
        raise ValueError("At least one reset report is required")
    aggregate = {
        key: value
        for key, value in reports[0].items()
        if key not in {"reset_index", "stage_seconds"}
    }
    aggregate["status"] = "PASS" if all(item["status"] == "PASS" for item in reports) else "FAIL"
    aggregate["num_resets"] = len(reports)
    aggregate["steps"] = sum(int(item["steps"]) for item in reports)
    aggregate["simulated_seconds"] = sum(float(item["simulated_seconds"]) for item in reports)
    aggregate["wall_seconds"] = sum(float(item["wall_seconds"]) for item in reports)
    for key in (
        "max_goal_error",
        "max_abs_control",
        "motion_span",
        "reset_initial_position_error",
        "reset_initial_velocity_max",
    ):
        values = [float(item.get(key, 0.0) or 0.0) for item in reports]
        aggregate[key] = max(values)
    goal_errors = np.asarray([item["max_goal_error"] for item in reports], dtype=float)
    aggregate["goal_error_mean"] = float(np.mean(goal_errors))
    aggregate["goal_error_std"] = float(np.std(goal_errors))
    stage_keys = {key for item in reports for key in item.get("stage_seconds", {})}
    aggregate["stage_seconds"] = {
        key: sum(float(item.get("stage_seconds", {}).get(key, 0.0)) for item in reports)
        for key in sorted(stage_keys)
    }
    wall = max(aggregate["wall_seconds"], 1.0e-12)
    aggregate["aggregate_control_hz"] = float(aggregate["num_envs"]) * aggregate["steps"] / wall
    aggregate["reset_reports"] = reports
    return aggregate


def _run_case(args, robot_cfg, agents, recorder=None) -> dict:
    from spark_pipeline import resolve_benchmark_test_case

    dynamics, mapping = _control_mapping(robot_cfg, args.control_name)
    control_index, dof_index, control_name, dof_name = mapping
    default_kp = getattr(robot_cfg, "conformance_position_kp", None)
    default_kd = getattr(robot_cfg, "conformance_velocity_kd", None)
    if default_kp is None:
        default_kp = 2.0 if dynamics.order == 1 else 4.0
    if default_kd is None:
        default_kd = 0.0 if dynamics.order == 1 else 4.0
    kp = float(args.position_kp if args.position_kp is not None else default_kp)
    kd = float(args.velocity_kd if args.velocity_kd is not None else default_kd)
    control_limit = abs(float(robot_cfg.ControlLimit[robot_cfg.Control(control_index)]))
    control_period = float(
        getattr(
            agents.primary,
            "control_period",
            getattr(agents.primary, "dt", 0.002) * getattr(agents.primary, "control_decimation", 1),
        )
    )
    requested_steps = max(1, int(math.ceil(float(args.duration) / control_period)))
    steps = min(requested_steps, int(args.max_episode_steps))
    test_case = resolve_benchmark_test_case(_canonical_test_case(args.test_case))
    reports = []
    visualizer = None
    # The full support-matrix orchestrator predates the standalone
    # ``--num-resets`` option and intentionally runs one reset per matrix cell.
    num_resets = int(getattr(args, "num_resets", 1))
    reset_index = 0
    while num_resets == -1 or reset_index < num_resets:
        is_running = getattr(agents.primary, "is_running", None)
        if callable(is_running) and not is_running():
            break
        reset_label = "continuous" if num_resets == -1 else str(num_resets)
        print(
            f"Benchmark reset {reset_index + 1}/{reset_label}: "
            f"{agents.num_envs} environments, {steps} control steps",
            flush=True,
        )
        _reset(agents)
        initial_feedback = _feedback(agents)
        initial_position = _batch(initial_feedback["dof_pos_fbk"], len(robot_cfg.DoFs))
        initial_velocity = _batch(initial_feedback["dof_vel_fbk"], len(robot_cfg.DoFs))
        initial_command = _batch(initial_feedback["dof_pos_cmd"], len(robot_cfg.DoFs))
        offsets = _sample_goal_offsets(
            agents.num_envs,
            args.amplitude,
            getattr(args, "seed", 0),
            reset_index,
        )
        target = initial_command.copy()
        target[:, dof_index] += offsets
        if not args.headless or recorder is not None:
            visualizer = _BenchmarkVisualizer(
                robot_cfg,
                agents.primary,
                args.backend,
                initial_position,
                target,
                test_case,
                args,
                reset_index=reset_index,
            )

        max_abs_control = 0.0
        stage_seconds = {"control": 0.0, "physics": 0.0, "feedback": 0.0, "render": 0.0}
        started = time.perf_counter()
        for step_index in range(steps):
            feedback_started = time.perf_counter()
            feedback = _feedback(agents)
            # Close the conformance controller around the plant output.  The
            # command state is an agent-internal drive/model target and can
            # legitimately lead a physical articulation; using it here hid
            # that lag in scalar runs and amplified it in CUDA-cloned runs.
            position = _batch(feedback["dof_pos_fbk"], len(robot_cfg.DoFs))
            velocity = _batch(feedback["dof_vel_fbk"], len(robot_cfg.DoFs))
            stage_seconds["feedback"] += time.perf_counter() - feedback_started

            control_started = time.perf_counter()
            control = np.zeros((agents.num_envs, len(robot_cfg.Control)), dtype=float)
            if args.mode == "benchmark":
                selected = kp * (target[:, dof_index] - position[:, dof_index])
                selected -= kd * velocity[:, dof_index]
            else:
                phase = 2.0 * math.pi * step_index / max(steps - 1, 1)
                selected = offsets * math.sin(phase)
            control[:, control_index] = np.clip(selected, -control_limit, control_limit)
            max_abs_control = max(max_abs_control, float(np.max(np.abs(control))))
            stage_seconds["control"] += time.perf_counter() - control_started

            physics_started = time.perf_counter()
            _step(agents, control)
            stage_seconds["physics"] += time.perf_counter() - physics_started

            if visualizer is not None and (step_index % max(1, int(args.render_every or 1)) == 0):
                render_started = time.perf_counter()
                latest = _feedback(agents)
                visualizer.render(
                    agents.primary,
                    latest["dof_pos_fbk"],
                    target,
                )
                agents.primary.render()
                if recorder is not None:
                    recorder.schedule((step_index + 1) * control_period)
                stage_seconds["render"] += time.perf_counter() - render_started

        wall_seconds = time.perf_counter() - started
        final_feedback = _feedback(agents)
        final_position = _batch(final_feedback["dof_pos_fbk"], len(robot_cfg.DoFs))
        final_velocity = _batch(final_feedback["dof_vel_fbk"], len(robot_cfg.DoFs))
        finite = bool(np.isfinite(final_position).all() and np.isfinite(final_velocity).all())
        goal_error = float(np.max(np.abs(final_position[:, dof_index] - target[:, dof_index])))
        motion_span = float(np.ptp(final_position[:, dof_index])) if agents.num_envs > 1 else 0.0
        passed = finite and (
            goal_error <= float(args.goal_tolerance)
            if args.mode == "benchmark"
            else max_abs_control > 0.0
        )
        reports.append(
            {
                "status": "PASS" if passed else "FAIL",
                "finite": finite,
                "robot_config": type(robot_cfg).__name__,
                "backend": args.backend,
                "device": agents.device,
                "mode": args.mode,
                "test_case": args.test_case,
                "num_envs": agents.num_envs,
                "dynamics_backend": args.dynamics_backend,
                "control": control_name,
                "dof": dof_name,
                "control_state_source": "measured_feedback",
                "safe_algo": args.safe_algo,
                "seed": int(getattr(args, "seed", 0)),
                "position_kp": kp,
                "velocity_kd": kd,
                "goal_tolerance": float(args.goal_tolerance),
                "steps": steps,
                "simulated_seconds": steps * control_period,
                "wall_seconds": wall_seconds,
                "max_goal_error": goal_error,
                "max_abs_control": max_abs_control,
                "motion_span": motion_span,
                "goal_offsets": offsets.tolist(),
                "first_obstacle_positions": (
                    visualizer.obstacle_positions[:, 0, :].tolist()
                    if visualizer is not None and visualizer.obstacle_positions.shape[1]
                    else []
                ),
                "reset_initial_position_error": float(
                    np.max(np.abs(initial_position - initial_command))
                ),
                "reset_initial_velocity_max": float(np.max(np.abs(initial_velocity))),
                "num_visualized_envs": (
                    len(visualizer.visualized_env_ids) if visualizer is not None else 0
                ),
                "num_collision_volume_envs": (
                    len(visualizer.collision_volume_env_ids) if visualizer is not None else 0
                ),
                "stage_seconds": stage_seconds,
                "reset_index": reset_index,
            }
        )
        reset_index += 1
    if not reports:
        raise RuntimeError("Benchmark stopped before completing its first reset")
    return _aggregate_reset_reports(reports)


def _validate_args(args) -> None:
    if args.num_envs < 1 or args.num_resets == 0 or args.num_resets < -1:
        raise SystemExit("--num-envs must be positive and --num-resets must be -1 or positive")
    if args.max_episode_steps < 1:
        raise SystemExit("--max-episode-steps must be positive")
    if args.duration <= 0.0 or args.amplitude <= 0.0 or args.goal_tolerance <= 0.0:
        raise SystemExit("--duration, --amplitude, and --goal-tolerance must be positive")
    if args.record_fps <= 0.0 or args.record_width < 1 or args.record_height < 1:
        raise SystemExit("recording FPS and dimensions must be positive")
    if args.max_visualized_envs is not None and args.max_visualized_envs < 1:
        raise SystemExit("--max-visualized-envs must be positive")
    if args.use_sim_dynamics is not None:
        requested = "simulator" if args.use_sim_dynamics else "model"
        if args.dynamics_backend != "simulator" and args.dynamics_backend != requested:
            raise SystemExit("Conflicting dynamics backend arguments")
        args.dynamics_backend = requested
    if args.device is None:
        args.device = "cpu" if args.num_envs == 1 else "cuda:0"


def main(argv=None) -> int:
    args = _parser().parse_args(argv)
    _validate_args(args)
    import spark_robot

    config_type = getattr(spark_robot, args.robot_config, None)
    if config_type is None:
        raise SystemExit(f"Unknown robot configuration: {args.robot_config}")
    robot_cfg = config_type()
    simulation_app = None
    agents = None
    recorder = None
    result = None
    try:
        if args.backend == "isaac":
            from isaacsim import SimulationApp

            render_requested = bool(
                not args.headless or args.record_video_path or args.record_gif_path
            )
            simulation_app = SimulationApp(
                {
                    "headless": bool(args.headless),
                    "hide_ui": bool(args.headless),
                    "multi_gpu": False,
                    "disable_viewport_updates": not render_requested,
                }
            )
        agents = _make_agents(args, robot_cfg, simulation_app)
        if args.backend == "isaac" and (args.record_video_path or args.record_gif_path):
            from spark_agent.simulation.isaac.viewport_recorder import IsaacViewportRecorder

            recorder = IsaacViewportRecorder(
                video_path=args.record_video_path,
                gif_path=args.record_gif_path,
                width=args.record_width,
                height=args.record_height,
                fps=args.record_fps,
            )
        result = _run_case(args, robot_cfg, agents, recorder=recorder)
        if recorder is not None:
            recorder.drain(agents.sim.render)
            recorder.close()
            result["recorded_frames"] = recorder.frames
        if args.report is not None:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
        print(json.dumps(result, indent=2, sort_keys=True), flush=True)
        return 0 if result["status"] == "PASS" else 1
    finally:
        if recorder is not None and not recorder.closed:
            if recorder.pending and agents is not None:
                recorder.drain(agents.sim.render)
            recorder.close()
        if agents is not None:
            _close_agents(agents, args.backend)
        if simulation_app is not None:
            simulation_app.close(wait_for_replicator=False)


if __name__ == "__main__":
    raise SystemExit(main())
