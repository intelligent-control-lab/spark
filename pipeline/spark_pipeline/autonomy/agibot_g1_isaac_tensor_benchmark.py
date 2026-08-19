"""Batched whole-body AgiBot G1 benchmark runtime for Isaac Sim.

Users should launch :mod:`example.agibot_g1.run_agibot_g1_benchmark`.  Isaac
is kept behind this process boundary so Kit is initialized before any Isaac
simulation modules are imported. State, nominal control, collision queries,
and the safety QP remain batched on one device, while bounded CPU IK runs only
when selected environment rows reset.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import time


ROBOT_CONFIGS = (
    "AgiBotG1MobileBaseDynamic1Config",
    "AgiBotG1MobileBaseDynamic2Config",
)
TEST_CASES = ("whole_goal_static_v0", "whole_goal_static_v1")
_GOAL_OFFSET_ENVIRONMENT_VARIABLE = "SPARK_AGIBOT_BENCHMARK_GOAL_OFFSETS"
_VIEWER_CONFIG_ENVIRONMENT_VARIABLE = "SPARK_AGIBOT_BENCHMARK_VIEWER_CONFIG"


def _build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-config", choices=ROBOT_CONFIGS, default=ROBOT_CONFIGS[0])
    parser.add_argument("--test-case", choices=TEST_CASES, default=TEST_CASES[0])
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
    parser.add_argument("--num-steps", type=int, default=10000)
    parser.add_argument("--warmup-steps", type=int, default=20)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--device", default=None)
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--show-simulation-info", action="store_true")
    parser.add_argument("--real-time", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument(
        "--dt",
        type=float,
        default=None,
        help="Physics step; defaults to the selected Agi robot configuration.",
    )
    parser.add_argument(
        "--control-decimation",
        type=int,
        default=None,
        help="Physics substeps per action; defaults to the Agi robot configuration.",
    )
    parser.add_argument(
        "--render-every",
        type=int,
        default=None,
        help="Present every Nth control step (default: approximately 20 Hz).",
    )
    parser.add_argument("--safe-algo", choices=("bypass", "ssa", "rssa"), default=None)
    parser.add_argument("--minimum-distance", type=float, default=None)
    parser.add_argument("--activation-distance", type=float, default=None)
    parser.add_argument("--safety-qp-iterations", type=int, default=20)
    parser.add_argument("--safety-slack-weight", type=float, default=1000.0)
    parser.add_argument(
        "--render-robot-collision-volumes",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument(
        "--render-safety-trigger-constraints",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Render blue closest-pair lines for constraints requiring correction.",
    )
    parser.add_argument(
        "--render-safety-violations",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Render purple closest-pair lines for residual post-filter violations.",
    )
    parser.add_argument(
        "--max-visualized-collision-envs",
        type=int,
        default=1,
        help="Render detailed collision spheres for only the first N clones.",
    )
    parser.add_argument(
        "--profile-frequency",
        action="store_true",
        help="Synchronize and report mean control-loop stage latency.",
    )
    parser.add_argument("--record-video-path")
    parser.add_argument("--record-gif-path")
    parser.add_argument("--record-fps", type=float, default=10.0)
    parser.add_argument("--record-width", type=int, default=960)
    parser.add_argument("--record-height", type=int, default=540)
    return parser


def _validate_args(args) -> None:
    if args.num_envs < 1:
        raise ValueError("--num-envs must be positive")
    if args.num_resets == 0 or args.num_resets < -1:
        raise ValueError("--num-resets must be positive or -1 for continuous reset")
    if args.max_episode_length < 1:
        raise ValueError("--max-episode-length must be positive")
    if args.num_steps == 0 or args.num_steps < -1:
        raise ValueError("--num-steps must be positive or -1")
    if args.dt is not None and args.dt <= 0.0:
        raise ValueError("--dt must be positive")
    if args.control_decimation is not None and args.control_decimation < 1:
        raise ValueError("--dt and --control-decimation must be positive")
    if args.render_every is not None and args.render_every < 1:
        raise ValueError("--render-every must be positive")
    if args.record_fps <= 0.0 or args.record_width < 1 or args.record_height < 1:
        raise ValueError("recording FPS, width, and height must be positive")
    if args.num_resets < 0 and args.num_steps < 0:
        # The viewer can still be closed interactively, so this is a supported
        # continuous mode rather than an accidental unbounded configuration.
        return


def _wrap_angle_torch(angle, torch):
    return torch.remainder(angle + torch.pi, 2.0 * torch.pi) - torch.pi


def _quaternion_rotate_xyzw(quaternion, vector, torch):
    q_vector = quaternion[..., :3]
    q_scalar = quaternion[..., 3:4]
    tangent = 2.0 * torch.cross(q_vector, vector, dim=-1)
    return vector + q_scalar * tangent + torch.cross(q_vector, tangent, dim=-1)


def _ik_position_error(info) -> float:
    if not isinstance(info, dict):
        return float("inf")
    result = info.get("ik_result")
    return float(getattr(result, "position_error", float("inf")))


def _run(args) -> int:
    from isaacsim import SimulationApp

    simulation_app = SimulationApp(
        {
            "headless": bool(args.headless),
            "hide_ui": bool(args.headless),
            "multi_gpu": False,
            "disable_viewport_updates": bool(args.headless),
        }
    )
    agent = None
    viewport_recorder = None
    try:
        import numpy as np
        import torch

        import spark_agent
        import spark_robot
        from spark_agent.simulation.viewer_config import DEFAULT_VIEWER_CONFIG
        from spark_pipeline.autonomy.benchmark_lifecycle import reset_quota_reached
        from spark_pipeline.autonomy.benchmark_test_cases import resolve_benchmark_test_case
        from spark_policy.safety.geometry import (
            PointCloudBatch,
            TorchSphereCollisionBackend,
            build_link_sphere_model,
        )
        from spark_policy.safety.tensor import (
            BatchedQPSafetyFilter,
            BatchedRelaxedQPSafetyFilter,
            FirstOrderTensorSafetyIndex,
            SecondOrderTensorSafetyIndex,
        )
        from spark_utils import VizColor, collision_volume_distance_color

        device = args.device or ("cpu" if args.num_envs == 1 else "cuda:0")
        args.device = device
        if device.startswith("cuda") and not torch.cuda.is_available():
            raise RuntimeError(
                f"CUDA device {device!r} was requested but PyTorch CUDA is unavailable"
            )

        robot_cfg = getattr(spark_robot, args.robot_config)()
        args.dt = float(robot_cfg.simulator_dynamics.physics_dt if args.dt is None else args.dt)
        args.control_decimation = int(
            robot_cfg.simulator_dynamics.control_decimation
            if args.control_decimation is None
            else args.control_decimation
        )
        if args.render_every is None:
            args.render_every = max(
                1,
                int(math.ceil(1.0 / (20.0 * args.dt * args.control_decimation))),
            )
        kinematics = spark_robot.AgiBotG1MobileBaseKinematics(robot_cfg)
        task = resolve_benchmark_test_case(args.test_case).parameters()
        task["max_episode_length"] = int(args.max_episode_length)
        serialized_offsets = os.environ.get(_GOAL_OFFSET_ENVIRONMENT_VARIABLE)
        if serialized_offsets is None:
            raise RuntimeError(
                "The AgiBot Isaac tensor runtime is internal; launch it through "
                "example/agibot_g1/run_agibot_g1_benchmark.py so runner-local "
                "benchmark offsets are supplied."
            )
        benchmark_goal_offsets = json.loads(serialized_offsets)
        for role, field in (
            ("left_arm_goal", "left_arm_goal_range"),
            ("right_arm_goal", "right_arm_goal_range"),
        ):
            offset = benchmark_goal_offsets[role]
            if len(offset) != 3:
                raise ValueError(f"Benchmark offset {role!r} must have three entries")
            task[field] = [
                (float(low) + float(offset[axis]), float(high) + float(offset[axis]))
                for axis, (low, high) in enumerate(task[field])
            ]

        if args.safe_algo is None:
            args.safe_algo = "bypass" if int(task["num_obstacle_task"]) == 0 else "rssa"
        minimum_distance = float(
            task.get("minimum_distance", 0.05)
            if args.minimum_distance is None
            else args.minimum_distance
        )
        activation_distance = float(
            task.get("safety_activation_distance", minimum_distance + 0.10)
            if args.activation_distance is None
            else args.activation_distance
        )
        if activation_distance < minimum_distance:
            raise ValueError(
                "activation distance must be greater than or equal to minimum distance"
            )

        viewer_config = dict(DEFAULT_VIEWER_CONFIG)
        serialized_viewer_config = os.environ.get(_VIEWER_CONFIG_ENVIRONMENT_VARIABLE)
        if serialized_viewer_config:
            viewer_config.update(json.loads(serialized_viewer_config))

        agent_class = getattr(spark_agent, "AgiBotG1MobileBaseIsaacAgent")
        agent = agent_class(
            robot_cfg,
            num_envs=args.num_envs,
            device=device,
            tensor_api=True,
            render=not args.headless,
            enable_viewer=not args.headless,
            viewer_show_simulation_info=args.show_simulation_info,
            viewport_layout_updates=8 if not args.headless else 0,
            render_on_step=False,
            dt=args.dt,
            control_decimation=args.control_decimation,
            dynamics_backend="simulator",
            viewer_config=viewer_config,
        )
        agent.attach_simulation_app(simulation_app)
        control_period = float(args.dt * args.control_decimation)
        if args.record_video_path or args.record_gif_path:
            from spark_agent.simulation.isaac.viewport_recorder import (
                IsaacViewportRecorder,
            )

            viewport_recorder = IsaacViewportRecorder(
                video_path=args.record_video_path,
                gif_path=args.record_gif_path,
                width=args.record_width,
                height=args.record_height,
                fps=args.record_fps,
            )
        num_dof = len(robot_cfg.DoFs)
        num_control = len(robot_cfg.Control)
        if num_dof != num_control:
            raise RuntimeError("Agi tensor PID currently requires one configured control per DoF")

        feedback = agent.get_feedback()
        body_kinematics = agent.get_body_kinematics(include_jacobian=False)
        collision_model = build_link_sphere_model(
            robot_cfg,
            body_kinematics["body_names"],
            device=device,
        )
        if collision_model.unmapped_frames:
            raise RuntimeError(
                "Agi collision frames do not map to Isaac bodies: "
                + ", ".join(collision_model.unmapped_frames)
            )
        body_name_to_id = {name: index for index, name in enumerate(body_kinematics["body_names"])}
        ee_body_ids = torch.tensor(
            [body_name_to_id["right_base_link"], body_name_to_id["left_base_link"]],
            device=device,
            dtype=torch.long,
        )
        ee_local_offsets = torch.tensor(
            [[0.0, 0.0, 0.10], [0.0, 0.0, 0.10]],
            device=device,
            dtype=torch.float32,
        )

        env_origins = agent.env_positions.clone()
        target_position = agent.default_dof_pos.clone()
        right_goal_local = torch.zeros(args.num_envs, 3, device=device)
        left_goal_local = torch.zeros_like(right_goal_local)
        base_goal = torch.zeros(args.num_envs, 3, device=device)
        obstacle_count = int(task["num_obstacle_task"])
        obstacle_positions_local = torch.zeros(args.num_envs, obstacle_count, 3, device=device)
        obstacle_positions_world = torch.zeros_like(obstacle_positions_local)
        obstacle_radii = torch.full(
            (args.num_envs, obstacle_count),
            float(task["obstacle_size"]),
            device=device,
        )
        obstacle_cloud = PointCloudBatch(
            positions=obstacle_positions_world,
            radii=obstacle_radii,
            valid_mask=torch.ones(args.num_envs, obstacle_count, device=device, dtype=torch.bool),
            velocities=torch.zeros_like(obstacle_positions_world),
        )
        episode_steps = torch.zeros(args.num_envs, device=device, dtype=torch.long)
        completed_episodes = torch.zeros_like(episode_steps)
        successful_episodes = torch.zeros_like(episode_steps)
        timeout_episodes = torch.zeros_like(episode_steps)
        last_finished_right_error = torch.full((args.num_envs,), torch.nan, device=device)
        last_finished_left_error = torch.full_like(last_finished_right_error, torch.nan)
        last_finished_base_error = torch.full_like(last_finished_right_error, torch.nan)
        last_finished_yaw_error = torch.full_like(last_finished_right_error, torch.nan)
        ik_queries = 0
        ik_failures = 0
        ik_time = 0.0

        collision_backend = TorchSphereCollisionBackend()
        safety_filter = None
        safety_index = None
        if obstacle_count and args.safe_algo != "bypass":
            if robot_cfg.dynamics_order == 1:
                safety_index = FirstOrderTensorSafetyIndex(
                    minimum_distance=minimum_distance,
                    activation_distance=activation_distance,
                    alpha=4.0,
                    mode="safe_set",
                )
            else:
                safety_index = SecondOrderTensorSafetyIndex(
                    minimum_distance=minimum_distance,
                    activation_distance=activation_distance,
                    position_gain=16.0,
                    velocity_gain=8.0,
                )
            filter_class = (
                BatchedRelaxedQPSafetyFilter if args.safe_algo == "rssa" else BatchedQPSafetyFilter
            )
            filter_kwargs = {
                "iterations": int(args.safety_qp_iterations),
                "warm_start": True,
            }
            if args.safe_algo == "rssa":
                filter_kwargs["slack_weight"] = float(args.safety_slack_weight)
            safety_filter = filter_class(**filter_kwargs)

        control_limits = torch.tensor(
            [float(robot_cfg.ControlLimit[control]) for control in robot_cfg.Control],
            device=device,
            dtype=torch.float32,
        )
        lower_control = -control_limits[None]
        upper_control = control_limits[None]
        previous_sphere_jacobian = None
        valid_previous_jacobian = torch.zeros(args.num_envs, device=device, dtype=torch.bool)
        closest_sphere_distance = torch.full(
            (args.num_envs, len(collision_model.body_names)),
            torch.inf,
            device=device,
        )
        safety_trigger_count = torch.zeros(args.num_envs, device=device, dtype=torch.long)
        minimum_observed_distance = torch.full((args.num_envs,), torch.inf, device=device)
        latest_safety_visualization = None

        def synchronize():
            if device.startswith("cuda"):
                torch.cuda.synchronize(device)

        def profile_call(name, function, profile):
            if not args.profile_frequency:
                return function()
            synchronize()
            started = time.perf_counter()
            result = function()
            synchronize()
            profile[name] += time.perf_counter() - started
            return result

        def current_sphere_geometry(*, include_jacobian=True):
            kin = agent.get_body_kinematics(include_jacobian=include_jacobian)
            positions = kin["body_position_w"][:, collision_model.body_ids]
            quaternions = kin["body_quaternion_w"][:, collision_model.body_ids]
            local = collision_model.local_offsets[None].expand(args.num_envs, -1, -1)
            world_offset = _quaternion_rotate_xyzw(quaternions, local, torch)
            return positions + world_offset, world_offset, kin

        def current_ee_positions(kin=None):
            if kin is None:
                kin = agent.get_body_kinematics(include_jacobian=False)
            positions = kin["body_position_w"][:, ee_body_ids]
            quaternions = kin["body_quaternion_w"][:, ee_body_ids]
            offsets = ee_local_offsets[None].expand(args.num_envs, -1, -1)
            return positions + _quaternion_rotate_xyzw(quaternions, offsets, torch)

        def arm_goals_world(dof_position):
            yaw = dof_position[:, int(robot_cfg.DoFs.RotYaw)]
            cosine, sine = torch.cos(yaw), torch.sin(yaw)
            base_xy = dof_position[:, [int(robot_cfg.DoFs.LinearX), int(robot_cfg.DoFs.LinearY)]]

            def transform(local):
                result = torch.empty_like(local)
                result[:, 0] = base_xy[:, 0] + cosine * local[:, 0] - sine * local[:, 1]
                result[:, 1] = base_xy[:, 1] + sine * local[:, 0] + cosine * local[:, 1]
                result[:, 2] = local[:, 2]
                return result + env_origins

            return transform(right_goal_local), transform(left_goal_local)

        def sample_uniform(rng, bounds):
            bounds = np.asarray(bounds, dtype=float)
            return rng.uniform(bounds[:, 0], bounds[:, 1])

        def sample_episode(env_id: int) -> None:
            nonlocal ik_queries, ik_failures, ik_time
            episode = int(completed_episodes[env_id].item())
            rng = np.random.RandomState(args.seed + 1009 * env_id + 7919 * episode)
            q_current = agent.command_pos[env_id].detach().cpu().numpy().astype(float)

            base_bounds = np.asarray(task["base_goal_range"], dtype=float)
            workspace = np.asarray(task["base_goal_workspace_range"], dtype=float)
            for _ in range(10000):
                candidate = sample_uniform(rng, base_bounds)
                candidate[:2] += q_current[
                    [int(robot_cfg.DoFs.LinearX), int(robot_cfg.DoFs.LinearY)]
                ]
                candidate[2] = rng.uniform(*task["base_goal_rot_range"])
                if (
                    np.linalg.norm(candidate[:2] - q_current[-3:-1])
                    >= float(task["base_goal_minimum_distance"])
                    and np.all(candidate[:2] >= workspace[:, 0])
                    and np.all(candidate[:2] <= workspace[:, 1])
                ):
                    break
            else:
                raise RuntimeError(f"Could not sample a base goal for environment {env_id}")
            base_goal[env_id] = torch.as_tensor(candidate, device=device, dtype=torch.float32)

            solved = None
            for _ in range(100):
                right = sample_uniform(rng, task["right_arm_goal_range"])
                left = sample_uniform(rng, task["left_arm_goal_range"])
                right_pose = np.eye(4)
                left_pose = np.eye(4)
                right_pose[:3, 3] = right
                left_pose[:3, 3] = left
                started = time.perf_counter()
                candidate_q, info = kinematics.inverse_kinematics(
                    [right_pose, left_pose], q_current
                )
                ik_time += time.perf_counter() - started
                ik_queries += 1
                # The shared benchmark scores end-effector position only, so
                # it may retain a useful solution even when the identity-
                # orientation solve reports non-convergence. Leave tracking
                # margin inside the visible goal sphere: a solution on its
                # boundary will miss the same tolerance after a gravity-loaded
                # PhysX drive contributes a small steady-state error.
                ik_position_tolerance = min(0.025, 0.5 * float(task["arm_goal_size"]))
                if _ik_position_error(info) <= ik_position_tolerance:
                    solved = np.asarray(candidate_q, dtype=float)
                    break
                ik_failures += 1
            if solved is None:
                raise RuntimeError(f"Could not solve dual-arm IK for environment {env_id}")
            right_goal_local[env_id] = torch.as_tensor(right, device=device)
            left_goal_local[env_id] = torch.as_tensor(left, device=device)
            target_position[env_id] = torch.as_tensor(solved, device=device, dtype=torch.float32)
            target_position[env_id, int(robot_cfg.DoFs.LinearX)] = base_goal[env_id, 0]
            target_position[env_id, int(robot_cfg.DoFs.LinearY)] = base_goal[env_id, 1]
            target_position[env_id, int(robot_cfg.DoFs.RotYaw)] = base_goal[env_id, 2]

            if obstacle_count:
                # Obstacle keepout is evaluated against the exact shared
                # configuration geometry at the row's reset posture.  This
                # avoids depending on stale PhysX link transforms immediately
                # after an indexed reset (no global physics step is allowed
                # there because other rows must keep advancing independently).
                reset_frames = kinematics.forward_kinematics(q_current)
                robot_centers = np.asarray(
                    [reset_frames[int(frame), :3, 3] for frame in robot_cfg.CollisionVol],
                    dtype=float,
                )
                robot_radii = np.asarray(
                    [geometry.size[0] for geometry in robot_cfg.CollisionVol.values()],
                    dtype=float,
                )
                sampled = []
                obstacle_bounds = np.asarray(task["obstacle_range"], dtype=float)
                obstacle_radius = float(task["obstacle_size"])
                robot_keepout = float(task.get("robot_keepout", 0.0))
                obstacle_keepout = float(task.get("obstacle_keepout", 0.0))
                for obstacle_id in range(obstacle_count):
                    for _ in range(10000):
                        point = sample_uniform(rng, obstacle_bounds)
                        robot_surface_distance = (
                            np.linalg.norm(robot_centers - point[None], axis=1)
                            - robot_radii
                            - obstacle_radius
                        )
                        separated_from_robot = bool(np.all(robot_surface_distance >= robot_keepout))
                        separated_from_obstacles = all(
                            np.linalg.norm(point - other) - 2.0 * obstacle_radius
                            >= obstacle_keepout
                            for other in sampled
                        )
                        if separated_from_robot and separated_from_obstacles:
                            sampled.append(point)
                            break
                    else:
                        raise RuntimeError(
                            f"Could not sample obstacle {obstacle_id} for environment {env_id}"
                        )
                local = torch.as_tensor(np.asarray(sampled), device=device, dtype=torch.float32)
                obstacle_positions_local[env_id] = local
                obstacle_positions_world[env_id] = local + env_origins[env_id]

        def reset_rows(env_ids, reason: str) -> None:
            nonlocal previous_sphere_jacobian
            env_ids = torch.as_tensor(env_ids, device=device, dtype=torch.long)
            if env_ids.numel() == 0:
                return
            if reason != "initial":
                completed_episodes[env_ids] += 1
                if reason == "goal_reached":
                    successful_episodes[env_ids] += 1
                else:
                    timeout_episodes[env_ids] += 1
            # Keep faster rows alive and resampling until every row has met
            # the quota. This prevents stale-goal clones in the viewer.
            agent.reset(env_ids=env_ids)
            episode_steps[env_ids] = 0
            valid_previous_jacobian[env_ids] = False
            if safety_filter is not None:
                safety_filter.reset(env_ids)
            for env_id in env_ids.detach().cpu().tolist():
                sample_episode(int(env_id))
            if not args.headless and obstacle_count:
                # Commit the same new tensor positions used by safety to
                # persistent per-clone markers before the next control step.
                agent.set_visual_obstacles(
                    obstacle_positions_local.detach().cpu().numpy(),
                    float(task["obstacle_size"]),
                    VizColor.obstacle_task,
                )
            print(
                f"[SPARK] {reason} reset: "
                + ", ".join(
                    f"env {env_id} -> episode {int(completed_episodes[env_id].item()) + 1}"
                    for env_id in env_ids.detach().cpu().tolist()
                )
                + ("; goals and obstacles resampled" if obstacle_count else "; goals resampled"),
                flush=True,
            )

        def nominal_control(feedback):
            if robot_cfg.dynamics_order == 1:
                position = feedback["dof_pos_fbk"]
                velocity = feedback["dof_vel_fbk"]
                # A unit gain only advances a simulator-owned position target
                # two percent of the residual per 20 ms cycle.  That target
                # lead is too small to preload AgiBot's gravity-loaded arms,
                # leaving their wrists outside a 5 cm benchmark goal.  Eight
                # keeps the first-order closed-loop response well damped while
                # matching the scalar agent's useful tracking authority.
                kp = torch.full((num_dof,), 8.0, device=device)
                kd = torch.full((num_dof,), 0.1, device=device)
                kp[int(robot_cfg.DoFs.LiftBody)] = 25.0
                kp[int(robot_cfg.DoFs.BodyPitch)] = 20.0
                kd[int(robot_cfg.DoFs.LiftBody)] = 0.2
                kd[int(robot_cfg.DoFs.BodyPitch)] = 0.2
                world_command = kp[None] * (target_position - position) - kd[None] * velocity
            else:
                position = agent.command_pos
                velocity = agent.command_vel
                world_command = 10.0 * (target_position - position) - 5.0 * velocity

            yaw_id = int(robot_cfg.DoFs.RotYaw)
            x_id = int(robot_cfg.DoFs.LinearX)
            y_id = int(robot_cfg.DoFs.LinearY)
            world_command[:, yaw_id] = (
                4.0 if robot_cfg.dynamics_order == 1 else 10.0
            ) * _wrap_angle_torch(target_position[:, yaw_id] - position[:, yaw_id], torch) - (
                0.1 if robot_cfg.dynamics_order == 1 else 5.0
            ) * velocity[:, yaw_id]
            cosine, sine = torch.cos(position[:, yaw_id]), torch.sin(position[:, yaw_id])
            x_world = world_command[:, x_id].clone()
            y_world = world_command[:, y_id].clone()
            world_command[:, x_id] = cosine * x_world + sine * y_world
            world_command[:, y_id] = -sine * x_world + cosine * y_world
            return torch.clamp(world_command, lower_control, upper_control)

        def sphere_control_jacobian():
            centers, world_offset, kin = current_sphere_geometry()
            link_jacobian = kin["body_jacobian_w"][:, collision_model.body_ids]
            column_offset = link_jacobian.shape[-1] - len(kin["joint_names"])
            joint_columns = column_offset + kin["body_joint_ids"]
            linear = link_jacobian[:, :, :3, joint_columns]
            angular = link_jacobian[:, :, 3:, joint_columns]
            jacobian = linear + torch.cross(
                angular.movedim(-1, -2),
                world_offset[:, :, None, :],
                dim=-1,
            ).movedim(-2, -1)

            yaw_id = int(robot_cfg.DoFs.RotYaw)
            x_id = int(robot_cfg.DoFs.LinearX)
            y_id = int(robot_cfg.DoFs.LinearY)
            yaw = agent.command_pos[:, yaw_id]
            cosine, sine = torch.cos(yaw), torch.sin(yaw)
            x_world = jacobian[..., x_id].clone()
            y_world = jacobian[..., y_id].clone()
            jacobian[..., x_id] = cosine[:, None, None] * x_world + sine[:, None, None] * y_world
            jacobian[..., y_id] = -sine[:, None, None] * x_world + cosine[:, None, None] * y_world
            return centers, jacobian

        def apply_safety(reference):
            nonlocal previous_sphere_jacobian, latest_safety_visualization
            if obstacle_count == 0:
                closest_sphere_distance.fill_(torch.inf)
                latest_safety_visualization = None
                return reference, None
            closest_sphere_distance.fill_(torch.inf)
            centers, jacobian = sphere_control_jacobian()
            query = collision_backend.query_environment_nearest(
                centers,
                collision_model.radii,
                obstacle_cloud,
                collision_model.environment_mask,
                nearest_k=1,
            )
            distances = torch.where(
                query.valid_mask, query.distance, torch.full_like(query.distance, torch.inf)
            )
            closest_sphere_distance.scatter_reduce_(
                1,
                query.robot_geometry_id,
                distances,
                reduce="amin",
                include_self=False,
            )
            minimum_observed_distance.copy_(
                torch.minimum(minimum_observed_distance, distances.min(dim=1).values)
            )
            if safety_filter is None:
                previous_sphere_jacobian = jacobian.detach().clone()
                latest_safety_visualization = None
                return reference, None
            batch_ids = torch.arange(args.num_envs, device=device)[:, None]
            point_jacobian = jacobian[batch_ids, query.robot_geometry_id]
            if robot_cfg.dynamics_order == 1:
                constraints = safety_index.build(query, point_jacobian)
            else:
                if previous_sphere_jacobian is None:
                    jacobian_dot = torch.zeros_like(jacobian)
                else:
                    jacobian_dot = (jacobian - previous_sphere_jacobian) / control_period
                    jacobian_dot = torch.where(
                        valid_previous_jacobian[:, None, None, None],
                        torch.clamp(jacobian_dot, -10.0, 10.0),
                        torch.zeros_like(jacobian_dot),
                    )
                point_jacobian_dot = jacobian_dot[batch_ids, query.robot_geometry_id]
                control_velocity = agent.command_vel.clone()
                yaw_id = int(robot_cfg.DoFs.RotYaw)
                x_id = int(robot_cfg.DoFs.LinearX)
                y_id = int(robot_cfg.DoFs.LinearY)
                cosine, sine = (
                    torch.cos(agent.command_pos[:, yaw_id]),
                    torch.sin(agent.command_pos[:, yaw_id]),
                )
                x_world = control_velocity[:, x_id].clone()
                y_world = control_velocity[:, y_id].clone()
                control_velocity[:, x_id] = cosine * x_world + sine * y_world
                control_velocity[:, y_id] = -sine * x_world + cosine * y_world
                constraints = safety_index.build(
                    query,
                    point_jacobian,
                    control_velocity=control_velocity,
                    point_jacobian_dot=point_jacobian_dot,
                )
            previous_sphere_jacobian = jacobian.detach().clone()
            valid_previous_jacobian.fill_(True)
            safe, info = safety_filter.filter(
                reference,
                constraints,
                lower_limit=lower_control,
                upper_limit=upper_control,
            )
            if args.render_safety_trigger_constraints or args.render_safety_violations:
                nominal_residual = constraints.lower - torch.einsum(
                    "bcu,bu->bc", constraints.A, reference
                )
                safe_residual = constraints.lower - torch.einsum("bcu,bu->bc", constraints.A, safe)
                latest_safety_visualization = {
                    "start": query.witness_robot,
                    "end": query.witness_environment,
                    "trigger_mask": constraints.active_mask & (nominal_residual > 1.0e-5),
                    "violation_mask": constraints.active_mask & (safe_residual > 1.0e-5),
                }
            else:
                latest_safety_visualization = None
            safety_trigger_count.add_(info["triggered"].long())
            return safe, info

        def completion(feedback):
            position = feedback["dof_pos_fbk"]
            right_goal_world, left_goal_world = arm_goals_world(position)
            ee = current_ee_positions()
            right_error = torch.linalg.vector_norm(ee[:, 0] - right_goal_world, dim=1)
            left_error = torch.linalg.vector_norm(ee[:, 1] - left_goal_world, dim=1)
            base_error = torch.linalg.vector_norm(
                position[:, [int(robot_cfg.DoFs.LinearX), int(robot_cfg.DoFs.LinearY)]]
                - base_goal[:, :2],
                dim=1,
            )
            yaw_error = torch.abs(
                _wrap_angle_torch(base_goal[:, 2] - position[:, int(robot_cfg.DoFs.RotYaw)], torch)
            )
            arm_reached = (right_error < float(task["arm_goal_size"])) & (
                left_error < float(task["arm_goal_size"])
            )
            base_reached = (base_error < float(task["base_goal_size"])) & (
                yaw_error < float(task["base_goal_yaw_size"])
            )
            return arm_reached & base_reached, right_error, left_error, base_error, yaw_error

        def render(feedback):
            if args.headless:
                return
            position = feedback["dof_pos_fbk"]
            right_world, left_world = arm_goals_world(position)
            right_cpu = right_world.detach().cpu().numpy()
            left_cpu = left_world.detach().cpu().numpy()
            base_marker = torch.empty_like(base_goal)
            base_marker[:, :2] = base_goal[:, :2]
            base_marker[:, 2] = float(task["base_goal_range"][2][0])
            base_cpu = (base_marker + env_origins).detach().cpu().numpy()
            for env_id in range(args.num_envs):
                agent.render_sphere(
                    right_cpu[env_id],
                    np.eye(3),
                    np.full(3, float(task["arm_goal_size"])),
                    VizColor.goal,
                )
                agent.render_sphere(
                    left_cpu[env_id],
                    np.eye(3),
                    np.full(3, float(task["arm_goal_size"])),
                    VizColor.goal,
                )
                agent.render_sphere(
                    base_cpu[env_id],
                    np.eye(3),
                    np.full(3, float(task["base_goal_size"])),
                    VizColor.goal,
                )
            if args.render_robot_collision_volumes:
                centers, _, _ = current_sphere_geometry(include_jacobian=False)
                centers_cpu = centers.detach().cpu().numpy()
                distances_cpu = closest_sphere_distance.detach().cpu().numpy()
                radii_cpu = collision_model.radii.detach().cpu().numpy()
                for env_id in range(
                    min(args.num_envs, max(0, int(args.max_visualized_collision_envs)))
                ):
                    for sphere_id, radius in enumerate(radii_cpu):
                        color = collision_volume_distance_color(
                            distances_cpu[env_id, sphere_id],
                            minimum_distance,
                            opacity_scale=agent.collision_volume_opacity_scale,
                            opacity_floor=agent.collision_volume_opacity_floor,
                        )
                        agent.render_sphere(
                            centers_cpu[env_id, sphere_id],
                            np.eye(3),
                            np.full(3, radius),
                            color,
                        )
            if latest_safety_visualization is not None:
                visual = latest_safety_visualization
                agent.set_visual_safety_constraints(
                    visual["start"].detach().cpu().numpy(),
                    visual["end"].detach().cpu().numpy(),
                    trigger_mask=(
                        visual["trigger_mask"].detach().cpu().numpy()
                        if args.render_safety_trigger_constraints
                        else None
                    ),
                    violation_mask=(
                        visual["violation_mask"].detach().cpu().numpy()
                        if args.render_safety_violations
                        else None
                    ),
                )
            agent.render()

        reset_rows(torch.arange(args.num_envs, device=device), "initial")
        base_dof_ids = [
            int(robot_cfg.DoFs.LinearX),
            int(robot_cfg.DoFs.LinearY),
            int(robot_cfg.DoFs.RotYaw),
        ]
        initial_base_offsets = (
            (base_goal - agent.command_pos[:, base_dof_ids]).detach().cpu().numpy()
        )
        unique_initial_goals = np.unique(np.round(initial_base_offsets, decimals=6), axis=0).shape[
            0
        ]
        print(
            f"[SPARK] AgiBot tensor benchmark ready: {args.num_envs} envs, "
            f"{len(collision_model.body_names)} collision spheres, "
            f"{obstacle_count} independent obstacles/env, device={device}, "
            f"control_period={control_period:.3f}s, safety={args.safe_algo}",
            flush=True,
        )
        print(
            "[SPARK] independently sampled initial base goals: "
            f"{unique_initial_goals}/{args.num_envs} unique; "
            f"relative X [{initial_base_offsets[:, 0].min():+.3f}, "
            f"{initial_base_offsets[:, 0].max():+.3f}] m, "
            f"Y [{initial_base_offsets[:, 1].min():+.3f}, "
            f"{initial_base_offsets[:, 1].max():+.3f}] m, "
            f"yaw [{initial_base_offsets[:, 2].min():+.3f}, "
            f"{initial_base_offsets[:, 2].max():+.3f}] rad",
            flush=True,
        )

        profile = {
            "feedback": 0.0,
            "nominal": 0.0,
            "safety": 0.0,
            "physics": 0.0,
            "completion_reset": 0.0,
            "render": 0.0,
        }
        measured_steps = 0
        measured_started = None
        step_index = 0
        latest_errors = None
        while simulation_app.is_running():
            if args.num_steps >= 0 and step_index >= args.num_steps:
                break
            cycle_started = time.perf_counter()
            feedback = profile_call("feedback", agent.get_feedback, profile)
            reference = profile_call("nominal", lambda: nominal_control(feedback), profile)
            safe_control, _ = profile_call("safety", lambda: apply_safety(reference), profile)
            profile_call("physics", lambda: agent.step(safe_control), profile)
            feedback = agent.get_feedback()

            def update_completion_and_resets():
                nonlocal latest_errors
                episode_steps.add_(1)
                reached, right_error, left_error, base_error, yaw_error = completion(feedback)
                latest_errors = (right_error, left_error, base_error, yaw_error)
                reached_ids = torch.nonzero(reached).flatten()
                timeout_ids = torch.nonzero(
                    (episode_steps >= args.max_episode_length) & ~reached
                ).flatten()
                finished_ids = torch.cat((reached_ids, timeout_ids))
                if finished_ids.numel():
                    last_finished_right_error[finished_ids] = right_error[finished_ids]
                    last_finished_left_error[finished_ids] = left_error[finished_ids]
                    last_finished_base_error[finished_ids] = base_error[finished_ids]
                    last_finished_yaw_error[finished_ids] = yaw_error[finished_ids]
                reset_rows(reached_ids, "goal_reached")
                reset_rows(timeout_ids, "timeout")
                return (
                    reset_quota_reached(completed_episodes, args.num_resets),
                    bool(reached_ids.numel() or timeout_ids.numel()),
                )

            benchmark_complete, reset_occurred = profile_call(
                "completion_reset", update_completion_and_resets, profile
            )
            if reset_occurred:
                # Render the post-reset articulation and the newly sampled
                # row-local goals together. Reusing pre-reset feedback made a
                # completed clone appear frozen until a later viewport update.
                feedback = agent.get_feedback()
            if not args.headless and step_index % args.render_every == 0:
                profile_call("render", lambda: render(feedback), profile)
                if viewport_recorder is not None:
                    viewport_recorder.schedule(step_index * control_period)

            if step_index == args.warmup_steps:
                synchronize()
                measured_started = time.perf_counter()
                measured_steps = 0
            elif step_index > args.warmup_steps:
                measured_steps += 1
            step_index += 1
            if benchmark_complete:
                break
            if args.real_time:
                remaining = control_period - (time.perf_counter() - cycle_started)
                if remaining > 0.0:
                    time.sleep(remaining)

        synchronize()
        measured_elapsed = (
            time.perf_counter() - measured_started
            if measured_started is not None and measured_steps
            else 0.0
        )
        control_rate = measured_steps / measured_elapsed if measured_elapsed > 0.0 else 0.0
        aggregate_rate = args.num_envs * control_rate
        if latest_errors is None:
            feedback = agent.get_feedback()
            _, right_error, left_error, base_error, yaw_error = completion(feedback)
        else:
            right_error, left_error, base_error, yaw_error = latest_errors
        right_error = torch.where(
            torch.isfinite(last_finished_right_error), last_finished_right_error, right_error
        )
        left_error = torch.where(
            torch.isfinite(last_finished_left_error), last_finished_left_error, left_error
        )
        base_error = torch.where(
            torch.isfinite(last_finished_base_error), last_finished_base_error, base_error
        )
        yaw_error = torch.where(
            torch.isfinite(last_finished_yaw_error), last_finished_yaw_error, yaw_error
        )
        print(
            "SPARK AgiBot Isaac tensor benchmark:\n"
            f"  environments: {args.num_envs}\n"
            f"  test case: {args.test_case}\n"
            f"  control steps: {step_index}\n"
            f"  control rate: {control_rate:.2f} Hz\n"
            f"  aggregate env steps/s: {aggregate_rate:.2f}\n"
            f"  simulated control period: {control_period:.4f} s\n"
            f"  completed episodes per env: {completed_episodes.detach().cpu().tolist()}\n"
            f"  successful episodes per env: {successful_episodes.detach().cpu().tolist()}\n"
            f"  timeout episodes per env: {timeout_episodes.detach().cpu().tolist()}\n"
            f"  final right EE error mean/max: {right_error.mean().item():.4f}/"
            f"{right_error.max().item():.4f} m\n"
            f"  final left EE error mean/max: {left_error.mean().item():.4f}/"
            f"{left_error.max().item():.4f} m\n"
            f"  final base position error mean/max: {base_error.mean().item():.4f}/"
            f"{base_error.max().item():.4f} m\n"
            f"  final base yaw error mean/max: {yaw_error.mean().item():.4f}/"
            f"{yaw_error.max().item():.4f} rad\n"
            f"  IK queries/failures/solve time: {ik_queries}/{ik_failures}/{ik_time:.3f} s",
            flush=True,
        )
        if obstacle_count:
            print(
                "Tensor safety:\n"
                f"  algorithm: {args.safe_algo}\n"
                f"  trigger steps per env: {safety_trigger_count.detach().cpu().tolist()}\n"
                f"  minimum distance per env: "
                f"{minimum_observed_distance.detach().cpu().tolist()}",
                flush=True,
            )
        if args.profile_frequency and step_index:
            denominator = max(step_index, 1)
            print("Synchronized mean stage latency:", flush=True)
            for name, elapsed in profile.items():
                print(f"  {name:18s}: {1000.0 * elapsed / denominator:8.3f} ms/step")
        if args.num_envs > 1:
            base_offsets = (base_goal - agent.default_dof_pos[:, -3:]).detach().cpu().numpy()
            print(
                "  sampled base goal offsets:\n    "
                + "\n    ".join(
                    f"env {env_id}: ({goal[0]:+.3f}, {goal[1]:+.3f}, {goal[2]:+.3f})"
                    for env_id, goal in enumerate(base_offsets)
                ),
                flush=True,
            )
        return 0
    except BaseException:
        # Isaac's fast shutdown can otherwise close Kit before Python emits an
        # unhandled traceback, which turns a failed benchmark into a silent
        # zero-output exit on some packaged Isaac Sim builds.
        import traceback

        traceback.print_exc()
        raise
    finally:
        if viewport_recorder is not None and not viewport_recorder.closed:
            if viewport_recorder.pending and agent is not None:
                viewport_recorder.drain(agent.render)
            viewport_recorder.close()
        if agent is not None:
            agent.close()
        simulation_app.close(wait_for_replicator=False)


def main(argv=None) -> int:
    args = _build_parser().parse_args(argv)
    _validate_args(args)
    return _run(args)


if __name__ == "__main__":
    raise SystemExit(main())
