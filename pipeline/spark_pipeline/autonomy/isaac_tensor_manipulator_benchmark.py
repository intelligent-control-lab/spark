"""Robot-neutral batched manipulator benchmark for Isaac Sim.

Robot launchers provide embodiment-specific end-effector and workspace
grounding through ``SPARK_MANIPULATOR_BENCHMARK_GROUNDING``.  Physics,
collision distance calculation, nominal control, safety filtering, resets,
and visualization remain batched on the selected Torch device.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import time


_GROUNDING_ENV = "SPARK_MANIPULATOR_BENCHMARK_GROUNDING"


def _parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-config", required=True)
    parser.add_argument(
        "--test-case",
        choices=(
            "arm_goal_static_v0",
            "arm_goal_static_v1",
            "whole_goal_static_v0",
            "whole_goal_static_v1",
        ),
    )
    parser.add_argument("--display-test-case", required=True)
    parser.add_argument("--num-envs", type=int, default=1)
    parser.add_argument("--num-resets", type=int, default=10)
    parser.add_argument("--max-episode-length", type=int, default=1000)
    parser.add_argument("--num-steps", type=int, default=10000)
    parser.add_argument("--warmup-steps", type=int, default=20)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--device")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--show-simulation-info", action="store_true")
    parser.add_argument("--real-time", action=argparse.BooleanOptionalAction, default=False)
    parser.add_argument("--dt", type=float, required=True)
    parser.add_argument("--control-decimation", type=int, required=True)
    parser.add_argument("--render-every", type=int, default=1)
    parser.add_argument("--safe-algo", choices=("bypass", "ssa", "rssa"))
    parser.add_argument("--minimum-distance", type=float)
    parser.add_argument("--activation-distance", type=float)
    parser.add_argument(
        "--enable-self-collision",
        action=argparse.BooleanOptionalAction,
        default=False,
    )
    parser.add_argument("--safety-qp-iterations", type=int, default=20)
    parser.add_argument("--safety-slack-weight", type=float, default=1000.0)
    parser.add_argument(
        "--render-robot-collision-volumes",
        action=argparse.BooleanOptionalAction,
        default=True,
    )
    parser.add_argument("--max-visualized-collision-envs", type=int, default=1)
    parser.add_argument("--profile-frequency", action="store_true")
    parser.add_argument("--record-video-path")
    parser.add_argument("--record-gif-path")
    parser.add_argument("--record-fps", type=float, default=10.0)
    parser.add_argument("--record-width", type=int, default=960)
    parser.add_argument("--record-height", type=int, default=540)
    return parser


def _validate(args):
    if args.num_envs < 1:
        raise ValueError("--num-envs must be positive")
    if args.num_resets == 0 or args.num_resets < -1:
        raise ValueError("--num-resets must be positive or -1")
    if args.max_episode_length < 1 or args.control_decimation < 1 or args.dt <= 0.0:
        raise ValueError("episode length and simulator timing must be positive")
    if args.num_steps == 0 or args.num_steps < -1:
        raise ValueError("--num-steps must be positive or -1")
    if args.record_fps <= 0.0 or args.record_width < 1 or args.record_height < 1:
        raise ValueError("recording FPS, width, and height must be positive")


def _rotate_xyzw(quaternion, vector, torch):
    q_vector = quaternion[..., :3]
    q_scalar = quaternion[..., 3:4]
    tangent = 2.0 * torch.cross(q_vector, vector, dim=-1)
    return vector + q_scalar * tangent + torch.cross(q_vector, tangent, dim=-1)


def _multiply_xyzw(left, right, torch):
    """Compose batched xyzw quaternions as ``left * right``."""

    left_xyz, left_w = left[..., :3], left[..., 3:4]
    right_xyz, right_w = right[..., :3], right[..., 3:4]
    xyz = left_w * right_xyz + right_w * left_xyz + torch.cross(left_xyz, right_xyz, dim=-1)
    w = left_w * right_w - torch.sum(left_xyz * right_xyz, dim=-1, keepdim=True)
    return torch.cat((xyz, w), dim=-1)


def _ik_position_error(info):
    if not isinstance(info, dict):
        return float("inf")
    return float(getattr(info.get("ik_result"), "position_error", float("inf")))


def _ik_orientation_error(info):
    if not isinstance(info, dict):
        return float("inf")
    return float(getattr(info.get("ik_result"), "orientation_error", float("inf")))


def _run(args):
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
        from scipy.spatial.transform import Rotation as R

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
            TensorSafetyConstraints,
        )
        from spark_task.autonomy.benchmark_goals import arm_goal_pair_is_separated
        from spark_utils import VizColor, collision_volume_distance_color

        serialized_grounding = os.environ.get(_GROUNDING_ENV)
        if serialized_grounding is None:
            raise RuntimeError(
                "This internal runtime requires a robot launcher to supply benchmark grounding"
            )
        grounding = json.loads(serialized_grounding)
        if grounding["robot_config"] != args.robot_config:
            raise ValueError("Benchmark grounding does not match --robot-config")
        use_dual_arm = bool(grounding["use_dual_arm"])
        mobile_base = bool(grounding.get("mobile_base", False))
        goal_rotation = np.asarray(
            grounding.get("arm_goal_rotation", np.eye(3)), dtype=float
        ).reshape(3, 3)

        device = args.device or ("cpu" if args.num_envs == 1 else "cuda:0")
        if device.startswith("cuda") and not torch.cuda.is_available():
            raise RuntimeError(
                f"CUDA device {device!r} was requested but Torch CUDA is unavailable"
            )
        robot_cfg = getattr(spark_robot, args.robot_config)()
        articulation_spec = getattr(robot_cfg, "isaac_articulation", None)
        physical_self_contact_enabled = bool(
            articulation_spec is not None and articulation_spec.allow_self_collision
        )
        self_collision_safety_enabled = bool(args.enable_self_collision)
        if robot_cfg.dynamics_order != 1:
            raise ValueError(
                "The manipulator tensor benchmark currently requires first-order dynamics"
            )
        kinematics = getattr(spark_robot, robot_cfg.kinematics_class_name)(robot_cfg)
        goal_quaternion = torch.as_tensor(
            R.from_matrix(goal_rotation).as_quat(), device=device, dtype=torch.float32
        )
        arm_goal_position_only = bool(grounding.get("arm_goal_position_only", False))
        arm_goal_orientation_size = float(grounding.get("arm_goal_orientation_size", 0.1))
        task = resolve_benchmark_test_case(args.test_case).parameters()
        task["max_episode_length"] = args.max_episode_length
        task["use_dual_arm"] = use_dual_arm
        task["right_arm_goal_range"] = grounding["right_arm_goal_range"]
        task["left_arm_goal_range"] = grounding["left_arm_goal_range"]
        task["arm_goal_pair_keepout"] = float(grounding.get("arm_goal_pair_keepout", 0.0))
        if "obstacle_range" in grounding:
            task["obstacle_range"] = grounding["obstacle_range"]
        if "robot_keepout" in grounding:
            task["robot_keepout"] = grounding["robot_keepout"]
        args.safe_algo = args.safe_algo or (
            "rssa" if int(task["num_obstacle_task"]) or self_collision_safety_enabled else "bypass"
        )
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
            raise ValueError("activation distance cannot be below minimum distance")

        # Declarative articulations share one Isaac Core tensor path.  This
        # also keeps reduced Unitree arm/mobile configurations independent of
        # the specialized IsaacLab plant used by WBT, Sport, and SONIC.
        agent_type = (
            spark_agent.ConfiguredIsaacTensorAgent
            if articulation_spec is not None
            else getattr(spark_agent, robot_cfg.agent_class_name("isaac"))
        )
        agent = agent_type(
            robot_cfg,
            num_envs=args.num_envs,
            device=device,
            tensor_api=True,
            render=not args.headless,
            enable_viewer=not args.headless,
            viewer_show_simulation_info=args.show_simulation_info,
            render_on_step=False,
            dt=args.dt,
            control_decimation=args.control_decimation,
            dynamics_backend="simulator",
            allow_self_collision=physical_self_contact_enabled,
            viewer_config=grounding.get("viewer_config", DEFAULT_VIEWER_CONFIG),
            preserve_viewer_orientation=True,
        )
        agent.attach_simulation_app(simulation_app)
        control_period = args.dt * args.control_decimation
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
        if num_dof != len(robot_cfg.Control):
            raise RuntimeError("Tensor joint tracking requires one configured control per DoF")
        if mobile_base and not all(
            name in robot_cfg.DoFs.__members__ for name in ("LinearX", "LinearY", "RotYaw")
        ):
            raise RuntimeError("Mobile benchmark grounding requires planar x/y/yaw DoFs")

        body_kinematics = agent.get_body_kinematics(include_jacobian=False)
        selected_joint_ids = set(agent.joint_indices.detach().cpu().tolist())
        passive_joint_ids = torch.as_tensor(
            [
                joint_id
                for joint_id in range(len(body_kinematics["joint_names"]))
                if joint_id not in selected_joint_ids
            ],
            device=device,
            dtype=torch.long,
        )
        joint_id_by_name = {
            name: joint_id for joint_id, name in enumerate(body_kinematics["joint_names"])
        }
        requested_hold_joint_names = tuple(grounding.get("hold_joint_names", ()))
        missing_hold_joint_names = [
            name for name in requested_hold_joint_names if name not in joint_id_by_name
        ]
        if missing_hold_joint_names:
            raise RuntimeError(
                "Held benchmark joints missing from Isaac: " + ", ".join(missing_hold_joint_names)
            )
        hold_joint_names = [name for name in requested_hold_joint_names if name in joint_id_by_name]
        selected_joint_to_dof = {
            joint_id: dof_id
            for dof_id, joint_id in enumerate(agent.joint_indices.detach().cpu().tolist())
        }
        hold_dof_ids_list = [
            selected_joint_to_dof[joint_id_by_name[name]]
            for name in hold_joint_names
            if joint_id_by_name[name] in selected_joint_to_dof
        ]
        hold_dof_ids = torch.as_tensor(
            hold_dof_ids_list,
            device=device,
            dtype=torch.long,
        )
        hold_passive_joint_ids = torch.as_tensor(
            [
                joint_id_by_name[name]
                for name in hold_joint_names
                if joint_id_by_name[name] not in selected_joint_ids
            ],
            device=device,
            dtype=torch.long,
        )
        collision_model = build_link_sphere_model(
            robot_cfg, body_kinematics["body_names"], device=device
        )
        if collision_model.unmapped_frames:
            raise RuntimeError(
                "Configured collision frames do not map to Isaac bodies: "
                + ", ".join(collision_model.unmapped_frames)
            )
        body_ids = {name: index for index, name in enumerate(body_kinematics["body_names"])}
        ee_names = [grounding["right_ee_body"]]
        if use_dual_arm:
            ee_names.append(grounding["left_ee_body"])
        missing_ee = [name for name in ee_names if name not in body_ids]
        if missing_ee:
            raise RuntimeError("End-effector bodies missing from Isaac: " + ", ".join(missing_ee))
        ee_body_ids = torch.as_tensor(
            [body_ids[name] for name in ee_names], device=device, dtype=torch.long
        )
        default_ee_offset = grounding["ee_local_offset"]
        ee_local_offsets = [grounding.get("right_ee_local_offset", default_ee_offset)]
        if use_dual_arm:
            ee_local_offsets.append(grounding.get("left_ee_local_offset", default_ee_offset))
        ee_local_offsets = torch.as_tensor(
            ee_local_offsets,
            device=device,
            dtype=torch.float32,
        )
        default_ee_rotation = grounding.get("ee_local_rotation", np.eye(3))
        ee_local_rotations = [
            np.asarray(
                grounding.get("right_ee_local_rotation", default_ee_rotation), dtype=float
            ).reshape(3, 3)
        ]
        if use_dual_arm:
            ee_local_rotations.append(
                np.asarray(
                    grounding.get("left_ee_local_rotation", default_ee_rotation), dtype=float
                ).reshape(3, 3)
            )
        ee_local_quaternions = torch.as_tensor(
            R.from_matrix(np.asarray(ee_local_rotations)).as_quat(),
            device=device,
            dtype=torch.float32,
        )

        # A fixed imported root may be raised above the clone origin (for
        # example, Unitree's pelvis is 0.793 m above the floor).  Benchmark
        # goals and obstacles live in the robot kinematic frame, not at the
        # ground-plane clone origin.
        env_origins = getattr(agent, "kinematic_origins", agent.env_positions).clone()
        target_position = agent.default_dof_pos.clone()
        right_goal_local = torch.zeros(args.num_envs, 3, device=device)
        left_goal_local = torch.zeros_like(right_goal_local)
        base_goal = torch.zeros(args.num_envs, 3, device=device)
        obstacle_count = int(task["num_obstacle_task"])
        obstacle_positions_local = torch.zeros(args.num_envs, obstacle_count, 3, device=device)
        obstacle_positions_world = torch.zeros_like(obstacle_positions_local)
        obstacle_radii = torch.full(
            (args.num_envs, obstacle_count), float(task["obstacle_size"]), device=device
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
        ik_queries = 0
        ik_failures = 0
        ik_time = 0.0

        collision_backend = TorchSphereCollisionBackend()
        safety_filter = None
        safety_index = None
        self_safety_index = None
        has_self_pairs = bool(self_collision_safety_enabled and collision_model.self_pair_i.numel())
        if (obstacle_count or has_self_pairs) and args.safe_algo != "bypass":
            safety_index = FirstOrderTensorSafetyIndex(
                minimum_distance=minimum_distance,
                activation_distance=activation_distance,
                alpha=4.0,
                mode="safe_set",
            )
            self_safety_index = FirstOrderTensorSafetyIndex(
                minimum_distance=0.0,
                activation_distance=activation_distance,
                alpha=4.0,
                mode="safe_set",
            )
            filter_type = (
                BatchedRelaxedQPSafetyFilter if args.safe_algo == "rssa" else BatchedQPSafetyFilter
            )
            filter_kwargs = {"iterations": args.safety_qp_iterations, "warm_start": True}
            if args.safe_algo == "rssa":
                filter_kwargs["slack_weight"] = args.safety_slack_weight
            safety_filter = filter_type(**filter_kwargs)

        control_limits = torch.as_tensor(
            [float(robot_cfg.ControlLimit[control]) for control in robot_cfg.Control],
            device=device,
            dtype=torch.float32,
        )
        lower_control = -control_limits[None]
        upper_control = control_limits[None]
        hold_dof_set = set(hold_dof_ids_list)
        hold_control_ids = [
            control_id for control_id, dof_id in agent.control_to_dof if dof_id in hold_dof_set
        ]
        if len(hold_control_ids) != len(hold_dof_ids_list):
            raise RuntimeError("Every selected held joint must have one configured control")
        if hold_control_ids:
            lower_control[:, hold_control_ids] = 0.0
            upper_control[:, hold_control_ids] = 0.0
        closest_sphere_distance = torch.full(
            (args.num_envs, len(collision_model.body_names)), torch.inf, device=device
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

        def current_spheres(*, include_jacobian=True):
            kin = agent.get_body_kinematics(include_jacobian=include_jacobian)
            positions = kin["body_position_w"][:, collision_model.body_ids]
            quaternions = kin["body_quaternion_w"][:, collision_model.body_ids]
            local = collision_model.local_offsets[None].expand(args.num_envs, -1, -1)
            world_offset = _rotate_xyzw(quaternions, local, torch)
            return positions + world_offset, world_offset, kin

        def current_end_effector_poses():
            kin = agent.get_body_kinematics(include_jacobian=False)
            positions = kin["body_position_w"][:, ee_body_ids]
            quaternions = kin["body_quaternion_w"][:, ee_body_ids]
            offsets = ee_local_offsets[None].expand(args.num_envs, -1, -1)
            local_quaternions = ee_local_quaternions[None].expand(args.num_envs, -1, -1)
            tool_quaternions = _multiply_xyzw(quaternions, local_quaternions, torch)
            return positions + _rotate_xyzw(quaternions, offsets, torch), tool_quaternions

        def current_end_effectors():
            return current_end_effector_poses()[0]

        def arm_goals_world(dof_position):
            if not mobile_base:
                return right_goal_local + env_origins, left_goal_local + env_origins
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

        def sample_episode(env_id):
            nonlocal ik_queries, ik_failures, ik_time
            episode = int(completed_episodes[env_id].item())
            rng = np.random.RandomState(args.seed + 1009 * env_id + 7919 * episode)
            q_current = agent.command_pos[env_id].detach().cpu().numpy().astype(float)
            if mobile_base:
                base_bounds = np.asarray(task["base_goal_range"], dtype=float)
                workspace = np.asarray(task["base_goal_workspace_range"], dtype=float)
                x_id = int(robot_cfg.DoFs.LinearX)
                y_id = int(robot_cfg.DoFs.LinearY)
                for _ in range(10000):
                    candidate_base = sample_uniform(rng, base_bounds)
                    if bool(task.get("base_goal_relative_to_current", False)):
                        candidate_base[:2] += q_current[[x_id, y_id]]
                    candidate_base[2] = rng.uniform(*task["base_goal_rot_range"])
                    if (
                        np.linalg.norm(candidate_base[:2] - q_current[[x_id, y_id]])
                        >= float(task["base_goal_minimum_distance"])
                        and np.all(candidate_base[:2] >= workspace[:, 0])
                        and np.all(candidate_base[:2] <= workspace[:, 1])
                    ):
                        break
                else:
                    raise RuntimeError(f"Could not sample a base goal for environment {env_id}")
                base_goal[env_id] = torch.as_tensor(
                    candidate_base, device=device, dtype=torch.float32
                )

            sampled_obstacles = []
            arm_goal_minimum_distance = float(task.get("arm_goal_minimum_distance", 0.0))
            reset_frames = None
            right_start = None
            left_start = None
            if arm_goal_minimum_distance > 0.0:
                reset_frames = kinematics.forward_kinematics(q_current)
                right_start = reset_frames[int(robot_cfg.Frames.R_ee), :3, 3]
                if use_dual_arm:
                    left_start = reset_frames[int(robot_cfg.Frames.L_ee), :3, 3]
            if obstacle_count:
                if reset_frames is None:
                    reset_frames = kinematics.forward_kinematics(q_current)
                robot_centers = np.asarray(
                    [reset_frames[int(frame), :3, 3] for frame in robot_cfg.CollisionVol]
                )
                robot_radii = np.asarray(
                    [geometry.size[0] for geometry in robot_cfg.CollisionVol.values()]
                )
                bounds = np.asarray(task["obstacle_range"], dtype=float)
                radius = float(task["obstacle_size"])
                # A safety benchmark must not begin in violation merely
                # because its layout-level keep-out is looser than the active
                # controller clearance. Keep a small mapping tolerance between
                # Pinocchio sampling geometry and Isaac link geometry.
                robot_keepout = max(float(task.get("robot_keepout", 0.0)), minimum_distance + 0.02)
                obstacle_keepout = float(task.get("obstacle_keepout", 0.0))
                base_keepout = float(task.get("base_goal_keepout", 0.3))
                for obstacle_id in range(obstacle_count):
                    for _ in range(10000):
                        point = sample_uniform(rng, bounds)
                        robot_distance = (
                            np.linalg.norm(robot_centers - point[None], axis=1)
                            - robot_radii
                            - radius
                        )
                        separated = bool(np.all(robot_distance >= robot_keepout)) and all(
                            np.linalg.norm(point - other) - 2.0 * radius >= obstacle_keepout
                            for other in sampled_obstacles
                        )
                        if mobile_base:
                            base_marker = np.array(
                                [
                                    candidate_base[0],
                                    candidate_base[1],
                                    float(task["base_goal_range"][2][0]),
                                ]
                            )
                            separated &= np.linalg.norm(point - base_marker) >= base_keepout
                        if separated:
                            sampled_obstacles.append(point)
                            break
                    else:
                        raise RuntimeError(
                            f"Could not sample obstacle {obstacle_id} for environment {env_id}"
                        )
                local = torch.as_tensor(
                    np.asarray(sampled_obstacles), device=device, dtype=torch.float32
                )
                obstacle_positions_local[env_id] = local
                # Obstacle ranges are expressed in the clone's ground frame.
                # Arm goals use ``kinematic_origins`` because their ranges are
                # relative to the raised robot root, but adding that root
                # height here would move the collision-query obstacle above
                # the USD marker created by ``set_visual_obstacles``.
                obstacle_positions_world[env_id] = local + agent.env_positions[env_id]

            solved = None
            for _ in range(100):
                right = sample_uniform(rng, task["right_arm_goal_range"])
                if right_start is not None and (
                    np.linalg.norm(right - right_start) < arm_goal_minimum_distance
                ):
                    continue
                right_pose = np.eye(4)
                right_pose[:3, 3] = right
                right_pose[:3, :3] = goal_rotation
                poses = [right_pose]
                left = right.copy()
                if use_dual_arm:
                    left = sample_uniform(rng, task["left_arm_goal_range"])
                    if left_start is not None and (
                        np.linalg.norm(left - left_start) < arm_goal_minimum_distance
                    ):
                        continue
                    if not arm_goal_pair_is_separated(
                        right,
                        left,
                        float(task.get("arm_goal_pair_keepout", 0.0)),
                    ):
                        continue
                    left_pose = np.eye(4)
                    left_pose[:3, 3] = left
                    left_pose[:3, :3] = goal_rotation
                    poses.append(left_pose)
                arm_keepout = float(task.get("arm_goal_keepout", 0.1))
                if any(
                    np.linalg.norm(goal - obstacle) < arm_keepout
                    for goal in ((right, left) if use_dual_arm else (right,))
                    for obstacle in sampled_obstacles
                ):
                    continue
                started = time.perf_counter()
                ik_options = (
                    [{"orientation_mask": (False, False, False)} for _ in poses]
                    if arm_goal_position_only
                    else None
                )
                if ik_options is None:
                    candidate, info = kinematics.inverse_kinematics(poses, q_current)
                else:
                    candidate, info = kinematics.inverse_kinematics(
                        poses, q_current, target_options=ik_options
                    )
                ik_time += time.perf_counter() - started
                ik_queries += 1
                pose_reached = _ik_position_error(info) <= float(task["arm_goal_size"])
                if not arm_goal_position_only:
                    pose_reached &= _ik_orientation_error(info) <= arm_goal_orientation_size
                if bool(info.get("success", False)) or pose_reached:
                    solved = np.asarray(candidate, dtype=float)
                    break
                ik_failures += 1
            if solved is None:
                raise RuntimeError(f"Could not sample reachable arm goals for environment {env_id}")
            right_goal_local[env_id] = torch.as_tensor(right, device=device)
            left_goal_local[env_id] = torch.as_tensor(left, device=device)
            target_position[env_id] = torch.as_tensor(solved, device=device)
            if hold_dof_ids.numel():
                target_position[env_id, hold_dof_ids] = agent.default_dof_pos[env_id, hold_dof_ids]
            if mobile_base:
                target_position[env_id, int(robot_cfg.DoFs.LinearX)] = base_goal[env_id, 0]
                target_position[env_id, int(robot_cfg.DoFs.LinearY)] = base_goal[env_id, 1]
                target_position[env_id, int(robot_cfg.DoFs.RotYaw)] = base_goal[env_id, 2]

        def reset_rows(env_ids, reason):
            env_ids = torch.as_tensor(env_ids, device=device, dtype=torch.long)
            if env_ids.numel() == 0:
                return
            if reason != "initial":
                completed_episodes[env_ids] += 1
                if reason == "goal_reached":
                    successful_episodes[env_ids] += 1
                else:
                    timeout_episodes[env_ids] += 1
            # Never freeze faster rows at their individual quota. Every
            # completed row gets a physical reset and a fresh task sample;
            # the outer loop stops the complete batch when all rows qualify.
            agent.reset(env_ids=env_ids)
            episode_steps[env_ids] = 0
            if safety_filter is not None:
                safety_filter.reset(env_ids)
            for env_id in env_ids.detach().cpu().tolist():
                sample_episode(int(env_id))
            if not args.headless and obstacle_count:
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
            position = feedback["dof_pos_fbk"]
            velocity = feedback["dof_vel_fbk"]
            position_kp = float(grounding.get("position_kp", 1.0))
            velocity_kd = float(grounding.get("velocity_kd", 0.1))
            reference = position_kp * (target_position - position)
            reference -= velocity_kd * velocity
            if mobile_base:
                yaw_id = int(robot_cfg.DoFs.RotYaw)
                x_id = int(robot_cfg.DoFs.LinearX)
                y_id = int(robot_cfg.DoFs.LinearY)
                yaw_error = (
                    torch.remainder(
                        target_position[:, yaw_id] - position[:, yaw_id] + torch.pi,
                        2.0 * torch.pi,
                    )
                    - torch.pi
                )
                reference[:, yaw_id] = position_kp * yaw_error - velocity_kd * velocity[:, yaw_id]
                cosine, sine = torch.cos(position[:, yaw_id]), torch.sin(position[:, yaw_id])
                x_world = reference[:, x_id].clone()
                y_world = reference[:, y_id].clone()
                reference[:, x_id] = cosine * x_world + sine * y_world
                reference[:, y_id] = -sine * x_world + cosine * y_world
            return torch.clamp(reference, lower_control, upper_control)

        def sphere_control_jacobian():
            centers, world_offset, kin = current_spheres()
            link_jacobian = kin["body_jacobian_w"][:, collision_model.body_ids]
            column_offset = link_jacobian.shape[-1] - len(kin["joint_names"])
            joint_columns = column_offset + kin["body_joint_ids"]
            linear = link_jacobian[:, :, :3, joint_columns]
            angular = link_jacobian[:, :, 3:, joint_columns]
            jacobian = linear + torch.cross(
                angular.movedim(-1, -2), world_offset[:, :, None, :], dim=-1
            ).movedim(-2, -1)
            if mobile_base:
                yaw_id = int(robot_cfg.DoFs.RotYaw)
                x_id = int(robot_cfg.DoFs.LinearX)
                y_id = int(robot_cfg.DoFs.LinearY)
                if jacobian.shape[-1] != len(robot_cfg.Control):
                    # Unitree's abstract mobile base is a free PhysX root, so
                    # its x/y/yaw coordinates have no articulation Jacobian
                    # columns. Expand the physical waist/arm columns into the
                    # robot-config order and analytically add the planar root.
                    model_jacobian = torch.zeros(
                        *jacobian.shape[:-1],
                        len(robot_cfg.Control),
                        device=jacobian.device,
                        dtype=jacobian.dtype,
                    )
                    body_model_ids = getattr(agent, "body_model_dof_ids_tensor", None)
                    if body_model_ids is None or body_model_ids.numel() != jacobian.shape[-1]:
                        raise RuntimeError(
                            "Mobile-base Jacobian does not expose a complete "
                            "physical-to-model DoF mapping"
                        )
                    model_jacobian[..., body_model_ids] = jacobian
                    model_jacobian[..., 0, x_id] = 1.0
                    model_jacobian[..., 1, y_id] = 1.0
                    model_position = agent.command_pos
                    base_world_x = model_position[:, x_id] + env_origins[:, 0]
                    base_world_y = model_position[:, y_id] + env_origins[:, 1]
                    model_jacobian[..., 0, yaw_id] = -(centers[..., 1] - base_world_y[:, None])
                    model_jacobian[..., 1, yaw_id] = centers[..., 0] - base_world_x[:, None]
                    jacobian = model_jacobian
                yaw = agent.command_pos[:, yaw_id]
                cosine, sine = torch.cos(yaw), torch.sin(yaw)
                x_world = jacobian[..., x_id].clone()
                y_world = jacobian[..., y_id].clone()
                jacobian[..., x_id] = (
                    cosine[:, None, None] * x_world + sine[:, None, None] * y_world
                )
                jacobian[..., y_id] = (
                    -sine[:, None, None] * x_world + cosine[:, None, None] * y_world
                )
            return centers, jacobian

        def apply_safety(reference):
            nonlocal latest_safety_visualization
            if obstacle_count == 0 and not has_self_pairs:
                closest_sphere_distance.fill_(torch.inf)
                latest_safety_visualization = None
                return reference
            closest_sphere_distance.fill_(torch.inf)
            centers, jacobian = sphere_control_jacobian()
            batch_ids = torch.arange(args.num_envs, device=device)[:, None]
            constraint_sets = []
            query_sets = []
            if obstacle_count:
                query = collision_backend.query_environment_nearest(
                    centers,
                    collision_model.radii,
                    obstacle_cloud,
                    collision_model.environment_mask,
                    nearest_k=1,
                )
                distances = torch.where(
                    query.valid_mask,
                    query.distance,
                    torch.full_like(query.distance, torch.inf),
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
                if safety_filter is not None:
                    constraint_sets.append(
                        safety_index.build(
                            query,
                            jacobian[batch_ids, query.robot_geometry_id],
                        )
                    )
                    query_sets.append(query)
            if has_self_pairs:
                self_query = collision_backend.query_self(
                    centers,
                    collision_model.radii,
                    collision_model.self_pair_i,
                    collision_model.self_pair_j,
                )
                self_distances = torch.where(
                    self_query.valid_mask,
                    self_query.distance,
                    torch.full_like(self_query.distance, torch.inf),
                )
                for geometry_ids in (
                    self_query.robot_geometry_id,
                    self_query.environment_geometry_id,
                ):
                    closest_sphere_distance.scatter_reduce_(
                        1,
                        geometry_ids,
                        self_distances,
                        reduce="amin",
                        include_self=False,
                    )
                minimum_observed_distance.copy_(
                    torch.minimum(
                        minimum_observed_distance,
                        self_distances.min(dim=1).values,
                    )
                )
                if safety_filter is not None:
                    constraint_sets.append(
                        self_safety_index.build(
                            self_query,
                            jacobian[batch_ids, self_query.robot_geometry_id],
                            other_point_jacobian=jacobian[
                                batch_ids, self_query.environment_geometry_id
                            ],
                        )
                    )
                    query_sets.append(self_query)
            if safety_filter is None:
                latest_safety_visualization = None
                return reference
            constraints = TensorSafetyConstraints(
                A=torch.cat([item.A for item in constraint_sets], dim=1),
                lower=torch.cat([item.lower for item in constraint_sets], dim=1),
                distance=torch.cat([item.distance for item in constraint_sets], dim=1),
                active_mask=torch.cat([item.active_mask for item in constraint_sets], dim=1),
                source="combined",
            )
            safe, info = safety_filter.filter(
                reference,
                constraints,
                lower_limit=lower_control,
                upper_limit=upper_control,
            )
            nominal_residual = constraints.lower - torch.einsum(
                "bcu,bu->bc", constraints.A, reference
            )
            safe_residual = constraints.lower - torch.einsum("bcu,bu->bc", constraints.A, safe)
            latest_safety_visualization = {
                "start": torch.cat([item.witness_robot for item in query_sets], dim=1),
                "end": torch.cat([item.witness_environment for item in query_sets], dim=1),
                "trigger_mask": constraints.active_mask & (nominal_residual > 1.0e-5),
                "violation_mask": constraints.active_mask & (safe_residual > 1.0e-5),
            }
            safety_trigger_count.add_(info["triggered"].long())
            return safe

        def completion(feedback):
            ee, ee_quaternions = current_end_effector_poses()
            right_world, left_world = arm_goals_world(feedback["dof_pos_fbk"])
            right_error = torch.linalg.vector_norm(ee[:, 0] - right_world, dim=1)
            if arm_goal_position_only:
                right_orientation_error = torch.zeros_like(right_error)
            else:
                right_dot = torch.abs(
                    torch.sum(ee_quaternions[:, 0] * goal_quaternion[None], dim=1)
                )
                right_orientation_error = 2.0 * torch.acos(torch.clamp(right_dot, 0.0, 1.0))
            if use_dual_arm:
                left_error = torch.linalg.vector_norm(ee[:, 1] - left_world, dim=1)
                if arm_goal_position_only:
                    left_orientation_error = torch.zeros_like(left_error)
                else:
                    left_dot = torch.abs(
                        torch.sum(ee_quaternions[:, 1] * goal_quaternion[None], dim=1)
                    )
                    left_orientation_error = 2.0 * torch.acos(torch.clamp(left_dot, 0.0, 1.0))
                reached = (right_error < float(task["arm_goal_size"])) & (
                    left_error < float(task["arm_goal_size"])
                )
                reached &= (right_orientation_error < arm_goal_orientation_size) & (
                    left_orientation_error < arm_goal_orientation_size
                )
            else:
                left_error = torch.zeros_like(right_error)
                left_orientation_error = torch.zeros_like(right_orientation_error)
                reached = (right_error < float(task["arm_goal_size"])) & (
                    right_orientation_error < arm_goal_orientation_size
                )
            base_error = torch.zeros_like(right_error)
            yaw_error = torch.zeros_like(right_error)
            if mobile_base:
                position = feedback["dof_pos_fbk"]
                base_error = torch.linalg.vector_norm(
                    position[:, [int(robot_cfg.DoFs.LinearX), int(robot_cfg.DoFs.LinearY)]]
                    - base_goal[:, :2],
                    dim=1,
                )
                yaw_error = torch.abs(
                    torch.remainder(
                        base_goal[:, 2] - position[:, int(robot_cfg.DoFs.RotYaw)] + torch.pi,
                        2.0 * torch.pi,
                    )
                    - torch.pi
                )
                reached &= (base_error < float(task["base_goal_size"])) & (
                    yaw_error < float(task["base_goal_yaw_size"])
                )
            return (
                reached,
                right_error,
                left_error,
                right_orientation_error,
                left_orientation_error,
                base_error,
                yaw_error,
            )

        def render(feedback):
            if args.headless:
                return
            right_goal_world, left_goal_world = arm_goals_world(feedback["dof_pos_fbk"])
            right_world = right_goal_world.detach().cpu().numpy()
            left_world = left_goal_world.detach().cpu().numpy()
            for env_id in range(args.num_envs):
                agent.render_sphere(
                    right_world[env_id],
                    np.eye(3),
                    np.full(3, float(task["arm_goal_size"])),
                    VizColor.goal,
                )
                if use_dual_arm:
                    agent.render_sphere(
                        left_world[env_id],
                        np.eye(3),
                        np.full(3, float(task["arm_goal_size"])),
                        VizColor.goal,
                    )
                if mobile_base:
                    # The task's base-goal z range describes the simulated
                    # root height, not a visualization height.  Draw the
                    # planar target on the floor, with the sphere tangent to
                    # it, so mobile goals do not appear on the robot torso.
                    base_goal_radius = float(task["base_goal_size"])
                    base_marker = (
                        np.array(
                            [
                                float(base_goal[env_id, 0].item()),
                                float(base_goal[env_id, 1].item()),
                                base_goal_radius,
                            ]
                        )
                        + env_origins[env_id].detach().cpu().numpy()
                    )
                    agent.render_sphere(
                        base_marker,
                        np.eye(3),
                        np.full(3, base_goal_radius),
                        VizColor.goal,
                    )
            if args.render_robot_collision_volumes:
                centers, _, _ = current_spheres(include_jacobian=False)
                centers = centers.detach().cpu().numpy()
                distances = closest_sphere_distance.detach().cpu().numpy()
                radii = collision_model.radii.detach().cpu().numpy()
                for env_id in range(min(args.num_envs, max(0, args.max_visualized_collision_envs))):
                    for sphere_id, radius in enumerate(radii):
                        color = collision_volume_distance_color(
                            distances[env_id, sphere_id],
                            minimum_distance,
                            opacity_scale=agent.collision_volume_opacity_scale,
                            opacity_floor=agent.collision_volume_opacity_floor,
                        )
                        agent.render_sphere(
                            centers[env_id, sphere_id],
                            np.eye(3),
                            np.full(3, radius),
                            color,
                        )
            if latest_safety_visualization is not None:
                visual = latest_safety_visualization
                agent.set_visual_safety_constraints(
                    visual["start"].detach().cpu().numpy(),
                    visual["end"].detach().cpu().numpy(),
                    trigger_mask=visual["trigger_mask"].detach().cpu().numpy(),
                    violation_mask=visual["violation_mask"].detach().cpu().numpy(),
                )
            agent.render()

        reset_rows(torch.arange(args.num_envs, device=device), "initial")
        print(
            f"[SPARK] tensor manipulator benchmark ready: {args.num_envs} envs, "
            f"{len(collision_model.body_names)} collision spheres, "
            f"{obstacle_count} independent obstacles/env, device={device}, "
            f"control_period={control_period:.3f}s, safety={args.safe_algo}",
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
        measured_started = None
        measured_steps = 0
        step_index = 0
        while simulation_app.is_running():
            if args.num_steps >= 0 and step_index >= args.num_steps:
                break
            cycle_started = time.perf_counter()
            feedback = profile_call("feedback", agent.get_feedback, profile)
            reference = profile_call("nominal", lambda: nominal_control(feedback), profile)
            safe = profile_call("safety", lambda: apply_safety(reference), profile)
            profile_call(
                "physics",
                lambda: agent.step(
                    safe,
                    hold_dof_indices=hold_dof_ids,
                ),
                profile,
            )
            # Completion and rendering must observe the state produced by the
            # physics step. Reusing the pre-step feedback can send one extra
            # command after a goal is reached and report a pose outside the
            # tolerance that actually completed the episode.
            feedback = agent.get_feedback()

            def update_completion():
                episode_steps.add_(1)
                reached, _, _, _, _, _, _ = completion(feedback)
                reached_ids = torch.nonzero(reached).flatten()
                timeout_ids = torch.nonzero(
                    (episode_steps >= args.max_episode_length) & ~reached
                ).flatten()
                reset_rows(reached_ids, "goal_reached")
                reset_rows(timeout_ids, "timeout")
                return reset_quota_reached(completed_episodes, args.num_resets)

            benchmark_complete = profile_call("completion_reset", update_completion, profile)
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
        control_rate = measured_steps / measured_elapsed if measured_elapsed else 0.0
        final_feedback = agent.get_feedback()
        (
            _,
            right_error,
            left_error,
            right_orientation_error,
            left_orientation_error,
            base_error,
            yaw_error,
        ) = completion(final_feedback)
        active_tracking_error = torch.abs(final_feedback["dof_pos_fbk"] - agent.command_pos)
        passive_joint_drift = None
        hold_joint_drift = None
        hold_drift_values = []
        if hold_dof_ids.numel():
            hold_drift_values.append(
                torch.max(
                    torch.abs(
                        final_feedback["dof_pos_fbk"][:, hold_dof_ids]
                        - agent.default_dof_pos[:, hold_dof_ids]
                    )
                )
            )
        if passive_joint_ids.numel():
            if hasattr(agent, "get_all_joint_positions"):
                all_joint_position = agent.get_all_joint_positions()
            else:
                all_joint_position = agent.articulation.get_joint_positions()
            passive_joint_drift = torch.max(
                torch.abs(
                    all_joint_position[:, passive_joint_ids]
                    - agent.default_all_joint_pos[:, passive_joint_ids]
                )
            ).item()
            if hold_passive_joint_ids.numel():
                hold_drift_values.append(
                    torch.max(
                        torch.abs(
                            all_joint_position[:, hold_passive_joint_ids]
                            - agent.default_all_joint_pos[:, hold_passive_joint_ids]
                        )
                    )
                )
        if hold_drift_values:
            hold_joint_drift = torch.max(torch.stack(hold_drift_values)).item()
        print(
            "SPARK Isaac tensor manipulator benchmark:\n"
            f"  environments: {args.num_envs}\n"
            f"  test case: {args.display_test_case}\n"
            f"  control steps: {step_index}\n"
            f"  control rate: {control_rate:.2f} Hz\n"
            f"  aggregate env steps/s: {args.num_envs * control_rate:.2f}\n"
            f"  completed episodes per env: {completed_episodes.detach().cpu().tolist()}\n"
            f"  successful episodes per env: {successful_episodes.detach().cpu().tolist()}\n"
            f"  timeout episodes per env: {timeout_episodes.detach().cpu().tolist()}\n"
            f"  final right EE error mean/max: {right_error.mean().item():.4f}/"
            f"{right_error.max().item():.4f} m\n"
            + (
                f"  final left EE error mean/max: {left_error.mean().item():.4f}/"
                f"{left_error.max().item():.4f} m\n"
                if use_dual_arm
                else ""
            )
            + (
                f"  final base error mean/max: {base_error.mean().item():.4f}/"
                f"{base_error.max().item():.4f} m\n"
                f"  final yaw error mean/max: {yaw_error.mean().item():.4f}/"
                f"{yaw_error.max().item():.4f} rad\n"
                if mobile_base
                else ""
            )
            + f"  IK queries/failures/solve time: {ik_queries}/{ik_failures}/{ik_time:.3f} s",
            flush=True,
        )
        print(
            "  final active-joint tracking error mean/max: "
            f"{active_tracking_error.mean().item():.6f}/"
            f"{active_tracking_error.max().item():.6f} rad",
            flush=True,
        )
        if passive_joint_drift is not None:
            print(f"  maximum passive-joint drift: {passive_joint_drift:.6f} rad", flush=True)
        if hold_joint_drift is not None:
            print(
                f"  maximum held-joint drift ({', '.join(hold_joint_names)}): "
                f"{hold_joint_drift:.6f} rad",
                flush=True,
            )
        if obstacle_count:
            print(
                "Tensor safety:\n"
                f"  algorithm: {args.safe_algo}\n"
                f"  trigger steps per env: {safety_trigger_count.detach().cpu().tolist()}\n"
                f"  minimum distance per env: {minimum_observed_distance.detach().cpu().tolist()}",
                flush=True,
            )
        if args.profile_frequency and step_index:
            final_ee = current_end_effectors()
            final_right_goal, final_left_goal = arm_goals_world(final_feedback["dof_pos_fbk"])
            print(
                "Final environment-0 Cartesian diagnostic:\n"
                f"  right EE/goal: {final_ee[0, 0].detach().cpu().tolist()} / "
                f"{final_right_goal[0].detach().cpu().tolist()}"
                f"\n  right orientation error: "
                f"{float(right_orientation_error[0].detach().cpu()):.4f} rad"
                + (
                    f"\n  left EE/goal: {final_ee[0, 1].detach().cpu().tolist()} / "
                    f"{final_left_goal[0].detach().cpu().tolist()}"
                    f"\n  left orientation error: "
                    f"{float(left_orientation_error[0].detach().cpu()):.4f} rad"
                    if use_dual_arm
                    else ""
                ),
                flush=True,
            )
            print("Synchronized mean stage latency:", flush=True)
            for name, elapsed in profile.items():
                print(f"  {name:18s}: {1000.0 * elapsed / step_index:8.3f} ms/step")
        return 0
    except BaseException:
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


def main(argv=None):
    args = _parser().parse_args(argv)
    _validate(args)
    return _run(args)


if __name__ == "__main__":
    raise SystemExit(main())
