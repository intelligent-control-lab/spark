from spark_task.base.base_goal_task import BaseGoalTask
from spark_task.utils.task_objects import TaskObject3D
from spark_task.utils.error_handle import ResamplingError
from spark_utils import Geometry, VizColor
from spark_utils import compute_masked_distance_matrix
import numpy as np
from scipy.spatial.transform import Rotation as R
from pathlib import Path
from spark_task.autonomy.benchmark_goals import (
    arm_goal_pair_is_separated,
    benchmark_scenario_fingerprint,
    evaluate_goal_completion,
    sample_benchmark_scenario,
    sample_bounded_position,
)


class BenchmarkTask(BaseGoalTask):
    def __init__(self, robot_cfg, robot_kinematics, agent, **kwargs):
        super().__init__(robot_cfg, robot_kinematics, agent, **kwargs)

        # Initialize task configuration
        self.task_name = kwargs.get("task_name", "BenchmarkTask")  # Name of the task
        self.max_episode_length = kwargs.get(
            "max_episode_length", 200
        )  # Maximum number of steps per episode
        self.robot_keepout = kwargs.get("robot_keepout", 0.0)  # Keepout distance for the robot
        # Benchmark objects can now be sampled independently. ``mode`` remains
        # the backward-compatible default used by generated legacy cases.
        self.obstacle_mode = kwargs.get("obstacle_mode", self.mode)
        self.arm_goal_mode = kwargs.get("arm_goal_mode", self.mode)
        self.base_goal_mode = kwargs.get("base_goal_mode", self.mode)
        self.environment_representation = kwargs.get("environment_representation", "sphere")
        self.points_per_obstacle = max(4, int(kwargs.get("points_per_obstacle", 64)))
        self.minimum_points_per_obstacle = max(1, int(kwargs.get("minimum_points_per_obstacle", 4)))
        self.dynamic_point_count = bool(kwargs.get("dynamic_point_count", True))
        self.regenerate_point_cloud_every_step = bool(
            kwargs.get("regenerate_point_cloud_every_step", False)
        )
        self.point_radius = float(kwargs.get("point_radius", 0.004))
        self.max_visualized_points = max(1, int(kwargs.get("max_visualized_points", 200)))
        self.object_mesh_path = kwargs.get("object_mesh_path", None)
        self.object_mesh_scale = float(kwargs.get("object_mesh_scale", 1.0))
        self.right_arm_goal_rotation = np.asarray(
            kwargs.get("right_arm_goal_rotation", np.eye(3)), dtype=float
        ).reshape(3, 3)
        self.left_arm_goal_rotation = np.asarray(
            kwargs.get("left_arm_goal_rotation", np.eye(3)), dtype=float
        ).reshape(3, 3)
        self._environment_surface_points = None
        self._environment_point_counts = None
        self._environment_point_geometries = None
        self._benchmark_scenario = None

    def _canonical_robot_spheres(self):
        """Return reset-pose collision spheres in the robot root frame."""

        default_dof = np.asarray(
            [self.robot_cfg.DefaultDoFVal[dof] for dof in self.robot_cfg.DoFs], dtype=float
        )
        frames = self.robot_kinematics.forward_kinematics(default_dof)
        centers = []
        radii = []
        for frame_id, geometry in self.robot_cfg.CollisionVol.items():
            if geometry.type != "sphere":
                continue
            centers.append(frames[int(frame_id), :3, 3])
            radii.append(float(geometry.size[0]))
        return np.asarray(centers, dtype=float).reshape(-1, 3), np.asarray(radii, dtype=float)

    def _task_object(self, *, velocity, direction, keep_direction_step, bound, smooth_weight, seed):
        return TaskObject3D(
            velocity=velocity,
            direction=direction,
            keep_direction_step=keep_direction_step,
            bound=bound,
            smooth_weight=smooth_weight,
            _seed=seed,
            dt=self.dt,
        )

    def _init_deterministic_scenario(self):
        """Build the exact NumPy scenario also consumed by Isaac tensor runs."""

        collision_centers, collision_radii = self._canonical_robot_spheres()
        default_dof = np.asarray(
            [self.robot_cfg.DefaultDoFVal[dof] for dof in self.robot_cfg.DoFs], dtype=float
        )
        canonical_frames = self.robot_kinematics.forward_kinematics(default_dof)
        # Scenario sampling is tied to the canonical reset state, not tiny
        # backend-dependent integration drift observed after reset.
        root_position = default_dof[:3].copy()
        root_yaw = 0.0
        right_start = (
            (np.eye(4) @ canonical_frames[self.robot_cfg.Frames.R_ee])[:3, 3] + root_position
            if hasattr(self.robot_cfg.Frames, "R_ee")
            else None
        )
        left_start = (
            (np.eye(4) @ canonical_frames[self.robot_cfg.Frames.L_ee])[:3, 3] + root_position
            if hasattr(self.robot_cfg.Frames, "L_ee")
            else None
        )
        scenario = sample_benchmark_scenario(
            seed=self._seed,
            root_position=root_position,
            root_yaw=root_yaw,
            arm_goal_enabled=self.arm_goal_enable,
            dual_arm=self.use_dual_arm,
            right_arm_goal_range=self.right_arm_goal_range,
            left_arm_goal_range=self.left_arm_goal_range,
            right_arm_start=right_start,
            left_arm_start=left_start,
            arm_goal_minimum_distance=self.arm_goal_minimum_distance,
            arm_goal_pair_keepout=self.arm_goal_pair_keepout,
            base_goal_enabled=self.base_goal_enable,
            base_goal_range=self.base_goal_range,
            base_goal_rot_range=self.base_goal_rot_range,
            base_goal_relative_to_current=self.base_goal_relative_to_current,
            base_goal_workspace_range=self.base_goal_workspace_range,
            base_goal_minimum_distance=self.base_goal_minimum_distance,
            num_obstacles=self.num_obstacle_task,
            obstacle_range=self.obstacle_range,
            obstacle_radius=self.obstacle_size,
            obstacle_keepout=self.obstacle_keepout,
            obstacle_robot_keepaway=self.robot_keepout,
            obstacle_goal_keepaway=self.obstacle_goal_keepaway,
            arm_goal_radius=self.arm_goal_size,
            base_goal_radius=self.base_goal_size,
            robot_collision_centers=collision_centers,
            robot_collision_radii=collision_radii,
        )
        self._benchmark_scenario = scenario

        self.obstacle_task = []
        self.obstacle_task_geom = []
        for index, position in enumerate(scenario.obstacle_positions):
            obstacle = self._task_object(
                velocity=self.obstacle_velocity,
                direction=self.obstacle_direction,
                keep_direction_step=self.obstacle_keep_direction_step,
                bound=self.obstacle_range,
                smooth_weight=self.obstacle_smooth_weight,
                seed=self._seed + index,
            )
            obstacle.frame[:3, 3] = position
            self.obstacle_task.append(obstacle)
            self.obstacle_task_geom.append(
                Geometry(type="sphere", radius=self.obstacle_size, color=VizColor.obstacle_task)
            )
        self._initialize_environment_representation()

        if self.arm_goal_enable:
            self.robot_goal_right = self._task_object(
                velocity=self.goal_right_velocity,
                direction=self.goal_right_direction,
                keep_direction_step=self.arm_goal_keep_direction_step,
                bound=self.right_arm_goal_range,
                smooth_weight=self.arm_goal_smooth_weight,
                seed=self._seed,
            )
            self.robot_goal_right.frame[:3, 3] = scenario.right_arm_goal
            self.robot_goal_right.frame[:3, :3] = self.right_arm_goal_rotation
            if self.use_dual_arm:
                self.robot_goal_left = self._task_object(
                    velocity=self.goal_left_velocity,
                    direction=self.goal_left_direction,
                    keep_direction_step=self.arm_goal_keep_direction_step,
                    bound=self.left_arm_goal_range,
                    smooth_weight=self.arm_goal_smooth_weight,
                    seed=self._seed,
                )
                self.robot_goal_left.frame[:3, 3] = scenario.left_arm_goal
                self.robot_goal_left.frame[:3, :3] = self.left_arm_goal_rotation

        if self.base_goal_enable:
            self.robot_goal_base = self._task_object(
                velocity=self.base_goal_velocity,
                direction=self.base_goal_direction,
                keep_direction_step=self.base_goal_keep_direction_step,
                bound=self.base_goal_range,
                smooth_weight=self.base_goal_smooth_weight,
                seed=self._seed,
            )
            self.robot_goal_base.frame[:3, 3] = scenario.base_goal[:3]
            self.robot_goal_base.frame[:3, :3] = R.from_euler(
                "xyz", [0.0, 0.0, scenario.base_goal[3]]
            ).as_matrix()
        print(
            f"[SPARK] benchmark scenario seed={scenario.seed} "
            f"fingerprint={benchmark_scenario_fingerprint(scenario)}",
            flush=True,
        )

    def _arm_goals_are_reachable(self, right_goal, left_goal=None):
        """Use the embodiment IK contract to reject infeasible reset samples."""

        if not self.validate_arm_goal_reachability:
            return True
        targets = [np.asarray(right_goal, dtype=float)]
        if left_goal is not None:
            targets.append(np.asarray(left_goal, dtype=float))
        current_state = np.asarray(self.agent_feedback["state"], dtype=float)
        current_dof = self.robot_cfg.decompose_state_to_dof_pos(current_state)
        target_options = None
        if self.arm_goal_position_only:
            target_options = [{"orientation_mask": (False, False, False)} for _ in targets]
        _, info = self.robot_kinematics.inverse_kinematics(
            targets,
            current_dof,
            target_options=target_options,
        )
        result = info.get("ik_result") if isinstance(info, dict) else None
        if result is None:
            return bool(info.get("success", False)) if isinstance(info, dict) else False
        position_ok = float(result.position_error) <= float(self.arm_goal_size)
        orientation_ok = self.arm_goal_position_only or (
            float(result.orientation_error) <= self.arm_goal_orientation_size
        )
        return bool(info.get("success", False) or (position_ok and orientation_ok))

    @staticmethod
    def _fibonacci_sphere(count):
        index = np.arange(count, dtype=float)
        z = 1.0 - 2.0 * (index + 0.5) / count
        radius = np.sqrt(np.maximum(0.0, 1.0 - z * z))
        angle = index * np.pi * (3.0 - np.sqrt(5.0))
        return np.stack((radius * np.cos(angle), radius * np.sin(angle), z), axis=1)

    def _sample_mesh_surface(self):
        if not self.object_mesh_path:
            raise ValueError("mesh environment representation requires object_mesh_path")
        import trimesh

        path = Path(self.object_mesh_path).expanduser().resolve()
        mesh = trimesh.load(str(path), force="mesh", process=True)
        triangles = np.asarray(mesh.triangles, dtype=float) * self.object_mesh_scale
        area = np.asarray(mesh.area_faces, dtype=float)
        if not len(triangles) or not np.any(area > 0.0):
            raise ValueError(f"Collision mesh has no valid triangles: {path}")
        triangle_ids = self.rs.choice(len(triangles), self.points_per_obstacle, p=area / area.sum())
        uv = self.rs.random((self.points_per_obstacle, 2))
        reflected = uv.sum(axis=1) > 1.0
        uv[reflected] = 1.0 - uv[reflected]
        selected = triangles[triangle_ids]
        return (
            selected[:, 0]
            + uv[:, :1] * (selected[:, 1] - selected[:, 0])
            + uv[:, 1:] * (selected[:, 2] - selected[:, 0])
        )

    def _initialize_environment_representation(self):
        if self.environment_representation == "sphere":
            return
        if self.environment_representation == "point_cloud":
            offsets = self._sample_point_cloud_surface()
        elif self.environment_representation == "mesh":
            offsets = self._sample_mesh_surface()
        else:
            raise ValueError(
                f"Unknown environment representation: {self.environment_representation}"
            )
        self._environment_surface_points = offsets
        self._environment_point_geometries = [
            Geometry(
                type="sphere",
                radius=self.point_radius,
                color=VizColor.obstacle_task,
            )
            for _ in range(self.num_obstacle_task * self.points_per_obstacle)
        ]
        minimum = min(self.minimum_points_per_obstacle, self.points_per_obstacle)
        if self.dynamic_point_count:
            self._environment_point_counts = self.rs.randint(
                minimum,
                self.points_per_obstacle + 1,
                size=self.num_obstacle_task,
            )
        else:
            self._environment_point_counts = np.full(
                self.num_obstacle_task, self.points_per_obstacle, dtype=int
            )

    def _sample_point_cloud_surface(self):
        if self.regenerate_point_cloud_every_step:
            offsets = self.rs.normal(size=(self.points_per_obstacle, 3))
            offsets /= np.maximum(np.linalg.norm(offsets, axis=1, keepdims=True), 1.0e-12)
        else:
            offsets = self._fibonacci_sphere(self.points_per_obstacle)
        return offsets * float(self.obstacle_size)

    def _regenerate_point_cloud(self):
        self._environment_surface_points = self._sample_point_cloud_surface()
        if self.dynamic_point_count:
            minimum = min(self.minimum_points_per_obstacle, self.points_per_obstacle)
            self._environment_point_counts = self.rs.randint(
                minimum,
                self.points_per_obstacle + 1,
                size=self.num_obstacle_task,
            )

    def _init_obstacle(self):
        # ------------------------------- init obstacle ------------------------------ #
        self.obstacle_task = []
        self.obstacle_task_geom = []
        cnt = 0
        while len(self.obstacle_task) < self.num_obstacle_task:
            # Randomly sample a point
            obstacle = TaskObject3D(
                velocity=self.obstacle_velocity,
                keep_direction_step=self.obstacle_keep_direction_step,
                bound=self.obstacle_range,
                direction=self.obstacle_direction,
                smooth_weight=self.obstacle_smooth_weight,
                _seed=self._seed + len(self.obstacle_task),
                dt=self.dt,
            )
            if self.obstacle_mode == "Velocity":
                obstacle.frame[:3, 3] = self.obstacle_init
                obstacle_geom = Geometry(
                    type="sphere", radius=self.obstacle_size, color=VizColor.obstacle_task
                )
                self.obstacle_task.append(obstacle)
                self.obstacle_task_geom.append(obstacle_geom)
            else:
                obstacle.direction = self.rs.uniform(low=-1.0, high=1.0, size=3)
                obstacle.frame[:3, 3] = sample_bounded_position(self.rs, obstacle.bound)
                obstacle_geom = Geometry(
                    type="sphere", radius=self.obstacle_size, color=VizColor.obstacle_task
                )
                valid_flag = True

                # check if the new obstacle is too close to existing obstacles or robot
                dist_robot_to_env, _ = compute_masked_distance_matrix(
                    frame_list_1=self.robot_frames_world,
                    geom_list_1=self.robot_cfg.CollisionVol.values(),
                    frame_list_2=[
                        obstacle.frame,
                    ],
                    geom_list_2=[
                        obstacle_geom,
                    ],
                )
                if dist_robot_to_env is not None and dist_robot_to_env.min() < self.robot_keepout:
                    valid_flag = False

                dist_obstacle_to_obstacle, _ = compute_masked_distance_matrix(
                    frame_list_1=[obs.frame for obs in self.obstacle_task],
                    geom_list_1=[obs_geom for obs_geom in self.obstacle_task_geom],
                    frame_list_2=[
                        obstacle.frame,
                    ],
                    geom_list_2=[
                        obstacle_geom,
                    ],
                )
                if (
                    dist_obstacle_to_obstacle is not None
                    and dist_obstacle_to_obstacle.min() < self.obstacle_keepout
                ):
                    valid_flag = False

                # add to task if valid
                if valid_flag:
                    self.obstacle_task.append(obstacle)
                    self.obstacle_task_geom.append(obstacle_geom)

                cnt += 1
                if cnt > 10000:
                    raise ResamplingError("Failed to sample layout of obstacles")

        self._initialize_environment_representation()
        return

    def _init_goal(self):
        # --------------------------------- init goal -------------------------------- #
        if self.arm_goal_enable:
            right_start_world = (
                self.robot_frames_world[self.robot_cfg.Frames.R_ee, :3, 3]
                if hasattr(self.robot_cfg.Frames, "R_ee")
                else None
            )
            left_start_world = (
                self.robot_frames_world[self.robot_cfg.Frames.L_ee, :3, 3]
                if self.use_dual_arm and hasattr(self.robot_cfg.Frames, "L_ee")
                else None
            )
            self.robot_goal_right = TaskObject3D(
                velocity=self.goal_right_velocity,
                direction=self.goal_right_direction,
                keep_direction_step=self.arm_goal_keep_direction_step,
                bound=self.right_arm_goal_range,
                smooth_weight=self.arm_goal_smooth_weight,
                _seed=self._seed,
                dt=self.dt,
            )
            if self.arm_goal_mode == "Velocity":
                self.robot_goal_right.frame[:3, 3] = self.goal_right_init
            else:
                self.robot_goal_right.frame[:3, 3] = sample_bounded_position(
                    self.rs, self.robot_goal_right.bound
                )
                # ------------------------------ Distance Check ------------------------------ #
                """
                Distance check between each arm goal and obstacles to prevent infeasible setting.
                """
                for _ in range(10_000_000):
                    rightGoal = np.eye(4)
                    rightGoal[:3, 3] = sample_bounded_position(self.rs, self.robot_goal_right.bound)
                    rightGoal_world = self.robot_base_frame @ rightGoal

                    valid_flag = right_start_world is None or (
                        np.linalg.norm(rightGoal_world[:3, 3] - right_start_world)
                        >= self.arm_goal_minimum_distance
                    )
                    for other_obstacle in self.obstacle_task:
                        if (
                            np.linalg.norm(rightGoal_world[:3, 3] - other_obstacle.frame[:3, 3])
                            < self.arm_goal_keepout
                        ):
                            valid_flag = False
                            break

                    if valid_flag and not self.use_dual_arm:
                        valid_flag = self._arm_goals_are_reachable(rightGoal)

                    if valid_flag:
                        self.robot_goal_right.frame[:3, 3] = rightGoal[:3, 3]
                        break
                else:
                    raise ResamplingError("Failed to sample layout of right arm goal")

            if self.use_dual_arm:
                self.robot_goal_left = TaskObject3D(
                    velocity=self.goal_left_velocity,
                    direction=self.goal_left_direction,
                    keep_direction_step=self.arm_goal_keep_direction_step,
                    bound=self.left_arm_goal_range,
                    smooth_weight=self.arm_goal_smooth_weight,
                    _seed=self._seed,
                    dt=self.dt,
                )
                if self.arm_goal_mode == "Velocity":
                    self.robot_goal_left.frame[:3, 3] = self.goal_left_init
                else:
                    self.robot_goal_left.frame[:3, 3] = sample_bounded_position(
                        self.rs, self.robot_goal_left.bound
                    )
                    # Sample the goals together. Retrying only the left side
                    # can be impossible when a fixed right target lies near
                    # the overlap between the two workspaces.
                    for _ in range(10_000):
                        rightGoal = np.eye(4)
                        rightGoal[:3, 3] = sample_bounded_position(
                            self.rs, self.robot_goal_right.bound
                        )
                        leftGoal = np.eye(4)
                        leftGoal[:3, 3] = sample_bounded_position(
                            self.rs, self.robot_goal_left.bound
                        )
                        leftGoal_world = self.robot_base_frame @ leftGoal
                        rightGoal_world = self.robot_base_frame @ rightGoal
                        valid_flag = arm_goal_pair_is_separated(
                            rightGoal_world[:3, 3],
                            leftGoal_world[:3, 3],
                            self.arm_goal_pair_keepout,
                        )
                        valid_flag &= right_start_world is None or (
                            np.linalg.norm(rightGoal_world[:3, 3] - right_start_world)
                            >= self.arm_goal_minimum_distance
                        )
                        valid_flag &= left_start_world is None or (
                            np.linalg.norm(leftGoal_world[:3, 3] - left_start_world)
                            >= self.arm_goal_minimum_distance
                        )
                        for other_obstacle in self.obstacle_task:
                            if (
                                np.linalg.norm(rightGoal_world[:3, 3] - other_obstacle.frame[:3, 3])
                                < self.arm_goal_keepout
                                or np.linalg.norm(
                                    leftGoal_world[:3, 3] - other_obstacle.frame[:3, 3]
                                )
                                < self.arm_goal_keepout
                            ):
                                valid_flag = False
                                break

                        if valid_flag:
                            valid_flag = self._arm_goals_are_reachable(rightGoal, leftGoal)

                        if valid_flag:
                            self.robot_goal_right.frame[:3, 3] = rightGoal[:3, 3]
                            self.robot_goal_left.frame[:3, 3] = leftGoal[:3, 3]
                            break
                    else:
                        raise ResamplingError("Failed to sample layout of paired arm goals")

            # ------------------------------ Distance Check ------------------------------ #

        if self.base_goal_enable:
            configured_base_goal_bound = np.asarray(self.base_goal_range, dtype=float).copy()
            self.robot_goal_base = TaskObject3D(
                velocity=self.base_goal_velocity,
                direction=self.base_goal_direction,
                keep_direction_step=self.base_goal_keep_direction_step,
                bound=self.base_goal_range,
                smooth_weight=self.base_goal_smooth_weight,
                _seed=self._seed,
                dt=self.dt,
            )
            workspace = None
            if self.base_goal_workspace_range is not None:
                workspace = np.asarray(self.base_goal_workspace_range, dtype=float).reshape(2, 2)
                self.robot_goal_base.bound = np.asarray(
                    self.robot_goal_base.bound, dtype=float
                ).copy()
                self.robot_goal_base.bound[:2] = workspace
            if self.base_goal_relative_to_current:
                # The configured XY bounds are displacements from the base at
                # reset. TaskObject3D applies its bounds on every move(), so
                # translate the live bounds into world coordinates as well as
                # translating the sampled point below. Otherwise the first
                # Brownian update reflects a valid world goal back into the
                # unshifted relative box.
                if workspace is None:
                    self.robot_goal_base.bound = np.asarray(
                        self.robot_goal_base.bound, dtype=float
                    ).copy()
                    self.robot_goal_base.bound[:2] += self.robot_base_frame[:2, 3, None]

            if self.base_goal_mode == "Velocity":
                self.robot_goal_base.frame[:3, 3] = self.base_goal_init
            else:
                self.robot_goal_base.frame[:3, 3] = sample_bounded_position(
                    self.rs, self.robot_goal_base.bound
                )

                for attempt in range(10000):
                    if self.base_goal_relative_to_current and attempt < 5000:
                        goal_frame = sample_bounded_position(self.rs, configured_base_goal_bound)
                        goal_frame[:2] += self.robot_base_frame[:2, 3]
                    else:
                        goal_frame = sample_bounded_position(self.rs, self.robot_goal_base.bound)
                    inside_workspace = (
                        workspace is None
                        or np.all(goal_frame[:2] >= workspace[:, 0])
                        and np.all(goal_frame[:2] <= workspace[:, 1])
                    )

                    valid_flag = inside_workspace and (
                        np.linalg.norm(goal_frame[:2] - self.robot_base_frame[:2, 3])
                        >= self.base_goal_minimum_distance
                    )
                    for other_obstacle in self.obstacle_task:
                        if (
                            np.linalg.norm(goal_frame[:2] - other_obstacle.frame[:2, 3])
                            < self.base_goal_keepout
                        ):
                            valid_flag = False
                            break

                    if valid_flag:
                        self.robot_goal_base.frame[:3, 3] = goal_frame
                        break
                else:
                    raise ResamplingError("Failed to sample layout of base goal")

                self.robot_goal_base.frame[:3, :3] = R.from_euler(
                    "xyz",
                    [
                        0,
                        0,
                        self.rs.uniform(self.base_goal_rot_range[0], self.base_goal_rot_range[1]),
                    ],
                ).as_matrix()

        if self.arm_goal_enable:
            self.robot_goal_right.frame[:3, :3] = self.right_arm_goal_rotation
            if self.use_dual_arm:
                self.robot_goal_left.frame[:3, :3] = self.left_arm_goal_rotation

        return

    def _update_goal(self):
        if self.arm_goal_enable:
            if self.use_dual_arm:
                self.robot_goal_left.move(self.arm_goal_mode)
            self.robot_goal_right.move(self.arm_goal_mode)
        if self.base_goal_enable:
            self.robot_goal_base.move(self.base_goal_mode)

    def _update_obstacle(self):
        for obstacle in self.obstacle_task:
            obstacle.move(self.obstacle_mode)
        if (
            self.environment_representation == "point_cloud"
            and self.regenerate_point_cloud_every_step
        ):
            self._regenerate_point_cloud()

    def _update_done(self):
        self._done = False
        # Preserve compatibility with lightweight task stubs and older
        # serialized benchmark state that predates pose-aware completion.
        arm_goal_position_only = bool(getattr(self, "arm_goal_position_only", True))
        arm_goal_orientation_size = float(getattr(self, "arm_goal_orientation_size", 0.1))

        # check goal reaching conditions
        arm_goal_reach_flag = False
        dist_goal_left = None
        dist_goal_right = None
        if self.arm_goal_enable:
            right_position = (
                self.robot_frames_world[self.robot_cfg.Frames.R_ee, :3, 3]
                if hasattr(self.robot_cfg.Frames, "R_ee")
                else (self.robot_base_frame @ self.robot_goal_right.frame)[:3, 3]
            )
            right_goal = (self.robot_base_frame @ self.robot_goal_right.frame)[:3, 3]
            if self.use_dual_arm:
                left_position = self.robot_frames_world[self.robot_cfg.Frames.L_ee, :3, 3]
                left_goal = (self.robot_base_frame @ self.robot_goal_left.frame)[:3, 3]
            else:
                left_position = right_position
                left_goal = right_goal
            completion = evaluate_goal_completion(
                left_position=left_position,
                right_position=right_position,
                left_goal=left_goal,
                right_goal=right_goal,
                left_orientation=(
                    self.robot_frames_world[self.robot_cfg.Frames.L_ee, :3, :3]
                    if self.use_dual_arm
                    else None
                ),
                right_orientation=(
                    self.robot_frames_world[self.robot_cfg.Frames.R_ee, :3, :3]
                    if hasattr(self.robot_cfg.Frames, "R_ee")
                    else (self.robot_base_frame @ self.robot_goal_right.frame)[:3, :3]
                ),
                left_goal_orientation=(
                    (self.robot_base_frame @ self.robot_goal_left.frame)[:3, :3]
                    if self.use_dual_arm
                    else None
                ),
                right_goal_orientation=(self.robot_base_frame @ self.robot_goal_right.frame)[
                    :3, :3
                ],
                arm_tolerance=self.arm_goal_size,
                arm_orientation_tolerance=arm_goal_orientation_size,
                arm_enabled=True,
                base_enabled=False,
                dual_arm=self.use_dual_arm,
                arm_position_only=arm_goal_position_only,
            )
            if self.use_dual_arm:
                dist_goal_left = float(completion["left_error"])
            dist_goal_right = float(completion["right_error"])
            arm_goal_reach_flag = bool(completion["arm_reached"])

        base_goal_reach_flag = False
        dist_goal_base = None
        yaw_error_base = None
        if self.base_goal_enable:
            dist_goal_base = np.linalg.norm(
                self.robot_base_frame[:2, 3] - self.robot_goal_base.frame[:2, 3]
            )
            robot_yaw = float(np.arctan2(self.robot_base_frame[1, 0], self.robot_base_frame[0, 0]))
            goal_yaw = float(
                np.arctan2(self.robot_goal_base.frame[1, 0], self.robot_goal_base.frame[0, 0])
            )
            yaw_error_base = float((goal_yaw - robot_yaw + np.pi) % (2.0 * np.pi) - np.pi)
            if dist_goal_base < self.base_goal_size and abs(yaw_error_base) < getattr(
                self, "base_goal_yaw_size", np.inf
            ):
                base_goal_reach_flag = True

        if self.arm_goal_enable and self.base_goal_enable:
            if self.arm_goal_reach_done and self.base_goal_reach_done:
                if self.completion_mode == "any_enabled_goal":
                    self._done |= arm_goal_reach_flag or base_goal_reach_flag
                elif self.completion_mode == "all_enabled_goals":
                    self._done |= arm_goal_reach_flag and base_goal_reach_flag
                else:
                    raise ValueError(f"Unknown completion_mode: {self.completion_mode!r}")
            elif self.arm_goal_reach_done:
                self._done |= arm_goal_reach_flag
            elif self.base_goal_reach_done:
                self._done |= base_goal_reach_flag
        elif self.arm_goal_enable:
            if self.arm_goal_reach_done:
                self._done |= arm_goal_reach_flag
        elif self.base_goal_enable:
            if self.base_goal_reach_done:
                self._done |= base_goal_reach_flag

        reached_condition = bool(self._done)
        timed_out = bool(
            self.max_episode_length >= 0 and self.episode_length >= self.max_episode_length
        )
        base_height = float(self.robot_base_frame[2, 3])
        fallen = bool(
            self.fall_height_threshold is not None
            and base_height < float(self.fall_height_threshold)
        )
        # Episode completion and reset policy are part of the task case.  A
        # reset always reconstructs goals and obstacles together in reset().
        self._done = bool(
            (reached_condition and self.reset_on_success)
            or (timed_out and self.reset_on_timeout)
            or fallen
        )
        self._done_info = {
            "reason": (
                "fallen"
                if fallen
                else "goal_reached"
                if reached_condition
                else "timeout"
                if timed_out
                else None
            ),
            "goal_reached": reached_condition,
            "timed_out": timed_out,
            "fallen": fallen,
            "base_height": base_height,
            "fall_height_threshold": self.fall_height_threshold,
            "arm_goal_reached": bool(arm_goal_reach_flag),
            "base_goal_reached": bool(base_goal_reach_flag),
            "arm_goal_distance_left": dist_goal_left,
            "arm_goal_distance_right": dist_goal_right,
            "arm_goal_orientation_distance_left": (
                None if not self.use_dual_arm else float(completion["left_orientation_error"])
            )
            if self.arm_goal_enable
            else None,
            "arm_goal_orientation_distance_right": (
                float(completion["right_orientation_error"]) if self.arm_goal_enable else None
            ),
            "arm_goal_position_only": arm_goal_position_only,
            "arm_goal_orientation_tolerance": arm_goal_orientation_size,
            "base_goal_distance": dist_goal_base,
            "base_goal_yaw_error": yaw_error_base,
            "base_goal_yaw_tolerance": getattr(self, "base_goal_yaw_size", np.inf),
            "episode_length": int(self.episode_length),
            "max_episode_length": int(self.max_episode_length),
            "reset_on_success": self.reset_on_success,
            "reset_on_timeout": self.reset_on_timeout,
            "completion_mode": self.completion_mode,
            "resample_arm_goals_on_reset": self.resample_arm_goals_on_reset,
            "resample_base_goal_on_reset": self.resample_base_goal_on_reset,
            "resample_obstacles_on_reset": self.resample_obstacles_on_reset,
        }

    def step(self, feedback):
        super().step(feedback)

    def get_info(self) -> dict:
        super().get_info()
        if (
            self.environment_representation != "sphere"
            and self._environment_surface_points is not None
        ):
            frame_batches, velocity_batches = [], []
            total_points = 0
            for obstacle_index, obstacle in enumerate(self.obstacle_task):
                count = int(self._environment_point_counts[obstacle_index])
                offsets = self._environment_surface_points[:count]
                world_points = (obstacle.frame[:3, :3] @ offsets.T).T + obstacle.frame[:3, 3]
                velocity = obstacle.velocity * np.concatenate((obstacle.direction, np.zeros(3)))
                frames = np.repeat(np.eye(4)[None], count, axis=0)
                frames[:, :3, 3] = world_points
                frame_batches.append(frames)
                velocity_batches.append(np.repeat(velocity[None], count, axis=0))
                total_points += count
            self.info["obstacle_task"]["frames_world"] = (
                np.concatenate(frame_batches, axis=0) if frame_batches else np.empty((0, 4, 4))
            )
            self.info["obstacle_task"]["geom"] = self._environment_point_geometries[:total_points]
            self.info["obstacle_task"]["velocity"] = (
                np.concatenate(velocity_batches, axis=0) if velocity_batches else np.empty((0, 6))
            )
            self.info["obstacle"]["frames_world"] = np.concatenate(
                [
                    self.info["obstacle_task"]["frames_world"],
                    self.info["obstacle_debug"]["frames_world"],
                ],
                axis=0,
            )
            self.info["obstacle"]["velocity"] = np.concatenate(
                [self.info["obstacle_task"]["velocity"], self.info["obstacle_debug"]["velocity"]],
                axis=0,
            )
            self.info["obstacle"]["geom"] = np.concatenate(
                [self.info["obstacle_task"]["geom"], self.info["obstacle_debug"]["geom"]],
                axis=0,
            )
            self.info["obstacle"]["num"] = len(self.info["obstacle"]["frames_world"])
            self.info["environment_representation"] = self.environment_representation
            self.info["max_visualized_points"] = self.max_visualized_points
        self.info["done_info"] = dict(self._done_info)
        return self.info


if __name__ == "__main__":
    pass
