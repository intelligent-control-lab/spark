import json
from pathlib import Path

import numpy as np

from spark_task.base.base_goal_task import BaseGoalTask


class CartesianTrajectoryTrackingTask(BaseGoalTask):
    """General teleop task that exposes Cartesian end-effector trajectories."""

    def __init__(self, robot_cfg, robot_kinematics, agent, **kwargs):
        super().__init__(robot_cfg, robot_kinematics, agent, **kwargs)

        self.task_name = kwargs.get("task_name", "CartesianTrajectoryTrackingTask")
        self.max_episode_length = int(kwargs.get("max_episode_length", -1))
        self.trajectory_path = kwargs.get("trajectory_path", None)
        self._command_payload_cache = None
        self.trajectory_frame = str(kwargs.get("trajectory_frame", "world"))
        self.step_mode = str(kwargs.get("step_mode", "full_trajectory"))
        self.waypoint_hold_steps = max(1, int(kwargs.get("waypoint_hold_steps", 1)))
        self.initial_hold_steps = max(0, int(kwargs.get("initial_hold_steps", 0)))
        self.repeat = bool(kwargs.get("repeat", False))
        self.render_goal_spheres = bool(kwargs.get("render_goal_spheres", False))
        self.render_debug_frames = bool(kwargs.get("render_debug_frames", True))
        self.debug_frame_stride = max(1, int(kwargs.get("debug_frame_stride", 8)))

        self.right_trajectory = self._load_trajectory(
            kwargs.get("right_trajectory", kwargs.get("trajectory", None)),
            side="right",
        )
        self.left_trajectory = self._load_trajectory(
            kwargs.get("left_trajectory", None), side="left"
        )
        self.object_trajectory = self._load_optional_pose_trajectory(
            kwargs.get("object_trajectory", None)
        )
        self.distribution_trajectories = self._load_optional_pose_trajectory_stack(
            kwargs.get("distribution_trajectories", None)
        )
        self.distribution_trajectory_colors = self._load_optional_array(
            kwargs.get("distribution_trajectory_colors", None),
            "distribution_trajectory_colors",
        )
        self.distribution_trajectory_radii = self._load_optional_array(
            kwargs.get("distribution_trajectory_radii", None),
            "distribution_trajectory_radii",
        )
        self.distribution_trajectory_widths = self._load_optional_array(
            kwargs.get("distribution_trajectory_widths", None),
            "distribution_trajectory_widths",
        )
        self.render_distribution_trajectories = bool(
            kwargs.get(
                "render_distribution_trajectories", self.distribution_trajectories is not None
            )
        )

        self.use_dual_arm = bool(kwargs.get("use_dual_arm", self.left_trajectory is not None))
        if self.use_dual_arm and self.left_trajectory is None:
            self.left_trajectory = self.right_trajectory.copy()

        self.base_goal_enable = bool(kwargs.get("base_goal_enable", False))
        self.arm_goal_enable = bool(kwargs.get("arm_goal_enable", True))
        self.right_gripper_goal = bool(kwargs.get("right_gripper_goal", False))
        self.left_gripper_goal = bool(kwargs.get("left_gripper_goal", False))
        self.grasp_target_name = kwargs.get("grasp_target_name", None)
        self.right_grasp_target_name = kwargs.get("right_grasp_target_name", self.grasp_target_name)
        self.left_grasp_target_name = kwargs.get("left_grasp_target_name", self.grasp_target_name)
        self.grasp_radius = kwargs.get("grasp_radius", None)
        self.right_grasp_radius = kwargs.get("right_grasp_radius", self.grasp_radius)
        self.left_grasp_radius = kwargs.get("left_grasp_radius", self.grasp_radius)

        self.object_name = kwargs.get("object_name", None)
        self.object_initial_pose = self._optional_pose_to_matrix(
            kwargs.get("object_initial_pose", None)
        )
        self.object_follow_gripper = bool(kwargs.get("object_follow_gripper", False))
        self.object_follow_side = str(kwargs.get("object_follow_side", "right")).lower()
        self.object_gripper_offset = self._optional_pose_to_matrix(
            kwargs.get("object_gripper_offset", None)
        )
        if self.object_gripper_offset is None:
            self.object_gripper_offset = np.eye(4, dtype=np.float64)
        self.target_object_name = kwargs.get("target_object_name", None)
        self.target_object_pose = self._optional_pose_to_matrix(
            kwargs.get("target_object_pose", None)
        )
        self.task_debug_frames = self.right_trajectory.copy()
        self.episode_length = 0

    def _load_command_payload(self) -> dict:
        if self._command_payload_cache is not None:
            return self._command_payload_cache
        if not self.trajectory_path:
            self._command_payload_cache = {}
            return self._command_payload_cache
        path = Path(self.trajectory_path).expanduser().resolve()
        if not path.exists():
            raise FileNotFoundError(f"Cartesian trajectory command not found: {path}")
        if path.suffix == ".npz":
            with np.load(path, allow_pickle=True) as payload:
                self._command_payload_cache = {key: payload[key] for key in payload.files}
                return self._command_payload_cache
        if path.suffix == ".npy":
            self._command_payload_cache = {"trajectory": np.load(path, allow_pickle=True)}
            return self._command_payload_cache
        if path.suffix == ".json":
            self._command_payload_cache = json.loads(path.read_text(encoding="utf-8"))
            return self._command_payload_cache
        raise ValueError(f"Unsupported trajectory command file: {path}")

    def _load_trajectory(self, configured, *, side: str) -> np.ndarray:
        payload = self._load_command_payload()
        key_order = (
            (f"{side}_trajectory", "trajectory") if side == "right" else (f"{side}_trajectory",)
        )
        value = configured
        for key in key_order:
            if value is None and key in payload:
                value = payload[key]
        if value is None:
            if side == "left":
                return None
            raise ValueError("A right/primary Cartesian trajectory is required.")
        return self._as_frame_trajectory(value, side=side)

    def _load_optional_pose_trajectory(self, configured) -> np.ndarray | None:
        payload = self._load_command_payload()
        value = configured if configured is not None else payload.get("object_trajectory", None)
        if value is None:
            return None
        return self._as_frame_trajectory(value, side="object")

    def _load_optional_pose_trajectory_stack(self, configured) -> np.ndarray | None:
        payload = self._load_command_payload()
        value = (
            configured if configured is not None else payload.get("distribution_trajectories", None)
        )
        if value is None:
            return None
        array = np.asarray(value, dtype=np.float64)
        if array.ndim == 4 and array.shape[-2:] == (4, 4):
            frames = array.copy()
        elif array.ndim == 3 and array.shape[1:] == (4, 4):
            frames = array.reshape(1, *array.shape).copy()
        elif array.ndim == 3 and array.shape[-1] == 16:
            frames = array.reshape(array.shape[0], array.shape[1], 4, 4).copy()
        elif array.ndim == 3 and array.shape[-1] >= 7:
            frames = np.stack(
                [self._as_frame_trajectory(item, side="object") for item in array], axis=0
            )
        else:
            raise ValueError(
                f"Expected distribution trajectories as [N,T,4,4], [N,T,16], or [N,T,7+], got {array.shape}."
            )
        if frames.shape[0] == 0 or frames.shape[1] < 2:
            return None
        frames[:, :, 3, :] = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        return frames

    def _load_optional_array(self, configured, key: str) -> np.ndarray | None:
        payload = self._load_command_payload()
        value = configured if configured is not None else payload.get(key, None)
        if value is None:
            return None
        return np.asarray(value, dtype=np.float64)

    def _as_frame_trajectory(self, value, *, side: str) -> np.ndarray:
        array = np.asarray(value, dtype=np.float64)
        if array.ndim == 3 and array.shape[1:] == (4, 4):
            frames = array.copy()
        elif array.ndim == 2 and array.shape[1] == 16:
            frames = array.reshape(-1, 4, 4).copy()
        elif array.ndim == 2 and array.shape[1] >= 14 and side in {"right", "left"}:
            start = 0 if side == "right" else 7
            frames = self._pose7_to_frames(array[:, start : start + 7])
        elif array.ndim == 3 and array.shape[-2:] == (2, 7) and side in {"right", "left"}:
            arm_index = 0 if side == "right" else 1
            frames = self._pose7_to_frames(array[:, arm_index, :])
        elif array.ndim == 2 and array.shape[1] >= 7:
            frames = self._pose7_to_frames(array[:, :7])
        else:
            raise ValueError(
                f"Expected {side} trajectory as [T,7], [T,16], [T,4,4], or dual pose array; got {array.shape}."
            )

        if frames.shape[0] == 0:
            raise ValueError(f"{side} trajectory cannot be empty.")
        frames[:, 3, :] = np.array([0.0, 0.0, 0.0, 1.0], dtype=np.float64)
        return frames

    def _pose7_to_frames(self, poses: np.ndarray) -> np.ndarray:
        poses = np.asarray(poses, dtype=np.float64)
        frames = np.repeat(np.eye(4, dtype=np.float64)[None, :, :], poses.shape[0], axis=0)
        frames[:, :3, 3] = poses[:, :3]
        for i, quat in enumerate(poses[:, 3:7]):
            frames[i, :3, :3] = self._quat_wxyz_to_rot(quat)
        return frames

    def _quat_wxyz_to_rot(self, quat: np.ndarray) -> np.ndarray:
        q = np.asarray(quat, dtype=np.float64).reshape(4)
        norm = np.linalg.norm(q)
        if norm < 1e-12:
            return np.eye(3, dtype=np.float64)
        w, x, y, z = q / norm
        return np.array(
            [
                [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
                [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
                [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
            ],
            dtype=np.float64,
        )

    def _optional_pose_to_matrix(self, value) -> np.ndarray | None:
        if value is None:
            return None
        return self._as_frame_trajectory(
            np.asarray(value, dtype=np.float64).reshape(1, -1), side="object"
        )[0]

    def _waypoint_index(self) -> int:
        shifted_step = max(0, self.episode_length - self.initial_hold_steps)
        return shifted_step // self.waypoint_hold_steps

    def _trajectory_for_info(self, trajectory: np.ndarray) -> np.ndarray:
        if self.step_mode == "current_waypoint":
            index = self._waypoint_index()
            if self.repeat:
                index = index % len(trajectory)
            else:
                index = min(index, len(trajectory) - 1)
            return trajectory[index : index + 1]
        return trajectory

    def _full_trajectory_for_debug(self, trajectory: np.ndarray) -> np.ndarray:
        return trajectory[:: self.debug_frame_stride]

    def _world_trajectory(self, trajectory: np.ndarray) -> np.ndarray:
        selected = self._trajectory_for_info(trajectory)
        return self._frames_to_world(selected)

    def _world_debug_trajectory(self, trajectory: np.ndarray) -> np.ndarray:
        return self._frames_to_world(self._full_trajectory_for_debug(trajectory))

    def _world_debug_trajectories(self, trajectories: np.ndarray) -> np.ndarray:
        return self._frames_to_world(trajectories)

    def _frames_to_world(self, selected: np.ndarray) -> np.ndarray:
        if self.trajectory_frame == "robot_base":
            if selected.ndim == 4:
                return self.robot_base_frame.reshape(1, 1, 4, 4) @ selected
            return self.robot_base_frame.reshape(1, 4, 4) @ selected
        if self.trajectory_frame != "world":
            raise ValueError(
                f"Unsupported trajectory_frame={self.trajectory_frame!r}; expected 'world' or 'robot_base'."
            )
        return selected

    def _maybe_set_object_pose(self, object_name: str | None, frame: np.ndarray | None) -> None:
        if object_name is None or frame is None or not hasattr(self.agent, "_set_object_pos"):
            return
        try:
            self.agent._set_object_pos(str(object_name), frame)
        except Exception as exc:
            print(f"[CartesianTrajectoryTrackingTask] failed to set object {object_name!r}: {exc}")

    def _gripper_world_frame(self, side: str) -> np.ndarray | None:
        frame_name = "L_ee" if side == "left" else "R_ee"
        if frame_name not in self.robot_cfg.Frames.__members__:
            return None
        return self.robot_frames_world[self.robot_cfg.Frames.__members__[frame_name]].copy()

    def _object_from_gripper_world_frame(self) -> np.ndarray | None:
        gripper_frame = self._gripper_world_frame(self.object_follow_side)
        if gripper_frame is None:
            return None
        return gripper_frame @ np.linalg.inv(self.object_gripper_offset)

    def _sync_scene_objects(self) -> None:
        if self.object_name and self.object_follow_gripper:
            self._maybe_set_object_pose(self.object_name, self._object_from_gripper_world_frame())
        elif self.object_name and self.object_trajectory is not None:
            self._maybe_set_object_pose(
                self.object_name, self._world_trajectory(self.object_trajectory)[0]
            )
        elif self.object_name and self.object_initial_pose is not None:
            self._maybe_set_object_pose(
                self.object_name,
                self._frames_to_world(self.object_initial_pose.reshape(1, 4, 4))[0],
            )
        if self.target_object_name and self.target_object_pose is not None:
            self._maybe_set_object_pose(
                self.target_object_name,
                self._frames_to_world(self.target_object_pose.reshape(1, 4, 4))[0],
            )

    def get_reset_info(self) -> dict:
        agent_reset_info = super().get_reset_info()
        if self.object_name and self.object_initial_pose is not None:
            agent_reset_info["reset_object_pos"] = {
                str(self.object_name): self.object_initial_pose.copy()
            }
        return agent_reset_info

    def reset(self, feedback):
        self.agent_feedback = feedback
        self.episode_length = 0
        self._done = False
        self._done_info = {}
        self._update_robot_state()
        self._sync_scene_objects()

    def step(self, feedback):
        self.agent_feedback = feedback
        self.episode_length += 1
        self._update_robot_state()
        self._sync_scene_objects()
        self._update_done()

    def _init_obstacle(self):
        return

    def _update_obstacle(self):
        return

    def _init_goal(self):
        return

    def _update_goal(self):
        return

    def _update_done(self):
        waypoint_index = self._waypoint_index()
        if self.max_episode_length >= 0 and self.episode_length >= self.max_episode_length:
            self._done = True
        elif (
            self.step_mode == "current_waypoint"
            and not self.repeat
            and waypoint_index >= len(self.right_trajectory)
        ):
            self._done = True
        else:
            self._done = False

    def get_info(self) -> dict:
        obstacle_debug_frames = self.agent_feedback.get("obstacle_debug_frame", np.empty((0, 4, 4)))
        obstacle_debug_geom = self.agent_feedback.get("obstacle_debug_geom", [])
        obstacle_debug_velocity = self.agent_feedback.get(
            "obstacle_debug_velocity", np.empty((0, 6))
        )

        goal_teleop = {
            "right": self._world_trajectory(self.right_trajectory),
            "right_gripper_goal": self.right_gripper_goal,
        }
        if self.use_dual_arm and self.left_trajectory is not None:
            goal_teleop["left"] = self._world_trajectory(self.left_trajectory)
            goal_teleop["left_gripper_goal"] = self.left_gripper_goal
        for key, value in (
            ("right_grasp_target_name", self.right_grasp_target_name),
            ("left_grasp_target_name", self.left_grasp_target_name),
            ("right_grasp_radius", self.right_grasp_radius),
            ("left_grasp_radius", self.left_grasp_radius),
            ("grasp_target_name", self.grasp_target_name),
            ("grasp_radius", self.grasp_radius),
        ):
            if value is not None:
                goal_teleop[key] = value

        self.info = {
            "done": self._done,
            "seed": self._seed,
            "episode_length": self.episode_length,
            "robot_base_frame": self.agent_feedback["robot_base_frame"],
            "arm_goal_enable": self.arm_goal_enable,
            "base_goal_enable": self.base_goal_enable,
            "goal_teleop": goal_teleop,
            "obstacle_task": {
                "frames_world": np.empty((0, 4, 4)),
                "geom": [],
                "velocity": np.empty((0, 6)),
            },
            "obstacle_debug": {
                "frames_world": obstacle_debug_frames,
                "geom": obstacle_debug_geom,
                "velocity": obstacle_debug_velocity,
            },
            "obstacle": {
                "frames_world": obstacle_debug_frames,
                "geom": obstacle_debug_geom,
                "velocity": obstacle_debug_velocity,
                "num": len(obstacle_debug_frames),
            },
            "robot_state": {
                "dof_pos_cmd": self.agent_feedback["dof_pos_cmd"],
                "dof_pos_fbk": self.agent_feedback["dof_pos_fbk"],
                "dof_vel_cmd": self.agent_feedback["dof_vel_cmd"],
            },
            "arm_goal_size": self.arm_goal_size if self.render_goal_spheres else None,
            "base_goal_size": self.base_goal_size,
        }
        if self.render_debug_frames:
            self.info["task_debug_frames"] = self._world_trajectory(
                self.object_trajectory
                if self.object_trajectory is not None
                else self.right_trajectory
            )

        if self.render_distribution_trajectories and self.distribution_trajectories is not None:
            self.info["task_debug_trajectories"] = self._world_debug_trajectories(
                self.distribution_trajectories
            )
            if self.distribution_trajectory_colors is not None:
                self.info["task_debug_trajectory_colors"] = self.distribution_trajectory_colors
            if self.distribution_trajectory_radii is not None:
                self.info["task_debug_trajectory_radii"] = self.distribution_trajectory_radii
            if self.distribution_trajectory_widths is not None:
                self.info["task_debug_trajectory_widths"] = self.distribution_trajectory_widths

        if len(self.info["obstacle"]["frames_world"]) == 0:
            self.info["obstacle"]["geom"] = []
        if self.object_name and self.object_trajectory is not None:
            followed_object_frame = (
                self._object_from_gripper_world_frame() if self.object_follow_gripper else None
            )
            self.info["object_trajectory"] = (
                followed_object_frame.reshape(1, 4, 4)
                if followed_object_frame is not None
                else self._world_trajectory(self.object_trajectory)
            )
            self.info["object_name"] = self.object_name
            if self.render_debug_frames:
                self.info["task_debug_frames"] = self._world_debug_trajectory(
                    self.object_trajectory
                )
        elif self.object_trajectory is None and self.render_debug_frames:
            self.info["task_debug_frames"] = self._world_debug_trajectory(self.right_trajectory)
        return self.info
