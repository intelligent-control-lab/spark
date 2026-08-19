from __future__ import annotations

from typing import TYPE_CHECKING

from spark_task.base.base_task import BaseTask
from spark_robot import RobotKinematics
import numpy as np
from abc import abstractmethod
from spark_task.autonomy.benchmark_goals import (
    DEFAULT_LEFT_ARM_GOAL_RANGE,
    DEFAULT_RIGHT_ARM_GOAL_RANGE,
)

if TYPE_CHECKING:
    from spark_agent import BaseAgent


class BaseGoalTask(BaseTask):
    def __init__(
        self,
        robot_cfg: RobotKinematics,
        robot_kinematics: RobotKinematics,
        agent: BaseAgent,
        **kwargs,
    ):
        super().__init__(robot_cfg, robot_kinematics, agent)

        # Initialize task configuration
        self.task_name = kwargs.get("task_name", "TeleopTask")  # Name of the task
        self.max_episode_length = kwargs.get(
            "max_episode_length", -1
        )  # Maximum number of steps per episode
        self.fall_height_threshold = kwargs.get("fall_height_threshold", None)
        self.mode = kwargs.get("mode", "Brownian")  # Mode for obstacle movement
        self.dt = kwargs.get("dt", 0.002)
        self._done = False
        self._done_info = {}
        self.reset_on_success = bool(kwargs.get("reset_on_success", True))
        self.reset_on_timeout = bool(kwargs.get("reset_on_timeout", True))
        self.completion_mode = kwargs.get("completion_mode", "all_enabled_goals")
        self.resample_arm_goals_on_reset = bool(kwargs.get("resample_arm_goals_on_reset", True))
        self.resample_base_goal_on_reset = bool(kwargs.get("resample_base_goal_on_reset", True))
        self.resample_obstacles_on_reset = bool(kwargs.get("resample_obstacles_on_reset", True))
        self.deterministic_scenario_sampling = bool(
            kwargs.get("deterministic_scenario_sampling", False)
        )
        self.reset_dof_pos = kwargs.get("reset_dof_pos", None)
        self.reset_dof_vel = kwargs.get("reset_dof_vel", None)

        # Obstacle configuration
        self.num_obstacle_task = kwargs.get(
            "num_obstacle_task", 3
        )  # Number of obstacles in the task
        self.obstacle_range = kwargs.get(
            "obstacle_range", [(-2, 2), (-2, 2), (0.8, 1.1)]
        )  # Range for obstacle placement [xmin, xmax, ymin, ymax, zmin, zmax]
        self.obstacle_size = kwargs.get("obstacle_size", 0.05)  # Size of each obstacle
        self.obstacle_keepout = kwargs.get(
            "obstacle_keepout", 0.05
        )  # Minimum keepout distance for obstacles
        self.obstacle_goal_keepaway = float(kwargs.get("obstacle_goal_keepaway", 0.0))
        self.obstacle_init = np.array(kwargs.get("obstacle_init", [0.2, 0.0, 1.0]))
        self.obstacle_velocity = kwargs.get("obstacle_velocity", 0.001)
        self.obstacle_direction = np.array(kwargs.get("obstacle_direction", [0.0, 0.0, 0.0]))
        self.obstacle_keep_direction_step = kwargs.get(
            "obstacle_keep_direction_step", 500
        )  # Steps to maintain obstacle direction
        self.obstacle_smooth_weight = kwargs.get(
            "obstacle_smooth_weight", 0.8
        )  # Smoothing weight for obstacle motion

        # Arm goal configuration
        self.arm_goal_enable = kwargs.get("arm_goal_enable", True)  # Enable arm goal functionality
        self.use_dual_arm = kwargs.get("use_dual_arm", True)  # Use single arm for teleoperation
        self.arm_goal_size = kwargs.get("arm_goal_size", 0.05)  # Size of arm goal markers
        self.arm_goal_keepout = kwargs.get(
            "arm_goal_keepout", 0.1
        )  # Keepout distance for arm goals
        self.arm_goal_pair_keepout = float(kwargs.get("arm_goal_pair_keepout", 0.0))
        if self.arm_goal_pair_keepout < 0.0:
            raise ValueError("arm_goal_pair_keepout cannot be negative")
        self.arm_goal_minimum_distance = float(kwargs.get("arm_goal_minimum_distance", 0.0))
        if self.arm_goal_minimum_distance < 0.0:
            raise ValueError("arm goal minimum distance cannot be negative")
        self.arm_goal_velocity = kwargs.get("arm_goal_velocity", 0.0)  # Velocity of arm goals
        self.arm_goal_keep_direction_step = kwargs.get(
            "arm_goal_keep_direction_step", 500
        )  # Steps to maintain arm goal direction
        self.arm_goal_smooth_weight = kwargs.get(
            "arm_goal_smooth_weight", 0.8
        )  # Smoothing weight for arm goal movement
        self.arm_goal_reach_done = kwargs.get(
            "arm_goal_reach_done", False
        )  # Flag to finish episode when arm goal is reached
        self.arm_goal_position_only = bool(kwargs.get("arm_goal_position_only", False))
        self.arm_goal_orientation_size = float(kwargs.get("arm_goal_orientation_size", 0.1))
        self.validate_arm_goal_reachability = bool(
            kwargs.get("validate_arm_goal_reachability", False)
        )

        self.left_arm_goal_range = kwargs.get(
            "left_arm_goal_range", DEFAULT_LEFT_ARM_GOAL_RANGE
        )  # Range for left arm goal [xmin, xmax, ymin, ymax, zmin, zmax] relative to the robot base
        self.goal_left_init = np.array(
            kwargs.get("goal_left_init", [0.3, 0.25, 0.0])
        )  # Initial position of left arm goal
        self.goal_left_velocity = kwargs.get("goal_left_velocity", 0.0)
        self.goal_left_direction = np.array(kwargs.get("goal_left_direction", [0.0, 0.0, 0.0]))
        self.right_arm_goal_range = kwargs.get(
            "right_arm_goal_range", DEFAULT_RIGHT_ARM_GOAL_RANGE
        )  # Range for right arm goal [xmin, xmax, ymin, ymax, zmin, zmax] relative to the robot base
        self.goal_right_init = np.array(
            kwargs.get("goal_right_init", [0.3, -0.25, 0.0])
        )  # Initial position of right arm goal
        self.goal_right_velocity = kwargs.get("goal_right_velocity", 0.0)
        self.goal_right_direction = np.array(kwargs.get("goal_right_direction", [0.0, 0.0, 0.0]))
        self.arm_goal_init_from_current_ee = bool(
            kwargs.get("arm_goal_init_from_current_ee", False)
        )
        self.robot_goal_right_home_frame = None
        self.robot_goal_left_home_frame = None

        # Base goal configuration
        self.base_goal_enable = kwargs.get(
            "base_goal_enable", True
        )  # Enable base goal functionality
        self.base_goal_range = kwargs.get(
            "base_goal_range", [(0, 0), (0, 0), (0.793, 0.793)]
        )  # Range for base goal [xmin, xmax, ymin, ymax, zmin, zmax]
        self.base_goal_rot_range = kwargs.get(
            "base_goal_rot_range", [0, 0]
        )  # Rotation range for base goal
        self.base_goal_size = kwargs.get("base_goal_size", 0.1)  # Size of the base goal marker
        # Keep infinity as the compatibility default; benchmark cases that
        # require full SE(2) arrival specify a finite yaw tolerance explicitly.
        self.base_goal_yaw_size = float(kwargs.get("base_goal_yaw_size", np.inf))
        self.base_goal_init = np.array(kwargs.get("base_goal_init", [0.0, 0.0, 0.793]))
        self.base_goal_keepout = kwargs.get(
            "base_goal_keepout", 0.1
        )  # Keepout distance for base goal
        self.base_goal_minimum_distance = float(kwargs.get("base_goal_minimum_distance", 0.0))
        self.base_goal_relative_to_current = bool(
            kwargs.get("base_goal_relative_to_current", False)
        )
        self.base_goal_workspace_range = kwargs.get("base_goal_workspace_range", None)
        self.base_goal_velocity = kwargs.get("base_goal_velocity", 0.0)  # Velocity of base goal
        self.base_goal_direction = np.array(kwargs.get("base_goal_direction", [1.0, 0.0, 0.0]))
        self.base_goal_keep_direction_step = kwargs.get(
            "base_goal_keep_direction_step", 500
        )  # Steps to maintain base goal direction
        self.base_goal_smooth_weight = kwargs.get(
            "base_goal_smooth_weight", 0.8
        )  # Smoothing weight for base goal movement
        self.base_goal_reach_done = kwargs.get(
            "base_goal_reach_done", False
        )  # Flag to finish episode when base goal is reached

        # Gripper goal configuration
        self.left_gripper_goal = kwargs.get("left_gripper_goal", False)  # State of the left gripper
        self.right_gripper_goal = kwargs.get(
            "right_gripper_goal", False
        )  # State of the right gripper

        # Seed configuration for random state
        self._seed = kwargs.get("seed", 0)  # Default random seed
        self.seed_list = kwargs.get("seed_list", [])  # List of seeds for resampling
        self._init_seed()
        # ------------------------------------ ROS ----------------------------------- #
        self.enable_ros = kwargs.get("enable_ros", False)  # Enable ROS integration

    def _init_seed(self):
        """
        Initialize seed to be one before the first seed to use
        reset() will increment the seed to the first seed to use
        """
        # self._seed = np.random.randint(2**32) if seed is None else seed
        self._seed_list_idx = -1

    def _update_robot_state(self):
        # ---------------------------- compute robot state --------------------------- #
        self.robot_base_frame = self.agent_feedback["robot_base_frame"]
        # compute transformations of robot collision volumes
        x = self.agent_feedback["state"]
        dof_pos = self.robot_cfg.decompose_state_to_dof_pos(x)
        robot_frames = self.robot_kinematics.forward_kinematics(dof_pos)
        self.robot_frames_world = np.zeros_like(robot_frames)
        for i in range(len(robot_frames)):
            self.robot_frames_world[i, :, :] = self.robot_base_frame @ robot_frames[i, :, :]
        return

    def reset(self, feedback):
        # ----------------------------------- init ----------------------------------- #
        self.agent_feedback = feedback
        self.episode_length = 0

        self._update_robot_state()
        if self.arm_goal_init_from_current_ee and self.arm_goal_enable:
            inv_base = np.linalg.inv(self.robot_base_frame)
            if hasattr(self.robot_cfg.Frames, "R_ee"):
                self.robot_goal_right_home_frame = (
                    inv_base @ self.robot_frames_world[self.robot_cfg.Frames.R_ee]
                )
            if self.use_dual_arm and hasattr(self.robot_cfg.Frames, "L_ee"):
                self.robot_goal_left_home_frame = (
                    inv_base @ self.robot_frames_world[self.robot_cfg.Frames.L_ee]
                )
        if self.deterministic_scenario_sampling:
            initializer = getattr(self, "_init_deterministic_scenario", None)
            if initializer is None:
                raise TypeError(
                    f"{type(self).__name__} does not implement deterministic scenario sampling"
                )
            initializer()
        else:
            self._init_obstacle()
            self._init_goal()
        # --------------------------------- init info -------------------------------- #
        self.info = {}
        self.info["goal_teleop"] = {}
        self.info["obstacle_task"] = {}
        self.info["obstacle_debug"] = {}
        self.info["obstacle"] = {}
        self.info["robot_frames"] = None
        self.info["robot_state"] = {}
        self.info["arm_goal_enable"] = self.arm_goal_enable
        self.info["arm_goal_position_only"] = self.arm_goal_position_only
        self.info["base_goal_enable"] = self.base_goal_enable
        scenario = getattr(self, "_benchmark_scenario", None)
        if scenario is not None:
            from spark_task.autonomy.benchmark_goals import benchmark_scenario_fingerprint

            self.info["scenario_seed"] = scenario.seed
            self.info["scenario_fingerprint"] = benchmark_scenario_fingerprint(scenario)
        if hasattr(self, "upper_body_joint_target_callback"):
            self.upper_body_joint_target_callback = None
        if hasattr(self, "upper_body_joint_target_mask_callback"):
            self.upper_body_joint_target_mask_callback = None
        if hasattr(self, "target_dof_pos_callback"):
            self.target_dof_pos_callback = None
        if hasattr(self, "teleop_upper_body_mode"):
            self.info["teleop_upper_body_mode"] = self.teleop_upper_body_mode
        self._done = False
        self._done_info = {}

    def get_reset_info(self) -> dict:
        # update seed
        if len(self.seed_list) > 0:
            self._seed_list_idx = (self._seed_list_idx + 1) % len(
                self.seed_list
            )  # Increment seed list index
            self._seed = self.seed_list[self._seed_list_idx]
        else:
            self._seed += 1  # Increment seed

        self.rs = np.random.RandomState(self._seed)
        # --------------------------------- reset info -------------------------------- #
        agent_reset_info = {}
        if self.reset_dof_pos is None:
            agent_reset_info["reset_dof_pos"] = np.array(
                [self.robot_cfg.DefaultDoFVal[dof] for dof in self.robot_cfg.DoFs]
            )
        else:
            agent_reset_info["reset_dof_pos"] = np.asarray(self.reset_dof_pos, dtype=float).copy()
        if self.reset_dof_vel is None:
            agent_reset_info["reset_dof_vel"] = np.zeros(self.robot_cfg.num_dof, dtype=float)
        else:
            agent_reset_info["reset_dof_vel"] = np.asarray(self.reset_dof_vel, dtype=float).copy()
        agent_reset_info["object_pos"] = {}

        return agent_reset_info

    def step(self, feedback):
        self.agent_feedback = feedback
        self.episode_length += 1

        # update task objects for next iteration
        self._update_robot_state()
        self._update_done()
        self._update_goal()
        self._update_obstacle()

    def get_info(self) -> dict:
        self.info["done"] = self._done

        self.info["seed"] = self._seed

        self.info["episode_length"] = self.episode_length

        self.info["robot_base_frame"] = self.agent_feedback["robot_base_frame"]

        if self.arm_goal_enable:
            self.info["arm_goal_size"] = self.arm_goal_size
            self.info["goal_teleop"]["right"] = (
                self.robot_base_frame @ self.robot_goal_right.frame.reshape(-1, 4, 4)
            )
            self.info["goal_teleop"]["right_gripper_goal"] = self.right_gripper_goal
            if self.use_dual_arm:
                self.info["goal_teleop"]["left"] = (
                    self.robot_base_frame @ self.robot_goal_left.frame.reshape(-1, 4, 4)
                )
                self.info["goal_teleop"]["left_gripper_goal"] = self.left_gripper_goal

        if self.base_goal_enable:
            self.info["base_goal_size"] = self.base_goal_size
            self.info["goal_teleop"]["base"] = self.robot_goal_base.frame.reshape(-1, 4, 4)

        self.info["obstacle_task"]["frames_world"] = (
            [obstacle.frame for obstacle in self.obstacle_task]
            if len(self.obstacle_task) > 0
            else np.empty((0, 4, 4))
        )
        self.info["obstacle_task"]["geom"] = self.obstacle_task_geom
        self.info["obstacle_task"]["velocity"] = (
            [
                obstacle.velocity * np.concatenate((obstacle.direction, np.zeros(3)))
                for obstacle in self.obstacle_task
            ]
            if len(self.obstacle_task) > 0
            else np.empty((0, 6))
        )
        self.info["obstacle_debug"]["frames_world"] = self.agent_feedback.get(
            "obstacle_debug_frame", np.empty((0, 4, 4))
        )
        self.info["obstacle_debug"]["geom"] = self.agent_feedback.get("obstacle_debug_geom", [])
        self.info["obstacle_debug"]["velocity"] = self.agent_feedback.get(
            "obstacle_debug_velocity", np.empty((0, 6))
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
            [self.info["obstacle_task"]["geom"], self.info["obstacle_debug"]["geom"]], axis=0
        )
        self.info["obstacle"]["num"] = len(self.info["obstacle"]["frames_world"])
        # robot state
        self.info["robot_state"]["dof_pos_cmd"] = self.agent_feedback["dof_pos_cmd"]
        self.info["robot_state"]["dof_pos_fbk"] = self.agent_feedback["dof_pos_fbk"]
        self.info["robot_state"]["dof_vel_cmd"] = self.agent_feedback["dof_vel_cmd"]
        self.info["camera"] = self.agent_feedback.get("camera", {})
        if getattr(self, "target_dof_pos_callback", None) is not None:
            self.info["target_dof_pos"] = self.target_dof_pos_callback.copy()
        if hasattr(self, "teleop_upper_body_mode"):
            self.info["teleop_upper_body_mode"] = self.teleop_upper_body_mode
        if getattr(self, "upper_body_joint_target_callback", None) is not None:
            self.info["teleop_joint_upper_body_target"] = (
                self.upper_body_joint_target_callback.copy()
            )
            if getattr(self, "upper_body_joint_target_mask_callback", None) is not None:
                self.info["teleop_joint_upper_body_target_mask"] = (
                    self.upper_body_joint_target_mask_callback.copy()
                )

        return self.info

    @abstractmethod
    def _init_obstacle(self):
        "'' Initialize obstacle task objects. ''"
        pass

    @abstractmethod
    def _update_obstacle(self):
        "'' Update obstacle task objects. ''"
        pass

    @abstractmethod
    def _init_goal(self):
        "'' Initialize goal task objects. ''"
        pass

    @abstractmethod
    def _update_goal(self):
        "'' Update goal task objects. ''"
        pass

    @abstractmethod
    def _update_done(self):
        "'' Update self._done to indicate whether the episode is finished. ''"
        pass
