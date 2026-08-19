from types import SimpleNamespace

import numpy as np

from spark_utils import initialize_class

from spark_robot import RobotConfig, RobotKinematics, UnitreeG1WholeBodyDynamic1Config
from spark_policy.core import Policy


def _config_namespace(value):
    if isinstance(value, dict):
        return SimpleNamespace(**{key: _config_namespace(item) for key, item in value.items()})
    if isinstance(value, list):
        return [_config_namespace(item) for item in value]
    return value


def _build_safety_policies(config, *, robot_cfg, robot_kinematics):
    monitor = initialize_class(config.safety_index, robot_kinematics=robot_kinematics)
    safety_filter = initialize_class(
        config.safe_algo,
        safety_index=monitor,
        safety_monitor=monitor,
        robot_cfg=robot_cfg,
        robot_kinematics=robot_kinematics,
    )
    if not hasattr(safety_filter, "safety_index"):
        safety_filter.safety_index = monitor
    return monitor, safety_filter


def _as_vector(value, length: int, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.shape == ():
        return np.full(length, float(array), dtype=float)
    array = array.reshape(-1)
    if array.shape[0] != length:
        raise ValueError(f"{name} must have {length} entries, got {array.shape[0]}")
    return array.copy()


class UnitreeG1SonicSafePolicy(Policy):
    """Cascade wrapper for Unitree G1 SONIC.

    The pre-SONIC safety stage uses a sport-mode abstraction: 17 upper-body
    controls plus named virtual base controls. SONIC exposes
    vLinearZ so the safe layer can request squat-height changes.
    """

    _SPORT_BODY_DOF_NAMES = (
        "WaistYaw",
        "WaistRoll",
        "WaistPitch",
        "LeftShoulderPitch",
        "LeftShoulderRoll",
        "LeftShoulderYaw",
        "LeftElbow",
        "LeftWristRoll",
        "LeftWristPitch",
        "LeftWristYaw",
        "RightShoulderPitch",
        "RightShoulderRoll",
        "RightShoulderYaw",
        "RightElbow",
        "RightWristRoll",
        "RightWristPitch",
        "RightWristYaw",
    )
    _WAIST_SLICE = slice(0, 3)
    _LEFT_ARM_SLICE = slice(3, 10)
    _RIGHT_ARM_SLICE = slice(10, 17)

    def __init__(
        self,
        cfg=None,
        robot_cfg: RobotConfig = None,
        robot_kinematics: RobotKinematics = None,
        **kwargs,
    ):
        self.cfg = _config_namespace(cfg if cfg is not None else kwargs)
        self.robot_cfg = robot_cfg
        self.robot_kinematics = robot_kinematics
        self.body_robot_cfg = UnitreeG1WholeBodyDynamic1Config()

        safe_robot_cfg_cfg = getattr(
            self.cfg,
            "safe_robot_cfg",
            SimpleNamespace(class_name="UnitreeG1MobileBaseDynamic1Config"),
        )
        self.safe_robot_cfg = initialize_class(safe_robot_cfg_cfg)

        safe_robot_kinematics_cfg = getattr(self.cfg, "safe_robot_kinematics", None)
        if safe_robot_kinematics_cfg is None:
            safe_robot_kinematics_cfg = SimpleNamespace(
                class_name=self.safe_robot_cfg.kinematics_class_name
            )
        elif getattr(safe_robot_kinematics_cfg, "class_name", None) is None:
            safe_robot_kinematics_cfg.class_name = self.safe_robot_cfg.kinematics_class_name
        self.safe_robot_kinematics = initialize_class(
            safe_robot_kinematics_cfg, robot_cfg=self.safe_robot_cfg
        )

        self.goal_tracking_type = str(getattr(self.cfg, "goal_tracking_type", "legged")).lower()
        if self.goal_tracking_type not in ("legged", "pid"):
            raise ValueError("goal_tracking_type must be 'legged' or 'pid'")
        goal_tracking_cfg = (
            self.cfg.goal_tracking_policy
            if self.goal_tracking_type == "legged"
            else getattr(
                self.cfg,
                "pid_goal_tracking_policy",
                SimpleNamespace(class_name="TeleopPIDPolicy"),
            )
        )
        self.goal_tracking_policy = initialize_class(
            goal_tracking_cfg,
            robot_cfg=self.safe_robot_cfg,
            robot_kinematics=self.safe_robot_kinematics,
        )
        self.policy = initialize_class(
            self.cfg.executor,
            robot_cfg=robot_cfg,
            robot_kinematics=robot_kinematics,
        )
        self.safety_monitor, self.safety_filter = _build_safety_policies(
            self._whole_body_safety_config(),
            robot_cfg=robot_cfg,
            robot_kinematics=robot_kinematics,
        )
        self.pre_safety_config = getattr(self.cfg, "pre_safe_controller", self.cfg.safe_controller)
        self.pre_safety_monitor, self.pre_safety_filter = _build_safety_policies(
            self.pre_safety_config,
            robot_cfg=self.safe_robot_cfg,
            robot_kinematics=self.safe_robot_kinematics,
        )
        self.pre_safe_enabled = bool(getattr(self.cfg, "pre_safe_enabled", True))
        self.pre_safe_target_dt = float(
            getattr(
                self.cfg, "pre_safe_target_dt", getattr(self.cfg.executor, "safe_target_dt", 0.02)
            )
        )
        self.pre_safe_hold_steps = max(0, int(getattr(self.cfg, "pre_safe_hold_steps", 0)))
        self.pre_safe_loco_command_limit = _as_vector(
            getattr(self.cfg, "pre_safe_loco_command_limit", [0.12, 0.10, 0.16]),
            3,
            "pre_safe_loco_command_limit",
        )
        self.pre_safe_loco_command_rate_limit = _as_vector(
            getattr(self.cfg, "pre_safe_loco_command_rate_limit", [0.05, 0.05, 0.08]),
            3,
            "pre_safe_loco_command_rate_limit",
        )
        self.pre_safe_height_command_limit = float(
            getattr(self.cfg, "pre_safe_height_command_limit", 0.0)
        )
        self.pre_safe_height_target_lookahead = float(
            getattr(self.cfg, "pre_safe_height_target_lookahead", 2.0)
        )
        self.pre_safe_height_min = float(getattr(self.cfg, "pre_safe_height_min", 0.2))
        self.pre_safe_height_max = float(getattr(self.cfg, "pre_safe_height_max", 1.0))
        self.pre_safe_height_recovery_enabled = bool(
            getattr(self.cfg, "pre_safe_height_recovery_enabled", True)
        )
        self.pre_safe_height_default = float(getattr(self.cfg, "pre_safe_height_default", 0.793))
        self.pre_safe_height_recovery_kp = float(
            getattr(self.cfg, "pre_safe_height_recovery_kp", 1.0)
        )
        self.pre_safe_height_recovery_deadband = float(
            getattr(self.cfg, "pre_safe_height_recovery_deadband", 0.015)
        )
        self.pre_safe_upper_body_velocity_limit = _as_vector(
            getattr(
                self.cfg, "pre_safe_upper_body_velocity_limit", [0.10, 0.10, 0.10] + [0.55] * 14
            ),
            17,
            "pre_safe_upper_body_velocity_limit",
        )
        self.pre_safe_upper_body_position_error_limit = _as_vector(
            getattr(self.cfg, "pre_safe_upper_body_position_error_limit", 0.30),
            17,
            "pre_safe_upper_body_position_error_limit",
        )
        self.pre_safe_upper_body_mode = getattr(self.cfg, "pre_safe_upper_body_mode", "integrated")
        if self.pre_safe_upper_body_mode not in ("integrated", "delta"):
            raise ValueError(
                f"Unsupported pre_safe_upper_body_mode={self.pre_safe_upper_body_mode}. "
                "Use 'integrated' or 'delta'."
            )
        self._pre_safe_hold_count = 0
        self._last_sport_u_safe = None
        self._last_sport_safe_delta = None
        self._last_pre_safe_active_sides = set()
        self._last_sonic_loco_command = None
        self._pre_safe_upper_body_cmd = None

    def reset(self, context=None) -> None:
        for component in (
            self.goal_tracking_policy,
            self.policy,
            self.safety_monitor,
            self.safety_filter,
            self.pre_safety_monitor,
            self.pre_safety_filter,
        ):
            reset = getattr(component, "reset", None)
            if callable(reset):
                reset(context)
        self._pre_safe_hold_count = 0
        self._last_sport_u_safe = None
        self._last_sport_safe_delta = None
        self._last_pre_safe_active_sides = set()
        self._last_sonic_loco_command = None
        self._pre_safe_upper_body_cmd = None

    def _whole_body_safety_config(self):
        return SimpleNamespace(
            class_name=None,
            safety_index=getattr(
                self.cfg.safe_controller,
                "safety_index",
                SimpleNamespace(class_name="FirstOrderCollisionSafetyIndex"),
            ),
            safe_algo=SimpleNamespace(class_name="ByPassSafeControl"),
        )

    def _body_feedback(self, agent_feedback: dict):
        qpos = np.asarray(
            agent_feedback.get(
                "body_qpos_fbk", agent_feedback.get("qpos_fbk", agent_feedback["dof_pos_fbk"])
            ),
            dtype=float,
        ).reshape(-1)
        qvel = np.asarray(
            agent_feedback.get(
                "body_qvel_fbk", agent_feedback.get("qvel_fbk", agent_feedback["dof_vel_fbk"])
            ),
            dtype=float,
        ).reshape(-1)
        return qpos, qvel

    def _sport_feedback_from_whole_body(self, agent_feedback: dict) -> dict:
        body_qpos, body_qvel = self._body_feedback(agent_feedback)
        sport_pos = np.array(
            [self.safe_robot_cfg.DefaultDoFVal[dof] for dof in self.safe_robot_cfg.DoFs],
            dtype=float,
        )
        sport_vel = np.zeros(len(self.safe_robot_cfg.DoFs), dtype=float)

        robot_base_frame = np.asarray(agent_feedback["robot_base_frame"], dtype=float)
        yaw = np.arctan2(robot_base_frame[1, 0], robot_base_frame[0, 0])
        sport_pos[self.safe_robot_cfg.DoFs.LinearX] = robot_base_frame[0, 3]
        sport_pos[self.safe_robot_cfg.DoFs.LinearY] = robot_base_frame[1, 3]
        if hasattr(self.safe_robot_cfg.DoFs, "LinearZ"):
            sport_pos[self.safe_robot_cfg.DoFs.LinearZ] = robot_base_frame[2, 3]
        sport_pos[self.safe_robot_cfg.DoFs.RotYaw] = yaw
        if body_qvel.shape[0] >= 6:
            sport_vel[self.safe_robot_cfg.DoFs.LinearX] = body_qvel[0]
            sport_vel[self.safe_robot_cfg.DoFs.LinearY] = body_qvel[1]
            if hasattr(self.safe_robot_cfg.DoFs, "LinearZ"):
                sport_vel[self.safe_robot_cfg.DoFs.LinearZ] = body_qvel[2]
            sport_vel[self.safe_robot_cfg.DoFs.RotYaw] = body_qvel[5]

        for name in self._SPORT_BODY_DOF_NAMES:
            if not hasattr(self.safe_robot_cfg.DoFs, name) or not hasattr(
                self.body_robot_cfg.DoFs, name
            ):
                continue
            sport_idx = int(getattr(self.safe_robot_cfg.DoFs, name))
            body_qpos_idx = int(getattr(self.body_robot_cfg.DoFs, name))
            body_qvel_idx = body_qpos_idx - 1
            if body_qpos_idx < body_qpos.shape[0]:
                sport_pos[sport_idx] = body_qpos[body_qpos_idx]
            if 0 <= body_qvel_idx < body_qvel.shape[0]:
                sport_vel[sport_idx] = body_qvel[body_qvel_idx]

        sport_feedback = dict(agent_feedback)
        sport_feedback["dof_pos_fbk"] = sport_pos.copy()
        sport_feedback["dof_vel_fbk"] = sport_vel.copy()
        sport_feedback["dof_pos_cmd"] = sport_pos.copy()
        sport_feedback["dof_vel_cmd"] = sport_vel.copy()
        sport_feedback["dof_acc_cmd"] = np.zeros_like(sport_vel)
        sport_feedback["state"] = self.safe_robot_cfg.compose_state_from_dof(sport_pos, sport_vel)
        return sport_feedback

    def _clip_safe_control(self, control: np.ndarray) -> np.ndarray:
        control = np.asarray(control, dtype=float).reshape(-1).copy()
        for control_id in self.safe_robot_cfg.Control:
            control[control_id] = np.clip(
                control[control_id],
                -self.safe_robot_cfg.ControlLimit[control_id],
                self.safe_robot_cfg.ControlLimit[control_id],
            )
        return control

    def _safe_control_index(self, name: str) -> int | None:
        control = getattr(self.safe_robot_cfg, "Control", None)
        if control is None or not hasattr(control, name):
            return None
        return int(getattr(control, name))

    def _zero_nominal_base_controls(self, control: np.ndarray) -> None:
        for name in ("vLinearX", "vLinearY", "vRotYaw", "vLinearZ"):
            index = self._safe_control_index(name)
            if index is not None and index < control.shape[0]:
                control[index] = 0.0

    def _planar_yaw_command_from_safe_control(
        self, control: np.ndarray, agent_feedback: dict
    ) -> np.ndarray:
        """Return SONIC's body-frame planar command.

        The legacy PID controller uses ``dynamics_g`` and therefore already
        returns native body-frame mobile-base controls.  The legged tracker
        deliberately publishes a world-frame locomotion abstraction, so only
        that path requires a world-to-body rotation here.
        """
        control = np.asarray(control, dtype=float).reshape(-1)
        command = np.zeros(3, dtype=float)
        for out_index, name in enumerate(("vLinearX", "vLinearY", "vRotYaw")):
            index = self._safe_control_index(name)
            if index is not None and index < control.shape[0]:
                command[out_index] = control[index]
        if self.goal_tracking_type == "pid":
            return command
        base = np.asarray(agent_feedback["robot_base_frame"], dtype=float)
        yaw = float(np.arctan2(base[1, 0], base[0, 0]))
        c, s = np.cos(yaw), np.sin(yaw)
        return np.array(
            [
                c * command[0] + s * command[1],
                -s * command[0] + c * command[1],
                command[2],
            ]
        )

    def _world_planar_command_from_safe_control(
        self, control: np.ndarray, agent_feedback: dict
    ) -> np.ndarray:
        """Return the planar command in world axes for facing diagnostics."""
        control = np.asarray(control, dtype=float).reshape(-1)
        command = np.zeros(2, dtype=float)
        for out_index, name in enumerate(("vLinearX", "vLinearY")):
            index = self._safe_control_index(name)
            if index is not None and index < control.shape[0]:
                command[out_index] = control[index]
        if self.goal_tracking_type != "pid":
            return command
        base = np.asarray(agent_feedback["robot_base_frame"], dtype=float)
        yaw = float(np.arctan2(base[1, 0], base[0, 0]))
        c, s = np.cos(yaw), np.sin(yaw)
        return np.array(
            [
                c * command[0] - s * command[1],
                s * command[0] + c * command[1],
            ]
        )

    def _facing_yaw_after_pre_safe(
        self,
        facing_yaw: float | None,
        nominal_sonic_command: np.ndarray,
        safe_sonic_command: np.ndarray,
        sport_u_safe: np.ndarray,
        agent_feedback: dict,
    ) -> float | None:
        if self.goal_tracking_type == "pid":
            if facing_yaw is None:
                return None
            return float(
                facing_yaw + float(safe_sonic_command[2]) - float(nominal_sonic_command[2])
            )

        world_safe = self._world_planar_command_from_safe_control(sport_u_safe, agent_feedback)
        if np.linalg.norm(world_safe) > 1.0e-4:
            return float(np.arctan2(world_safe[1], world_safe[0]))
        return facing_yaw

    def _height_command_from_safe_control(self, control: np.ndarray) -> float:
        index = self._safe_control_index("vLinearZ")
        if index is None:
            return 0.0
        control = np.asarray(control, dtype=float).reshape(-1)
        if index >= control.shape[0]:
            return 0.0
        command = float(control[index])
        if self.pre_safe_height_command_limit > 0.0:
            command = float(
                np.clip(
                    command, -self.pre_safe_height_command_limit, self.pre_safe_height_command_limit
                )
            )
        return command

    def _sonic_height_target_from_command(
        self, agent_feedback: dict, height_command: float
    ) -> float | None:
        if abs(height_command) <= 1.0e-5:
            return None
        robot_base_frame = np.asarray(agent_feedback["robot_base_frame"], dtype=float)
        current_height = float(robot_base_frame[2, 3])
        target_height = (
            current_height + float(height_command) * self.pre_safe_height_target_lookahead
        )
        target_height = float(
            np.clip(target_height, self.pre_safe_height_min, self.pre_safe_height_max)
        )
        if (
            abs(target_height - self.pre_safe_height_default)
            <= self.pre_safe_height_recovery_deadband
        ):
            return None
        return target_height

    def _height_recovery_command(self, agent_feedback: dict, task_info: dict) -> float:
        if not self.pre_safe_height_recovery_enabled:
            return 0.0
        if self._base_goal_enabled(task_info):
            return 0.0

        robot_base_frame = np.asarray(agent_feedback["robot_base_frame"], dtype=float)
        current_height = float(robot_base_frame[2, 3])
        height_error = self.pre_safe_height_default - current_height
        if abs(height_error) <= self.pre_safe_height_recovery_deadband:
            return 0.0

        command = self.pre_safe_height_recovery_kp * height_error
        if self.pre_safe_height_target_lookahead > 1.0e-6:
            recovery_limit = height_error / self.pre_safe_height_target_lookahead
            if height_error > 0.0:
                command = min(command, recovery_limit)
            else:
                command = max(command, recovery_limit)
        if self.pre_safe_height_command_limit > 0.0:
            command = float(
                np.clip(
                    command, -self.pre_safe_height_command_limit, self.pre_safe_height_command_limit
                )
            )
        return float(command)

    def _base_goal_enabled(self, task_info: dict) -> bool:
        return bool(task_info.get("base_goal_enable", True))

    def _nominal_sonic_command(self, sport_u_ref: np.ndarray, agent_feedback: dict) -> np.ndarray:
        command = self._planar_yaw_command_from_safe_control(sport_u_ref, agent_feedback)
        return np.concatenate([command, np.zeros(2, dtype=float)])

    def _add_gripper_info(self, task_info: dict, action_info: dict) -> None:
        goal_teleop = task_info.get("goal_teleop", {})
        for key in (
            "left_gripper_goal",
            "right_gripper_goal",
            "gripper_goal",
            "left_grasp_target_name",
            "right_grasp_target_name",
            "grasp_target_name",
            "left_grasp_radius",
            "right_grasp_radius",
            "grasp_radius",
        ):
            if key in goal_teleop:
                action_info[key] = goal_teleop[key]
            elif key in task_info:
                action_info[key] = task_info[key]

    def _has_obstacles(self, task_info: dict) -> bool:
        obstacle_info = task_info.get("obstacle", {})
        num_obstacle = int(obstacle_info.get("num", 0))
        return num_obstacle > 0 and len(obstacle_info.get("frames_world", [])) > 0

    def _apply_pre_safe_hold(
        self,
        sport_u_ref: np.ndarray,
        sport_u_safe: np.ndarray,
        trigger_safe: bool,
        active_sides: set[str],
    ) -> tuple[np.ndarray, bool, set[str]]:
        if trigger_safe:
            self._pre_safe_hold_count = self.pre_safe_hold_steps
            self._last_sport_u_safe = np.asarray(sport_u_safe, dtype=float).reshape(-1).copy()
            self._last_sport_safe_delta = np.asarray(sport_u_safe, dtype=float).reshape(
                -1
            ) - np.asarray(sport_u_ref, dtype=float).reshape(-1)
            self._last_pre_safe_active_sides = set(active_sides)
            return sport_u_safe, False, set(active_sides)

        if self._pre_safe_hold_count > 0 and self._last_sport_safe_delta is not None:
            self._pre_safe_hold_count -= 1
            sport_u_ref = np.asarray(sport_u_ref, dtype=float).reshape(-1)
            return (
                sport_u_ref + self._last_sport_safe_delta,
                True,
                set(self._last_pre_safe_active_sides),
            )

        self._last_sport_u_safe = None
        self._last_sport_safe_delta = None
        self._last_pre_safe_active_sides = set()
        return sport_u_safe, False, set()

    def _smooth_sonic_loco_command(
        self,
        nominal_loco_command: np.ndarray,
        safe_loco_command: np.ndarray,
        safe_active: bool,
    ) -> np.ndarray:
        nominal_loco_command = np.asarray(nominal_loco_command, dtype=float).reshape(3)
        safe_loco_command = np.asarray(safe_loco_command, dtype=float).reshape(3)
        if not safe_active:
            self._last_sonic_loco_command = nominal_loco_command.copy()
            return safe_loco_command

        target = np.clip(
            safe_loco_command,
            -self.pre_safe_loco_command_limit,
            self.pre_safe_loco_command_limit,
        )
        if self._last_sonic_loco_command is None:
            self._last_sonic_loco_command = nominal_loco_command.copy()

        delta = np.clip(
            target - self._last_sonic_loco_command,
            -self.pre_safe_loco_command_rate_limit,
            self.pre_safe_loco_command_rate_limit,
        )
        smoothed = self._last_sonic_loco_command + delta
        self._last_sonic_loco_command = smoothed.copy()
        return smoothed

    def _active_pre_safe_sides(self, safe_control_info: dict) -> set[str]:
        phi_env = safe_control_info.get("phi_safe_mat_env", None)
        if phi_env is None:
            return set()

        phi_env = np.asarray(phi_env, dtype=float)
        if phi_env.size == 0:
            return set()

        active_mask = phi_env > 0.0
        env_mask = safe_control_info.get("env_collision_mask", None)
        if env_mask is not None:
            env_mask = np.asarray(env_mask, dtype=bool)
            if env_mask.shape == active_mask.shape:
                active_mask &= env_mask

        if not np.any(active_mask):
            return set()

        frame_ids = getattr(self.pre_safety_filter.safety_index, "collision_vol_frame_ids", [])
        active_sides = set()
        for robot_vol_idx in np.where(np.any(active_mask, axis=1))[0]:
            if robot_vol_idx >= len(frame_ids):
                continue
            frame_name = getattr(frame_ids[robot_vol_idx], "name", str(frame_ids[robot_vol_idx]))
            if frame_name.startswith("left_") or frame_name == "L_ee":
                active_sides.add("left")
            elif frame_name.startswith("right_") or frame_name == "R_ee":
                active_sides.add("right")
            elif frame_name.startswith("waist_") or frame_name.startswith("torso_"):
                active_sides.add("waist")
        return active_sides

    def visualization_context(
        self,
        agent_feedback: dict,
        task_info: dict,
        action_info: dict,
        robot_frames: np.ndarray,
        dist_robot_to_env: np.ndarray,
        robot_cfg: RobotConfig,
    ) -> dict | None:
        if (
            "pre_safe_phi_safe_mat_env" not in action_info
            and "pre_safe_phi_safe_mat_self" not in action_info
        ):
            return None

        sport_feedback = self._sport_feedback_from_whole_body(agent_feedback)
        sport_task_info = {
            **task_info,
            "robot_base_frame": agent_feedback["robot_base_frame"],
        }
        robot_collision_vol, obstacle_collision_vol = (
            self.pre_safety_filter.safety_index.get_vol_info(
                sport_feedback["state"],
                sport_task_info,
            )
        )
        pre_safe_robot_frames = np.asarray([vol.frame for vol in robot_collision_vol])
        pre_safe_distance_vector, _, pre_safe_normal, _ = (
            self.pre_safety_filter.safety_index.compute_pairwise_info(
                robot_collision_vol,
                obstacle_collision_vol,
            )
        )
        pre_safe_dist_env = -np.einsum("...i,...i->...", pre_safe_distance_vector, pre_safe_normal)

        visual_action_info = dict(action_info)
        for key in (
            "phi_safe_mat_env",
            "phi_safe_mat_self",
            "phi_hold_mat_env",
            "phi_hold_mat_self",
            "violation_mat_env",
            "violation_mat_self",
        ):
            pre_safe_key = f"pre_safe_{key}"
            if pre_safe_key in action_info:
                visual_action_info[key] = action_info[pre_safe_key]

        return {
            "action_info": visual_action_info,
            "safety_index": self.pre_safety_filter.safety_index,
            "robot_frames": pre_safe_robot_frames,
            "dist_robot_to_env": pre_safe_dist_env,
            "robot_cfg": self.safe_robot_cfg,
        }

    def observability_context(self) -> dict:
        # General pipeline metrics use the full-body geometry. The reduced
        # mobile-base pre-safety model is supplied separately by
        # visualization_context when its constraints are rendered.
        monitor = self.safety_monitor
        return {
            "constraint_monitor": monitor,
            "environment_constraint_mask": getattr(monitor, "env_collision_mask", None),
            "self_constraint_mask": getattr(monitor, "self_collision_mask", None),
            "constraint_gain": getattr(monitor, "k", None),
        }

    def _upper_body_delta_from_sport_control(
        self,
        agent_feedback: dict,
        sport_u_ref: np.ndarray,
        sport_u_safe: np.ndarray,
        safe_active: bool,
        active_sides: set[str],
    ) -> tuple[np.ndarray | None, np.ndarray | None, np.ndarray | None]:
        if not safe_active:
            self._pre_safe_upper_body_cmd = None
            return None, None, None

        sport_u_ref = np.asarray(sport_u_ref, dtype=float).reshape(-1)
        sport_u_safe = np.asarray(sport_u_safe, dtype=float).reshape(-1)
        # Only apply the upper-body correction to branches whose collision
        # volumes activated the safety constraint.  The previous all-true
        # mask made a left-arm or base obstacle overwrite both arms and the
        # waist, which visually appeared as a torso-first avoidance policy.
        mask = np.zeros(17, dtype=bool)
        if "waist" in active_sides:
            mask[self._WAIST_SLICE] = True
        if "left" in active_sides:
            mask[self._LEFT_ARM_SLICE] = True
        if "right" in active_sides:
            mask[self._RIGHT_ARM_SLICE] = True
        if not np.any(mask):
            self._pre_safe_upper_body_cmd = None
            return None, None, None

        dt = max(self.pre_safe_target_dt, 1.0e-6)

        if self.pre_safe_upper_body_mode == "delta":
            safe_delta = sport_u_safe[:17] - sport_u_ref[:17]
            if not np.any(np.abs(safe_delta[mask]) > 1.0e-9):
                return None, None, None
            upper_body_delta = np.zeros(17, dtype=float)
            safe_delta = np.clip(
                safe_delta,
                -self.pre_safe_upper_body_velocity_limit,
                self.pre_safe_upper_body_velocity_limit,
            )
            upper_body_delta[mask] = safe_delta[mask] * dt
            return None, None, upper_body_delta

        body_qpos, _ = self._body_feedback(agent_feedback)
        if body_qpos.shape[0] < self.body_robot_cfg.num_state:
            return None, None, None

        current_upper_body = body_qpos[19:36].copy()
        if self._pre_safe_upper_body_cmd is None:
            self._pre_safe_upper_body_cmd = current_upper_body.copy()

        # Match sport-mode semantics: integrate the full post-safe PID velocity
        # command into a persistent command state. Non-active branches stay tied
        # to the normal SONIC IK target through the mask below.
        safe_upper_body_velocity = np.clip(
            sport_u_safe[:17],
            -self.pre_safe_upper_body_velocity_limit,
            self.pre_safe_upper_body_velocity_limit,
        )
        self._pre_safe_upper_body_cmd[mask] += safe_upper_body_velocity[mask] * dt
        target_error = np.clip(
            self._pre_safe_upper_body_cmd - current_upper_body,
            -self.pre_safe_upper_body_position_error_limit,
            self.pre_safe_upper_body_position_error_limit,
        )
        self._pre_safe_upper_body_cmd = current_upper_body + target_error
        return self._pre_safe_upper_body_cmd.copy(), mask.copy(), None

    def act(self, agent_feedback, task_info):
        sport_feedback = self._sport_feedback_from_whole_body(agent_feedback)
        sport_u_ref, sport_action_info = self.goal_tracking_policy.act(
            agent_feedback=sport_feedback,
            task_info=task_info,
        )
        sport_u_ref = self._clip_safe_control(sport_u_ref)
        base_goal_enabled = self._base_goal_enabled(task_info)
        if not base_goal_enabled:
            self._zero_nominal_base_controls(sport_u_ref)

        if self.pre_safe_enabled and self._has_obstacles(task_info):
            sport_u_safe, safe_control_info = self.pre_safety_filter.safe_control(
                x=sport_feedback["state"],
                u_ref=sport_u_ref,
                agent_feedback=sport_feedback,
                task_info=task_info,
                action_info=sport_action_info,
            )
            sport_u_safe = self._clip_safe_control(sport_u_safe)
        else:
            sport_u_safe = sport_u_ref.copy()
            safe_control_info = {"trigger_safe": False}
        pre_safe_active_sides = self._active_pre_safe_sides(safe_control_info)
        sport_u_safe, pre_safe_hold_active, pre_safe_active_sides = self._apply_pre_safe_hold(
            sport_u_ref,
            sport_u_safe,
            bool(safe_control_info.get("trigger_safe", False)),
            pre_safe_active_sides,
        )
        sport_u_safe = self._clip_safe_control(sport_u_safe)

        nominal_sonic_command = self._nominal_sonic_command(sport_u_ref, agent_feedback)
        safe_sonic_command = nominal_sonic_command.copy()
        pre_safe_active = bool(safe_control_info.get("trigger_safe", False) or pre_safe_hold_active)
        pre_safe_loco_applied = pre_safe_active
        safe_planar_yaw_command = self._planar_yaw_command_from_safe_control(
            sport_u_safe, agent_feedback
        )
        safe_height_command = 0.0
        if pre_safe_loco_applied:
            safe_sonic_command[:3] = self._smooth_sonic_loco_command(
                nominal_sonic_command[:3],
                safe_planar_yaw_command,
                True,
            )
            safe_height_command = self._height_command_from_safe_control(sport_u_safe)
            safe_sonic_command[3] = safe_height_command
        else:
            safe_sonic_command[:3] = self._smooth_sonic_loco_command(
                nominal_sonic_command[:3],
                nominal_sonic_command[:3],
                False,
            )
            safe_height_command = self._height_recovery_command(agent_feedback, task_info)
            safe_sonic_command[3] = safe_height_command
        pre_safe_height_target = self._sonic_height_target_from_command(
            agent_feedback, safe_height_command
        )
        (
            upper_body_target_override,
            upper_body_target_override_mask,
            upper_body_target_delta,
        ) = self._upper_body_delta_from_sport_control(
            agent_feedback,
            sport_u_ref,
            sport_u_safe,
            pre_safe_active,
            pre_safe_active_sides,
        )

        sonic_task_info = {
            **task_info,
            "sonic_command": safe_sonic_command.copy(),
            "sonic_command_nominal": nominal_sonic_command.copy(),
            "sport_u_ref": sport_u_ref.copy(),
            "sport_u_safe": sport_u_safe.copy(),
            "sonic_loco_safety_override": bool(pre_safe_loco_applied),
            "sonic_external_tracking": True,
        }
        facing_yaw = sport_action_info.get("locomotion_facing_yaw", None)
        if pre_safe_loco_applied:
            # PID translation and yaw are independent mobile-base channels.
            # A safe lateral correction must not replace the pose yaw with
            # the movement direction. Preserve the PID target and add only
            # the yaw correction introduced by the QP. Legged tracking keeps
            # its movement-facing safety behavior.
            facing_yaw = self._facing_yaw_after_pre_safe(
                facing_yaw,
                nominal_sonic_command,
                safe_sonic_command,
                sport_u_safe,
                agent_feedback,
            )
        if facing_yaw is not None:
            sonic_task_info["locomotion_facing_yaw"] = float(facing_yaw)
        if pre_safe_height_target is not None:
            sonic_task_info["sonic_height"] = pre_safe_height_target
        if upper_body_target_override is not None:
            sonic_task_info["upper_body_target_override"] = upper_body_target_override
            sonic_task_info["upper_body_target_override_mask"] = upper_body_target_override_mask
        if upper_body_target_delta is not None:
            sonic_task_info["upper_body_target_delta"] = upper_body_target_delta
        sonic_u_ref, sonic_action_info = self.policy.act(
            agent_feedback=agent_feedback,
            task_info=sonic_task_info,
        )

        pre_safe_control_info = {
            f"pre_safe_{key}": value for key, value in safe_control_info.items()
        }
        action_info = {
            **sonic_action_info,
            **pre_safe_control_info,
            "trigger_safe": bool(
                safe_control_info.get("trigger_safe", False) or pre_safe_hold_active
            ),
            "sport_action_info": sport_action_info,
            "sport_u_ref": sport_u_ref.copy(),
            "sport_u_safe": sport_u_safe.copy(),
            "goal_tracking_type": self.goal_tracking_type,
            "sonic_command_nominal": nominal_sonic_command.copy(),
            "sonic_command_safe": safe_sonic_command.copy(),
            "sonic_loco_command_raw_safe": safe_planar_yaw_command.copy(),
            "pre_safe_height_command": float(safe_height_command),
            "pre_safe_height_target": -1.0
            if pre_safe_height_target is None
            else float(pre_safe_height_target),
            "base_goal_enabled": bool(base_goal_enabled),
            "pre_safe_loco_applied": bool(pre_safe_loco_applied),
            "pre_safe_upper_body_target_applied": (
                upper_body_target_delta is not None or upper_body_target_override is not None
            ),
            "pre_safe_upper_body_delta_applied": upper_body_target_delta is not None,
            "pre_safe_upper_body_delta": (
                upper_body_target_delta.copy()
                if upper_body_target_delta is not None
                else np.zeros(17, dtype=float)
            ),
            "pre_safe_upper_body_override_applied": upper_body_target_override is not None,
            "pre_safe_upper_body_override": (
                upper_body_target_override.copy()
                if upper_body_target_override is not None
                else np.zeros(17, dtype=float)
            ),
            "pre_safe_upper_body_override_mask": (
                upper_body_target_override_mask.copy()
                if upper_body_target_override_mask is not None
                else np.zeros(17, dtype=bool)
            ),
            "pre_safe_upper_body_mode": self.pre_safe_upper_body_mode,
            "pre_safe_active_sides": tuple(sorted(pre_safe_active_sides)),
            "pre_safe_hold_active": bool(pre_safe_hold_active),
            "pre_safe_hold_count": int(self._pre_safe_hold_count),
            "pre_safe_enabled": self.pre_safe_enabled,
            "u_ref": np.asarray(sonic_u_ref, dtype=float).copy(),
            "u_safe": np.asarray(sonic_u_ref, dtype=float).copy(),
        }
        self._add_gripper_info(task_info, action_info)
        return sonic_u_ref, action_info
