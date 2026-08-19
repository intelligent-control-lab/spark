from __future__ import annotations

from typing import Any, Dict, Tuple

import numpy as np

from spark_policy.core.policy import BasePolicy
from spark_policy.control.whole_body.unitree_g1.wbt.config import (
    WBT_MOTOR_KDS,
    WBT_MOTOR_KPS,
)
from spark_robot import RobotConfig, RobotKinematics, UnitreeG1WholeBodyDynamic1Config

from .client import SonicClient


SONIC_LOCOMOTION_IDLE = 0
SONIC_LOCOMOTION_SLOW_WALK = 1
SONIC_LOCOMOTION_WALK = 2
SONIC_LOCOMOTION_RUN = 3
SONIC_LOCOMOTION_IDLE_SQUAT = 4

DEFAULT_UPPER_BODY_MOTOR_KPS = [
    300.0,
    300.0,
    300.0,
    100.0,
    100.0,
    50.0,
    50.0,
    20.0,
    20.0,
    20.0,
    100.0,
    100.0,
    50.0,
    50.0,
    20.0,
    20.0,
    20.0,
]
DEFAULT_UPPER_BODY_MOTOR_KDS = [
    3.0,
    3.0,
    3.0,
    2.0,
    2.0,
    2.0,
    2.0,
    1.0,
    1.0,
    1.0,
    2.0,
    2.0,
    2.0,
    2.0,
    1.0,
    1.0,
    1.0,
]

# SPARK upper-body order:
#   waist, all-left-arm, all-right-arm.
# SONIC planner upper_body_position order:
#   waist, interleaved left/right joints.
SPARK_TO_SONIC_UPPER_BODY = np.asarray(
    [0, 1, 2, 3, 10, 4, 11, 5, 12, 6, 13, 7, 14, 8, 15, 9, 16],
    dtype=int,
)


def _as_vector(value, length: int, name: str) -> np.ndarray:
    array = np.asarray(value, dtype=float)
    if array.shape == ():
        return np.full(length, float(array), dtype=float)
    array = array.reshape(-1)
    if array.shape[0] != length:
        raise ValueError(f"{name} must have {length} entries, got {array.shape[0]}")
    return array.copy()


def _wrap_to_pi(angle: float) -> float:
    return (float(angle) + np.pi) % (2.0 * np.pi) - np.pi


class SonicBaseTracker:
    """Shape final nominal/safety base intent for SONIC locomotion.

    The goal controller and safety filter operate in a virtual mobile-base
    space. SONIC realizes that intent through a gait, so its final input needs
    bounded acceleration and jerk rather than per-cycle PID discontinuities.
    This tracker is deliberately local to the concrete SONIC policy.
    """

    def __init__(
        self,
        dt: float,
        acceleration_limit,
        jerk_limit,
        safety_acceleration_limit,
        safety_jerk_limit,
    ) -> None:
        self.dt = float(dt)
        self.acceleration_limit = _as_vector(
            acceleration_limit, 5, "base_tracker_acceleration_limit"
        )
        self.jerk_limit = _as_vector(jerk_limit, 5, "base_tracker_jerk_limit")
        self.safety_acceleration_limit = _as_vector(
            safety_acceleration_limit, 5, "base_tracker_safety_acceleration_limit"
        )
        self.safety_jerk_limit = _as_vector(safety_jerk_limit, 5, "base_tracker_safety_jerk_limit")
        self.command = np.zeros(5, dtype=float)
        self.acceleration = np.zeros(5, dtype=float)
        self.phase = "idle"

    def reset(self) -> None:
        self.command.fill(0.0)
        self.acceleration.fill(0.0)
        self.phase = "idle"

    def update(self, target, *, safety_active: bool = False) -> np.ndarray:
        target = np.asarray(target, dtype=float).reshape(5)
        dt = max(self.dt, 1.0e-6)
        acceleration_limit = (
            self.safety_acceleration_limit if safety_active else self.acceleration_limit
        )
        jerk_limit = self.safety_jerk_limit if safety_active else self.jerk_limit
        desired_acceleration = np.clip(
            (target - self.command) / dt, -acceleration_limit, acceleration_limit
        )
        acceleration_delta = np.clip(
            desired_acceleration - self.acceleration,
            -jerk_limit * dt,
            jerk_limit * dt,
        )
        self.acceleration = np.clip(
            self.acceleration + acceleration_delta,
            -acceleration_limit,
            acceleration_limit,
        )
        previous_error = target - self.command
        candidate = self.command + self.acceleration * dt
        new_error = target - candidate
        reached = previous_error * new_error <= 0.0
        candidate[reached] = target[reached]
        self.acceleration[reached] = 0.0
        self.command = candidate

        planar_speed = float(np.linalg.norm(self.command[:2]))
        target_speed = float(np.linalg.norm(target[:2]))
        if (
            planar_speed < 0.01
            and target_speed < 0.01
            and abs(self.command[2]) < 0.02
            and abs(target[2]) < 0.02
        ):
            self.phase = "idle"
        elif target_speed + 1.0e-4 < planar_speed:
            self.phase = "braking"
        elif safety_active:
            self.phase = "safety"
        else:
            self.phase = "tracking"
        return self.command.copy()


class UnitreeG1SonicPolicy(BasePolicy):
    """SPARK policy wrapper for an external SONIC whole-body policy server.

    The wrapper keeps the SPARK whole-body interface: Cartesian arm goals come in
    through ``task_info['goal_teleop']`` and the policy returns whole-body joint
    position targets for ``UnitreeG1WholeBodyMujocoAgent``.
    """

    def __init__(self, robot_cfg: RobotConfig, robot_kinematics: RobotKinematics, **kwargs) -> None:
        super().__init__(robot_cfg, robot_kinematics)

        self.body_robot_cfg = UnitreeG1WholeBodyDynamic1Config()
        self.default_qpos = np.array(
            [self.body_robot_cfg.DefaultDoFVal[dof] for dof in self.body_robot_cfg.DoFs],
            dtype=np.float64,
        )
        self.model_default_qpos = np.array(
            [self.robot_cfg.DefaultDoFVal[dof] for dof in self.robot_cfg.DoFs],
            dtype=np.float64,
        )
        self.model_actuated_qpos_indices = self._model_actuated_qpos_indices()
        self.body_actuated_qpos_indices = self._body_actuated_qpos_indices()
        self.num_actuated = self.model_actuated_qpos_indices.shape[0]
        sonic_default_actuated_pos = kwargs.get("sonic_default_actuated_pos", None)
        if sonic_default_actuated_pos is not None:
            sonic_default_actuated_pos = np.asarray(
                sonic_default_actuated_pos, dtype=np.float64
            ).reshape(-1)
            if sonic_default_actuated_pos.shape[0] != 29:
                raise ValueError(
                    f"sonic_default_actuated_pos must have 29 entries, got {sonic_default_actuated_pos.shape[0]}"
                )
            self.default_qpos[7:36] = sonic_default_actuated_pos
            for model_qpos_idx, value in zip(
                self.model_actuated_qpos_indices[:29], sonic_default_actuated_pos
            ):
                self.model_default_qpos[int(model_qpos_idx)] = value

        self.client = SonicClient(
            endpoint=kwargs.get("sonic_endpoint", "tcp://127.0.0.1:5560"),
            timeout_ms=int(kwargs.get("sonic_timeout_ms", 50)),
            linger_ms=int(kwargs.get("sonic_linger_ms", 0)),
        )
        self.protocol = str(kwargs.get("sonic_protocol", "spark_sonic_policy_v1"))
        self.fallback_mode = str(kwargs.get("sonic_fallback_mode", "hold"))
        if self.fallback_mode not in ("hold", "default", "raise"):
            raise ValueError("sonic_fallback_mode must be one of: hold, default, raise")

        self.safe_target_dt = float(kwargs.get("safe_target_dt", 0.02))
        self.control_dt = float(kwargs.get("control_dt", self.safe_target_dt))
        self.apply_safe_control_targets = bool(kwargs.get("apply_safe_control_targets", False))
        self.safe_control_only_when_triggered = bool(
            kwargs.get("safe_control_only_when_triggered", True)
        )
        self.use_policy_motor_gains = bool(kwargs.get("use_policy_motor_gains", False))
        self.post_sonic_upper_body_overwrite = bool(
            kwargs.get("post_sonic_upper_body_overwrite", False)
        )
        self.override_sonic_upper_body_target = bool(
            kwargs.get("override_sonic_upper_body_target", False)
        )
        self.base_disabled_identity_facing = bool(kwargs.get("base_disabled_identity_facing", True))
        self.base_disabled_zero_waist_upper_body_input = bool(
            kwargs.get("base_disabled_zero_waist_upper_body_input", True)
        )

        self.base_position_kp = np.asarray(
            kwargs.get("base_position_kp", [0.80, 0.80, 0.60]), dtype=float
        )
        self.base_velocity_kd = np.asarray(
            kwargs.get("base_velocity_kd", [0.35, 0.35]), dtype=float
        )
        self.base_orientation_kp = np.asarray(
            kwargs.get("base_orientation_kp", [0.50, 0.50, 0.50]), dtype=float
        )
        self.base_xy_yaw_limit = np.asarray(
            kwargs.get("base_xy_yaw_limit", [0.35, 0.35, 0.50]), dtype=float
        )
        self.base_planar_velocity_limit = float(kwargs.get("base_planar_velocity_limit", 0.12))
        self.base_goal_xy_deadband = float(kwargs.get("base_goal_xy_deadband", 0.12))
        self.base_goal_resume_distance = float(kwargs.get("base_goal_resume_distance", 0.22))
        self.base_goal_yaw_deadband = float(kwargs.get("base_goal_yaw_deadband", 0.05))
        self.base_goal_yaw_resume_distance = float(
            kwargs.get("base_goal_yaw_resume_distance", 0.10)
        )
        self.base_goal_slowdown_distance = float(kwargs.get("base_goal_slowdown_distance", 0.50))
        self.base_goal_slowdown_min_speed = float(kwargs.get("base_goal_slowdown_min_speed", 0.20))
        self.base_goal_slowdown_enabled = bool(kwargs.get("base_goal_slowdown_enabled", True))
        self.base_stop_lookahead_time = float(kwargs.get("base_stop_lookahead_time", 0.80))
        self.base_stop_distance_bias = float(kwargs.get("base_stop_distance_bias", 0.03))
        self.base_settle_velocity = float(kwargs.get("base_settle_velocity", 0.04))
        self.base_settle_cycles = max(1, int(kwargs.get("base_settle_cycles", 5)))
        self.base_z_pitch_limit = np.asarray(
            kwargs.get("base_z_pitch_limit", [0.03, 0.03]), dtype=float
        )
        self.base_command_rate_limit = np.asarray(
            kwargs.get("base_command_rate_limit", [0.08, 0.08, 0.12, 0.004, 0.004]),
            dtype=float,
        )
        self.base_command_deadband = np.asarray(
            kwargs.get("base_command_deadband", [0.010, 0.010, 0.020, 0.004, 0.004]),
            dtype=float,
        )
        self.base_tracker = SonicBaseTracker(
            self.control_dt,
            kwargs.get("base_tracker_acceleration_limit", [0.20, 0.20, 0.50, 0.10, 0.10]),
            kwargs.get("base_tracker_jerk_limit", [0.80, 0.80, 2.00, 0.40, 0.40]),
            kwargs.get("base_tracker_safety_acceleration_limit", [0.50, 0.50, 1.20, 0.20, 0.20]),
            kwargs.get("base_tracker_safety_jerk_limit", [3.00, 3.00, 6.00, 1.00, 1.00]),
        )
        self.sonic_planar_speed_scale = float(kwargs.get("sonic_planar_speed_scale", 1.5))
        self.sonic_slow_min_speed = float(kwargs.get("sonic_slow_min_speed", 0.2))
        self.sonic_slow_walk_max_speed = float(kwargs.get("sonic_slow_walk_max_speed", 0.8))
        self.sonic_walk_min_speed = float(kwargs.get("sonic_walk_min_speed", 0.8))
        self.sonic_walk_max_speed = float(kwargs.get("sonic_walk_max_speed", 2.5))
        self.sonic_run_min_speed = float(kwargs.get("sonic_run_min_speed", 2.5))
        self.sonic_max_speed = float(kwargs.get("sonic_max_speed", 3.0))
        self.sonic_motion_deadband = float(kwargs.get("sonic_motion_deadband", 0.02))
        self.sonic_speed_quantization = float(kwargs.get("sonic_speed_quantization", 0.05))
        self.sonic_direction_update_angle = float(kwargs.get("sonic_direction_update_angle", 0.12))
        self.sonic_facing_update_angle = float(kwargs.get("sonic_facing_update_angle", 0.12))
        self.sonic_safety_speed_quantization = float(
            kwargs.get("sonic_safety_speed_quantization", 0.025)
        )
        self.sonic_safety_direction_update_angle = float(
            kwargs.get("sonic_safety_direction_update_angle", 0.06)
        )
        self.sonic_safety_facing_update_angle = float(
            kwargs.get("sonic_safety_facing_update_angle", 0.06)
        )
        self.sonic_align_enter_angle = float(kwargs.get("sonic_align_enter_angle", 0.60))
        self.sonic_align_exit_angle = float(kwargs.get("sonic_align_exit_angle", 0.25))
        self.sonic_align_min_distance = float(kwargs.get("sonic_align_min_distance", 0.35))
        self.sonic_locomotion_mode = self._normalize_locomotion_mode(
            kwargs.get("sonic_locomotion_mode", "slow")
        )
        self.sonic_height_control_enabled = bool(kwargs.get("sonic_height_control_enabled", True))
        self.sonic_default_height = float(kwargs.get("sonic_default_height", 0.793))
        self.sonic_height_min = float(kwargs.get("sonic_height_min", 0.2))
        self.sonic_height_max = float(kwargs.get("sonic_height_max", 1.0))
        self.sonic_height_deadband = float(kwargs.get("sonic_height_deadband", 0.01))
        self.sonic_height_filter_gain = float(kwargs.get("sonic_height_filter_gain", 0.35))
        self.sonic_height_rate_limit = float(kwargs.get("sonic_height_rate_limit", 0.006))
        # Learned SONIC locomotion is substantially more stable when its torso
        # faces the direction of travel. Final task yaw is completed after the
        # translational approach instead of demanding long side/back strides.
        self.sonic_facing_mode = str(kwargs.get("sonic_facing_mode", "movement"))
        if self.sonic_facing_mode not in ("movement", "goal", "command"):
            raise ValueError("sonic_facing_mode must be one of: movement, goal, command")

        self.upper_body_target_rate_limit = _as_vector(
            kwargs.get("upper_body_target_rate_limit", [0.006, 0.006, 0.006] + [0.014] * 14),
            17,
            "upper_body_target_rate_limit",
        )
        self.override_sonic_upper_body_motor_gains = bool(
            kwargs.get("override_sonic_upper_body_motor_gains", False)
        )
        self.use_wbt_motor_gains = bool(kwargs.get("use_wbt_motor_gains", False))
        self.disable_upper_body_motor_gain_override_when_base_goal_disabled = bool(
            kwargs.get("disable_upper_body_motor_gain_override_when_base_goal_disabled", True)
        )
        self.upper_body_motor_kps = _as_vector(
            kwargs.get("upper_body_motor_kps", DEFAULT_UPPER_BODY_MOTOR_KPS),
            17,
            "upper_body_motor_kps",
        )
        self.upper_body_motor_kds = _as_vector(
            kwargs.get("upper_body_motor_kds", DEFAULT_UPPER_BODY_MOTOR_KDS),
            17,
            "upper_body_motor_kds",
        )
        self.upper_body_target_filter_gain = float(kwargs.get("upper_body_target_filter_gain", 1.0))
        self.disable_upper_body_target_rate_limit = bool(
            kwargs.get("disable_upper_body_target_rate_limit", False)
        )
        self.upper_body_gravity_compensation = bool(
            kwargs.get("upper_body_gravity_compensation", True)
        )
        self.last_upper_body_target = self.default_qpos[19:36].astype(np.float64).copy()
        self._last_filtered_upper_body_target = self.last_upper_body_target.copy()
        self._last_base_command = None
        self._last_base_xy = None
        self._last_sonic_height = self.sonic_default_height
        self._base_goal_arrived = False
        self._base_goal_hold_xy = None
        self._base_translation_stopped = False
        self._base_translation_hold_xy = None
        self._base_settle_count = 0
        self._seq = 0
        self._last_target_actuated_pos = self.default_qpos[7:36].astype(float).copy()
        self._last_target_qpos = self.default_qpos.copy()
        self._reset_requested = True
        self._last_planner_movement = None
        self._last_planner_facing = None
        self._sonic_aligning = False

        self.use_fixed_base_arm_kinematics = bool(kwargs.get("use_fixed_base_arm_kinematics", True))
        self.arm_kinematics_mode = "robot"
        self.arm_robot_cfg = None
        self.arm_kinematics = self.robot_kinematics
        if self.use_fixed_base_arm_kinematics:
            try:
                from spark_robot.unitree_g1.config.unitree_g1_fixed_base_dynamic_1_config import (
                    UnitreeG1FixedBaseDynamic1Config,
                )
                from spark_robot.unitree_g1.kinematics.unitree_g1_fixed_base_kinematics import (
                    UnitreeG1FixedBaseKinematics,
                )

                self.arm_robot_cfg = UnitreeG1FixedBaseDynamic1Config()
                self.arm_kinematics = UnitreeG1FixedBaseKinematics(self.arm_robot_cfg)
                self.arm_kinematics_mode = "fixed_base"
            except Exception as exc:
                print("UnitreeG1SonicPolicy falling back to provided arm kinematics", exc)
                self.arm_robot_cfg = None
                self.arm_kinematics = self.robot_kinematics
                self.arm_kinematics_mode = "robot"

    def _model_actuated_qpos_indices(self) -> np.ndarray:
        indices = []
        for motor in self.robot_cfg.MujocoMotors:
            if not hasattr(self.robot_cfg.MujocoDoFs, motor.name):
                raise KeyError(f"No MuJoCo DoF named {motor.name} for motor {motor}")
            indices.append(int(getattr(self.robot_cfg.MujocoDoFs, motor.name)))
        return np.asarray(indices, dtype=int)

    def _body_actuated_qpos_indices(self) -> np.ndarray:
        indices = []
        for motor in self.robot_cfg.MujocoMotors:
            if not hasattr(self.body_robot_cfg.DoFs, motor.name):
                continue
            indices.append(int(getattr(self.body_robot_cfg.DoFs, motor.name)))
        if len(indices) != 29:
            raise ValueError(f"SONIC body policy expects 29 body motors, got {len(indices)}")
        return np.asarray(indices, dtype=int)

    def _raw_qpos_qvel_from_feedback(self, agent_feedback: dict) -> Tuple[np.ndarray, np.ndarray]:
        has_body_feedback = "body_qpos_fbk" in agent_feedback
        qpos_key = "body_qpos_fbk" if has_body_feedback else "qpos_fbk"
        qvel_key = "body_qvel_fbk" if has_body_feedback else "qvel_fbk"
        qpos = np.asarray(
            agent_feedback.get(qpos_key, agent_feedback["dof_pos_fbk"]), dtype=float
        ).reshape(-1)
        qvel = np.asarray(
            agent_feedback.get(qvel_key, agent_feedback.get("dof_vel_fbk", np.zeros(0))),
            dtype=float,
        ).reshape(-1)
        if has_body_feedback:
            return qpos, qvel

        if qpos.shape[0] < self.model_default_qpos.shape[0]:
            full = self.model_default_qpos.copy()
            full[: qpos.shape[0]] = qpos
            qpos = full
        return qpos, qvel

    def _virtual_qpos(self, qpos: np.ndarray) -> np.ndarray:
        qpos = np.asarray(qpos, dtype=float).reshape(-1)
        if qpos.shape[0] == self.default_qpos.shape[0]:
            return qpos.copy()
        state = np.asarray(self.robot_cfg.compose_state_from_dof(qpos, None), dtype=float).reshape(
            -1
        )
        if state.shape[0] == self.default_qpos.shape[0]:
            return state
        full = self.default_qpos.copy()
        full[: min(full.shape[0], qpos.shape[0])] = qpos[: min(full.shape[0], qpos.shape[0])]
        return full

    def _virtual_qvel(self, qpos: np.ndarray, qvel: np.ndarray) -> np.ndarray:
        qpos = np.asarray(qpos, dtype=float).reshape(-1)
        qvel = np.asarray(qvel, dtype=float).reshape(-1)
        if qpos.shape[0] == self.default_qpos.shape[0] and qvel.shape[0] >= 35:
            return qvel[:35].copy()

        virtual_qvel = np.zeros(35, dtype=float)
        qpos_qvel_offset = 1 if qpos.shape[0] - qvel.shape[0] == 1 else 0
        for model_qpos_idx, body_qpos_idx in zip(
            self.model_actuated_qpos_indices, self.body_actuated_qpos_indices
        ):
            model_qvel_idx = int(model_qpos_idx) - qpos_qvel_offset
            body_qvel_idx = int(body_qpos_idx) - 1
            if 0 <= model_qvel_idx < qvel.shape[0] and 0 <= body_qvel_idx < virtual_qvel.shape[0]:
                virtual_qvel[body_qvel_idx] = qvel[model_qvel_idx]
        return virtual_qvel

    def _qpos_qvel_from_feedback(self, agent_feedback: dict) -> Tuple[np.ndarray, np.ndarray]:
        qpos_raw, qvel_raw = self._raw_qpos_qvel_from_feedback(agent_feedback)
        return self._virtual_qpos(qpos_raw), self._virtual_qvel(qpos_raw, qvel_raw)

    def _actuated_qpos_from_raw(self, qpos: np.ndarray) -> np.ndarray:
        qpos = np.asarray(qpos, dtype=float).reshape(-1)
        if qpos.shape[0] > int(np.max(self.model_actuated_qpos_indices)):
            return qpos[self.model_actuated_qpos_indices[:29]].copy()
        return self._actuated_qpos_from_virtual(self._virtual_qpos(qpos))

    def _actuated_qpos_from_virtual(self, qpos: np.ndarray) -> np.ndarray:
        return np.asarray(qpos, dtype=float).reshape(-1)[self.body_actuated_qpos_indices].copy()

    def _control_limit_vector(self) -> np.ndarray:
        return np.asarray(
            [self.robot_cfg.ControlLimit[control_id] for control_id in self.robot_cfg.Control],
            dtype=float,
        )

    def _normalize_locomotion_mode(self, mode: Any) -> str:
        mode = str(mode).lower()
        if mode == "auto":
            mode = "hybrid"
        if mode not in ("slow", "walk", "run", "hybrid"):
            raise ValueError("sonic_locomotion_mode must be one of: slow, walk, run, hybrid")
        return mode

    def _zero_base_command(self) -> np.ndarray:
        return self._filter_base_command(np.zeros(5, dtype=float))

    def _estimate_base_xy_velocity(self, base_xy: np.ndarray) -> np.ndarray:
        base_xy = np.asarray(base_xy, dtype=float).reshape(2)
        if self._last_base_xy is None:
            velocity = np.zeros(2, dtype=float)
        else:
            dt = max(self.control_dt, 1.0e-6)
            velocity = (base_xy - self._last_base_xy) / dt
        self._last_base_xy = base_xy.copy()
        return velocity

    def _set_base_goal_arrived(self, goal_xy: np.ndarray) -> np.ndarray:
        self._base_goal_arrived = True
        self._base_goal_hold_xy = np.asarray(goal_xy, dtype=float).reshape(2).copy()
        return self._zero_base_command()

    def _reset_base_goal_arrival(self) -> None:
        self._base_goal_arrived = False
        self._base_goal_hold_xy = None
        self._base_translation_stopped = False
        self._base_translation_hold_xy = None
        self._base_settle_count = 0

    def _safe_control_reference(
        self, current_actuated_pos: np.ndarray, target_actuated_pos: np.ndarray
    ) -> np.ndarray:
        dt = max(self.safe_target_dt, 1.0e-6)
        u_ref = (target_actuated_pos - current_actuated_pos[: target_actuated_pos.shape[0]]) / dt
        control_limit = self._control_limit_vector()
        if u_ref.shape == control_limit.shape:
            u_ref = np.clip(u_ref, -control_limit, control_limit)
        return u_ref

    def _target_model_qpos(self, qpos_raw: np.ndarray, target_qpos: np.ndarray) -> np.ndarray:
        """Return a target in the active MuJoCo model qpos layout.

        SONIC predicts the canonical 29 body motors only. The hand MuJoCo
        model adds finger DoFs, so keep those finger joints at their current
        state and overwrite only the 29 body motor positions.
        """
        qpos_raw = np.asarray(qpos_raw, dtype=float).reshape(-1)
        target_qpos = np.asarray(target_qpos, dtype=float).reshape(-1)
        if qpos_raw.shape[0] == target_qpos.shape[0]:
            return target_qpos.copy()
        if qpos_raw.shape[0] > int(np.max(self.model_actuated_qpos_indices)):
            model_target = qpos_raw.copy()
            model_target[self.model_actuated_qpos_indices[:29]] = target_qpos[7:36]
            return model_target
        return target_qpos.copy()

    def _add_gripper_info(self, task_info: dict, action_info: Dict[str, Any]) -> None:
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

    def _filter_base_command(
        self, command: np.ndarray, *, safety_active: bool = False
    ) -> np.ndarray:
        command = np.asarray(command, dtype=float).reshape(5).copy()
        command[:3] = np.clip(command[:3], -self.base_xy_yaw_limit, self.base_xy_yaw_limit)
        command[3:] = np.clip(command[3:], -self.base_z_pitch_limit, self.base_z_pitch_limit)

        planar_norm = float(np.linalg.norm(command[:2]))
        if self.base_planar_velocity_limit > 0.0 and planar_norm > self.base_planar_velocity_limit:
            command[:2] *= self.base_planar_velocity_limit / max(planar_norm, 1.0e-6)

        if self.base_command_deadband.shape == command.shape:
            command[np.abs(command) < self.base_command_deadband] = 0.0

        command = self.base_tracker.update(command, safety_active=safety_active)
        self._last_base_command = command.copy()
        return command

    def _base_command_from_goal(
        self, agent_feedback: dict, task_info: dict, *, apply_tracker: bool = True
    ) -> np.ndarray:
        safety_active = bool(task_info.get("sonic_loco_safety_override", False))

        def finish(command):
            command = np.asarray(command, dtype=float).reshape(5)
            if not apply_tracker:
                return command
            return self._filter_base_command(command, safety_active=safety_active)

        base_goal_enabled = bool(task_info.get("base_goal_enable", True))
        allow_disabled_base_command = bool(
            task_info.get("allow_base_command_when_base_goal_disabled", False)
            or task_info.get("sonic_loco_safety_override", False)
        )
        if not bool(task_info.get("base_goal_enable", True)):
            if not allow_disabled_base_command:
                self._reset_base_goal_arrival()
                return finish(np.zeros(5, dtype=float))

        for key in ("sonic_command", "base_command", "locomotion_command"):
            if key in task_info:
                self._reset_base_goal_arrival()
                command = np.asarray(task_info[key], dtype=float).reshape(-1)
                if command.shape[0] == 3:
                    command = np.concatenate([command, np.zeros(2)])
                if command.shape[0] != 5:
                    raise ValueError(f"{key} must have 3 or 5 entries.")
                return finish(command)

        goal_teleop = task_info.get("goal_teleop", {})
        base_goal = goal_teleop.get("base", None)
        if base_goal is None or not base_goal_enabled:
            self._reset_base_goal_arrival()
            return finish(np.zeros(5, dtype=float))

        base_goal = np.asarray(base_goal, dtype=float).reshape(-1, 4, 4)[0]
        robot_base_frame = np.asarray(agent_feedback["robot_base_frame"], dtype=float)
        base_xy = robot_base_frame[:2, 3]
        goal_xy = base_goal[:2, 3]
        base_xy_velocity = self._estimate_base_xy_velocity(base_xy)

        yaw = np.arctan2(robot_base_frame[1, 0], robot_base_frame[0, 0])
        target_yaw = np.arctan2(base_goal[1, 0], base_goal[0, 0])
        yaw_error = _wrap_to_pi(target_yaw - yaw)
        pos_error_world = base_goal[:3, 3] - robot_base_frame[:3, 3]
        c, s = np.cos(yaw), np.sin(yaw)
        xy_body = np.array(
            [
                c * pos_error_world[0] + s * pos_error_world[1],
                -s * pos_error_world[0] + c * pos_error_world[1],
            ],
            dtype=float,
        )
        xy_velocity_body = np.array(
            [
                c * base_xy_velocity[0] + s * base_xy_velocity[1],
                -s * base_xy_velocity[0] + c * base_xy_velocity[1],
            ],
            dtype=float,
        )
        xy_distance = float(np.linalg.norm(xy_body))
        resume_distance = max(self.base_goal_resume_distance, self.base_goal_xy_deadband)
        yaw_resume_distance = max(self.base_goal_yaw_resume_distance, self.base_goal_yaw_deadband)
        if self._base_goal_arrived:
            goal_unchanged = (
                self._base_goal_hold_xy is not None
                and float(np.linalg.norm(goal_xy - self._base_goal_hold_xy)) < resume_distance
            )
            robot_still_at_goal = (
                xy_distance < resume_distance and abs(yaw_error) < yaw_resume_distance
            )
            if goal_unchanged and robot_still_at_goal:
                return finish(np.zeros(5, dtype=float))
            self._reset_base_goal_arrival()

        if (
            xy_distance <= self.base_goal_xy_deadband
            and abs(yaw_error) <= self.base_goal_yaw_deadband
        ):
            self._base_goal_arrived = True
            self._base_goal_hold_xy = goal_xy.copy()
            return finish(np.zeros(5, dtype=float))

        stop_translation = False
        if self._base_translation_stopped:
            same_translation_goal = (
                self._base_translation_hold_xy is not None
                and float(np.linalg.norm(goal_xy - self._base_translation_hold_xy))
                < resume_distance
            )
            if same_translation_goal:
                # Finish the current gait before issuing a correction in the
                # opposite direction. Immediate reversal after an overshoot is
                # interpreted by SONIC as a new backward gait and commonly
                # produces the near-goal backward lean/limit cycle.
                tracker = getattr(self, "base_tracker", None)
                tracker_command = getattr(tracker, "command", np.zeros(5, dtype=float))
                tracker_speed = float(np.linalg.norm(np.asarray(tracker_command)[:2]))
                measured_speed = float(np.linalg.norm(base_xy_velocity))
                settled = tracker_speed <= getattr(
                    self, "sonic_motion_deadband", 0.02
                ) and measured_speed <= getattr(self, "base_settle_velocity", 0.04)
                settle_count = getattr(self, "_base_settle_count", 0)
                self._base_settle_count = settle_count + 1 if settled else 0
                stop_translation = (
                    self._base_settle_count < getattr(self, "base_settle_cycles", 5)
                    or xy_distance <= self.base_goal_xy_deadband
                )
                if not stop_translation:
                    self._base_translation_stopped = False
                    self._base_translation_hold_xy = None
                    self._base_settle_count = 0
            else:
                self._base_translation_stopped = False
                self._base_translation_hold_xy = None
                self._base_settle_count = 0
        if xy_distance <= self.base_goal_xy_deadband:
            stop_translation = True
        if xy_distance > 1.0e-6 and self.base_stop_lookahead_time > 0.0:
            direction_to_goal = pos_error_world[:2] / xy_distance
            closing_speed = float(np.dot(base_xy_velocity, direction_to_goal))
            stop_distance = (
                max(0.0, closing_speed) * self.base_stop_lookahead_time
                + self.base_stop_distance_bias
            )
            if closing_speed > 0.0 and xy_distance <= max(
                self.base_goal_xy_deadband, stop_distance
            ):
                stop_translation = True
        if stop_translation:
            self._base_translation_stopped = True
            self._base_translation_hold_xy = goal_xy.copy()

        pitch = np.arctan2(
            -robot_base_frame[2, 0], np.hypot(robot_base_frame[2, 1], robot_base_frame[2, 2])
        )
        target_pitch = np.arctan2(-base_goal[2, 0], np.hypot(base_goal[2, 1], base_goal[2, 2]))

        command = np.array(
            [
                self.base_position_kp[0] * xy_body[0]
                - self.base_velocity_kd[0] * xy_velocity_body[0],
                self.base_position_kp[1] * xy_body[1]
                - self.base_velocity_kd[1] * xy_velocity_body[1],
                self.base_orientation_kp[2] * yaw_error,
                self.base_position_kp[2] * pos_error_world[2],
                self.base_orientation_kp[1] * _wrap_to_pi(target_pitch - pitch),
            ],
            dtype=float,
        )
        # Translational and rotational arrival are independent. Continuing to
        # command x/y strides merely because yaw is still converging makes a
        # discrete gait orbit and repeatedly overshoot an otherwise reached
        # planar goal. SONIC can adjust its facing while idle.
        if stop_translation:
            command[:2] = 0.0
        return finish(command)

    def _fallback_goal_frame(self, qpos: np.ndarray, frame_name: str) -> np.ndarray:
        frames = self.robot_kinematics.forward_kinematics(qpos)
        frame_id = getattr(self.robot_cfg.Frames, frame_name)
        return frames[frame_id].copy()

    def _solve_upper_body_ik(
        self, right_base: np.ndarray, left_base: np.ndarray, qpos: np.ndarray, qvel: np.ndarray
    ) -> np.ndarray:
        if self.arm_kinematics_mode == "fixed_base":
            target, _ = self.arm_kinematics.inverse_kinematics(
                [right_base, left_base], qpos[19:36], None
            )
            target = np.asarray(target, dtype=float).reshape(-1)
            if target.shape[0] != 17:
                raise ValueError(
                    f"Fixed-base arm IK returned {target.shape[0]} entries; expected 17"
                )
            return target.copy()

        if self.arm_robot_cfg is not None:
            current_mobile_dof = np.concatenate([qpos[19:36], np.zeros(5, dtype=float)])
            target, _ = self.arm_kinematics.inverse_kinematics(
                [right_base, left_base], current_mobile_dof, None
            )
            target = np.asarray(target, dtype=float).reshape(-1)
            if target.shape[0] < 17:
                raise ValueError(
                    f"Mobile-base arm IK returned {target.shape[0]} entries; expected at least 17"
                )
            return target[:17].copy()

        target, _ = self.robot_kinematics.inverse_kinematics([right_base, left_base], qpos, qvel)
        target = np.asarray(target, dtype=float).reshape(-1)
        if target.shape[0] < 17:
            raise ValueError(f"Arm IK returned {target.shape[0]} entries; expected at least 17")
        return target[-17:].copy()

    def _clip_upper_body_target(self, target: np.ndarray) -> np.ndarray:
        target = np.asarray(target, dtype=float).reshape(17).copy()
        real_motors = getattr(self.body_robot_cfg, "RealMotors", None)
        real_limits = getattr(self.body_robot_cfg, "RealMotorPosLimit", {})
        if real_motors is None:
            return target

        for local_idx, qpos_idx in enumerate(range(19, 36)):
            dof_name = self.body_robot_cfg.DoFs(qpos_idx).name
            if dof_name not in real_motors.__members__:
                continue
            motor = real_motors.__members__[dof_name]
            if motor in real_limits:
                target[local_idx] = np.clip(target[local_idx], *real_limits[motor])
        return target

    def _filter_upper_body_target(self, target: np.ndarray) -> np.ndarray:
        target = self._clip_upper_body_target(target)
        if self.disable_upper_body_target_rate_limit:
            self._last_filtered_upper_body_target = target.copy()
            return target.copy()

        delta = self.upper_body_target_filter_gain * (
            target - self._last_filtered_upper_body_target
        )
        delta = np.clip(
            delta, -self.upper_body_target_rate_limit, self.upper_body_target_rate_limit
        )
        filtered = self._clip_upper_body_target(self._last_filtered_upper_body_target + delta)
        self._last_filtered_upper_body_target = filtered.copy()
        return filtered

    def _apply_upper_body_safety_target(self, target: np.ndarray, task_info: dict) -> np.ndarray:
        target = self._clip_upper_body_target(target)

        override = task_info.get("upper_body_target_override", None)
        if override is not None:
            override = self._clip_upper_body_target(override)
            mask = task_info.get("upper_body_target_override_mask", None)
            if mask is None:
                target = override
            else:
                mask = np.asarray(mask, dtype=bool).reshape(-1)
                if mask.shape[0] != 17:
                    raise ValueError(
                        f"upper_body_target_override_mask must have 17 entries, got {mask.shape[0]}"
                    )
                target = target.copy()
                target[mask] = override[mask]

        delta = task_info.get("upper_body_target_delta", None)
        if delta is None:
            return self._clip_upper_body_target(target)

        delta = np.asarray(delta, dtype=float).reshape(-1)
        if delta.shape[0] != 17:
            raise ValueError(f"upper_body_target_delta must have 17 entries, got {delta.shape[0]}")
        return self._clip_upper_body_target(np.asarray(target, dtype=float).reshape(17) + delta)

    def _upper_body_target(
        self, qpos: np.ndarray, qvel: np.ndarray, agent_feedback: dict, task_info: dict
    ) -> Tuple[np.ndarray, bool]:
        if "target_dof_pos" in task_info:
            target = np.asarray(task_info["target_dof_pos"], dtype=float).reshape(-1)
            if target.shape[0] >= 36:
                target = self._apply_upper_body_safety_target(
                    self._virtual_qpos(target)[19:36], task_info
                )
                self.last_upper_body_target = self._filter_upper_body_target(target)
                return self.last_upper_body_target.copy(), True

        goal_teleop = task_info.get("goal_teleop", {})
        if not task_info.get("arm_goal_enable", True):
            target = self._apply_upper_body_safety_target(self.last_upper_body_target, task_info)
            self.last_upper_body_target = self._filter_upper_body_target(target)
            return self.last_upper_body_target.copy(), True

        robot_base_frame = np.asarray(agent_feedback["robot_base_frame"], dtype=float)
        inv_base = np.linalg.inv(robot_base_frame)

        right_goal = goal_teleop.get("right", None)
        left_goal = goal_teleop.get("left", None)
        right_base = (
            inv_base @ np.asarray(right_goal, dtype=float).reshape(-1, 4, 4)[0]
            if right_goal is not None
            else self._fallback_goal_frame(qpos, "R_ee")
        )
        left_base = (
            inv_base @ np.asarray(left_goal, dtype=float).reshape(-1, 4, 4)[0]
            if left_goal is not None
            else self._fallback_goal_frame(qpos, "L_ee")
        )

        try:
            target = self._apply_upper_body_safety_target(
                self._solve_upper_body_ik(right_base, left_base, qpos, qvel),
                task_info,
            )
            self.last_upper_body_target = self._filter_upper_body_target(target)
            return self.last_upper_body_target.copy(), True
        except Exception as exc:
            print("UnitreeG1SonicPolicy inverse_kinematics error", exc)
            target = self._apply_upper_body_safety_target(self.last_upper_body_target, task_info)
            self.last_upper_body_target = self._filter_upper_body_target(target)
            return self.last_upper_body_target.copy(), False

    def _sonic_upper_body_order(self, spark_upper_body: np.ndarray) -> np.ndarray:
        return (
            np.asarray(spark_upper_body, dtype=float).reshape(17)[SPARK_TO_SONIC_UPPER_BODY].copy()
        )

    def _height_command(self, task_info: dict) -> float:
        if not self.sonic_height_control_enabled:
            return -1.0

        # A benchmark base goal is an SE(2) target (x, y, yaw).  Its frame z
        # is visualization/nominal-posture data, not a locomotion objective.
        # Height remains an independent channel that a safety composition can
        # request explicitly without coupling it to ordinary goal tracking.
        if "sonic_height" in task_info:
            height = float(
                np.clip(task_info["sonic_height"], self.sonic_height_min, self.sonic_height_max)
            )
            if abs(height - self.sonic_default_height) < self.sonic_height_deadband:
                return -1.0
            return height
        return -1.0

    def _filtered_height_command(self, height: float) -> float:
        if height < 0.0:
            self._last_sonic_height = self.sonic_default_height
            return -1.0

        target = float(np.clip(height, self.sonic_height_min, self.sonic_height_max))
        delta = self.sonic_height_filter_gain * (target - self._last_sonic_height)
        if self.sonic_height_rate_limit > 0.0:
            delta = float(
                np.clip(delta, -self.sonic_height_rate_limit, self.sonic_height_rate_limit)
            )
        self._last_sonic_height = float(
            np.clip(self._last_sonic_height + delta, self.sonic_height_min, self.sonic_height_max)
        )
        if abs(self._last_sonic_height - self.sonic_default_height) < self.sonic_height_deadband:
            return -1.0
        return self._last_sonic_height

    def _base_goal_xy_distance(
        self, robot_base_frame: np.ndarray, base_goal: np.ndarray | None
    ) -> float | None:
        if base_goal is None:
            return None
        base_goal = np.asarray(base_goal, dtype=float).reshape(-1, 4, 4)[0]
        return float(np.linalg.norm(base_goal[:2, 3] - robot_base_frame[:2, 3]))

    def _apply_near_goal_speed_profile(
        self,
        speed: float,
        locomotion_mode: str,
        xy_distance: float | None,
    ) -> tuple[int | None, float]:
        if (
            not self.base_goal_slowdown_enabled
            or xy_distance is None
            or self.base_goal_slowdown_distance <= self.base_goal_xy_deadband
            or xy_distance > self.base_goal_slowdown_distance
            or locomotion_mode == "run"
        ):
            return None, speed

        alpha = (xy_distance - self.base_goal_xy_deadband) / max(
            self.base_goal_slowdown_distance - self.base_goal_xy_deadband,
            1.0e-6,
        )
        alpha = float(np.clip(alpha, 0.0, 1.0))
        near_goal_max_speed = self.base_goal_slowdown_min_speed + alpha * (
            self.sonic_slow_walk_max_speed - self.base_goal_slowdown_min_speed
        )
        speed = float(np.clip(speed, self.base_goal_slowdown_min_speed, near_goal_max_speed))
        if locomotion_mode == "walk":
            # Near a pose goal, reduced target velocity must also select the
            # slow-walk expert. Feeding sub-walk speeds to the normal WALK
            # expert still produces full strides and causes limit cycles.
            return SONIC_LOCOMOTION_SLOW_WALK, speed
        return SONIC_LOCOMOTION_SLOW_WALK, speed

    def _planner_command_from_base_command(
        self, command: np.ndarray, agent_feedback: dict, task_info: dict
    ) -> dict[str, Any]:
        robot_base_frame = np.asarray(agent_feedback["robot_base_frame"], dtype=float)
        yaw = float(np.arctan2(robot_base_frame[1, 0], robot_base_frame[0, 0]))
        base_goal_enabled = bool(task_info.get("base_goal_enable", True))

        goal_teleop = task_info.get("goal_teleop", {})
        base_goal = goal_teleop.get("base", None)
        goal_facing_yaw = None
        xy_distance = None
        if base_goal is not None:
            base_goal = np.asarray(base_goal, dtype=float).reshape(-1, 4, 4)[0]
            goal_facing_yaw = float(np.arctan2(base_goal[1, 0], base_goal[0, 0]))
            xy_distance = self._base_goal_xy_distance(robot_base_frame, base_goal)

        denom = self.base_orientation_kp[2] if abs(self.base_orientation_kp[2]) > 1.0e-6 else 1.0
        command_facing_yaw = yaw if not base_goal_enabled else yaw + float(command[2]) / denom

        body_vx, body_vy = float(command[0]), float(command[1])
        c, s = np.cos(yaw), np.sin(yaw)
        world_xy = np.array([c * body_vx - s * body_vy, s * body_vx + c * body_vy], dtype=float)
        raw_speed = float(np.linalg.norm(world_xy))
        height = self._filtered_height_command(self._height_command(task_info))

        facing_mode = str(task_info.get("sonic_facing_mode", self.sonic_facing_mode))
        if facing_mode not in ("movement", "goal", "command"):
            raise ValueError("sonic_facing_mode must be one of: movement, goal, command")

        explicit_facing_yaw = task_info.get("locomotion_facing_yaw", None)
        if explicit_facing_yaw is not None:
            facing_yaw = float(explicit_facing_yaw)
        elif not base_goal_enabled:
            facing_yaw = 0.0 if self.base_disabled_identity_facing else yaw
        elif raw_speed >= self.sonic_motion_deadband and facing_mode == "movement":
            facing_yaw = float(np.arctan2(world_xy[1], world_xy[0]))
        elif facing_mode == "goal" and goal_facing_yaw is not None:
            facing_yaw = goal_facing_yaw
        else:
            facing_yaw = command_facing_yaw
        facing = [float(np.cos(facing_yaw)), float(np.sin(facing_yaw)), 0.0]

        safety_override = bool(task_info.get("sonic_loco_safety_override", False))
        external_tracking = bool(task_info.get("sonic_external_tracking", False))
        if (
            not external_tracking
            and raw_speed >= self.sonic_motion_deadband
            and facing_mode == "movement"
        ):
            travel_yaw = float(np.arctan2(world_xy[1], world_xy[0]))
            travel_heading_error = abs(_wrap_to_pi(travel_yaw - yaw))
            far_enough_to_align = (
                xy_distance is None or xy_distance >= self.sonic_align_min_distance
            )
            if not safety_override and far_enough_to_align:
                if self._sonic_aligning:
                    self._sonic_aligning = travel_heading_error > self.sonic_align_exit_angle
                else:
                    self._sonic_aligning = travel_heading_error > self.sonic_align_enter_angle
            else:
                self._sonic_aligning = False
        else:
            self._sonic_aligning = False

        if self._sonic_aligning:
            # Ask the planner for an in-place facing transition before enabling
            # translation. This prevents lateral/backward goals from becoming
            # a long circular walk while the gait attempts both operations.
            self._last_planner_movement = None
            self._last_planner_facing = np.asarray(facing[:2], dtype=float)
            mode = SONIC_LOCOMOTION_IDLE_SQUAT if height >= 0.0 else SONIC_LOCOMOTION_IDLE
            return {
                "mode": int(mode),
                "movement": [0.0, 0.0, 0.0],
                "facing": facing,
                "speed": -1.0,
                "height": float(height),
            }

        if raw_speed < self.sonic_motion_deadband:
            self._last_planner_movement = None
            self._last_planner_facing = None
            mode = SONIC_LOCOMOTION_IDLE_SQUAT if height >= 0.0 else SONIC_LOCOMOTION_IDLE
            return {
                "mode": int(mode),
                "movement": [0.0, 0.0, 0.0],
                "facing": facing,
                "speed": -1.0,
                "height": float(height),
            }

        speed = min(raw_speed * self.sonic_planar_speed_scale, self.sonic_max_speed)
        locomotion_mode = self._normalize_locomotion_mode(
            task_info.get("sonic_locomotion_mode", self.sonic_locomotion_mode)
        )
        near_goal_mode, speed = self._apply_near_goal_speed_profile(
            speed, locomotion_mode, xy_distance
        )
        if near_goal_mode is not None:
            mode = near_goal_mode
        elif locomotion_mode == "slow":
            mode = SONIC_LOCOMOTION_SLOW_WALK
            speed = float(np.clip(speed, self.sonic_slow_min_speed, self.sonic_slow_walk_max_speed))
        elif locomotion_mode == "walk":
            mode = SONIC_LOCOMOTION_WALK
            speed = float(np.clip(speed, self.sonic_walk_min_speed, self.sonic_walk_max_speed))
        elif locomotion_mode == "run":
            mode = SONIC_LOCOMOTION_RUN
            speed = float(np.clip(speed, self.sonic_run_min_speed, self.sonic_max_speed))
        elif speed <= self.sonic_slow_walk_max_speed:
            mode = SONIC_LOCOMOTION_SLOW_WALK
        elif speed <= self.sonic_walk_max_speed:
            mode = SONIC_LOCOMOTION_WALK
        else:
            mode = SONIC_LOCOMOTION_RUN

        movement = world_xy / max(raw_speed, 1.0e-6)
        facing_array = np.asarray(facing[:2], dtype=float)

        def hold_direction(candidate, previous, threshold):
            candidate = np.asarray(candidate, dtype=float).reshape(2)
            if previous is None:
                return candidate.copy()
            dot = float(np.clip(np.dot(candidate, previous), -1.0, 1.0))
            return previous.copy() if np.arccos(dot) < threshold else candidate.copy()

        movement = hold_direction(
            movement,
            self._last_planner_movement,
            self.sonic_safety_direction_update_angle
            if safety_override
            else self.sonic_direction_update_angle,
        )
        facing_array = hold_direction(
            facing_array,
            self._last_planner_facing,
            self.sonic_safety_facing_update_angle
            if safety_override
            else self.sonic_facing_update_angle,
        )
        self._last_planner_movement = movement.copy()
        self._last_planner_facing = facing_array.copy()
        facing = [float(facing_array[0]), float(facing_array[1]), 0.0]
        speed_quantization = (
            self.sonic_safety_speed_quantization
            if safety_override
            else self.sonic_speed_quantization
        )
        if speed_quantization > 0.0:
            speed = round(speed / speed_quantization) * speed_quantization
            speed = max(self.sonic_slow_min_speed, speed)
        return {
            "mode": int(mode),
            "movement": [float(movement[0]), float(movement[1]), 0.0],
            "facing": facing,
            "speed": float(speed),
            "height": float(height),
        }

    def _build_request(
        self,
        qpos: np.ndarray,
        qvel: np.ndarray,
        agent_feedback: dict,
        planner_command: dict[str, Any],
        upper_body_sonic: np.ndarray,
    ) -> dict[str, Any]:
        self._seq += 1
        base_quat = np.asarray(qpos[3:7], dtype=float).reshape(4)
        base_ang_vel = (
            np.asarray(qvel[3:6], dtype=float).reshape(3) if qvel.shape[0] >= 6 else np.zeros(3)
        )
        body_qpos = np.asarray(qpos, dtype=float).reshape(-1)
        body_qvel = np.asarray(qvel, dtype=float).reshape(-1)

        return {
            "protocol": self.protocol,
            "seq": int(self._seq),
            "dt": float(self.control_dt),
            "body_qpos": body_qpos.tolist(),
            "body_qvel": body_qvel.tolist(),
            "actuated_qpos": body_qpos[7:36].tolist(),
            "actuated_qvel": body_qvel[6:35].tolist()
            if body_qvel.shape[0] >= 35
            else np.zeros(29).tolist(),
            "base_quat": base_quat.tolist(),
            "base_ang_vel": base_ang_vel.tolist(),
            "planner": planner_command,
            "upper_body_position": upper_body_sonic.tolist(),
            "start": True,
            "reset": bool(self._reset_requested),
        }

    def reset(self, context=None) -> None:
        """Reset local episode state and request a server-side history reset."""
        del context
        self._seq = 0
        self.last_upper_body_target = self.default_qpos[19:36].astype(np.float64).copy()
        self._last_filtered_upper_body_target = self.last_upper_body_target.copy()
        self._last_base_command = None
        self.base_tracker.reset()
        self._last_base_xy = None
        self._last_sonic_height = self.sonic_default_height
        self._base_goal_arrived = False
        self._base_goal_hold_xy = None
        self._base_translation_stopped = False
        self._base_translation_hold_xy = None
        self._last_target_actuated_pos = self.default_qpos[7:36].astype(float).copy()
        self._last_target_qpos = self.default_qpos.copy()
        self._reset_base_goal_arrival()
        self._reset_requested = True
        self._last_planner_movement = None
        self._last_planner_facing = None
        self._sonic_aligning = False
        # Recreate the REQ socket so an episode boundary also clears any
        # outstanding request/reply state after a timeout.
        self.client.reconnect()

    def _target_from_response(
        self, response: dict[str, Any], qpos: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        if "target_actuated_pos" in response:
            target_actuated_pos = np.asarray(response["target_actuated_pos"], dtype=float).reshape(
                -1
            )
            if target_actuated_pos.shape[0] != 29:
                raise ValueError(
                    f"target_actuated_pos must have 29 entries, got {target_actuated_pos.shape[0]}"
                )
            target_qpos = qpos.copy()
            target_qpos[7:36] = target_actuated_pos
            return target_qpos, target_actuated_pos

        if "target_dof_pos" in response:
            target_qpos = np.asarray(response["target_dof_pos"], dtype=float).reshape(-1)
            if target_qpos.shape[0] == 29:
                full = qpos.copy()
                full[7:36] = target_qpos
                target_qpos = full
            elif target_qpos.shape[0] != 36:
                raise ValueError(
                    f"target_dof_pos must have 29 or 36 entries, got {target_qpos.shape[0]}"
                )
            return target_qpos, target_qpos[7:36].copy()

        raise KeyError("SONIC response must include target_actuated_pos or target_dof_pos")

    def _apply_upper_body_target_override(
        self,
        target_qpos: np.ndarray,
        target_actuated_pos: np.ndarray,
        upper_target: np.ndarray,
        override_enabled: bool | None = None,
    ) -> tuple[np.ndarray, np.ndarray]:
        if override_enabled is None:
            override_enabled = self.override_sonic_upper_body_target
        if not override_enabled:
            return target_qpos, target_actuated_pos

        target_qpos = np.asarray(target_qpos, dtype=float).reshape(-1).copy()
        target_actuated_pos = np.asarray(target_actuated_pos, dtype=float).reshape(-1).copy()
        upper_target = self._clip_upper_body_target(upper_target)

        if target_qpos.shape[0] >= 36:
            target_qpos[19:36] = upper_target
        if target_actuated_pos.shape[0] >= 29:
            target_actuated_pos[12:29] = upper_target
        return target_qpos, target_actuated_pos

    def _fallback_target(self, qpos: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        if self.fallback_mode == "raise":
            raise TimeoutError(f"No response from SONIC server at {self.client.endpoint}")
        if self.fallback_mode == "default":
            target_qpos = qpos.copy()
            target_qpos[7:36] = self.default_qpos[7:36]
            return target_qpos, target_qpos[7:36].copy()
        return self._last_target_qpos.copy(), self._last_target_actuated_pos.copy()

    def _default_body_motor_gains(self) -> tuple[np.ndarray, np.ndarray]:
        kps = []
        kds = []
        for motor in self.robot_cfg.MujocoMotors:
            if not hasattr(self.body_robot_cfg.MujocoDoFs, motor.name):
                continue
            kps.append(float(self.robot_cfg.MujocoMotorKps[motor]))
            kds.append(float(self.robot_cfg.MujocoMotorKds[motor]))
        if len(kps) != 29 or len(kds) != 29:
            return np.zeros(29, dtype=float), np.zeros(29, dtype=float)
        return np.asarray(kps, dtype=float), np.asarray(kds, dtype=float)

    def _motor_gains_from_response(
        self,
        response: dict[str, Any] | None,
        override_enabled: bool | None = None,
    ) -> tuple[np.ndarray | None, np.ndarray | None]:
        motor_kps = None
        motor_kds = None
        if response:
            if "motor_kps" in response:
                motor_kps = np.asarray(response["motor_kps"], dtype=float).reshape(-1)
            if "motor_kds" in response:
                motor_kds = np.asarray(response["motor_kds"], dtype=float).reshape(-1)

        if self.use_wbt_motor_gains:
            return WBT_MOTOR_KPS.copy(), WBT_MOTOR_KDS.copy()

        if override_enabled is None:
            override_enabled = self.override_sonic_upper_body_motor_gains

        if override_enabled:
            default_kps, default_kds = self._default_body_motor_gains()
            if motor_kps is None or motor_kps.shape[0] != 29:
                motor_kps = default_kps
            else:
                motor_kps = motor_kps.copy()
            if motor_kds is None or motor_kds.shape[0] != 29:
                motor_kds = default_kds
            else:
                motor_kds = motor_kds.copy()
            motor_kps[12:29] = self.upper_body_motor_kps
            motor_kds[12:29] = self.upper_body_motor_kds

        return motor_kps, motor_kds

    def act(self, agent_feedback: dict, task_info: dict):
        qpos_raw, _ = self._raw_qpos_qvel_from_feedback(agent_feedback)
        qpos_model = np.asarray(
            agent_feedback.get("mujoco_qpos_fbk", agent_feedback.get("qpos_fbk", qpos_raw)),
            dtype=float,
        ).reshape(-1)
        qpos, qvel = self._qpos_qvel_from_feedback(agent_feedback)
        base_goal_enabled = bool(task_info.get("base_goal_enable", True))
        command = self._base_command_from_goal(agent_feedback, task_info)
        upper_target, ik_success = self._upper_body_target(qpos, qvel, agent_feedback, task_info)
        upper_body_sonic = self._sonic_upper_body_order(upper_target)
        if not base_goal_enabled and self.base_disabled_zero_waist_upper_body_input:
            upper_body_sonic[:3] = 0.0
        planner_command = self._planner_command_from_base_command(
            command, agent_feedback, task_info
        )
        request = self._build_request(qpos, qvel, agent_feedback, planner_command, upper_body_sonic)

        response = self.client.request(request)
        if response is not None:
            self._reset_requested = False
        sonic_ok = response is not None and bool(response.get("ok", True))
        sonic_error = (
            None
            if sonic_ok
            else (None if response is None else response.get("error", "SONIC response not ok"))
        )

        try:
            if sonic_ok:
                target_qpos, target_actuated_pos = self._target_from_response(response, qpos)
            else:
                target_qpos, target_actuated_pos = self._fallback_target(qpos)
        except Exception as exc:
            sonic_ok = False
            sonic_error = str(exc)
            target_qpos, target_actuated_pos = self._fallback_target(qpos)

        upper_body_override_enabled = bool(
            self.override_sonic_upper_body_target
            or self.post_sonic_upper_body_overwrite
            # A composed safety policy may provide a per-step target without
            # globally replacing SONIC's nominal upper-body output.  Dropping
            # that target here made the safety QP correction visible in
            # diagnostics but absent from the command sent to the robot.
            or task_info.get("upper_body_target_override", None) is not None
            or task_info.get("upper_body_target_delta", None) is not None
        )
        upper_body_motor_gain_override_enabled = bool(self.override_sonic_upper_body_motor_gains)
        if (
            self.disable_upper_body_motor_gain_override_when_base_goal_disabled
            and not base_goal_enabled
        ):
            upper_body_motor_gain_override_enabled = False

        target_qpos, target_actuated_pos = self._apply_upper_body_target_override(
            target_qpos,
            target_actuated_pos,
            upper_target,
            upper_body_override_enabled,
        )

        current_actuated_pos = self._actuated_qpos_from_raw(qpos_raw)
        self._last_target_qpos = target_qpos.copy()
        self._last_target_actuated_pos = target_actuated_pos.copy()

        u_ref = self._safe_control_reference(current_actuated_pos, target_actuated_pos)
        target_model_qpos = self._target_model_qpos(qpos_model, target_qpos)
        motor_kps, motor_kds = self._motor_gains_from_response(
            response,
            upper_body_motor_gain_override_enabled,
        )

        info: Dict[str, Any] = {
            "ik_success": bool(ik_success),
            "sonic_ok": bool(sonic_ok),
            "sonic_fallback": not bool(sonic_ok),
            "sonic_error": "" if sonic_error is None else str(sonic_error),
            "sonic_seq": int(request["seq"]),
            "sonic_endpoint": self.client.endpoint,
            "sonic_planner_command": planner_command,
            "sonic_height_control_enabled": bool(self.sonic_height_control_enabled),
            "base_goal_enabled": bool(base_goal_enabled),
            "base_disabled_identity_facing": bool(self.base_disabled_identity_facing),
            "base_disabled_zero_waist_upper_body_input": bool(
                self.base_disabled_zero_waist_upper_body_input
            ),
            "sonic_mode": "sonic",
            "sonic_command": command.copy(),
            "sonic_base_tracker_phase": self.base_tracker.phase,
            "sonic_base_tracker_acceleration": self.base_tracker.acceleration.copy(),
            "target_dof_pos": target_model_qpos.copy(),
            "target_dof_pos_nominal": target_model_qpos.copy(),
            "target_actuated_pos": target_actuated_pos.copy(),
            "target_actuated_pos_nominal": target_actuated_pos.copy(),
            "target_actuated_pos_safe": target_actuated_pos.copy(),
            "target_dof_vel": np.zeros(29, dtype=float),
            "target_lower_body_pos": target_qpos[7:19].copy(),
            "sonic_output_waist_pos": target_qpos[19:22].copy(),
            "target_upper_body_pos": upper_target.copy(),
            "target_upper_body_pos_sonic_order": upper_body_sonic.copy(),
            "post_sonic_upper_body_overwrite": bool(self.post_sonic_upper_body_overwrite),
            "effective_post_sonic_upper_body_overwrite": bool(
                upper_body_override_enabled and self.post_sonic_upper_body_overwrite
            ),
            "override_sonic_upper_body_target": bool(self.override_sonic_upper_body_target),
            "effective_override_sonic_upper_body_target": bool(upper_body_override_enabled),
            "arm_kinematics_mode": self.arm_kinematics_mode,
            "upper_body_target_rate_limit_disabled": bool(
                self.disable_upper_body_target_rate_limit
            ),
            "upper_body_target_filter_gain": float(self.upper_body_target_filter_gain),
            "upper_body_gravity_compensation": bool(self.upper_body_gravity_compensation),
            "override_sonic_upper_body_motor_gains": bool(
                self.override_sonic_upper_body_motor_gains
            ),
            "effective_override_sonic_upper_body_motor_gains": bool(
                upper_body_motor_gain_override_enabled
            ),
            "disable_upper_body_motor_gain_override_when_base_goal_disabled": bool(
                self.disable_upper_body_motor_gain_override_when_base_goal_disabled
            ),
        }
        if motor_kps is not None:
            info["motor_kps"] = motor_kps
        if motor_kds is not None:
            info["motor_kds"] = motor_kds
        if response:
            if "debug" in response:
                info["sonic_debug"] = response["debug"]
        self._add_gripper_info(task_info, info)
        return u_ref, info

    def post_safe_control(
        self,
        agent_feedback: dict,
        task_info: dict,
        action_info: Dict[str, Any],
        u_ref: np.ndarray,
        u_safe: np.ndarray,
        safe_control_info: Dict[str, Any],
    ) -> Dict[str, Any]:
        if not self.apply_safe_control_targets:
            return action_info

        trigger_safe = bool(action_info.get("trigger_safe", False))
        if self.safe_control_only_when_triggered and not trigger_safe:
            return action_info

        qpos_raw, _ = self._raw_qpos_qvel_from_feedback(agent_feedback)
        current_actuated_pos = self._actuated_qpos_from_raw(qpos_raw)
        u_safe = np.asarray(u_safe, dtype=float).reshape(-1)
        if u_safe.shape[0] != current_actuated_pos[:29].shape[0]:
            return action_info

        safe_actuated_pos = current_actuated_pos[:29] + u_safe * self.safe_target_dt
        target_qpos = np.asarray(action_info["target_dof_pos"], dtype=float).copy()
        if target_qpos.shape[0] > int(np.max(self.model_actuated_qpos_indices)):
            target_qpos[self.model_actuated_qpos_indices[:29]] = safe_actuated_pos
        else:
            target_qpos[7:36] = safe_actuated_pos
        action_info["target_dof_pos"] = target_qpos
        action_info["target_actuated_pos"] = safe_actuated_pos.copy()
        action_info["target_actuated_pos_safe"] = safe_actuated_pos.copy()
        action_info["safe_control_applied_to_target"] = True
        return action_info

    def close(self) -> None:
        self.client.close()
