"""Unitree G1 articulated-hand support for MuJoCo agents."""

from __future__ import annotations

import mujoco
import numpy as np

from spark_robot.unitree_g1.actuation import UNITREE_G1_GRIPPER_SPECS


class UnitreeG1GripperMixin:
    """Keep G1 hand discovery and PD control out of the generic backend."""

    def _init_embodiment_control(self, **kwargs) -> None:
        self._unitree_gripper_control_available = False
        self._init_unitree_gripper_control_if_available(**kwargs)

    def _unitree_gravity_compensated_pd(
        self,
        target_pos,
        current_pos,
        kp,
        target_vel,
        current_vel,
        kd,
        qvel_indices,
    ):
        """Track reduced G1 joints without gravity-induced target ratcheting.

        Simulator-owned first-order dynamics deliberately integrate each
        command from measured state.  That prevents hidden target windup, but
        a small one-step position error alone cannot both cancel arm gravity
        and move toward a Cartesian target.  MuJoCo exposes the exact bias
        force for the controlled coordinates, so add it as feed-forward while
        retaining the feedback-anchored command contract.
        """

        tau = self._pd_control(
            target_pos,
            current_pos,
            kp,
            target_vel,
            current_vel,
            kd,
        )
        bias = np.asarray(self.data.qfrc_bias[qvel_indices], dtype=float)
        if bias.shape == tau.shape:
            tau = tau + bias
        return tau

    def _unitree_hand_joint_indices(self, joint_names):
        qpos_idx = []
        qvel_idx = []
        ctrl_idx = []
        for joint_name in joint_names:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint_name)
            if joint_id < 0 or actuator_id < 0:
                return None
            qpos_idx.append(int(self.model.jnt_qposadr[joint_id]))
            qvel_idx.append(int(self.model.jnt_dofadr[joint_id]))
            ctrl_idx.append(int(actuator_id))
        return (
            np.asarray(qpos_idx, dtype=int),
            np.asarray(qvel_idx, dtype=int),
            np.asarray(ctrl_idx, dtype=int),
        )

    def _init_unitree_gripper_control_if_available(self, **kwargs):
        left_spec = UNITREE_G1_GRIPPER_SPECS["left"]
        right_spec = UNITREE_G1_GRIPPER_SPECS["right"]
        left_indices = self._unitree_hand_joint_indices(left_spec.joint_names)
        right_indices = self._unitree_hand_joint_indices(right_spec.joint_names)
        self._unitree_gripper_control_sides = set()
        if left_indices is None and right_indices is None:
            return

        if left_indices is not None:
            (
                self.left_unitree_hand_qpos_index,
                self.left_unitree_hand_qvel_index,
                self.left_unitree_hand_ctrl_index,
            ) = left_indices
            self._unitree_gripper_control_sides.add("left")
        if right_indices is not None:
            (
                self.right_unitree_hand_qpos_index,
                self.right_unitree_hand_qvel_index,
                self.right_unitree_hand_ctrl_index,
            ) = right_indices
            self._unitree_gripper_control_sides.add("right")
        self.left_unitree_hand_close_pos = left_spec.target(True)
        self.left_unitree_hand_open_pos = left_spec.target(False)
        self.right_unitree_hand_close_pos = right_spec.target(True)
        self.right_unitree_hand_open_pos = right_spec.target(False)
        self.left_unitree_hand_upper_joint_limit = np.asarray(left_spec.upper_limit, dtype=float)
        self.left_unitree_hand_lower_joint_limit = np.asarray(left_spec.lower_limit, dtype=float)
        self.right_unitree_hand_upper_joint_limit = np.asarray(right_spec.upper_limit, dtype=float)
        self.right_unitree_hand_lower_joint_limit = np.asarray(right_spec.lower_limit, dtype=float)

        hand_kp = float(kwargs.get("hand_kp", left_spec.kp))
        hand_kd = float(kwargs.get("hand_kd", left_spec.kd))
        self.left_unitree_hand_kp = hand_kp * np.ones(7, dtype=float)
        self.left_unitree_hand_kd = hand_kd * np.ones(7, dtype=float)
        self.right_unitree_hand_kp = hand_kp * np.ones(7, dtype=float)
        self.right_unitree_hand_kd = hand_kd * np.ones(7, dtype=float)
        self._unitree_gripper_control_available = True

    def _unitree_gripper_target(self, side: str, action_info: dict):
        control_key = f"{side}_gripper_control"
        if control_key in action_info:
            target = np.asarray(action_info[control_key], dtype=float).reshape(-1)
            if target.size != 7:
                raise ValueError(f"{control_key} must have 7 entries.")
            open_pos = getattr(self, f"{side}_unitree_hand_open_pos")
            lower = getattr(self, f"{side}_unitree_hand_lower_joint_limit")
            upper = getattr(self, f"{side}_unitree_hand_upper_joint_limit")
            goal = bool(np.linalg.norm(target - open_pos) > 1e-6)
            self._set_resolved_gripper_goal(side, goal)
            return np.clip(target, lower, upper)

        goal = self._resolve_binary_gripper_goal(side, action_info)
        target = getattr(
            self,
            f"{side}_unitree_hand_{'close' if goal else 'open'}_pos",
        )
        lower = getattr(self, f"{side}_unitree_hand_lower_joint_limit")
        upper = getattr(self, f"{side}_unitree_hand_upper_joint_limit")
        return np.clip(target, lower, upper)

    def _apply_unitree_gripper_pd(self, target_pos, qpos_idx, qvel_idx, ctrl_idx, kp, kd):
        tau = self._pd_control(
            target_pos,
            self.data.qpos[qpos_idx],
            kp,
            np.zeros_like(kd),
            self.data.qvel[qvel_idx],
            kd,
        )
        self.data.ctrl[ctrl_idx] = tau

    def _apply_unitree_gripper_control(self, action_info: dict):
        if not (self.enable_hand_control and self._unitree_gripper_control_available):
            return
        action_info = {} if action_info is None else action_info
        for side in ("left", "right"):
            if side not in self._unitree_gripper_control_sides:
                continue
            self._apply_unitree_gripper_pd(
                self._unitree_gripper_target(side, action_info),
                getattr(self, f"{side}_unitree_hand_qpos_index"),
                getattr(self, f"{side}_unitree_hand_qvel_index"),
                getattr(self, f"{side}_unitree_hand_ctrl_index"),
                getattr(self, f"{side}_unitree_hand_kp"),
                getattr(self, f"{side}_unitree_hand_kd"),
            )

    def _reset_unitree_gripper_control(self):
        if not self._unitree_gripper_control_available:
            return
        self._keyboard_gripper_goal_override.clear()
        self.left_gripper_goal = False
        self.right_gripper_goal = False
        self.left_gripper_debug_state = False
        self.right_gripper_debug_state = False
        for side in ("left", "right"):
            if side not in self._unitree_gripper_control_sides:
                continue
            self.data.qpos[getattr(self, f"{side}_unitree_hand_qpos_index")] = getattr(
                self, f"{side}_unitree_hand_open_pos"
            )
            self.data.qvel[getattr(self, f"{side}_unitree_hand_qvel_index")] = 0.0

    def _add_unitree_gripper_feedback(self, ret: dict) -> dict:
        ret["left_gripper_debug_state"] = bool(self.left_gripper_debug_state)
        ret["right_gripper_debug_state"] = bool(self.right_gripper_debug_state)
        ret["left_gripper_goal"] = bool(self.left_gripper_goal)
        ret["right_gripper_goal"] = bool(self.right_gripper_goal)
        return ret
