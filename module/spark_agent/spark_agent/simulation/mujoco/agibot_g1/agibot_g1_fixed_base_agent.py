from typing import Optional, Sequence
import warnings

import mujoco
import numpy as np

from spark_agent.simulation.mujoco.mujoco_agent import MujocoAgent
from spark_robot.agibot_g1.config.joint_defaults import (
    AGIBOT_G1_LOCKED_JOINT_DEFAULTS,
)
from spark_robot import RobotConfig


_GRIPPER_JOINT_ORDER = (
    "narrow1_joint",
    "narrow3_joint",
    "narrow4_joint",
    "narrow2_joint",
    "wide1_joint",
    "wide3_joint",
    "wide4_joint",
    "wide2_joint",
)

# The A2D Omnipicker is closed at q=0. Its official coupled linkage opens from
# one negative inner master angle q: inner=(q, .1q, .25q, .65q) and
# outer=(-q, .1q, -.25q, -.45q). The final coefficients account for the old
# open-chain narrow2/wide2 representation used by SPARK's MuJoCo asset.
_GRIPPER_CLOSED_QPOS = np.zeros(len(_GRIPPER_JOINT_ORDER), dtype=np.float64)
_GRIPPER_OPEN_MASTER_ANGLE = -0.60
_GRIPPER_OPEN_QPOS = np.array(
    [
        _GRIPPER_OPEN_MASTER_ANGLE,
        0.10 * _GRIPPER_OPEN_MASTER_ANGLE,
        0.25 * _GRIPPER_OPEN_MASTER_ANGLE,
        0.65 * _GRIPPER_OPEN_MASTER_ANGLE,
        -_GRIPPER_OPEN_MASTER_ANGLE,
        0.10 * _GRIPPER_OPEN_MASTER_ANGLE,
        -0.25 * _GRIPPER_OPEN_MASTER_ANGLE,
        -0.45 * _GRIPPER_OPEN_MASTER_ANGLE,
    ],
    dtype=np.float64,
)
_GRIPPER_COMMAND_THRESHOLD = 0.5
_GRIPPER_TARGET_SPEED = 1.5


class _AgiBotG1AgentBase(MujocoAgent):
    def __init__(self, robot_cfg: RobotConfig, **kwargs) -> None:
        dynamics_backend = str(kwargs.get("dynamics_backend", "simulator")).lower()
        use_sim_dynamics = kwargs.get("use_sim_dynamics")
        if use_sim_dynamics is not None:
            dynamics_backend = "simulator" if use_sim_dynamics else "model"
        variant = str(getattr(robot_cfg, "dynamics_variant", ""))
        if dynamics_backend == "simulator" and variant not in {
            "single_integrator",
            "double_integrator",
        }:
            warnings.warn(
                f"{type(robot_cfg).__name__} selects the custom {variant!r} analytical "
                "dynamics with simulator-owned execution. The shared AgiBot mobile-base "
                "agent will execute its physical planar-drive contract, not reproduce the "
                "custom analytical equations exactly. Use --dynamics-backend model when "
                "the analytical model must own state evolution.",
                UserWarning,
                stacklevel=2,
            )
        super().__init__(robot_cfg, **kwargs)
        self.sim_use_bias_compensation = bool(kwargs.get("sim_use_bias_compensation", True))
        self.sim_spring_damping_scale = float(kwargs.get("sim_spring_damping_scale", 2.0))
        self.arm_actuator_indices = np.array(
            [int(mj_motor) for mj_motor in self.robot_cfg.MujocoMotors],
            dtype=np.int32,
        )
        self.arm_control_indices = np.array(
            [
                int(self.robot_cfg.MujocoMotor_to_Control[mj_motor])
                for mj_motor in self.robot_cfg.MujocoMotors
            ],
            dtype=np.int32,
        )
        self.left_gripper_actuator_ids = self._resolve_gripper_actuator_ids("left")
        self.right_gripper_actuator_ids = self._resolve_gripper_actuator_ids("right")
        self._configure_uncontrolled_actuator_holds(
            controlled_actuator_ids=self.arm_actuator_indices,
            excluded_actuator_ids=[
                *[item for item in self.left_gripper_actuator_ids if item is not None],
                *[item for item in self.right_gripper_actuator_ids if item is not None],
            ],
            joint_names=getattr(self.robot_cfg, "joint_to_lock", None),
        )
        self._configure_controlled_joint_springs()

    def _configure_controlled_joint_springs(self) -> None:
        """Use MuJoCo's implicit joint springs for stable position tracking."""
        self._controlled_joint_springs = []
        for motor_index, mj_motor in enumerate(self.robot_cfg.MujocoMotors):
            actuator_id = int(mj_motor)
            joint_id = self._actuator_joint_id(actuator_id)
            if joint_id is None:
                continue
            qpos_adr = int(self.model.jnt_qposadr[joint_id])
            dof_adr = int(self.model.jnt_dofadr[joint_id])
            mj_dof = getattr(self.robot_cfg.MujocoDoFs, mj_motor.name)
            state_idx = int(self.robot_cfg.MujocoDoF_to_DoF[mj_dof])
            stiffness = float(self.kps[motor_index])
            damping = float(self.kds[motor_index])
            if stiffness > 0.0:
                self.model.jnt_stiffness[joint_id] = max(
                    float(self.model.jnt_stiffness[joint_id]),
                    stiffness,
                )
            if damping > 0.0:
                self.model.dof_damping[dof_adr] = max(
                    float(self.model.dof_damping[dof_adr]),
                    damping * self.sim_spring_damping_scale,
                )
            self._controlled_joint_springs.append(
                {
                    "actuator_id": actuator_id,
                    "joint_id": joint_id,
                    "qpos_adr": qpos_adr,
                    "dof_adr": dof_adr,
                    "state_idx": state_idx,
                    "stiffness": stiffness,
                    "damping": damping,
                }
            )

    def _update_controlled_joint_spring_targets(self, target_pos: np.ndarray) -> None:
        target_pos = self._clip_target_pos_to_mujoco_limits(target_pos)
        for entry in self._controlled_joint_springs:
            if entry["stiffness"] <= 0.0:
                continue
            self.model.qpos_spring[entry["qpos_adr"]] = target_pos[entry["state_idx"]]

    def reset(self, agent_reset_info=None) -> None:
        super().reset()
        agent_reset_info = agent_reset_info or {}

        if "reset_dof_pos" in agent_reset_info:
            self.default_dof_pos = np.asarray(agent_reset_info["reset_dof_pos"], dtype=np.float64)
        else:
            self.default_dof_pos = np.array(
                [self.robot_cfg.DefaultDoFVal[dof] for dof in self.robot_cfg.DoFs],
                dtype=np.float64,
            )
        default_dof_vel = np.asarray(
            agent_reset_info.get("reset_dof_vel", np.zeros(self.num_dof)),
            dtype=np.float64,
        ).reshape(self.num_dof)

        self.left_gripper_goal = False
        self.right_gripper_goal = False
        self.dof_pos_cmd = self.default_dof_pos.copy()
        self.dof_vel_cmd = default_dof_vel.copy()
        self.dof_acc_cmd = np.zeros(self.num_dof, dtype=np.float64)
        self._set_dof_pos(self.default_dof_pos)
        self._update_controlled_joint_spring_targets(self.default_dof_pos)
        self._set_locked_joint_defaults()
        self._set_gripper_qpos("left", _GRIPPER_OPEN_QPOS)
        self._set_gripper_qpos("right", _GRIPPER_OPEN_QPOS)
        self._refresh_uncontrolled_actuator_hold_targets()
        self._enforce_uncontrolled_actuator_holds()

        if len(self.data.ctrl) > 0:
            ctrl = np.zeros(len(self.data.ctrl), dtype=np.float64)
            self._apply_uncontrolled_actuator_holds(ctrl)
            self._apply_gripper_ctrl(ctrl, {})
            self.data.ctrl[:] = ctrl

        self._mujoco_step()

    def _set_dof_pos(self, dof_pos: np.ndarray) -> None:
        for mj_dof_idx in self.robot_cfg.MujocoDoFs:
            dof_idx = self.robot_cfg.MujocoDoF_to_DoF[mj_dof_idx]
            self.data.qpos[int(mj_dof_idx)] = dof_pos[dof_idx]

        self.data.qvel[:] = 0
        self.data.qacc[:] = 0
        self.data.qfrc_applied[:] = 0
        self.data.xfrc_applied[:, :] = 0

    def _set_locked_joint_defaults(self) -> None:
        controlled_joint_ids = set()
        for mj_dof_idx in self.robot_cfg.MujocoDoFs:
            if int(mj_dof_idx) < self.model.nv:
                controlled_joint_ids.add(int(self.model.dof_jntid[int(mj_dof_idx)]))
        for joint_name in getattr(self.robot_cfg, "joint_to_lock", ()):
            if joint_name not in AGIBOT_G1_LOCKED_JOINT_DEFAULTS:
                continue
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, str(joint_name))
            if joint_id == -1 or joint_id in controlled_joint_ids:
                continue
            qpos_adr = int(self.model.jnt_qposadr[joint_id])
            self.data.qpos[qpos_adr] = float(AGIBOT_G1_LOCKED_JOINT_DEFAULTS[joint_name])

    def _resolve_gripper_joint_name(self, side: str, joint_suffix: str):
        joint_name = f"{side}_{joint_suffix}"
        joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        return joint_name if joint_id != -1 else None

    def _resolve_gripper_actuator_ids(self, side: str):
        actuator_ids = []
        for joint_suffix in _GRIPPER_JOINT_ORDER:
            actuator_name = f"{side}_{joint_suffix}_act"
            actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
            actuator_ids.append(actuator_id if actuator_id != -1 else None)
        return actuator_ids

    def _coerce_gripper_target(self, command) -> np.ndarray:
        if np.isscalar(command):
            return (
                _GRIPPER_CLOSED_QPOS
                if float(command) >= _GRIPPER_COMMAND_THRESHOLD
                else _GRIPPER_OPEN_QPOS
            )

        target = np.asarray(command, dtype=np.float64).reshape(-1)
        if target.shape[0] == 1:
            return (
                _GRIPPER_CLOSED_QPOS
                if float(target[0]) >= _GRIPPER_COMMAND_THRESHOLD
                else _GRIPPER_OPEN_QPOS
            )
        if target.shape[0] != len(_GRIPPER_JOINT_ORDER):
            raise ValueError(
                "AgiBot gripper command must be a scalar target angle or an 8-element joint target vector."
            )
        return target

    def _coerce_gripper_goal(self, goal_value) -> bool:
        if isinstance(goal_value, (bool, np.bool_)):
            return bool(goal_value)

        target = np.asarray(goal_value, dtype=np.float64).reshape(-1)
        if target.size == 0:
            return False
        if target.size == len(_GRIPPER_JOINT_ORDER):
            return np.linalg.norm(target - _GRIPPER_CLOSED_QPOS) <= np.linalg.norm(
                target - _GRIPPER_OPEN_QPOS
            )
        return float(target[0]) >= _GRIPPER_COMMAND_THRESHOLD

    def _resolve_gripper_target(self, side: str, action_info: dict) -> np.ndarray:
        control_key = f"{side}_gripper_control"
        if control_key in action_info:
            target = self._coerce_gripper_target(action_info[control_key])
            self._set_resolved_gripper_goal(
                side,
                np.linalg.norm(target - _GRIPPER_CLOSED_QPOS)
                <= np.linalg.norm(target - _GRIPPER_OPEN_QPOS),
            )
            return target
        if "gripper_control" in action_info:
            target = self._coerce_gripper_target(action_info["gripper_control"])
            self._set_resolved_gripper_goal(
                side,
                np.linalg.norm(target - _GRIPPER_CLOSED_QPOS)
                <= np.linalg.norm(target - _GRIPPER_OPEN_QPOS),
            )
            return target

        goal = self._resolve_binary_gripper_goal(
            side,
            action_info,
            coerce=self._coerce_gripper_goal,
        )
        return _GRIPPER_CLOSED_QPOS if goal else _GRIPPER_OPEN_QPOS

    def _set_gripper_qpos(self, side: str, qpos_target: Sequence[float]) -> None:
        for joint_suffix, joint_target in zip(_GRIPPER_JOINT_ORDER, qpos_target):
            joint_name = self._resolve_gripper_joint_name(side, joint_suffix)
            if joint_name is None:
                continue
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            qpos_adr = self.model.jnt_qposadr[joint_id]
            self.data.qpos[qpos_adr] = joint_target

    def _apply_gripper_ctrl(self, ctrl: np.ndarray, action_info: dict) -> None:
        maximum_step = _GRIPPER_TARGET_SPEED * self.dt * self.control_decimation

        def apply_target(actuator_ids, target):
            for actuator_id, requested in zip(actuator_ids, target):
                if actuator_id is None:
                    continue
                joint_id = int(self.model.actuator_trnid[actuator_id, 0])
                qpos_index = int(self.model.jnt_qposadr[joint_id])
                current = float(self.data.qpos[qpos_index])
                # The light open-chain fingers have little passive damping.
                # Applying the complete 0.6 rad binary transition in one
                # control frame makes them overshoot through the q=0 closed
                # pose and settle against the gripper body. Slew only the
                # actuator target; PhysX/MuJoCo contact remains authoritative.
                ctrl[actuator_id] = np.clip(
                    requested,
                    current - maximum_step,
                    current + maximum_step,
                )

        right_target = self._resolve_gripper_target("right", action_info)
        apply_target(self.right_gripper_actuator_ids, right_target)

        left_target = self._resolve_gripper_target("left", action_info)
        apply_target(self.left_gripper_actuator_ids, left_target)

    def _current_arm_state(self):
        q = np.array(
            [self.data.qpos[int(mj_dof)] for mj_dof in self.robot_cfg.MujocoDoFs], dtype=np.float64
        )
        dq = np.array(
            [self.data.qvel[int(mj_dof)] for mj_dof in self.robot_cfg.MujocoDoFs], dtype=np.float64
        )
        return q, dq

    def _apply_arm_ctrl(
        self,
        ctrl: np.ndarray,
        target_pos: np.ndarray,
        target_vel: np.ndarray,
        kd_scale: float,
    ) -> None:
        target_pos = self._clip_target_pos_to_mujoco_limits(target_pos)
        self._update_controlled_joint_spring_targets(target_pos)
        for entry in self._controlled_joint_springs:
            actuator_id = entry["actuator_id"]
            state_idx = entry["state_idx"]
            velocity_feedforward = entry["damping"] * kd_scale * target_vel[state_idx]
            bias_control = (
                self._actuator_bias_control(actuator_id) if self.sim_use_bias_compensation else 0.0
            )
            ctrl[actuator_id] = self._clip_actuator_control(
                actuator_id,
                bias_control + velocity_feedforward,
            )

    def _apply_arm_bias_ctrl(self, ctrl: np.ndarray) -> None:
        for actuator_id in self.arm_actuator_indices:
            actuator_id = int(actuator_id)
            ctrl[actuator_id] = self._clip_actuator_control(
                actuator_id,
                self._actuator_bias_control(actuator_id),
            )

    def _set_controlled_dof_state(
        self, dof_pos: np.ndarray, dof_vel: Optional[np.ndarray] = None
    ) -> None:
        dof_pos = self._clip_target_pos_to_mujoco_limits(dof_pos)
        for mj_dof_idx in self.robot_cfg.MujocoDoFs:
            dof_idx = int(self.robot_cfg.MujocoDoF_to_DoF[mj_dof_idx])
            mj_dof_idx = int(mj_dof_idx)
            self.data.qpos[mj_dof_idx] = dof_pos[dof_idx]
            self.data.qvel[mj_dof_idx] = 0.0 if dof_vel is None else dof_vel[dof_idx]

    def _send_control_modeled_dynamics(self, command, **kwargs):
        x = self._compose_cmd_state()
        x = self._advance_configured_dynamics(x, command, **kwargs)

        self.dof_pos_cmd = self.robot_cfg.decompose_state_to_dof_pos(x)
        self.dof_vel_cmd = self.robot_cfg.decompose_state_to_dof_vel(x)

        action_info = kwargs.get("action_info", {})
        self._set_dof_pos(self.dof_pos_cmd)
        self._set_gripper_qpos("right", self._resolve_gripper_target("right", action_info))
        self._set_gripper_qpos("left", self._resolve_gripper_target("left", action_info))

        if len(self.data.ctrl) > 0:
            ctrl = np.zeros(len(self.data.ctrl), dtype=np.float64)
            self._apply_gripper_ctrl(ctrl, action_info)
            self.data.ctrl[:] = ctrl


class AgiBotG1FixedBaseAgent(_AgiBotG1AgentBase):
    def __init__(self, robot_cfg: RobotConfig, **kwargs) -> None:
        super().__init__(robot_cfg, **kwargs)

    def get_feedback(self) -> None:
        ret = {}

        global_position = self.data.body("robot").xpos.copy()
        global_orientation = self.data.body("robot").xmat.copy().reshape(3, 3)

        robot_base_frame = np.eye(4)
        robot_base_frame[:3, :3] = global_orientation
        robot_base_frame[:3, 3] = global_position
        ret["robot_base_frame"] = robot_base_frame

        self.dof_pos_fbk = np.zeros(self.num_dof) if self.dof_pos_fbk is None else self.dof_pos_fbk
        self.dof_vel_fbk = np.zeros(self.num_dof) if self.dof_vel_fbk is None else self.dof_vel_fbk

        for dof in self.robot_cfg.DoFs:
            mj_dof = self.robot_cfg.DoF_to_MujocoDoF[dof]
            self.dof_pos_fbk[dof] = self.data.qpos[int(mj_dof)]
            self.dof_vel_fbk[dof] = self.data.qvel[int(mj_dof)]

        self.dof_pos_cmd = self.dof_pos_fbk.copy() if self.dof_pos_cmd is None else self.dof_pos_cmd
        self.dof_vel_cmd = self.dof_vel_fbk.copy() if self.dof_vel_cmd is None else self.dof_vel_cmd
        self.dof_acc_cmd = np.zeros(self.num_dof) if self.dof_acc_cmd is None else self.dof_acc_cmd

        if not self.use_sim_dynamics:
            self.dof_pos_fbk = self.dof_pos_cmd.copy()
            self.dof_vel_fbk = self.dof_vel_cmd.copy()

        ret["dof_pos_fbk"] = self.dof_pos_fbk.copy()
        ret["dof_vel_fbk"] = self.dof_vel_fbk.copy()
        ret["dof_pos_cmd"] = self.dof_pos_cmd.copy()
        ret["dof_vel_cmd"] = self.dof_vel_cmd.copy()
        ret["dof_acc_cmd"] = self.dof_acc_cmd.copy()
        ret["state"] = self._compose_fbk_state()

        ret["obstacle_debug_frame"] = self.obstacle_debug_frame
        ret["obstacle_debug_geom"] = self.obstacle_debug_geom
        ret["obstacle_debug_velocity"] = (
            np.array(self.obstacle_debug_velocity)
            if len(self.obstacle_debug_velocity) > 0
            else np.empty((0, 6))
        )

        ret["robot_goal_left_offset"] = self.left_goal_debug_frame
        ret["robot_goal_right_offset"] = self.right_goal_debug_frame
        ret["robot_goal_base_offset"] = self.base_goal_debug_frame
        ret["left_gripper_goal"] = bool(self.left_gripper_goal)
        ret["right_gripper_goal"] = bool(self.right_gripper_goal)
        ret["left_gripper_debug_state"] = bool(self.left_gripper_debug_state)
        ret["right_gripper_debug_state"] = bool(self.right_gripper_debug_state)
        return ret

    def _send_control_sim_dynamics(self, command, **kwargs):
        x = self._compose_simulator_dynamics_state()
        x = self._advance_configured_dynamics(x, command, **kwargs)

        self.dof_pos_cmd = self.robot_cfg.decompose_state_to_dof_pos(x)
        self.dof_vel_cmd = self.robot_cfg.decompose_state_to_dof_vel(x)

        target_pos = self.dof_pos_cmd.copy()
        target_vel = self.dof_vel_cmd.copy()

        action_info = kwargs.get("action_info", {})
        for _ in range(self.control_decimation):
            ctrl = np.zeros(len(self.data.ctrl), dtype=np.float64)
            if self.stabilize_sim_dynamics_joint_positions:
                self._set_controlled_dof_state(target_pos, target_vel)
                self._enforce_uncontrolled_actuator_holds()
                self._apply_arm_bias_ctrl(ctrl)
            else:
                self._apply_arm_ctrl(ctrl, target_pos, target_vel, kd_scale=2.0)
            self._apply_uncontrolled_actuator_holds(ctrl)
            self._apply_gripper_ctrl(ctrl, action_info)
            self.data.ctrl[:] = ctrl
            self.counter += 1
            mujoco.mj_step(self.model, self.data)
            if self.stabilize_sim_dynamics_joint_positions:
                self._set_controlled_dof_state(target_pos, target_vel)
            self._enforce_uncontrolled_actuator_holds()
