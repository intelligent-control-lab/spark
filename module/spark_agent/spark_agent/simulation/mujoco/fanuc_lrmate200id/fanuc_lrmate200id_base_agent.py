"""Shared MuJoCo execution for FANUC LR Mate 200iD embodiments."""

from typing import List, Optional

import mujoco
import numpy as np

from spark_agent.simulation.mujoco.mujoco_agent import MujocoAgent
from spark_robot import RobotConfig


_GRIPPER_JOINT_ORDER = (
    "finger_A_joint_0",
    "finger_A_joint_1",
    "finger_A_joint_2",
    "finger_A_joint_3",
    "finger_B_joint_0",
    "finger_B_joint_1",
    "finger_B_joint_2",
    "finger_B_joint_3",
    "finger_C_joint_0",
    "finger_C_joint_1",
    "finger_C_joint_2",
    "finger_C_joint_3",
)


class _FanucLRMate200iDAgentBase(MujocoAgent):
    """Shared MuJoCo implementation for FANUC LR Mate 200iD variants."""

    ARM_DOF_PER_ARM = 6

    def __init__(self, robot_cfg: RobotConfig, **kwargs) -> None:
        super().__init__(robot_cfg, **kwargs)
        self.sim_use_bias_compensation = bool(kwargs.get("sim_use_bias_compensation", True))
        self.sim_force_limit_margin = float(kwargs.get("sim_force_limit_margin", 0.7))
        if not 0.0 < self.sim_force_limit_margin <= 1.0:
            raise ValueError("sim_force_limit_margin must be in (0, 1]")
        self.arm_actuator_indices = np.asarray(
            [int(motor) for motor in self.robot_cfg.MujocoMotors], dtype=np.int32
        )
        self.arm_control_indices = np.asarray(
            [
                int(self.robot_cfg.MujocoMotor_to_Control[motor])
                for motor in self.robot_cfg.MujocoMotors
            ],
            dtype=np.int32,
        )
        self.arm_dof_indices = np.asarray(
            [
                int(self.robot_cfg.MujocoDoF_to_DoF[getattr(self.robot_cfg.MujocoDoFs, motor.name)])
                for motor in self.robot_cfg.MujocoMotors
            ],
            dtype=np.int32,
        )
        self.right_gripper_actuator_ids = self._resolve_gripper_actuator_ids("right")
        self.left_gripper_actuator_ids = self._resolve_gripper_actuator_ids("left")

    @property
    def num_arms(self) -> int:
        return max(1, self.num_dof // self.ARM_DOF_PER_ARM)

    def reset(self, agent_reset_info=None) -> None:
        super().reset()
        reset_info = dict(agent_reset_info or {})
        self.default_dof_pos = np.asarray(
            reset_info.get(
                "reset_dof_pos",
                [self.robot_cfg.DefaultDoFVal[dof] for dof in self.robot_cfg.DoFs],
            ),
            dtype=float,
        )
        self.left_gripper_goal = False
        self.right_gripper_goal = False
        self.left_gripper_debug_state = False
        self.right_gripper_debug_state = False
        self._set_dof_pos(self.default_dof_pos)
        for object_name, object_pos in reset_info.get("reset_object_pos", {}).items():
            self._set_object_pos(object_name, object_pos)
        self._mujoco_step()

    def get_feedback(self) -> dict:
        feedback = {"object_pos": {}}
        for body_id in range(1, self.model.nbody):
            name = (
                mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body_id)
                or f"body_{body_id}"
            )
            feedback["object_pos"][name] = np.hstack(
                (self.data.xpos[body_id].copy(), self.data.xquat[body_id].copy())
            )
        robot_base_frame = np.eye(4)
        robot_base_frame[:3, :3] = self.data.body("robot").xmat.copy().reshape(3, 3)
        robot_base_frame[:3, 3] = self.data.body("robot").xpos.copy()
        feedback["robot_base_frame"] = robot_base_frame

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

        feedback.update(
            dof_pos_fbk=self.dof_pos_fbk.copy(),
            dof_vel_fbk=self.dof_vel_fbk.copy(),
            dof_pos_cmd=self.dof_pos_cmd.copy(),
            dof_vel_cmd=self.dof_vel_cmd.copy(),
            dof_acc_cmd=self.dof_acc_cmd.copy(),
            state=self._compose_fbk_state(),
            obstacle_debug_frame=self.obstacle_debug_frame,
            obstacle_debug_geom=self.obstacle_debug_geom,
            obstacle_debug_velocity=(
                np.asarray(self.obstacle_debug_velocity)
                if len(self.obstacle_debug_velocity) > 0
                else np.empty((0, 6))
            ),
            robot_goal_left_offset=self.left_goal_debug_frame,
            robot_goal_right_offset=self.right_goal_debug_frame,
            robot_goal_base_offset=self.base_goal_debug_frame,
            left_gripper_goal=bool(self.left_gripper_goal),
            right_gripper_goal=bool(self.right_gripper_goal),
            left_gripper_debug_state=bool(self.left_gripper_debug_state),
            right_gripper_debug_state=bool(self.right_gripper_debug_state),
        )
        return feedback

    def _set_dof_pos(self, dof_pos: np.ndarray) -> None:
        for mj_dof_idx in self.robot_cfg.MujocoDoFs:
            dof_idx = self.robot_cfg.MujocoDoF_to_DoF[mj_dof_idx]
            self.data.qpos[int(mj_dof_idx)] = dof_pos[dof_idx]
        self.data.qvel[:] = 0.0
        self.data.qacc[:] = 0.0
        self.data.qfrc_applied[:] = 0.0
        self.data.xfrc_applied[:, :] = 0.0

    def _resolve_joint_name(self, side: str, joint_name: str) -> Optional[str]:
        for candidate in (f"{side}/{joint_name}", joint_name):
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, candidate)
            if joint_id != -1:
                return candidate
        return None

    def _resolve_actuator_name(self, side: str, actuator_name: str) -> Optional[str]:
        for candidate in (f"{side}/{actuator_name}", actuator_name):
            actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, candidate)
            if actuator_id != -1:
                return candidate
        return None

    def _resolve_gripper_actuator_ids(self, side: str):
        actuator_ids = []
        for actuator_name in _GRIPPER_JOINT_ORDER:
            resolved_name = self._resolve_actuator_name(side, actuator_name)
            if resolved_name is None:
                actuator_ids.append(None)
                continue
            actuator_ids.append(
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, resolved_name)
            )
        return actuator_ids

    def _default_gripper_control(self, closed: bool) -> List[float]:
        return [255, 255, 255, 137] if closed else [0, 0, 0, 137]

    def _resolve_gripper_action(self, side: str, action_info: dict):
        control_key = f"{side}_gripper_control"
        icf_key = f"{side}_gripper_icf"

        if control_key in action_info:
            command = action_info[control_key]
            self._set_resolved_gripper_goal(side, command[0] > 0)
            return command, bool(action_info.get(icf_key, False))
        if "gripper_control" in action_info:
            command = action_info["gripper_control"]
            self._set_resolved_gripper_goal(side, command[0] > 0)
            return command, bool(action_info.get("gripper_icf", False))

        goal = self._resolve_binary_gripper_goal(side, action_info)
        return self._default_gripper_control(goal), False

    def _gripper_target(self, side: str, action_info: dict) -> np.ndarray:
        command, icf = self._resolve_gripper_action(side, action_info)
        return self.controlGripperRegister(
            POA=command[0],
            POB=command[1],
            POC=command[2],
            POS=command[3],
            ICF=icf,
        )

    def _apply_arm_ctrl(
        self,
        ctrl: np.ndarray,
        target_pos: np.ndarray,
        target_vel: np.ndarray | None = None,
    ) -> None:
        """Realize the configured target through force-limited affine servos."""

        target_pos = self._clip_target_pos_to_mujoco_limits(target_pos)
        target_vel = (
            np.zeros(self.num_dof, dtype=float)
            if target_vel is None
            else np.asarray(target_vel, dtype=float)
        )
        for motor_index, actuator_id in enumerate(self.arm_actuator_indices):
            actuator_id = int(actuator_id)
            dof_index = int(self.arm_dof_indices[motor_index])
            target = float(target_pos[dof_index])
            gain = float(self.model.actuator_gainprm[actuator_id, 0])
            damping = max(0.0, -float(self.model.actuator_biasprm[actuator_id, 2]))
            if abs(gain) > 1.0e-9:
                target += damping * float(target_vel[dof_index]) / gain
                if self.sim_use_bias_compensation:
                    target += self._actuator_bias_control(actuator_id) / gain
            target = self._limit_affine_actuator_force(actuator_id, target)
            ctrl[actuator_id] = self._clip_actuator_control(actuator_id, target)

    def _limit_affine_actuator_force(self, actuator_id: int, target: float) -> float:
        if not bool(self.model.actuator_forcelimited[actuator_id]):
            return float(target)
        gain = float(self.model.actuator_gainprm[actuator_id, 0])
        if abs(gain) <= 1.0e-9:
            return float(target)
        bias = float(
            self.model.actuator_biasprm[actuator_id, 0]
            + self.model.actuator_biasprm[actuator_id, 1] * self.data.actuator_length[actuator_id]
            + self.model.actuator_biasprm[actuator_id, 2] * self.data.actuator_velocity[actuator_id]
        )
        low, high = self.sim_force_limit_margin * self.model.actuator_forcerange[actuator_id]
        target_at_low = (float(low) - bias) / gain
        target_at_high = (float(high) - bias) / gain
        return float(
            np.clip(target, min(target_at_low, target_at_high), max(target_at_low, target_at_high))
        )

    def _simulator_target_velocity(self, command: np.ndarray) -> np.ndarray:
        if self.dynamic_order != 1:
            return self.dof_vel_cmd.copy()
        command = np.asarray(command, dtype=float)
        target_vel = np.zeros(self.num_dof, dtype=float)
        for motor_index, dof_index in enumerate(self.arm_dof_indices):
            control_index = int(self.arm_control_indices[motor_index])
            target_vel[int(dof_index)] = command[control_index]
        return target_vel

    def _apply_gripper_ctrl(self, ctrl: np.ndarray, action_info: dict) -> None:
        right_target = self._gripper_target("right", action_info)
        for actuator_id, actuator_target in zip(self.right_gripper_actuator_ids, right_target):
            if actuator_id is not None:
                ctrl[actuator_id] = actuator_target

        if self.num_arms > 1:
            left_target = self._gripper_target("left", action_info)
            for actuator_id, actuator_target in zip(self.left_gripper_actuator_ids, left_target):
                if actuator_id is not None:
                    ctrl[actuator_id] = actuator_target

    def _set_gripper_qpos(self, side: str, qpos_target: np.ndarray) -> None:
        for joint_name, joint_target in zip(_GRIPPER_JOINT_ORDER, qpos_target):
            resolved_name = self._resolve_joint_name(side, joint_name)
            if resolved_name is None:
                continue
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, resolved_name)
            qpos_adr = self.model.jnt_qposadr[joint_id]
            self.data.qpos[qpos_adr] = joint_target

    def _send_control_sim_dynamics(self, command, **kwargs):
        x = self._compose_simulator_dynamics_state()
        x = self._advance_configured_dynamics(x, command, **kwargs)

        self.dof_pos_cmd = self.robot_cfg.decompose_state_to_dof_pos(x)
        self.dof_vel_cmd = self.robot_cfg.decompose_state_to_dof_vel(x)

        self.target_pos = self.dof_pos_cmd.copy()
        self.target_vel = self._simulator_target_velocity(command)

        action_info = kwargs.get("action_info", {})
        for _ in range(self.control_decimation):
            ctrl = np.zeros(len(self.data.ctrl), dtype=float)
            self._apply_arm_ctrl(ctrl, self.target_pos, self.target_vel)
            self._apply_gripper_ctrl(ctrl, action_info)
            self.data.ctrl[:] = ctrl
            self.counter += 1
            mujoco.mj_step(self.model, self.data)

    def _send_control_modeled_dynamics(self, command, **kwargs):
        x = self._compose_cmd_state()
        x = self._advance_configured_dynamics(x, command, **kwargs)

        self.dof_pos_cmd = self.robot_cfg.decompose_state_to_dof_pos(x)
        self.dof_vel_cmd = self.robot_cfg.decompose_state_to_dof_vel(x)
        action_info = kwargs.get("action_info", {})

        self._set_dof_pos(self.dof_pos_cmd)

        if len(self.data.ctrl) > 0:
            ctrl = np.zeros(len(self.data.ctrl), dtype=float)
            self._apply_arm_ctrl(ctrl, self.dof_pos_cmd, self.dof_vel_cmd)
            self._apply_gripper_ctrl(ctrl, action_info)
            self.data.ctrl[:] = ctrl

        right_target = self._gripper_target("right", action_info)
        self._set_gripper_qpos("right", right_target)
        if self.num_arms > 1:
            left_target = self._gripper_target("left", action_info)
            self._set_gripper_qpos("left", left_target)

    def controlGripperRegister(self, POA=0, POB=0, POC=0, POS=137, ICF=False):
        COEF_SCISSORS = 26 / 220
        COEF_JOINT_1 = 62 / 140
        COEF_JOINT_2 = 90 / 100

        fa0 = 0
        fb0 = COEF_SCISSORS * min(POS, 220) - 10
        fc0 = -fb0

        fa1 = COEF_JOINT_1 * min(POA, 140)
        fa2 = COEF_JOINT_2 * min(max(POA - 140, 0), 100)
        fa3 = -COEF_JOINT_1 * min(POA, 110)

        if ICF is False:
            fb1, fb2, fb3 = fa1, fa2, fa3
            fc1, fc2, fc3 = fa1, fa2, fa3
        else:
            fb1 = COEF_JOINT_1 * min(POB, 140)
            fb2 = COEF_JOINT_2 * min(max(POB - 140, 0), 100)
            fb3 = -COEF_JOINT_1 * min(POB, 110)
            fc1 = COEF_JOINT_1 * min(POC, 140)
            fc2 = COEF_JOINT_2 * min(max(POC - 140, 0), 100)
            fc3 = -COEF_JOINT_1 * min(POC, 110)

        command_gripper = np.array(
            [
                fa0,
                fa1,
                fa2,
                fa3,
                fb0,
                fb1,
                fb2,
                fb3,
                fc0,
                fc1,
                fc2,
                fc3,
            ]
        )
        return np.deg2rad(command_gripper)
