"""Shared MuJoCo execution for fixed-base KUKA iiwa 14 embodiments."""

import mujoco
import numpy as np

from spark_agent.simulation.mujoco.mujoco_agent import MujocoAgent
from spark_robot import RobotConfig


class _KukaIIWA14AgentBase(MujocoAgent):
    """Keep single- and dual-arm iiwa simulator behavior identical."""

    ARM_DOF_PER_ARM = 7

    def __init__(self, robot_cfg: RobotConfig, **kwargs) -> None:
        super().__init__(robot_cfg, **kwargs)
        self.sim_use_bias_compensation = bool(kwargs.get("sim_use_bias_compensation", True))
        self.sim_force_limit_margin = float(kwargs.get("sim_force_limit_margin", 0.6))
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
        self.right_gripper_actuator = self._resolve_actuator_id("right/fingers_actuator")
        self.left_gripper_actuator = self._resolve_actuator_id("left/fingers_actuator")

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
            mujoco_dof = int(self.robot_cfg.DoF_to_MujocoDoF[dof])
            self.dof_pos_fbk[dof] = self.data.qpos[mujoco_dof]
            self.dof_vel_fbk[dof] = self.data.qvel[mujoco_dof]
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
        for mujoco_dof in self.robot_cfg.MujocoDoFs:
            dof = self.robot_cfg.MujocoDoF_to_DoF[mujoco_dof]
            self.data.qpos[int(mujoco_dof)] = dof_pos[dof]
        self.data.qvel[:] = 0.0
        self.data.qacc[:] = 0.0
        self.data.qfrc_applied[:] = 0.0
        self.data.xfrc_applied[:, :] = 0.0

    def _resolve_actuator_id(self, name: str):
        actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        return actuator_id if actuator_id >= 0 else None

    def _apply_arm_ctrl(
        self,
        ctrl: np.ndarray,
        target_pos: np.ndarray,
        target_vel: np.ndarray | None = None,
    ) -> None:
        """Realize the configured joint target through the iiwa affine servos."""

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

    def _apply_arm_acceleration_ctrl(self, ctrl: np.ndarray, command: np.ndarray) -> None:
        """Express inverse-dynamics acceleration force as affine targets."""

        for actuator_id, actuator_force in self._joint_acceleration_actuator_forces(
            command
        ).items():
            gain = float(self.model.actuator_gainprm[actuator_id, 0])
            if abs(gain) <= 1.0e-9:
                continue
            bias = float(
                self.model.actuator_biasprm[actuator_id, 0]
                + self.model.actuator_biasprm[actuator_id, 1]
                * self.data.actuator_length[actuator_id]
                + self.model.actuator_biasprm[actuator_id, 2]
                * self.data.actuator_velocity[actuator_id]
            )
            target = (actuator_force - bias) / gain
            ctrl[actuator_id] = self._clip_actuator_control(
                actuator_id,
                self._limit_affine_actuator_force(actuator_id, target),
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
        self.right_gripper_goal = self._resolve_binary_gripper_goal("right", action_info)
        self.left_gripper_goal = self._resolve_binary_gripper_goal("left", action_info)
        if self.right_gripper_actuator is not None:
            ctrl[self.right_gripper_actuator] = 255.0 if self.right_gripper_goal else 0.0
        if self.num_arms > 1 and self.left_gripper_actuator is not None:
            ctrl[self.left_gripper_actuator] = 255.0 if self.left_gripper_goal else 0.0

    def _send_control_sim_dynamics(self, command, **kwargs):
        state = self._compose_simulator_dynamics_state()
        state = self._advance_configured_dynamics(state, command, **kwargs)
        self.dof_pos_cmd = self.robot_cfg.decompose_state_to_dof_pos(state)
        self.dof_vel_cmd = self.robot_cfg.decompose_state_to_dof_vel(state)
        self.target_pos = self.dof_pos_cmd.copy()
        self.target_vel = self._simulator_target_velocity(command)
        action_info = kwargs.get("action_info", {})
        for _ in range(self.control_decimation):
            ctrl = np.zeros(len(self.data.ctrl), dtype=float)
            if self.dynamic_order == 2:
                self._apply_arm_acceleration_ctrl(ctrl, command)
            else:
                self._apply_arm_ctrl(ctrl, self.target_pos, self.target_vel)
            self._apply_gripper_ctrl(ctrl, action_info)
            self.data.ctrl[:] = ctrl
            self.counter += 1
            mujoco.mj_step(self.model, self.data)

    def _send_control_modeled_dynamics(self, command, **kwargs):
        state = self._compose_cmd_state()
        state = self._advance_configured_dynamics(state, command, **kwargs)
        self.dof_pos_cmd = self.robot_cfg.decompose_state_to_dof_pos(state)
        self.dof_vel_cmd = self.robot_cfg.decompose_state_to_dof_vel(state)
        action_info = kwargs.get("action_info", {})
        self._set_dof_pos(self.dof_pos_cmd)
        if len(self.data.ctrl) > 0:
            ctrl = np.zeros(len(self.data.ctrl), dtype=float)
            self._apply_arm_ctrl(ctrl, self.dof_pos_cmd, self.dof_vel_cmd)
            self._apply_gripper_ctrl(ctrl, action_info)
            self.data.ctrl[:] = ctrl
        self._set_robotiq_2f85_qpos("right", self.right_gripper_goal)
        if self.num_arms > 1:
            self._set_robotiq_2f85_qpos("left", self.left_gripper_goal)
        mujoco.mj_forward(self.model, self.data)
