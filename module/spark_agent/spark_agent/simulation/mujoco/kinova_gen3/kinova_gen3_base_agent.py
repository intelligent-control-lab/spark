from spark_agent.simulation.mujoco.mujoco_agent import MujocoAgent
import mujoco
import numpy as np
from spark_robot import RobotConfig


class _KinovaGen3AgentBase(MujocoAgent):
    """Shared MuJoCo implementation for fixed-base Kinova Gen3 variants."""

    ARM_DOF_PER_ARM = 7

    def __init__(self, robot_cfg: RobotConfig, **kwargs) -> None:
        super().__init__(robot_cfg, **kwargs)
        self.sim_use_bias_compensation = bool(kwargs.get("sim_use_bias_compensation", True))
        self.sim_force_limit_margin = float(kwargs.get("sim_force_limit_margin", 0.6))
        if not 0.0 < self.sim_force_limit_margin <= 1.0:
            raise ValueError("sim_force_limit_margin must be in (0, 1]")
        self.arm_actuator_indices = np.array(
            [int(mj_motor) for mj_motor in self.robot_cfg.MujocoMotors], dtype=np.int32
        )
        self.arm_control_indices = np.array(
            [
                int(self.robot_cfg.MujocoMotor_to_Control[mj_motor])
                for mj_motor in self.robot_cfg.MujocoMotors
            ],
            dtype=np.int32,
        )
        self.arm_dof_indices = np.array(
            [
                int(
                    self.robot_cfg.MujocoDoF_to_DoF[
                        getattr(self.robot_cfg.MujocoDoFs, mj_motor.name)
                    ]
                )
                for mj_motor in self.robot_cfg.MujocoMotors
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
        agent_reset_info = agent_reset_info or {}

        if "reset_dof_pos" in agent_reset_info:
            self.default_dof_pos = agent_reset_info["reset_dof_pos"]
        else:
            self.default_dof_pos = np.array(
                [self.robot_cfg.DefaultDoFVal[dof] for dof in self.robot_cfg.DoFs]
            )

        self._set_dof_pos(self.default_dof_pos)
        self._mujoco_step()

    def get_feedback(self) -> None:
        ret = {}

        ret["object_pos"] = {}
        for b in range(1, self.model.nbody):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, b) or f"body_{b}"
            p = self.data.xpos[b].copy()
            q = self.data.xquat[b].copy()
            ret["object_pos"][name] = np.hstack((p, q))

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
            self.dof_pos_fbk[dof] = self.data.qpos[mj_dof]
            self.dof_vel_fbk[dof] = self.data.qvel[mj_dof]

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
        ret["left_gripper_goal"] = bool(getattr(self, "left_gripper_goal", False))
        ret["right_gripper_goal"] = bool(getattr(self, "right_gripper_goal", False))
        return ret

    def _set_dof_pos(self, dof_pos: np.ndarray) -> None:
        for mj_dof_idx in self.robot_cfg.MujocoDoFs:
            dof_idx = self.robot_cfg.MujocoDoF_to_DoF[mj_dof_idx]
            self.data.qpos[mj_dof_idx] = dof_pos[dof_idx]

        self.data.qvel[:] = 0
        self.data.qacc[:] = 0
        self.data.qfrc_applied[:] = 0
        self.data.xfrc_applied[:, :] = 0

    def _resolve_actuator_id(self, actuator_name: str):
        actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator_name)
        return actuator_id if actuator_id != -1 else None

    def _apply_arm_ctrl(
        self,
        ctrl: np.ndarray,
        target_pos: np.ndarray,
        target_vel: np.ndarray | None = None,
    ) -> None:
        """Apply position/velocity targets with bias-force feed-forward.

        The Gen3 MJCF uses affine position actuators.  Simulator-owned
        velocity commands are deliberately integrated from measured state,
        so an uncompensated zero command otherwise follows gravity-induced
        drift instead of holding the requested pose.  Express the generalized
        bias force as the equivalent actuator-position offset while retaining
        MuJoCo as the state owner.

        MuJoCo's affine position actuator implements ``kp * (target - q) -
        kd * dq``.  A position target alone therefore discards the desired
        velocity and makes a measured-state-anchored velocity step much slower
        than its command.  Add ``kd / kp * target_vel`` to the actuator target
        so the same actuator realizes the full PD expression without reducing
        its stopping damping.
        """

        target_pos = self._clip_target_pos_to_mujoco_limits(target_pos)
        target_vel = (
            np.zeros(self.num_dof, dtype=np.float64)
            if target_vel is None
            else np.asarray(target_vel, dtype=np.float64)
        )
        for motor_index, actuator_id in enumerate(self.arm_actuator_indices):
            dof_index = int(self.arm_dof_indices[motor_index])
            target = float(target_pos[dof_index])
            gain = float(self.model.actuator_gainprm[int(actuator_id), 0])
            damping = max(0.0, -float(self.model.actuator_biasprm[int(actuator_id), 2]))
            if abs(gain) > 1.0e-9:
                target += damping * float(target_vel[dof_index]) / gain
            if self.sim_use_bias_compensation:
                if abs(gain) > 1.0e-9:
                    target += self._actuator_bias_control(int(actuator_id)) / gain
            target = self._limit_affine_actuator_force(int(actuator_id), target)
            ctrl[int(actuator_id)] = self._clip_actuator_control(int(actuator_id), target)

    def _limit_affine_actuator_force(self, actuator_id: int, target: float) -> float:
        """Keep the implicit position servo below MuJoCo's force clamp.

        MuJoCo 3.8 corrected implicit actuator derivatives when ``forcerange``
        clamps an actuator.  The legacy Gen3 gains can repeatedly cross that
        discontinuity during a large Cartesian move.  Bound the affine target
        using the current actuator length and velocity so MuJoCo retains its
        damping derivative on both the 3.4 and 3.8 runtime stacks.
        """

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
        """Return the joint-velocity target represented by this control step."""

        if self.dynamic_order != 1:
            return self.dof_vel_cmd.copy()

        command = np.asarray(command, dtype=np.float64)
        target_vel = np.zeros(self.num_dof, dtype=np.float64)
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
        x = self._compose_simulator_dynamics_state()
        x = self._advance_configured_dynamics(x, command, **kwargs)

        self.dof_pos_cmd = self.robot_cfg.decompose_state_to_dof_pos(x)
        self.dof_vel_cmd = self.robot_cfg.decompose_state_to_dof_vel(x)

        self.target_pos = self.dof_pos_cmd.copy()
        self.target_vel = self._simulator_target_velocity(command)

        action_info = kwargs.get("action_info", {})
        for _ in range(self.control_decimation):
            ctrl = np.zeros(len(self.data.ctrl))
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
            ctrl = np.zeros(len(self.data.ctrl))
            self._apply_arm_ctrl(ctrl, self.dof_pos_cmd, self.dof_vel_cmd)
            self._apply_gripper_ctrl(ctrl, action_info)
            self.data.ctrl[:] = ctrl
        self._set_robotiq_2f85_qpos("right", self.right_gripper_goal)
        if self.num_arms > 1:
            self._set_robotiq_2f85_qpos("left", self.left_gripper_goal)
        mujoco.mj_forward(self.model, self.data)
