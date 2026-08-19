import numpy as np
import mujoco
from scipy.spatial.transform import Rotation as R

from spark_robot import RobotConfig
from spark_agent.simulation.mujoco.agibot_g1.agibot_g1_fixed_base_agent import (
    _AgiBotG1AgentBase,
)


class AgiBotG1MobileBaseAgent(_AgiBotG1AgentBase):
    """AgiBot G1 mobile-base MuJoCo agent."""

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

        rot = R.from_matrix(robot_base_frame[:3, :3])
        euler = rot.as_euler("xyz")

        self.dof_pos_fbk = np.zeros(self.num_dof) if self.dof_pos_fbk is None else self.dof_pos_fbk
        self.dof_vel_fbk = np.zeros(self.num_dof) if self.dof_vel_fbk is None else self.dof_vel_fbk

        for dof in self.robot_cfg.DoFs:
            if dof.name == "LinearX":
                self.dof_pos_fbk[dof] = robot_base_frame[0, 3]
                self.dof_vel_fbk[dof] = self.data.qvel[int(self.robot_cfg.DoF_to_MujocoDoF[dof])]
            elif dof.name == "LinearY":
                self.dof_pos_fbk[dof] = robot_base_frame[1, 3]
                self.dof_vel_fbk[dof] = self.data.qvel[int(self.robot_cfg.DoF_to_MujocoDoF[dof])]
            elif dof.name == "RotYaw":
                self.dof_pos_fbk[dof] = euler[2]
                self.dof_vel_fbk[dof] = self.data.qvel[int(self.robot_cfg.DoF_to_MujocoDoF[dof])]
            else:
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
        state_velocity = np.asarray(self.robot_cfg.dynamics_f(x), dtype=np.float64).reshape(
            -1
        ) + np.asarray(self.robot_cfg.dynamics_g(x), dtype=np.float64) @ np.asarray(
            command, dtype=np.float64
        ).reshape(-1)
        x = self._advance_configured_dynamics(x, command, **kwargs)

        self.dof_pos_cmd = self.robot_cfg.decompose_state_to_dof_pos(x)
        self.dof_vel_cmd = self.robot_cfg.decompose_state_to_dof_vel(x)

        target_pos = self.dof_pos_cmd.copy()
        target_vel = self.dof_vel_cmd.copy()
        if self.dynamic_order == 1:
            # The high-level planar velocity is robot-frame relative whereas
            # MuJoCo's x/y slide joints are world-frame relative.  Use the
            # configured dynamics derivative so holonomic and unicycle views
            # share the same correct frame conversion.
            target_vel = state_velocity.copy()

        action_info = kwargs.get("action_info", {})
        for _ in range(self.control_decimation):
            ctrl = np.zeros(len(self.data.ctrl), dtype=np.float64)
            if self.stabilize_sim_dynamics_joint_positions:
                self._set_controlled_dof_state(target_pos, target_vel)
                self._enforce_uncontrolled_actuator_holds()
                self._apply_arm_bias_ctrl(ctrl)
            else:
                self._apply_arm_ctrl(ctrl, target_pos, target_vel, kd_scale=1.0)
            self._apply_uncontrolled_actuator_holds(ctrl)
            self._apply_gripper_ctrl(ctrl, action_info)
            self.data.ctrl[:] = ctrl
            self.counter += 1
            mujoco.mj_step(self.model, self.data)
            if self.stabilize_sim_dynamics_joint_positions:
                self._set_controlled_dof_state(target_pos, target_vel)
            self._enforce_uncontrolled_actuator_holds()
