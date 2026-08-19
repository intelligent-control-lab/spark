from spark_agent.simulation.mujoco.mujoco_agent import MujocoAgent
import mujoco
import numpy as np
from spark_robot import RobotConfig
from scipy.spatial.transform import Rotation as R
from .unitree_g1_gripper_mixin import UnitreeG1GripperMixin


class UnitreeG1MobileBaseMujocoAgent(UnitreeG1GripperMixin, MujocoAgent):
    """
    A Mujoco-based agent for the G1 robot, extending MujocoAgent.
    Provides methods to compose command and feedback states.
    """

    def __init__(self, robot_cfg: RobotConfig, **kwargs) -> None:
        """
        Initializes the Mujoco agent with robot configuration and simulation parameters.

        Args:
            robot_cfg (RobotConfig): The configuration for the robot.
            **kwargs: Additional keyword arguments such as simulation dynamics, model path, viewer settings, etc.
        """
        super().__init__(robot_cfg, **kwargs)
        self._controlled_mujoco_motor_indices = np.array(
            [int(motor) for motor in self.robot_cfg.MujocoMotors],
            dtype=int,
        )
        self._controlled_mujoco_qpos_indices = np.array(
            [
                int(getattr(self.robot_cfg.MujocoDoFs, motor.name))
                for motor in self.robot_cfg.MujocoMotors
            ],
            dtype=int,
        )
        self._controlled_mujoco_qvel_indices = self._controlled_mujoco_qpos_indices.copy()

    # -------------------------------- Simulation Helpers -------------------------------- #

    def reset(self, agent_reset_info) -> None:
        """Resets the robot to its default position and configuration."""

        # Call the parent reset method (if any)
        super().reset()

        # Set the default position for the robot's DoFs
        if "reset_dof_pos" in agent_reset_info:
            self.default_dof_pos = agent_reset_info["reset_dof_pos"]
        else:
            self.default_dof_pos = np.array(
                [self.robot_cfg.DefaultDoFVal[dof] for dof in self.robot_cfg.DoFs]
            )
        self._reset_unitree_gripper_control()

        # Step through the simulation to update the robot's configuration
        self._mujoco_step()

    def get_feedback(self) -> None:
        ret = {}

        # Feedback: Robot frame in world frame
        # Extract robot's global position and orientation
        global_position = self.data.body("robot").xpos.copy()  # [x, y, z]
        global_orientation = (
            self.data.body("robot").xmat.copy().reshape(3, 3)
        )  # 3x3 rotation matrix

        # Construct the 4x4 transformation matrix for the robot's base frame
        robot_base_frame = np.eye(4)  # Start with identity matrix
        robot_base_frame[:3, :3] = global_orientation  # Top-left 3x3 is the rotation matrix
        robot_base_frame[:3, 3] = global_position  # Translation vector

        ret["robot_base_frame"] = robot_base_frame

        # Feedback: Degrees of Freedom (DoF) positions
        rot = R.from_matrix(robot_base_frame[:3, :3])  # Rotation matrix to Euler angles
        euler = rot.as_euler("xyz")  # Euler angles in XYZ order

        # Initialize or reset feedback arrays for position and velocity
        self.dof_pos_fbk = np.zeros(self.num_dof) if self.dof_pos_fbk is None else self.dof_pos_fbk
        self.dof_vel_fbk = np.zeros(self.num_dof) if self.dof_vel_fbk is None else self.dof_vel_fbk

        # Update DoF position feedback based on the robot configuration
        for dof in self.robot_cfg.DoFs:
            if dof.name == "LinearX":
                self.dof_pos_fbk[dof] = robot_base_frame[0, 3]
            elif dof.name == "LinearY":
                self.dof_pos_fbk[dof] = robot_base_frame[1, 3]
            elif dof.name == "RotYaw":
                self.dof_pos_fbk[dof] = euler[2]
            else:
                mj_dof = self.robot_cfg.DoF_to_MujocoDoF[dof]
                self.dof_pos_fbk[dof] = self.data.qpos[mj_dof]
                self.dof_vel_fbk[dof] = self.data.qvel[mj_dof]

        # Initialize commanded DoF values if they are None
        self.dof_pos_cmd = self.dof_pos_fbk.copy() if self.dof_pos_cmd is None else self.dof_pos_cmd
        self.dof_vel_cmd = self.dof_vel_fbk.copy() if self.dof_vel_cmd is None else self.dof_vel_cmd
        self.dof_acc_cmd = np.zeros(self.num_dof) if self.dof_acc_cmd is None else self.dof_acc_cmd

        # Collect and return feedback
        ret["dof_pos_fbk"] = self.dof_pos_fbk.copy()
        ret["dof_vel_fbk"] = self.dof_vel_fbk.copy()
        ret["dof_pos_cmd"] = self.dof_pos_cmd.copy()
        ret["dof_vel_cmd"] = self.dof_vel_cmd.copy()
        ret["dof_acc_cmd"] = self.dof_acc_cmd.copy()

        # Dynamics state feedback
        ret["state"] = self._compose_fbk_state()

        # Virtual obstacle feedback
        ret["obstacle_debug_frame"] = self.obstacle_debug_frame
        ret["obstacle_debug_geom"] = self.obstacle_debug_geom
        ret["obstacle_debug_velocity"] = (
            np.array(self.obstacle_debug_velocity)
            if len(self.obstacle_debug_velocity) > 0
            else np.empty((0, 6))
        )

        # Update goal position
        ret["robot_goal_left_offset"] = self.left_goal_debug_frame
        ret["robot_goal_right_offset"] = self.right_goal_debug_frame
        ret["robot_goal_base_offset"] = self.base_goal_debug_frame
        self._add_unitree_gripper_feedback(ret)

        return ret

    def _set_dof_pos(self, dof_pos: np.ndarray) -> None:
        """Sets the Degrees of Freedom (DoF) positions for the robot."""

        qpos = np.zeros(self.model.nq)  # Initialize qpos with zeros
        # Map the DoF positions to Mujoco DoFs based on the robot configuration
        for mj_dof_idx in self.robot_cfg.MujocoDoFs:
            dof_idx = self.robot_cfg.MujocoDoF_to_DoF[mj_dof_idx]
            qpos[mj_dof_idx] = dof_pos[dof_idx]

        # Set the robot's configuration in the model data
        self.data.qpos = qpos
        # Reset velocities, accelerations, and applied forces
        self.data.qvel[:] = 0  # Clear velocities
        self.data.qacc[:] = 0  # Clear accelerations
        self.data.qfrc_applied[:] = 0  # Clear applied forces
        self.data.xfrc_applied[:, :] = 0  # Clear external forces

    def _send_control_sim_dynamics(self, command, **kwargs):
        """
        This method integrates the robot's dynamics and updates its state based on modeled dynamics.
        """

        # Get the current state of the robot and apply the commanded dynamics
        x = self._compose_simulator_dynamics_state()
        x = self._advance_configured_dynamics(x, command, **kwargs)

        # Decompose the state into degree-of-freedom (DoF) positions and velocities
        self.dof_pos_cmd = self.robot_cfg.decompose_state_to_dof_pos(x)
        self.dof_vel_cmd = self.robot_cfg.decompose_state_to_dof_vel(x)

        # Set target positions and velocities for other modes
        self.target_pos = np.zeros(self.model.nq)
        self.target_vel = np.zeros(self.model.nv)

        # Apply control for each degree-of-freedom (DoF)
        for dof in self.robot_cfg.MujocoDoFs:
            dof_idx = self.robot_cfg.MujocoDoF_to_DoF[dof]
            mj_dof_idx = int(dof)
            if dof.name == "LinearX":
                self.target_pos[mj_dof_idx] = self.dof_pos_cmd[dof_idx]
                self.target_vel[mj_dof_idx] = (
                    self.dof_vel_cmd[dof_idx]
                    if self.dynamic_order == 2
                    else command[self.robot_cfg.Control.vLinearX]
                )
            elif dof.name == "LinearY":
                self.target_pos[mj_dof_idx] = self.dof_pos_cmd[dof_idx]
                self.target_vel[mj_dof_idx] = (
                    self.dof_vel_cmd[dof_idx]
                    if self.dynamic_order == 2
                    else command[self.robot_cfg.Control.vLinearY]
                )
            elif dof.name == "RotYaw":
                self.target_pos[mj_dof_idx] = self.dof_pos_cmd[dof_idx]
                self.target_vel[mj_dof_idx] = (
                    self.dof_vel_cmd[dof_idx]
                    if self.dynamic_order == 2
                    else command[self.robot_cfg.Control.vRotYaw]
                )
            else:
                self.target_pos[mj_dof_idx] = self.dof_pos_cmd[dof_idx]
                self.target_vel[mj_dof_idx] = self.dof_vel_cmd[
                    dof_idx
                ]  # Feedforward velocity is zero for non-mobile DoFs; only track position

        # Apply PD control to the robot's actuators
        for _ in range(self.control_decimation):
            tau = self._unitree_gravity_compensated_pd(
                self.target_pos[self._controlled_mujoco_qpos_indices],
                self.data.qpos[self._controlled_mujoco_qpos_indices],
                self.kps,
                self.target_vel[self._controlled_mujoco_qvel_indices],
                self.data.qvel[self._controlled_mujoco_qvel_indices],
                self.kds,
                self._controlled_mujoco_qvel_indices,
            )
            ctrl = np.zeros(len(self.data.ctrl), dtype=np.float64)
            ctrl[self._controlled_mujoco_motor_indices] = tau
            self.data.ctrl[:] = ctrl
            self._apply_unitree_gripper_control(kwargs.get("action_info", {}))
            self.counter += 1
            mujoco.mj_step(self.model, self.data)

    def _send_control_modeled_dynamics(self, command, **kwargs):
        """
        This method invokes modeled dynamics to simulate the robot's state changes and applies control.
        """
        # Get the current robot state and apply the dynamics model
        x = self._compose_cmd_state()
        x = self._advance_configured_dynamics(x, command, **kwargs)

        # Decompose the state to update degree-of-freedom (DoF) positions
        self.dof_pos_cmd = self.robot_cfg.decompose_state_to_dof_pos(x)
        self.dof_vel_cmd = self.robot_cfg.decompose_state_to_dof_vel(x)

        # Set the degree-of-freedom (DoF) positions based on the updated state
        self._set_dof_pos(self.dof_pos_cmd)
