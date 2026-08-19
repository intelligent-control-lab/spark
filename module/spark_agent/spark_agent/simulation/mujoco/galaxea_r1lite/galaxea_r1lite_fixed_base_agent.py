from spark_agent.simulation.mujoco.mujoco_agent import MujocoAgent
import mujoco
import mujoco.viewer
import numpy as np
import os
import time
from scipy.spatial.transform import Rotation as R
from spark_agent import SPARK_AGENT_ROOT
from spark_robot import RobotConfig, SPARK_ROBOT_RESOURCE_DIR
from spark_utils import Geometry, VizColor
import cv2
import imageio
from typing import Optional


class GalaxeaR1LiteFixedBaseAgent(MujocoAgent):
    """
    A Mujoco-based agent for the Galaxea R1 Lite fixed-base robot.
    Provides methods to compose command and feedback states.
    """

    def __init__(self, robot_cfg: RobotConfig, **kwargs) -> None:
        """
        Initializes the Mujoco agent with robot configuration and simulation parameters.

        Args:
            robot_cfg (RobotConfig): The configuration for the robot.
            **kwargs: Additional keyword arguments such as simulation dynamics, model path, viewer settings, etc.
        """
        kwargs.setdefault("enable_hand_control", True)
        super().__init__(robot_cfg, **kwargs)
        # R1 Lite's shoulder and torso joints are strongly gravity-loaded.
        # Make the qualified physical-servo behavior the agent default so
        # generic runners and robot-local workflows use the same plant.
        self.sim_use_bias_compensation = bool(kwargs.get("sim_use_bias_compensation", True))
        self.controlled_actuator_indices = np.array(
            [int(mj_motor) for mj_motor in self.robot_cfg.MujocoMotors],
            dtype=np.int32,
        )
        self._configure_uncontrolled_actuator_holds(
            controlled_actuator_ids=self.controlled_actuator_indices,
            excluded_actuator_ids=self._r1lite_gripper_actuator_ids,
            joint_names=getattr(self.robot_cfg, "joint_to_lock", None),
        )

    def _init_embodiment_control(self, **kwargs) -> None:
        """Resolve optional finger joints and actuators by their declared names."""

        del kwargs
        self._r1lite_grippers = {}
        articulation = getattr(self.robot_cfg, "isaac_articulation", None)
        for spec in getattr(articulation, "grippers", ()):
            actuator_names = spec.actuator_names or spec.joint_names
            joint_ids = [
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
                for name in spec.joint_names
            ]
            actuator_ids = [
                mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
                for name in actuator_names
            ]
            if any(index < 0 for index in (*joint_ids, *actuator_ids)):
                continue
            self._r1lite_grippers[spec.side] = {
                "spec": spec,
                "qpos": np.asarray(
                    [self.model.jnt_qposadr[index] for index in joint_ids], dtype=np.int32
                ),
                "qvel": np.asarray(
                    [self.model.jnt_dofadr[index] for index in joint_ids], dtype=np.int32
                ),
                "actuator": np.asarray(actuator_ids, dtype=np.int32),
            }
        self._r1lite_gripper_actuator_ids = np.asarray(
            [
                actuator
                for record in self._r1lite_grippers.values()
                for actuator in record["actuator"]
            ],
            dtype=np.int32,
        )

    def _r1lite_gripper_target(self, side: str, action_info: dict) -> np.ndarray:
        record = self._r1lite_grippers[side]
        spec = record["spec"]
        control_key = f"{side}_gripper_control"
        goal_key = f"{side}_gripper_goal"
        if control_key in action_info:
            target = np.asarray(action_info[control_key], dtype=float).reshape(-1)
            if target.size != len(spec.joint_names):
                raise ValueError(f"{control_key} must have {len(spec.joint_names)} entries")
            closed = bool(
                np.linalg.norm(target - np.asarray(spec.open_positions, dtype=float)) > 1e-6
            )
            self._set_resolved_gripper_goal(side, closed)
        else:
            closed = self._resolve_binary_gripper_goal(side, action_info)
            target = np.asarray(
                spec.closed_positions if closed else spec.open_positions, dtype=float
            )
        return target

    def _apply_r1lite_gripper_control(self, ctrl: np.ndarray, action_info=None) -> None:
        if not self.enable_hand_control:
            return
        action_info = dict(action_info or {})
        for side, record in self._r1lite_grippers.items():
            ctrl[record["actuator"]] = self._r1lite_gripper_target(side, action_info)

    def _reset_r1lite_grippers(self) -> None:
        self._keyboard_gripper_goal_override.clear()
        for side, record in self._r1lite_grippers.items():
            spec = record["spec"]
            setattr(self, f"{side}_gripper_goal", False)
            setattr(self, f"{side}_gripper_debug_state", False)
            self.data.qpos[record["qpos"]] = np.asarray(spec.open_positions, dtype=float)
            self.data.qvel[record["qvel"]] = 0.0

    def _write_r1lite_gripper_configuration(self, action_info=None) -> None:
        if not self.enable_hand_control:
            return
        action_info = dict(action_info or {})
        for side, record in self._r1lite_grippers.items():
            self.data.qpos[record["qpos"]] = self._r1lite_gripper_target(side, action_info)
            self.data.qvel[record["qvel"]] = 0.0

    # ---------------------------------- Setup Helpers --------------------------------- #

    def reset(self, agent_reset_info=None) -> None:
        """Resets the robot to its default position and configuration."""

        agent_reset_info = dict(agent_reset_info or {})

        # Call the parent reset method (if any)
        super().reset()

        # Set the default position for the robot's DoFs
        if "reset_dof_pos" in agent_reset_info:
            self.default_dof_pos = agent_reset_info["reset_dof_pos"]
        else:
            self.default_dof_pos = np.array(
                [self.robot_cfg.DefaultDoFVal[dof] for dof in self.robot_cfg.DoFs]
            )
        self.dof_pos_cmd = np.asarray(self.default_dof_pos, dtype=np.float64).copy()
        self.dof_vel_cmd = np.zeros(self.num_dof, dtype=np.float64)
        self.dof_acc_cmd = np.zeros(self.num_dof, dtype=np.float64)

        # Set the robot's DoF positions to the default ones
        self._set_dof_pos(self.default_dof_pos)
        self.data.qvel[:] = 0
        self.data.qacc[:] = 0
        self.data.qfrc_applied[:] = 0
        self.data.xfrc_applied[:, :] = 0
        self._reset_r1lite_grippers()
        self._refresh_uncontrolled_actuator_hold_targets()
        self._enforce_uncontrolled_actuator_holds()

        if len(self.data.ctrl) > 0:
            ctrl = np.zeros(len(self.data.ctrl), dtype=np.float64)
            self._apply_uncontrolled_actuator_holds(ctrl)
            self.data.ctrl[:] = ctrl

        # Step through the simulation to update the robot's configuration
        self._mujoco_step()

    def get_feedback(self) -> None:
        ret = {}

        ret["object_pos"] = {}
        for b in range(1, self.model.nbody):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, b) or f"body_{b}"
            p = self.data.xpos[b].copy()  # (3,)
            q = self.data.xquat[b].copy()  # (4,) quaternion (w, x, y, z)
            ret["object_pos"][name] = np.hstack((p, q))  # Combine position and orientation

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

        # Initialize or reset feedback arrays for position and velocity
        self.dof_pos_fbk = np.zeros(self.num_dof) if self.dof_pos_fbk is None else self.dof_pos_fbk
        self.dof_vel_fbk = np.zeros(self.num_dof) if self.dof_vel_fbk is None else self.dof_vel_fbk

        # Update DoF position feedback based on the robot configuration
        for dof in self.robot_cfg.DoFs:
            mj_dof = self.robot_cfg.DoF_to_MujocoDoF[dof]
            self.dof_pos_fbk[dof] = self.data.qpos[mj_dof]
            self.dof_vel_fbk[dof] = self.data.qvel[mj_dof]

        # Initialize commanded DoF values if they are None
        self.dof_pos_cmd = self.dof_pos_fbk.copy() if self.dof_pos_cmd is None else self.dof_pos_cmd
        self.dof_vel_cmd = self.dof_vel_fbk.copy() if self.dof_vel_cmd is None else self.dof_vel_cmd
        self.dof_acc_cmd = np.zeros(self.num_dof) if self.dof_acc_cmd is None else self.dof_acc_cmd

        # If not using simulated dynamics, set feedback to commanded values
        if not self.use_sim_dynamics:
            self.dof_pos_fbk = self.dof_pos_cmd.copy()
            self.dof_vel_fbk = self.dof_vel_cmd.copy()

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
        ret["left_gripper_goal"] = bool(self.left_gripper_goal)
        ret["right_gripper_goal"] = bool(self.right_gripper_goal)
        ret["left_gripper_debug_state"] = bool(self.left_gripper_debug_state)
        ret["right_gripper_debug_state"] = bool(self.right_gripper_debug_state)

        return ret

    def _set_dof_pos(self, dof_pos: np.ndarray) -> None:
        """Sets the Degrees of Freedom (DoF) positions for the robot."""
        for mj_dof_idx in self.robot_cfg.MujocoDoFs:
            dof_idx = self.robot_cfg.MujocoDoF_to_DoF[mj_dof_idx]
            self.data.qpos[mj_dof_idx] = dof_pos[dof_idx]

    def _current_controlled_state(self):
        q = np.zeros(self.num_dof, dtype=np.float64)
        dq = np.zeros(self.num_dof, dtype=np.float64)
        for dof in self.robot_cfg.DoFs:
            mj_dof = int(self.robot_cfg.DoF_to_MujocoDoF[dof])
            q[int(dof)] = self.data.qpos[mj_dof]
            dq[int(dof)] = self.data.qvel[mj_dof]
        return q, dq

    def _apply_controlled_joint_pd(
        self, ctrl: np.ndarray, target_pos: np.ndarray, target_vel: np.ndarray
    ) -> None:
        current_q, current_dq = self._current_controlled_state()
        target_pos = self._clip_target_pos_to_mujoco_limits(target_pos)
        target_pos = self._limit_target_pos_error(target_pos, current_q)
        for motor_index, mj_motor in enumerate(self.robot_cfg.MujocoMotors):
            mj_dof = getattr(self.robot_cfg.MujocoDoFs, mj_motor.name)
            dof_idx = int(self.robot_cfg.MujocoDoF_to_DoF[mj_dof])
            pd_torque = self.kps[motor_index] * (
                target_pos[dof_idx] - current_q[dof_idx]
            ) + self.kds[motor_index] * (target_vel[dof_idx] - current_dq[dof_idx])
            actuator_id = int(mj_motor)
            bias_control = (
                self._actuator_bias_control(actuator_id) if self.sim_use_bias_compensation else 0.0
            )
            ctrl[actuator_id] = self._clip_actuator_control(
                actuator_id,
                bias_control + pd_torque,
            )

    def _apply_controlled_joint_bias(self, ctrl: np.ndarray) -> None:
        for mj_motor in self.robot_cfg.MujocoMotors:
            actuator_id = int(mj_motor)
            ctrl[actuator_id] = self._clip_actuator_control(
                actuator_id,
                self._actuator_bias_control(actuator_id) if self.sim_use_bias_compensation else 0.0,
            )

    def _apply_controlled_joint_acceleration(self, ctrl: np.ndarray, command: np.ndarray) -> None:
        """Realize second-order controls through the raw torque actuators.

        The R1 Lite MJCF uses ``motor`` actuators for the torso and arms.  A
        position-PD target is therefore not an actuator position command.  In
        particular, applying the 3000-gain first-order servo to an
        acceleration-integrated target injects anti-damping proportional to
        the measured velocity.  Map the requested joint accelerations through
        MuJoCo's current mass matrix instead and compensate bias/passive force.
        """

        for actuator_id, actuator_force in self._joint_acceleration_actuator_forces(
            command
        ).items():
            ctrl[actuator_id] = self._clip_actuator_control(
                actuator_id,
                actuator_force,
            )

    def _set_controlled_dof_state(
        self, dof_pos: np.ndarray, dof_vel: Optional[np.ndarray] = None
    ) -> None:
        dof_pos = self._clip_target_pos_to_mujoco_limits(dof_pos)
        for dof in self.robot_cfg.DoFs:
            mj_dof = int(self.robot_cfg.DoF_to_MujocoDoF[dof])
            dof_index = int(dof)
            self.data.qpos[mj_dof] = dof_pos[dof_index]
            self.data.qvel[mj_dof] = 0.0 if dof_vel is None else dof_vel[dof_index]

    def _send_control_sim_dynamics(self, command, **kwargs):
        """
        This method integrates the robot's dynamics and updates its state based on modeled dynamics.
        """
        # Get the current state of the robot and apply the commanded dynamics
        # Anchor each simulator-owned velocity step to measured state. This
        # prevents command-state windup when the physical servo lags IK.
        x = self._compose_simulator_dynamics_state()
        x = self._advance_configured_dynamics(x, command, **kwargs)

        # Decompose the state into degree-of-freedom (DoF) positions and velocities
        self.dof_pos_cmd = self.robot_cfg.decompose_state_to_dof_pos(x)
        self.dof_vel_cmd = self.robot_cfg.decompose_state_to_dof_vel(x)

        # Set target positions and velocities for other modes
        self.target_pos = np.zeros_like(self.dof_pos_cmd)
        self.target_vel = np.zeros_like(self.dof_vel_cmd)

        self.target_pos = self.dof_pos_cmd.copy()
        self.target_vel = self.dof_vel_cmd.copy()

        action_info = kwargs.get("action_info", dict())
        # Apply PD control to the robot's actuators
        for _ in range(self.control_decimation):
            ctrl = np.zeros(len(self.data.ctrl), dtype=np.float64)
            if self.stabilize_sim_dynamics_joint_positions:
                self._set_controlled_dof_state(self.target_pos, self.target_vel)
                self._enforce_uncontrolled_actuator_holds()
                self._apply_controlled_joint_bias(ctrl)
            elif self.dynamic_order == 2:
                self._apply_controlled_joint_acceleration(ctrl, command)
            else:
                self._apply_controlled_joint_pd(ctrl, self.target_pos, self.target_vel)
            self._apply_uncontrolled_actuator_holds(ctrl)
            self._apply_r1lite_gripper_control(ctrl, action_info)

            self.data.ctrl[:] = ctrl
            self.counter += 1
            mujoco.mj_step(self.model, self.data)
            if self.stabilize_sim_dynamics_joint_positions:
                self._set_controlled_dof_state(self.target_pos, self.target_vel)
            self._enforce_uncontrolled_actuator_holds()

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
        self._write_r1lite_gripper_configuration(kwargs.get("action_info"))
        mujoco.mj_forward(self.model, self.data)
