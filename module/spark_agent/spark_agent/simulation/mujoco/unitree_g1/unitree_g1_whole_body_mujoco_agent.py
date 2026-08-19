from spark_agent.simulation.mujoco.mujoco_agent import MujocoAgent
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation as R
from spark_robot import RobotConfig, UnitreeG1WholeBodyDynamic1Config
from spark_robot.unitree_g1.actuation import UNITREE_G1_GRIPPER_SPECS


class UnitreeG1WholeBodyMujocoAgent(MujocoAgent):
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

        self._validate_qpos_compatible_config()
        self.use_action_gains = bool(kwargs.get("use_action_gains", True))
        self.feedforward_torque_scale = float(kwargs.get("feedforward_torque_scale", 1.0))
        self.with_hand = self._has_unitree_hand_actuators()
        self.body_only_feedback = bool(kwargs.get("body_only_feedback", True))
        self.body_robot_cfg = (
            UnitreeG1WholeBodyDynamic1Config() if self.with_hand else self.robot_cfg
        )
        graspable_body_names = self._normalize_body_names(kwargs.get("graspable_body_names", None))
        self.graspable_body_names = (
            None if graspable_body_names is None else set(graspable_body_names)
        )
        self.grasp_exclude_body_names = set(
            self._normalize_body_names(kwargs.get("grasp_exclude_body_names", [])) or []
        )
        self.grasp_exclude_body_names.add("robot")
        self.grasp_radius = float(kwargs.get("grasp_radius", 0.10))
        self.left_gripper_goal = False
        self.right_gripper_goal = False
        self.left_gripper_debug_state = False
        self.right_gripper_debug_state = False
        self.left_hand_picked_object = None
        self.right_hand_picked_object = None
        self._init_whole_body_mode()
        if self.with_hand:
            self._init_hand_control()

    def _init_whole_body_mode(self):
        """
        Initializes the whole-body mode configuration for the Mujoco agent.
        This includes setting up target positions and velocities.
        """
        # Initialize target positions and velocities for whole-body control.
        self.target_pos = self.data.qpos.copy()
        self.target_vel = np.zeros_like(self.data.qvel)

    # -------------------------------- Simulation Helpers -------------------------------- #

    def reset(self, agent_reset_info) -> None:
        """Resets the robot to its default position and configuration."""

        # Reset all MuJoCo integration and constraint-solver state, not only
        # qpos/qvel.  In particular qacc_warmstart, ctrl, actuator state and
        # simulation time otherwise leak across benchmark episodes and make an
        # identical learned-locomotion task diverge after several resets.
        mujoco.mj_resetData(self.model, self.data)

        # Call the parent reset method (if any)
        super().reset()

        # Set the default position for the robot's DoFs
        if "reset_dof_pos" in agent_reset_info:
            self.default_dof_pos = agent_reset_info["reset_dof_pos"]
        else:
            self.default_dof_pos = np.array(
                [self.robot_cfg.DefaultDoFVal[dof] for dof in self.robot_cfg.DoFs]
            )

        # Set the robot's DoF positions to the default ones
        self._set_dof_pos(self.default_dof_pos)
        if self.with_hand:
            self.left_gripper_goal = False
            self.right_gripper_goal = False
            self.left_gripper_debug_state = False
            self.right_gripper_debug_state = False
            self.left_hand_picked_object = None
            self.right_hand_picked_object = None
            self._set_hand_qpos(
                self.left_hand_qpos_index, self.left_hand_qvel_index, self.left_hand_open_pos
            )
            self._set_hand_qpos(
                self.right_hand_qpos_index, self.right_hand_qvel_index, self.right_hand_open_pos
            )

        # Step through the simulation to update the robot's configuration
        self._mujoco_step()
        self.target_pos = self.data.qpos.copy()
        self.target_vel = np.zeros_like(self.data.qvel)

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

        body_qpos_fbk, body_qvel_fbk = self._body_qpos_qvel_feedback()

        # Feedback for whole body mode: expose both the MuJoCo state and the
        # canonical 29-body-joint state used by WBT-style policies.
        ret["mujoco_qpos_fbk"] = self.data.qpos.copy()
        ret["mujoco_qvel_fbk"] = self.data.qvel.copy()
        ret["body_qpos_fbk"] = body_qpos_fbk
        ret["body_qvel_fbk"] = body_qvel_fbk
        ret["qpos_fbk"] = self.data.qpos.copy()
        ret["qvel_fbk"] = self.data.qvel.copy()
        self.dof_pos_fbk = self.data.qpos.copy()  # Set commanded position to feedback position
        self.dof_vel_fbk = self.data.qvel.copy()  # Set commanded velocity to feedback velocity

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

        camera_feedback = self.get_camera_feedback()
        if camera_feedback:
            ret["camera"] = camera_feedback

        return ret

    def _body_qpos_qvel_feedback(self):
        if not self.with_hand or not self.body_only_feedback:
            return self.data.qpos.copy(), self.data.qvel.copy()

        body_qpos = np.array(
            [self.body_robot_cfg.DefaultDoFVal[dof] for dof in self.body_robot_cfg.DoFs],
            dtype=float,
        )
        body_qvel = np.zeros(len(self.body_robot_cfg.DoFs) - 1, dtype=float)
        body_qpos[:7] = self.data.qpos[:7]
        body_qvel[:6] = self.data.qvel[:6]

        model_qpos_to_qvel_offset = 1 if self.model.nq - self.model.nv == 1 else 0
        for motor in self.robot_cfg.MujocoMotors:
            if not hasattr(self.body_robot_cfg.MujocoDoFs, motor.name):
                continue
            model_qpos_idx = int(getattr(self.robot_cfg.MujocoDoFs, motor.name))
            body_qpos_idx = int(getattr(self.body_robot_cfg.MujocoDoFs, motor.name))
            model_qvel_idx = model_qpos_idx - model_qpos_to_qvel_offset
            body_qvel_idx = body_qpos_idx - 1

            body_qpos[body_qpos_idx] = self.data.qpos[model_qpos_idx]
            if (
                0 <= model_qvel_idx < self.data.qvel.shape[0]
                and 0 <= body_qvel_idx < body_qvel.shape[0]
            ):
                body_qvel[body_qvel_idx] = self.data.qvel[model_qvel_idx]

        return body_qpos, body_qvel

    def _set_dof_pos(self, dof_pos: np.ndarray) -> None:
        """Sets the Degrees of Freedom (DoF) positions for the robot."""

        qpos = np.asarray(dof_pos, dtype=float).reshape(-1)
        if qpos.shape[0] != self.model.nq:
            raise ValueError(
                f"{self.__class__.__name__} requires a robot config whose DoFs match the MuJoCo qpos length. "
                f"Got {qpos.shape[0]} default DoFs for model nq={self.model.nq}. "
                "Use a whole-body configuration whose DoFs describe the complete MuJoCo qpos."
            )

        # Set the robot's configuration in the model data
        self.data.qpos[:] = qpos
        # Reset velocities, accelerations, and applied forces
        self.data.qvel[:] = 0  # Clear velocities
        self.data.qacc[:] = 0  # Clear accelerations
        self.data.qfrc_applied[:] = 0  # Clear applied forces
        self.data.xfrc_applied[:, :] = 0  # Clear external forces

    def _validate_qpos_compatible_config(self) -> None:
        default_qpos_len = len(self.robot_cfg.DefaultDoFVal)
        if default_qpos_len != self.model.nq:
            raise ValueError(
                f"{self.__class__.__name__} cannot run with {self.robot_cfg.__class__.__name__}: "
                f"config DoF count is {default_qpos_len}, but MuJoCo model nq is {self.model.nq}. "
                "Whole-body MuJoCo configurations must describe the complete simulator qpos."
            )

    def _has_unitree_hand_actuators(self) -> bool:
        hand_actuator_names = tuple(
            joint_name
            for spec in UNITREE_G1_GRIPPER_SPECS.values()
            for joint_name in spec.joint_names
        )
        return all(
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name) >= 0
            for name in hand_actuator_names
        )

    def _normalize_body_names(self, names):
        if names is None:
            return None
        if isinstance(names, str):
            return [names]
        return [str(name) for name in names]

    def _body_id_from_name(self, body_name: str):
        body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, body_name)
        return None if body_id < 0 else int(body_id)

    def _hand_joint_indices(self, joint_names):
        qpos_idx = []
        qvel_idx = []
        ctrl_idx = []
        for joint_name in joint_names:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            actuator_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, joint_name)
            if joint_id < 0 or actuator_id < 0:
                raise KeyError(f"Missing Unitree G1 hand joint or actuator: {joint_name}")
            qpos_idx.append(int(self.model.jnt_qposadr[joint_id]))
            qvel_idx.append(int(self.model.jnt_dofadr[joint_id]))
            ctrl_idx.append(int(actuator_id))
        return (
            np.asarray(qpos_idx, dtype=int),
            np.asarray(qvel_idx, dtype=int),
            np.asarray(ctrl_idx, dtype=int),
        )

    def _init_hand_control(self):
        left_spec = UNITREE_G1_GRIPPER_SPECS["left"]
        right_spec = UNITREE_G1_GRIPPER_SPECS["right"]
        self.left_hand_qpos_index, self.left_hand_qvel_index, self.left_hand_ctrl_index = (
            self._hand_joint_indices(left_spec.joint_names)
        )
        self.right_hand_qpos_index, self.right_hand_qvel_index, self.right_hand_ctrl_index = (
            self._hand_joint_indices(right_spec.joint_names)
        )
        self.left_hand_close_pos = left_spec.target(True)
        self.left_hand_open_pos = left_spec.target(False)
        self.right_hand_close_pos = right_spec.target(True)
        self.right_hand_open_pos = right_spec.target(False)
        self.left_hand_upper_joint_limit = np.asarray(left_spec.upper_limit, dtype=float)
        self.left_hand_lower_joint_limit = np.asarray(left_spec.lower_limit, dtype=float)
        self.right_hand_upper_joint_limit = np.asarray(right_spec.upper_limit, dtype=float)
        self.right_hand_lower_joint_limit = np.asarray(right_spec.lower_limit, dtype=float)
        self.left_hand_kp = left_spec.kp * np.ones(7)
        self.left_hand_kd = left_spec.kd * np.ones(7)
        self.right_hand_kp = right_spec.kp * np.ones(7)
        self.right_hand_kd = right_spec.kd * np.ones(7)

    def _set_hand_qpos(
        self, qpos_idx: np.ndarray, qvel_idx: np.ndarray, target_pos: np.ndarray
    ) -> None:
        self.data.qpos[qpos_idx] = target_pos
        self.data.qvel[qvel_idx] = 0.0

    def _object_pick_allowed(self, body_name: str) -> bool:
        if body_name in self.grasp_exclude_body_names:
            return False
        if self.graspable_body_names is not None and body_name not in self.graspable_body_names:
            return False
        return True

    def _candidate_body_ids(self, target_name=None):
        if target_name is not None:
            body_id = self._body_id_from_name(str(target_name))
            if body_id is None or not self._object_pick_allowed(str(target_name)):
                return []
            return [body_id]

        body_ids = []
        for body_id in range(1, self.model.nbody):
            if self.model.body_parentid[body_id] != 0:
                continue
            body_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body_id)
            if body_name is None or not self._object_pick_allowed(body_name):
                continue
            body_ids.append(body_id)
        return body_ids

    def _body_transform(self, body_name: str) -> np.ndarray:
        body_id = self.model.body(body_name).id
        transform = np.eye(4)
        transform[:3, 3] = self.data.xpos[body_id]
        transform[:3, :3] = self.data.xmat[body_id].reshape(3, 3)
        return transform

    def _apply_hand_pd(
        self, target_pos, qpos_idx, qvel_idx, ctrl_idx, kp, kd, lower, upper
    ) -> None:
        target = np.clip(target_pos, lower, upper)
        tau = self._pd_control(
            target,
            self.data.qpos[qpos_idx],
            kp,
            np.zeros_like(kd),
            self.data.qvel[qvel_idx],
            kd,
        )
        self.data.ctrl[ctrl_idx] = tau

    def _try_pick_object(
        self, close_cmd, picked_attr, thumb_body, wrist_body, target_name=None, radius=None
    ) -> None:
        if not close_cmd or getattr(self, picked_attr) is not None:
            return

        thumb_pos = self.data.xpos[self.model.body(thumb_body).id]
        wrist_transform = self._body_transform(wrist_body)
        pick_radius = self.grasp_radius if radius is None else float(radius)

        for body_id in self._candidate_body_ids(target_name):
            body_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_BODY, body_id)
            if body_name is None:
                continue
            if np.linalg.norm(self.data.xpos[body_id] - thumb_pos) >= pick_radius:
                continue

            object_transform = np.eye(4)
            object_transform[:3, 3] = self.data.xpos[body_id]
            object_transform[:3, :3] = self.data.xmat[body_id].reshape(3, 3)
            setattr(
                self, picked_attr, {body_name: np.linalg.inv(wrist_transform) @ object_transform}
            )
            return

    def _move_picked_object(self, picked_attr, wrist_body) -> None:
        picked_object = getattr(self, picked_attr)
        if picked_object is None:
            return

        wrist_transform = self._body_transform(wrist_body)
        for body_name, relative_transform in picked_object.items():
            self._set_object_pos(body_name, wrist_transform @ relative_transform)

    def _send_control_hand(self, action_info: dict) -> None:
        self.left_gripper_goal = self._resolve_binary_gripper_goal("left", action_info)
        self.right_gripper_goal = self._resolve_binary_gripper_goal("right", action_info)

        self._apply_hand_pd(
            self.left_hand_close_pos if self.left_gripper_goal else self.left_hand_open_pos,
            self.left_hand_qpos_index,
            self.left_hand_qvel_index,
            self.left_hand_ctrl_index,
            self.left_hand_kp,
            self.left_hand_kd,
            self.left_hand_lower_joint_limit,
            self.left_hand_upper_joint_limit,
        )
        self._apply_hand_pd(
            self.right_hand_close_pos if self.right_gripper_goal else self.right_hand_open_pos,
            self.right_hand_qpos_index,
            self.right_hand_qvel_index,
            self.right_hand_ctrl_index,
            self.right_hand_kp,
            self.right_hand_kd,
            self.right_hand_lower_joint_limit,
            self.right_hand_upper_joint_limit,
        )

        self._try_pick_object(
            self.left_gripper_goal,
            "left_hand_picked_object",
            "left_hand_thumb_2_link",
            "left_wrist_yaw_link",
            target_name=action_info.get(
                "left_grasp_target_name", action_info.get("grasp_target_name", None)
            ),
            radius=action_info.get("left_grasp_radius", action_info.get("grasp_radius", None)),
        )
        self._try_pick_object(
            self.right_gripper_goal,
            "right_hand_picked_object",
            "right_hand_thumb_2_link",
            "right_wrist_yaw_link",
            target_name=action_info.get(
                "right_grasp_target_name", action_info.get("grasp_target_name", None)
            ),
            radius=action_info.get("right_grasp_radius", action_info.get("grasp_radius", None)),
        )

        if not self.left_gripper_goal:
            self.left_hand_picked_object = None
        if not self.right_gripper_goal:
            self.right_hand_picked_object = None

        self._move_picked_object("left_hand_picked_object", "left_wrist_yaw_link")
        self._move_picked_object("right_hand_picked_object", "right_wrist_yaw_link")

    def _actuated_qpos_indices(self) -> np.ndarray:
        qpos_indices = []
        for motor in self.robot_cfg.MujocoMotors:
            if not hasattr(self.robot_cfg.MujocoDoFs, motor.name):
                raise KeyError(f"No MuJoCo DoF named {motor.name} for motor {motor}")
            qpos_indices.append(int(getattr(self.robot_cfg.MujocoDoFs, motor.name)))
        return np.asarray(qpos_indices, dtype=int)

    def _actuated_qvel_indices(self) -> np.ndarray:
        qpos_indices = self._actuated_qpos_indices()
        if self.model.nq == self.model.nv:
            return qpos_indices.copy()
        if self.model.nq - self.model.nv == 1:
            return qpos_indices - 1
        raise ValueError(
            f"Unsupported MuJoCo qpos/qvel dimensions for actuated index mapping: "
            f"nq={self.model.nq}, nv={self.model.nv}"
        )

    def _actuated_motor_indices(self) -> np.ndarray:
        return np.asarray([int(idx) for idx in self.robot_cfg.MujocoMotors], dtype=int)

    def _upper_body_actuated_mask(self) -> np.ndarray:
        upper_body_prefixes = (
            "Waist",
            "LeftShoulder",
            "LeftElbow",
            "LeftWrist",
            "RightShoulder",
            "RightElbow",
            "RightWrist",
        )
        return np.asarray(
            [motor.name.startswith(upper_body_prefixes) for motor in self.robot_cfg.MujocoMotors],
            dtype=bool,
        )

    def _coerce_actuated_position_target(self, action_info: dict) -> np.ndarray:
        if "target_actuated_pos" in action_info:
            target = np.asarray(action_info["target_actuated_pos"], dtype=float).reshape(-1)
            if target.shape[0] != len(self.robot_cfg.MujocoMotors):
                raise ValueError(
                    f"target_actuated_pos must have {len(self.robot_cfg.MujocoMotors)} entries; "
                    f"got {target.shape[0]}"
                )
            return target

        if "target_dof_pos" not in action_info:
            raise KeyError(
                "Expected action_info['target_dof_pos'] or action_info['target_actuated_pos']"
            )

        target = np.asarray(action_info["target_dof_pos"], dtype=float).reshape(-1)
        if target.shape[0] == self.model.nq:
            return target[self._actuated_qpos_indices()]
        if target.shape[0] == len(self.robot_cfg.MujocoMotors):
            return target
        if target.shape[0] == len(self.robot_cfg.DoFs):
            values = []
            for motor in self.robot_cfg.MujocoMotors:
                mj_dof = getattr(self.robot_cfg.MujocoDoFs, motor.name)
                dof = self.robot_cfg.MujocoDoF_to_DoF.get(mj_dof, mj_dof)
                values.append(target[int(dof)])
            return np.asarray(values, dtype=float)

        raise ValueError(
            "target_dof_pos must be full qpos, full DoF position, or actuated position; "
            f"got shape {target.shape}"
        )

    def _coerce_actuated_velocity_target(self, action_info: dict) -> np.ndarray:
        if "target_actuated_vel" in action_info:
            target = np.asarray(action_info["target_actuated_vel"], dtype=float).reshape(-1)
        elif "target_dof_vel" in action_info:
            target = np.asarray(action_info["target_dof_vel"], dtype=float).reshape(-1)
            if target.shape[0] == self.model.nv:
                target = target[self._actuated_qvel_indices()]
            elif target.shape[0] == len(self.robot_cfg.DoFs):
                values = []
                for motor in self.robot_cfg.MujocoMotors:
                    mj_dof = getattr(self.robot_cfg.MujocoDoFs, motor.name)
                    dof = self.robot_cfg.MujocoDoF_to_DoF.get(mj_dof, mj_dof)
                    values.append(target[int(dof)])
                target = np.asarray(values, dtype=float)
        else:
            return np.zeros(len(self.robot_cfg.MujocoMotors), dtype=float)

        if target.shape[0] != len(self.robot_cfg.MujocoMotors):
            raise ValueError(
                f"target velocity must have {len(self.robot_cfg.MujocoMotors)} actuated entries; "
                f"got {target.shape[0]}"
            )
        return target

    def _gains_from_action_info(self, action_info: dict):
        if not self.use_action_gains:
            return self.kps, self.kds

        kps = np.asarray(
            action_info.get("target_actuated_kps", action_info.get("motor_kps", self.kps)),
            dtype=float,
        ).reshape(-1)
        kds = np.asarray(
            action_info.get("target_actuated_kds", action_info.get("motor_kds", self.kds)),
            dtype=float,
        ).reshape(-1)
        if kps.shape[0] != len(self.robot_cfg.MujocoMotors):
            kps = self.kps
        if kds.shape[0] != len(self.robot_cfg.MujocoMotors):
            kds = self.kds
        return kps, kds

    def _feedforward_torque_from_action_info(self, action_info: dict) -> np.ndarray:
        feedforward_torque = np.asarray(
            action_info.get(
                "feedforward_torque",
                action_info.get("sol_torque", np.zeros(len(self.robot_cfg.MujocoMotors))),
            ),
            dtype=float,
        ).reshape(-1)
        if feedforward_torque.shape[0] != len(self.robot_cfg.MujocoMotors):
            feedforward_torque = np.zeros(len(self.robot_cfg.MujocoMotors), dtype=float)
        # Dexterous hands add enough distal mass that an otherwise valid body
        # hold can excite the wrists and torso. The released with-hand WBT and
        # Sonic policies already request this compensation explicitly; use the
        # same safe default for direct agent use while preserving an explicit
        # False override and the handless behavior.
        if bool(action_info.get("upper_body_gravity_compensation", self.with_hand)):
            qvel_idx = self._actuated_qvel_indices()
            bias_torque = np.asarray(self.data.qfrc_bias[qvel_idx], dtype=float).reshape(-1)
            if bias_torque.shape == feedforward_torque.shape:
                upper_body_mask = self._upper_body_actuated_mask()
                feedforward_torque[upper_body_mask] += bias_torque[upper_body_mask]
        return self.feedforward_torque_scale * feedforward_torque

    def _send_control_position_target(self, action_info: dict):
        target_actuated_pos = self._coerce_actuated_position_target(action_info)
        target_actuated_vel = self._coerce_actuated_velocity_target(action_info)
        kps, kds = self._gains_from_action_info(action_info)
        feedforward_torque = self._feedforward_torque_from_action_info(action_info)

        qpos_idx = self._actuated_qpos_indices()
        qvel_idx = self._actuated_qvel_indices()
        motor_idx = self._actuated_motor_indices()

        self.target_pos = self.data.qpos.copy()
        self.target_pos[qpos_idx] = target_actuated_pos
        self.target_vel = np.zeros_like(self.data.qvel)
        self.target_vel[qvel_idx] = target_actuated_vel

        self.dof_pos_cmd = self.target_pos.copy()
        self.dof_vel_cmd = self.target_vel.copy()
        self.dof_acc_cmd = np.zeros_like(self.target_vel)

        for _ in range(self.control_decimation):
            tau = self._pd_control(
                target_actuated_pos,
                self.data.qpos[qpos_idx],
                kps,
                target_actuated_vel,
                self.data.qvel[qvel_idx],
                kds,
            )
            self.data.ctrl[motor_idx] = feedforward_torque + tau
            if self.with_hand:
                self._send_control_hand(action_info)
            self.counter += 1
            mujoco.mj_step(self.model, self.data)

    def _send_control_whole_body_mode(self, command, action_info):
        """
        This method applies control signals directly to the whole body of the robot.
        """
        if "target_dof_pos" in action_info or "target_actuated_pos" in action_info:
            self._send_control_position_target(action_info)
            return

        missing = [key for key in ("sol_acc", "sol_torque") if key not in action_info]
        if missing:
            raise ValueError(
                "Unitree whole-body simulator dynamics requires either "
                "action_info['target_actuated_pos'] or both WBT outputs "
                "action_info['sol_acc'] and action_info['sol_torque']; "
                f"missing {missing}"
            )

        # Apply the given command to all actuators (joints)
        target_dof_acc = action_info["sol_acc"][6:]
        target_torque = action_info["sol_torque"]

        dof_pos_fbk = self.data.qpos.copy()[7:]
        dof_vel_fbk = self.data.qvel.copy()[6:]
        target_dof_vel = dof_vel_fbk + target_dof_acc * self.dt
        target_dof_pos = dof_pos_fbk + 0.5 * target_dof_acc * (self.dt) ** 2
        ctrl = (
            target_torque
            + self.kps * (target_dof_pos - dof_pos_fbk)
            + self.kds * (target_dof_vel - dof_vel_fbk)
        )
        self.data.ctrl = ctrl

        self.counter += 1
        mujoco.mj_step(self.model, self.data)

    def _send_control_sim_dynamics(self, command, **kwargs):
        """
        This method integrates the robot's dynamics and updates its state based on modeled dynamics.
        """
        self._send_control_whole_body_mode(command, kwargs.get("action_info", {}))

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

        """
        Compose the feedback state representation based on DoF position and velocity feedback.

        Returns:
            np.ndarray: Composed feedback state.
        """
        return self.robot_cfg.compose_state_from_dof(self.dof_pos_fbk, self.dof_vel_fbk)
