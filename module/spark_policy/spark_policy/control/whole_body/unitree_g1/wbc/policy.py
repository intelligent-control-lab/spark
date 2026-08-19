from spark_policy.core.policy import BasePolicy
from spark_robot import RobotKinematics, RobotConfig
import numpy as np
from scipy.spatial.transform import Rotation as R
from spark_utils import pos_quat_to_transformation, rpy2quat, transformation_to_pos_quat, quat2rpy
from spark_policy.safety import SecondOrderCollisionSafetyIndex
from spark_utils import initialize_class
import proxsuite

import time


class UnitreeG1WBCPolicy(BasePolicy):
    def __init__(self, robot_cfg: RobotConfig, robot_kinematics: RobotKinematics, **kwargs) -> None:
        super().__init__(robot_cfg, robot_kinematics)

        self.safety_index = SecondOrderCollisionSafetyIndex(
            robot_kinematics=self.robot_kinematics,
            robot_cfg=robot_kinematics.robot_cfg,
            phi_n=1.0,
            phi_k=0.1,
            min_distance={"environment": 0.05, "self": 0.01},
            enable_self_collision=False,
        )
        self.eta_default = 1.0

        self.contact_frames = [
            "fr_left_foot",
            "br_left_foot",
            "fl_left_foot",
            "bl_left_foot",
            "fr_right_foot",
            "br_right_foot",
            "fl_right_foot",
            "bl_right_foot",
        ]
        # zmp_edges represents the edges of the support polygon in x-y plane
        # Each row represents a set of parameters a, b and c to define the edge
        # a * x + b * y + c  <= 0
        self.zmp_edges = np.array(
            [
                [1, 0, -0.05],  # x <= 0.01
                [-1, 0, -0.05],  # x >= -0.01
                [0, 1, -0.05],  # y <= 0.01
                [0, -1, -0.05],  # y >= -0.01
            ]
        )

        self.num_contacts = len(self.contact_frames)  # Number of contact points
        self.num_ctrl = len(self.robot_cfg.Control)
        self.num_dof = len(self.robot_cfg.DoFs) - 1  # Floating base + actuated joints
        self.num_slack = 6
        self.num_decision_vars = (
            self.num_dof + 3 * self.num_contacts + self.num_ctrl + self.num_slack
        )  # Decision variables: [ddq, tau, lambda]
        self.torque_limits = 100.0 * np.ones(self.num_ctrl)  # Example max torque limits
        self.acceleration_limits = np.ones(self.num_dof)  # Example acceleration limits
        self.acceleration_limits[:6] = kwargs.get("floating_base_acceleration_limit", 3.0)
        self.acceleration_limits[-self.num_ctrl :] = [
            self.robot_cfg.ControlLimit[ctrl_idx] for ctrl_idx in self.robot_cfg.Control
        ]  # Example max acceleration limits
        self.friction_coeff = 0.5  # Example friction coefficient
        self.contact_constraint_warmup_steps = max(
            0, int(kwargs.get("contact_constraint_warmup_steps", 0))
        )
        self.zmp_constraint_warmup_steps = max(
            0, int(kwargs.get("zmp_constraint_warmup_steps", 100))
        )
        self.track_base_goal = kwargs.get("track_base_goal", True)
        self.base_position_track_axes = np.array(
            kwargs.get("base_position_track_axes", [False, False, True]), dtype=bool
        )
        self.base_orientation_track_axes = np.array(
            kwargs.get("base_orientation_track_axes", [False, True, False]), dtype=bool
        )
        self.base_linear_weight = kwargs.get("base_linear_weight", 50.0)
        self.base_angular_weight = kwargs.get("base_angular_weight", 20.0)
        self.joint_acceleration_weight = kwargs.get("joint_acceleration_weight", 1.0)
        self.posture_acceleration_weight = kwargs.get("posture_acceleration_weight", 8.0)
        self.floating_base_acceleration_weight = kwargs.get(
            "floating_base_acceleration_weight", 0.0
        )
        self.base_height_min = kwargs.get("base_height_min", 0.65)
        self.base_height_max = kwargs.get("base_height_max", 1.05)
        self.base_pitch_min = kwargs.get("base_pitch_min", -0.10)
        self.base_pitch_max = kwargs.get("base_pitch_max", 0.10)
        if self.base_pitch_min > self.base_pitch_max:
            raise ValueError("base_pitch_min must be less than or equal to base_pitch_max")
        self.standing_base_height = kwargs.get("standing_base_height", 0.793)
        self.squat_base_height = kwargs.get("squat_base_height", self.base_height_min)
        self.base_reference_dt = kwargs.get("base_reference_dt", 0.002)
        self.base_linear_rate_limit = self._axis_limit(kwargs.get("base_linear_rate_limit", 1.0))
        self.base_angular_rate_limit = self._axis_limit(kwargs.get("base_angular_rate_limit", 0.5))
        self.base_velocity_des = np.zeros(6)  # Desired base velocity
        self.base_pose_des = np.array(
            kwargs.get("base_pose_des", [0.0, 0.0, 0.793, 0.0, 0.0, 0.0]), dtype=float
        )
        self.base_acceleration_des = np.zeros(6)  # Desired base acceleration
        self.base_angular_kp = kwargs.get(
            "base_angular_kp", 100
        )  # Proportional gain for angular velocity
        self.base_angular_kd = kwargs.get(
            "base_angular_kd", 30
        )  # Derivative gain for angular velocity
        self.base_linear_kp = kwargs.get(
            "base_linear_kp", 100
        )  # Proportional gain for linear velocity
        self.base_linear_kd = kwargs.get(
            "base_linear_kd", 30
        )  # Derivative gain for linear velocity
        self.direction = -1
        self.ref_ddq = np.zeros(self.num_dof)
        self.last_ddq = np.zeros(self.num_dof)
        self.last_contact_force = np.zeros(3 * self.num_contacts)
        self.last_torque = np.zeros(self.num_ctrl)
        self.nominal_dof_pos = np.array(
            [self.robot_cfg.DefaultDoFVal[dof] for dof in self.robot_cfg.DoFs], dtype=float
        )
        self.nominal_actuated_pos = self.nominal_dof_pos[7:].copy()
        self.posture_actuated_indices = np.asarray(
            kwargs.get("posture_actuated_indices", list(range(15))), dtype=int
        )
        self.posture_kp = kwargs.get("posture_kp", 80.0)
        self.posture_kd = kwargs.get("posture_kd", 16.0)
        squat_posture_offset = kwargs.get("squat_posture_offset", None)
        if squat_posture_offset is None:
            self.squat_posture_offset = np.zeros_like(self.nominal_actuated_pos)
            self.squat_posture_offset[[0, 6]] = -0.65
            self.squat_posture_offset[[3, 9]] = 1.20
            self.squat_posture_offset[[4, 10]] = -0.55
        else:
            self.squat_posture_offset = np.asarray(squat_posture_offset, dtype=float).reshape(
                self.nominal_actuated_pos.shape
            )
        self.max_solver_failure_prints = max(0, int(kwargs.get("max_solver_failure_prints", 10)))
        self.solver_failure_print_interval = max(
            1, int(kwargs.get("solver_failure_print_interval", 100))
        )
        self.solver_failure_count = 0
        self.use_safe = True

    @staticmethod
    def _axis_limit(value):
        value = np.asarray(value, dtype=float)
        if value.ndim == 0:
            value = np.repeat(float(value), 3)
        return np.maximum(value.reshape(3), 0.0)

    def _update_base_reference_from_goal(self, task_info: dict):
        if not self.track_base_goal:
            return

        goal_teleop = task_info.get("goal_teleop", {})
        base_goal = goal_teleop.get("base", None)
        if base_goal is None:
            return

        base_goal = np.asarray(base_goal, dtype=float).reshape(-1, 4, 4)[0]
        target_pose = self.base_pose_des.copy()
        target_pose[:3] = base_goal[:3, 3]
        target_pose[2] = np.clip(
            target_pose[2],
            self.base_height_min,
            self.base_height_max,
        )
        target_pose[3:] = R.from_matrix(base_goal[:3, :3]).as_euler("xyz")
        target_pose[4] = np.clip(
            target_pose[4],
            self.base_pitch_min,
            self.base_pitch_max,
        )

        reference_dt = max(float(self.base_reference_dt), 1e-6)
        linear_step = self.base_linear_rate_limit * reference_dt
        angular_step = self.base_angular_rate_limit * reference_dt
        linear_delta = np.clip(target_pose[:3] - self.base_pose_des[:3], -linear_step, linear_step)
        self.base_pose_des[:3] += linear_delta
        angle_delta = (target_pose[3:] - self.base_pose_des[3:] + np.pi) % (2.0 * np.pi) - np.pi
        angular_delta = np.clip(angle_delta, -angular_step, angular_step)
        self.base_pose_des[3:] += angular_delta
        self.base_velocity_des[:3] = linear_delta / reference_dt
        self.base_velocity_des[3:] = angular_delta / reference_dt
        self.base_acceleration_des[:] = 0.0

    def _get_base_pose_feedback(self):
        frame = self.robot_kinematics.get_frame_transformation("pelvis_link_1")
        return np.concatenate(
            (
                frame[:3, 3],
                R.from_matrix(frame[:3, :3]).as_euler("xyz"),
            )
        )

    def _get_actuated_posture_target(self):
        height_span = max(self.standing_base_height - self.squat_base_height, 1e-6)
        squat_ratio = np.clip(
            (self.standing_base_height - self.base_pose_des[2]) / height_span, 0.0, 1.0
        )
        return self.nominal_actuated_pos + squat_ratio * self.squat_posture_offset

    def _init_qp(self):

        if self.use_safe and not self.use_wbc:
            self.weights_functions = [
                (self.formulate_angular_acceleration_task, self.base_angular_weight),
                (self.formulate_linear_acceleration_task, self.base_linear_weight),
                # (self.formulate_contact_force_task, 10),
                (self.formulate_joint_acceleration_task, self.joint_acceleration_weight),
                (self.formulate_posture_acceleration_task, self.posture_acceleration_weight),
                # (self.formulate_delta_joint_acceleration_task, 20),
                # (self.formulate_delta_joint_torque_task, 1),
                # (self.formulate_zmp_task, 10000),
                # (self.formulate_slack_variable_task, 1)
            ]

            self.constraints_functions = [
                # (self.formulate_floating_base_eom_task, self.num_dof, 0),
                # (self.formulate_torque_limits_task, 0, 2 * self.num_ctrl),
                (self.formulate_acceleration_limits_task, 0, 2 * self.num_dof),
                # (self.formulate_friction_cone_task, 0, 5 * self.num_contacts),
                (self.formulate_safety_task, 0, 6),
                # (self.formulate_contact_point_acceleration_task, 3 * self.num_contacts, 0),
                # (self.formulate_zmp_task, 0, self.zmp_edges.shape[0]),
            ]

        elif self.use_safe and self.use_wbc:
            self.weights_functions = [
                (self.formulate_angular_acceleration_task, self.base_angular_weight),
                (self.formulate_linear_acceleration_task, self.base_linear_weight),
                # (self.formulate_contact_force_task, 10),
                (self.formulate_joint_acceleration_task, self.joint_acceleration_weight),
                (self.formulate_posture_acceleration_task, self.posture_acceleration_weight),
                # (self.formulate_delta_joint_acceleration_task, 20),
                # (self.formulate_delta_joint_torque_task, 1),
                # (self.formulate_zmp_task, 10000),
                # (self.formulate_slack_variable_task, 1)
            ]
            self.constraints_functions = [
                (self.formulate_floating_base_eom_task, self.num_dof, 0),
                (self.formulate_torque_limits_task, 0, 2 * self.num_ctrl),
                (self.formulate_acceleration_limits_task, 0, 2 * self.num_dof),
                (self.formulate_friction_cone_task, 0, 5 * self.num_contacts),
                (self.formulate_safety_task, 0, 6),
                (self.formulate_contact_point_acceleration_task, 3 * self.num_contacts, 0),
                # (self.formulate_zmp_task, 0, self.zmp_edges.shape[0]),
            ]

        elif self.use_wbc and not self.use_safe:
            self.weights_functions = [
                (self.formulate_angular_acceleration_task, self.base_angular_weight),
                (self.formulate_linear_acceleration_task, self.base_linear_weight),
                # (self.formulate_contact_force_task, 10),
                (self.formulate_joint_acceleration_task, self.joint_acceleration_weight),
                (self.formulate_posture_acceleration_task, self.posture_acceleration_weight),
                # (self.formulate_delta_joint_acceleration_task, 20),
                # (self.formulate_delta_joint_torque_task, 1),
                # (self.formulate_zmp_task, 10000),
                # (self.formulate_slack_variable_task, 1)
            ]
            self.constraints_functions = [
                (self.formulate_floating_base_eom_task, self.num_dof, 0),
                (self.formulate_torque_limits_task, 0, 2 * self.num_ctrl),
                (self.formulate_acceleration_limits_task, 0, 2 * self.num_dof),
                (self.formulate_friction_cone_task, 0, 5 * self.num_contacts),
                # (self.formulate_safety_task, 0, 6 ),
                (self.formulate_contact_point_acceleration_task, 3 * self.num_contacts, 0),
                # (self.formulate_zmp_task, 0, self.zmp_edges.shape[0]),
            ]

        self.num_equality_constraints = sum(
            [constraint[1] for constraint in self.constraints_functions]
        )
        self.num_inequality_constraints = sum(
            [constraint[2] for constraint in self.constraints_functions]
        )

        self.qp = proxsuite.proxqp.dense.QP(
            self.num_decision_vars, self.num_equality_constraints, self.num_inequality_constraints
        )

        # self.qp_init = False
        self.prev_sol = np.zeros((self.num_decision_vars,))
        # self.qp.settings.verbose = True                # Print solver info each iteration
        self.qp.settings.max_iter = 1000  # Maximum number of iterations
        self.qp.settings.eps_abs = 1e-5  # Absolute tolerance
        self.qp.settings.eps_rel = 1e-5  # Relative tolerance
        self.qp.settings.compute_timings = True  # Track solve timing (optional)

    def formulate_floating_base_eom_task(self):
        """
        Formulate the floating base equations of motion task."""

        s = np.zeros((self.num_ctrl, self.num_dof))
        s[:, 6:] = np.eye(self.num_ctrl)

        a = np.zeros((self.num_dof, self.num_decision_vars))
        a[:, : self.num_dof] = self.M
        a[:, self.num_dof : self.num_dof + 3 * self.num_contacts] = -self.J_c.T
        a[
            :,
            self.num_dof + 3 * self.num_contacts : self.num_dof
            + 3 * self.num_contacts
            + self.num_ctrl,
        ] = -s.T

        # a = np.hstack([
        #     self.M,
        #     -self.J_c.T,
        #     -s.T,
        # ])

        b = -self.nle

        return a, b, np.empty((0, self.num_decision_vars)), np.empty(0)

    def formulate_torque_limits_task(self):
        """
        Formulate the torque limits constraint.
        """
        d = np.zeros((2 * self.num_ctrl, self.num_decision_vars))

        identity_block = np.eye(self.num_ctrl)
        start_col = self.num_dof + 3 * self.num_contacts

        d[: self.num_ctrl, start_col : start_col + self.num_ctrl] = identity_block
        d[self.num_ctrl :, start_col : start_col + self.num_ctrl] = -identity_block

        f = np.zeros(2 * self.num_ctrl)
        f[: self.num_ctrl] = self.torque_limits
        f[self.num_ctrl :] = self.torque_limits
        return np.empty((0, self.num_decision_vars)), np.empty(0), d, f

    def formulate_acceleration_limits_task(self):
        """
        Formulate the acceleration limits constraint.
        """
        d = np.zeros((2 * self.num_dof, self.num_decision_vars))

        identity_block = np.eye(self.num_dof)
        start_col = 0

        d[: self.num_dof, start_col : start_col + self.num_dof] = identity_block
        d[self.num_dof :, start_col : start_col + self.num_dof] = -identity_block

        f = np.zeros(2 * self.num_dof)
        f[: self.num_dof] = self.acceleration_limits
        f[self.num_dof :] = self.acceleration_limits
        return np.empty((0, self.num_decision_vars)), np.empty(0), d, f

    def formulate_friction_cone_task(self):
        """
        Formulate the friction cone constraint.
        """
        num_three_dof_contacts = self.num_contacts
        # NOTE Only consider when two feet are in contact
        num_contacts = self.num_contacts
        contact_flag = np.ones(num_three_dof_contacts, dtype=bool)
        # Initialize matrix 'a' with zeros
        a = np.zeros((3 * (num_three_dof_contacts - num_contacts), self.num_decision_vars))

        # Populate matrix 'a' based on contactFlag_
        j = 0
        for i in range(num_three_dof_contacts):
            if not contact_flag[i]:
                a[3 * j : 3 * (j + 1), self.num_dof + 3 * i : self.num_dof + 3 * (i + 1)] = np.eye(
                    3
                )
                j += 1

        # Initialize vector 'b' with zeros
        b = np.zeros(a.shape[0])

        # Define friction pyramid matrix
        friction_pyramid = np.array(
            [
                [0, 0, -1],
                [1, 0, -self.friction_coeff],
                [-1, 0, -self.friction_coeff],
                [0, 1, -self.friction_coeff],
                [0, -1, -self.friction_coeff],
            ]
        )

        # Initialize matrix 'd' with zeros
        d = np.zeros(
            (5 * num_contacts + 3 * (num_three_dof_contacts - num_contacts), self.num_decision_vars)
        )

        # Populate matrix 'd' based on contactFlag_
        j = 0
        for i in range(num_three_dof_contacts):
            if contact_flag[i]:
                d[5 * j : 5 * (j + 1), self.num_dof + 3 * i : self.num_dof + 3 * (i + 1)] = (
                    friction_pyramid
                )
                j += 1

        # Initialize vector 'f' with zeros
        f = np.zeros(d.shape[0])

        return a, b, d, f

    def formulate_contact_force_task(self):
        """
        Formulate the contact force task.
        """
        # Initialize matrix 'a' with zeros
        a = np.zeros((3 * self.num_contacts, self.num_decision_vars))
        b = np.zeros(a.shape[0])

        for j in range(self.num_contacts):
            a[3 * j : 3 * (j + 1), self.num_dof + 3 * j : self.num_dof + 3 * (j + 1)] = np.eye(3)
            if j % 2 == 0:
                b[3 * j : 3 * (j + 1)] = np.array([0.0, 0.0, 50.0])
            else:
                b[3 * j : 3 * (j + 1)] = np.array([0.0, 0.0, 32.0])

        return a, b, np.empty((0, self.num_decision_vars)), np.empty(0)

    def formulate_contact_point_acceleration_task(self):
        """
        Formulate the contact point task.
        """
        # Initialize matrix 'a' with zeros
        a = np.zeros((3 * self.num_contacts, self.num_decision_vars))
        b = np.zeros(a.shape[0])
        for j in range(self.num_contacts):
            a[3 * j : 3 * (j + 1), : self.num_dof] = self.J_c[3 * j : 3 * (j + 1), : self.num_dof]
            b[3 * j : 3 * (j + 1)] = -self.dJ_c[3 * j : 3 * (j + 1), : self.num_dof] @ self.qvel_fbk

        if self.time_step < self.contact_constraint_warmup_steps:
            a *= 0.0
            b *= 0.0

        return a, b, np.empty((0, self.num_decision_vars)), np.empty(0)

    def formulate_zmp_task(self):
        """
        Formulate the ZMP task.
        """

        num_edges = self.zmp_edges.shape[0]
        d = np.zeros((num_edges, self.num_decision_vars))
        f = np.zeros(num_edges)

        # Precompute S_x, S_y, S_z from contact locations
        S_x = np.zeros((3 * self.num_contacts, 1))
        S_y = np.zeros((3 * self.num_contacts, 1))
        S_z = np.zeros((3 * self.num_contacts, 1))
        for c in range(self.num_contacts):
            S_x[c * 3 + 2] = self.frame_c[c, 0]  # p_x of contact
            S_y[c * 3 + 2] = self.frame_c[c, 1]  # p_y of contact
            S_z[c * 3 + 2] = 1.0  # f_z term

        # Build inequality for each edge
        for j in range(num_edges):
            normal_x, normal_y, offset = self.zmp_edges[j]
            S_proj = normal_x * S_x + normal_y * S_y + offset * S_z
            d[j, self.num_dof : self.num_dof + 3 * self.num_contacts] = S_proj.flatten()
            f[j] = 0.0

        if self.time_step < self.zmp_constraint_warmup_steps:
            d *= 0.0
            f *= 0.0

        return np.empty((0, self.num_decision_vars)), np.empty(0), d, f

    def formulate_angular_acceleration_task(self):
        """
        Formulate the base acceleration weight.
        """
        # Initialize matrices and vectors
        a = np.zeros((3, self.num_decision_vars))
        b = np.zeros(3)

        frame_name = "pelvis_link_1"
        J_frame = self.robot_kinematics.get_jacobian(frame_name)
        dJ_frame = self.robot_kinematics.get_jacobian_dot(frame_name)

        a[:, : self.num_dof] = J_frame[3:, : self.num_dof]

        v = J_frame[3:, :] @ self.qvel_fbk
        rot = self.robot_kinematics.get_frame_transformation(frame_name)[:3, :3]

        target_v = self.base_velocity_des[-3:]
        target_a = self.base_acceleration_des[-3:]
        target_euler = self.base_pose_des[-3:]

        target_rot = R.from_euler("xyz", target_euler).as_matrix()

        R_error = target_rot @ rot.T
        error = (
            np.array(
                [
                    R_error[2, 1] - R_error[1, 2],
                    R_error[0, 2] - R_error[2, 0],
                    R_error[1, 0] - R_error[0, 1],
                ]
            )
            * 0.5
        )

        # Compute b vector
        b = (
            target_a
            + self.base_angular_kp * error
            + self.base_angular_kd * (target_v - v)
            - dJ_frame[3:6, : self.num_dof] @ self.qvel_fbk
        )

        a = a[self.base_orientation_track_axes]
        b = b[self.base_orientation_track_axes]

        return a, b, np.empty((0, 0)), np.empty(0)

    def formulate_linear_acceleration_task(self):
        """
        Formulate the base linear acceleration task.
        """
        # Initialize matrices and vectors
        a = np.zeros((3, self.num_decision_vars))
        b = np.zeros(3)

        frame_name = "pelvis_link_1"
        J_frame = self.robot_kinematics.get_jacobian(frame_name)
        dJ_frame = self.robot_kinematics.get_jacobian_dot(frame_name)

        a[:, : self.num_dof] = J_frame[:3, : self.num_dof]

        p = self.robot_kinematics.get_frame_transformation(frame_name)[:3, 3]
        v = J_frame[:3, :] @ self.qvel_fbk

        target_p = self.base_pose_des[:3]
        target_v = self.base_velocity_des[:3]
        target_a = self.base_acceleration_des[:3]

        b = (
            target_a
            + self.base_linear_kp * (target_p - p)
            + self.base_linear_kd * (target_v - v)
            - dJ_frame[:3, : self.num_dof] @ self.qvel_fbk
        )

        a = a[self.base_position_track_axes]
        b = b[self.base_position_track_axes]

        return a, b, np.empty((0, 0)), np.empty(0)

    def formulate_joint_acceleration_task(self):
        """
        Formulate the joint acceleration task.
        """
        # Initialize matrices and vectors
        a = np.zeros((self.num_dof, self.num_decision_vars))
        b = np.zeros(self.num_dof)

        # Extract relevant block from base_j
        weights = np.ones(self.num_dof)
        weights[:6] = self.floating_base_acceleration_weight
        weights[6:-17] *= 10.0
        a[:, : self.num_dof] = np.diag(weights)

        # Compute b vector
        b = self.ref_ddq

        return a, b, np.empty((0, 0)), np.empty(0)

    def formulate_posture_acceleration_task(self):
        """
        Formulate a lower-body posture task for base-height tracking.
        """
        if self.posture_acceleration_weight <= 0.0 or self.posture_actuated_indices.size == 0:
            return np.empty((0, self.num_decision_vars)), np.empty(0), np.empty((0, 0)), np.empty(0)

        qpos_actuated = self.qpos_fbk[7:]
        qvel_actuated = self.qvel_fbk[6:]
        target_qpos = self._get_actuated_posture_target()

        actuated_indices = self.posture_actuated_indices
        qvel_indices = 6 + actuated_indices

        a = np.zeros((len(actuated_indices), self.num_decision_vars))
        b = np.zeros(len(actuated_indices))
        a[:, qvel_indices] = np.eye(len(actuated_indices))
        b[:] = (
            self.posture_kp * (target_qpos[actuated_indices] - qpos_actuated[actuated_indices])
            - self.posture_kd * qvel_actuated[actuated_indices]
        )

        return a, b, np.empty((0, 0)), np.empty(0)

    def formulate_delta_joint_acceleration_task(self):
        """
        Formulate the joint acceleration task.
        """
        # Initialize matrices and vectors
        a = np.zeros((self.num_dof, self.num_decision_vars))
        b = np.zeros(self.num_dof)

        # Extract relevant block from base_j
        weights = np.ones(self.num_dof)
        weights[6:-17] *= 10.0
        a[:, : self.num_dof] = np.diag(weights)

        # Compute b vector
        b = self.last_ddq

        return a, b, np.empty((0, 0)), np.empty(0)

    def formulate_joint_torque_task(self):
        """
        Formulate the joint torque task.
        """
        # Initialize matrices and vectors
        a = np.zeros((self.num_ctrl, self.num_decision_vars))
        b = np.zeros(self.num_ctrl)

        # Extract relevant block from base_j
        a[
            :,
            self.num_dof + 3 * self.num_contacts : self.num_dof
            + 3 * self.num_contacts
            + self.num_ctrl,
        ] = np.eye(self.num_ctrl)

        # Compute b vector
        b = np.zeros(self.num_ctrl)

        return a, b, np.empty((0, 0)), np.empty(0)

    def formulate_delta_joint_torque_task(self):
        """
        Formulate the joint torque task.
        """
        # Initialize matrices and vectors
        a = np.zeros((self.num_ctrl, self.num_decision_vars))
        b = np.zeros(self.num_ctrl)

        # Extract relevant block from base_j
        a[
            :,
            self.num_dof + 3 * self.num_contacts : self.num_dof
            + 3 * self.num_contacts
            + self.num_ctrl,
        ] = np.eye(self.num_ctrl)

        # Compute b vector
        b = self.last_torque

        return a, b, np.empty((0, 0)), np.empty(0)

    def eta_fn(self, phi):
        eta_values = np.where(
            (self.safety_index.phi_mask.reshape(-1, 1) > 0) & (phi.reshape(-1, 1) >= 0),
            np.ones((self.safety_index.num_constraint, 1)) * self.eta_default,
            # np.ones((self.safety_index.num_constraint, 1)) * self.eta_default * phi.reshape(-1, 1),
            np.ones((self.safety_index.num_constraint, 1)) * -np.inf,
        )
        return eta_values.reshape(-1, 1)

    def formulate_safety_task(self):
        """
        Formulate the safety task.
        """
        use_slack = False
        a = np.empty((0, self.num_decision_vars))
        b = np.empty(0)
        if not use_slack:
            # Initialize matrices and vectors
            d = np.zeros((self.phi.shape[0], self.num_decision_vars))
            f = np.zeros(self.phi.shape[0])
            # print(self.phi[self.safety_index.phi_mask].max())

            d[:, : self.num_dof] = self.Lg

            # Compute f vector
            f = -self.Lf.reshape(
                -1,
            ) - self.eta_fn(self.phi).reshape(
                -1,
            )

        else:
            d = np.zeros((self.phi.shape[0] + self.num_slack, self.num_decision_vars))
            f = np.zeros(self.phi.shape[0] + self.num_slack)

            d[: self.phi.shape[0], : self.num_dof] = self.Lg
            f[: self.phi.shape[0]] = -self.Lf.reshape(
                -1,
            ) - self.eta_fn(self.phi).reshape(
                -1,
            )

            d[-self.num_slack :, -self.num_slack :] = -np.eye(self.num_slack)
            f[-self.num_slack :] = np.inf * np.ones(self.num_slack)

        return a, b, d, f

    def formulate_slack_variable_task(self):
        a = np.zeros((self.num_slack, self.num_decision_vars))
        b = np.zeros(self.num_slack)
        a[:, -self.num_slack :] = 1e3 * np.eye(self.num_slack)

        return a, b, np.empty((0, 0)), np.empty(0)

    def formulate_constraints(self):
        """
        Formulate the constraints for the optimization problem."""

        constraints = [fun() for fun, _, _ in self.constraints_functions]

        # Equality constraints aX = b
        a = np.vstack([constraint[0] for constraint in constraints])  # (35, 76)
        b = np.hstack([constraint[1] for constraint in constraints])  # (35,)
        # # Inequality constraints dX <= f
        d = np.vstack([constraint[2] for constraint in constraints])  # (78, 76)
        f = np.hstack([constraint[3] for constraint in constraints])  # (78,)

        # Combine all constraints
        A = np.vstack([a, d])  # A: (113, 76)
        lbA = np.hstack([b, -np.inf * np.ones(f.shape[0])])  # lbA: (113,)
        ubA = np.hstack([b, f])  # ubA: (113,)

        return A, lbA, ubA, a, d, b, f

    def formulate_weights(self):
        """
        Formulate the weights for the optimization problem.
        """
        weights = [(fun(), weight) for fun, weight in self.weights_functions]

        a = np.vstack([weight[0][0] * weight[1] for weight in weights])
        b = np.hstack([weight[0][1] * weight[1] for weight in weights])

        P = a.T @ a
        q = -a.T @ b

        return P, q

    def calculate_zmp(self):
        """
        Calculate the Zero Moment Point (ZMP) based on the contact forces.
        """
        # Get the contact forces
        contact_forces = self.prev_sol[self.num_dof : self.num_dof + 3 * self.num_contacts].reshape(
            -1, 3
        )

        # Get the positions of the contact points
        contact_positions = np.array(
            [
                self.robot_kinematics.get_frame_transformation(frame_name)[:3, 3]
                for frame_name in self.contact_frames
            ]
        )

        # Calculate the ZMP
        zmp = np.sum(
            contact_positions[:, :2] * contact_forces[:, 2][:, np.newaxis], axis=0
        ) / np.sum(contact_forces[:, 2])

        return np.concatenate((zmp, np.zeros(1)))  # Add a zero for the z-coordinate of ZMP

    def solve_wbc(self):
        self._init_qp()

        self.M, self.nle, self.J_c, self.dJ_c, self.frame_c = self.robot_kinematics.get_dynamics(
            self.qpos_fbk, self.qvel_fbk, self.contact_frames
        )
        # J_contact = self.robot_kinematics.get_contact_jacobian(self.qpos_fbk, self.qvel_fbk, self.contact_frames)
        # import ipdb;ipdb.set_trace()
        A, lbA, ubA, a, d, b, f = self.formulate_constraints()
        P, q = self.formulate_weights()
        self.qp.init(P, q, a, b, d, None, f)
        # if not self.qp_init: # in first iteration we initialize the model
        #     self.qp.init(P, q, a, b, d, None, f)
        #     self.qp_init = True
        # else: # otherwise, we update the model
        #     self.qp.update(P, q, a, b, d, None, f)

        # Let's solve the QP
        self.qp.solve()

        status = self.qp.results.info.status
        if status == proxsuite.proxqp.QPSolverOutput.PROXQP_SOLVED:
            # print("Solver succeeded!")
            self.prev_sol = np.copy(self.qp.results.x)

            sol_acc = self.qp.results.x[: self.num_dof].flatten()
            sol_contact_force = self.qp.results.x[
                self.num_dof : self.num_dof + 3 * self.num_contacts
            ].flatten()
            sol_torque = self.qp.results.x[
                self.num_dof + 3 * self.num_contacts : self.num_dof
                + 3 * self.num_contacts
                + self.num_ctrl
            ].flatten()
            # print("ZMP:", np.round(zmp, 3))

            constarint_violation = (
                self.Lf + self.Lg @ sol_acc.reshape(-1, 1) + self.eta_fn(self.phi.reshape(-1, 1))
            )
            constarint_violation[constarint_violation < 0] = 0
            # print(np.round(sol_acc, 3))

            return sol_acc, sol_contact_force, sol_torque

        else:
            self.solver_failure_count += 1
            if (
                self.solver_failure_count <= self.max_solver_failure_prints
                or self.time_step % self.solver_failure_print_interval == 0
            ):
                print(f"{self.time_step} Solver failed with status: {status}")

            return None, None, None

    def act(self, agent_feedback: dict, task_info: dict):
        info = {}
        self.time_step = task_info["episode_length"]
        self.qpos_fbk = agent_feedback["qpos_fbk"]
        self.qvel_fbk = agent_feedback["qvel_fbk"]
        self._update_base_reference_from_goal(task_info)
        self.phi, self.Lg, self.Lf, self.phi0, self.phi0dot = self.safety_index.phi(
            agent_feedback["state"], task_info
        )

        if task_info["ik_success"] is False:
            # If IK failed, hold the last valid WBC solution so the stance
            # controller keeps publishing complete whole-body action fields.
            info["ik_success"] = False
            info["qp_success"] = False
            info["sol_acc"] = self.last_ddq.copy()
            info["sol_contact_force"] = self.last_contact_force.copy()
            info["sol_torque"] = self.last_torque.copy()
            info["base_pose_des"] = self.base_pose_des.copy()
            info["base_pose_fbk"] = self._get_base_pose_feedback()
            sol_torque = self.last_torque.copy()
        else:
            # If IK succeeded, proceed with WBC
            info["ik_success"] = True
            ddq_ctrl = task_info["ref_ddq"]
            self.ref_ddq[-17:] = ddq_ctrl[-17:]

            self.use_safe = True
            self.use_wbc = True
            sol_acc, sol_contact_force, sol_torque = self.solve_wbc()

            if sol_acc is None:
                # If WBC failed, hold the last valid solution instead of dropping
                # required action_info fields used by the MuJoCo agent.
                info["qp_success"] = False
                info["sol_acc"] = self.last_ddq.copy()
                info["sol_contact_force"] = self.last_contact_force.copy()
                info["sol_torque"] = self.last_torque.copy()
                info["base_pose_des"] = self.base_pose_des.copy()
                info["base_pose_fbk"] = self._get_base_pose_feedback()
                sol_torque = self.last_torque.copy()
            else:
                # If WBC succeeded, return the solution
                info["qp_success"] = True
                info["sol_acc"] = sol_acc
                info["sol_contact_force"] = sol_contact_force
                info["sol_torque"] = sol_torque
                info["zmp"] = self.calculate_zmp()
                info["phi"] = self.phi
                info["base_pose_des"] = self.base_pose_des.copy()
                info["base_pose_fbk"] = self._get_base_pose_feedback()

                self.last_ddq = sol_acc
                self.last_contact_force = sol_contact_force
                self.last_torque = sol_torque

        return sol_torque, info
