from spark_policy.base.base_policy import BasePolicy
from spark_agent import BaseAgent
from spark_robot import RobotKinematics, RobotConfig
import numpy as np
from scipy.spatial.transform import Rotation as R
from spark_utils import pos_quat_to_transformation, rpy2quat, transformation_to_pos_quat, quat2rpy
from spark_policy.safe import SecondOrderCollisionSafetyIndex
from spark_utils import initialize_class
import proxsuite

import time
class G1WBCPolicy(BasePolicy):
    def __init__(self, robot_cfg: RobotConfig, robot_kinematics: RobotKinematics) -> None:
        super().__init__(robot_cfg, robot_kinematics)

        self.safety_index = SecondOrderCollisionSafetyIndex(robot_kinematics=self.robot_kinematics,
                                                            robot_cfg=robot_kinematics.robot_cfg,
                                                            phi_n=1.0,
                                                            phi_k=0.1, 
                                                            min_distance = {
                                                                "environment": 0.03,
                                                                "self": 0.01
                                                            },
                                                            enable_self_collision = False)
        self.eta_default = 1.0
        
        self.contact_frames = ["fr_left_foot", 
                                "br_left_foot", 
                                "fl_left_foot", 
                                "bl_left_foot", 
                                "fr_right_foot", 
                                "br_right_foot", 
                                "fl_right_foot", 
                                "bl_right_foot"]
        # zmp_edges represents the edges of the support polygon in x-y plane
        # Each row represents a set of parameters a, b and c to define the edge
        # a * x + b * y + c  <= 0
        self.zmp_edges = np.array([
            [1, 0, -0.05], # x <= 0.01
            [-1, 0, -0.05],# x >= -0.01
            [0, 1, -0.05], # y <= 0.01
            [0, -1, -0.05],# y >= -0.01
        ])
        
        self.num_contacts = len(self.contact_frames)  # Number of contact points
        self.num_ctrl = len(self.robot_cfg.Control)
        self.num_dof = len(self.robot_cfg.DoFs) - 1  # Floating base + actuated joints
        self.num_slack = 6
        self.num_decision_vars = self.num_dof + 3 * self.num_contacts + self.num_ctrl + self.num_slack  # Decision variables: [ddq, tau, lambda]
        self.torque_limits = 100.0 * np.ones(self.num_ctrl)  # Example max torque limits
        self.acceleration_limits = np.ones(self.num_dof)  # Example acceleration limits
        self.acceleration_limits[-self.num_ctrl:] = [self.robot_cfg.ControlLimit[ctrl_idx] for ctrl_idx in self.robot_cfg.Control]  # Example max acceleration limits
        self.friction_coeff = 0.5  # Example friction coefficient
        self.base_velocity_des = np.zeros(6)  # Desired base velocity
        # self.base_velocity_des[2] = -0.1
        self.base_pose_des = np.zeros(6)  # Desired base pose
        self.base_pose_des[2] = 0.91 #0.937
        self.base_acceleration_des = np.zeros(6)  # Desired base acceleration
        self.base_angular_kp = 100  # Proportional gain for angular velocity
        self.base_angular_kd = 30  # Derivative gain for angular velocity
        self.base_linear_kp = 100  # Proportional gain for linear velocity
        self.base_linear_kd = 30  # Derivative gain for linear velocity
        self.direction = -1
        self.ref_ddq = np.zeros(self.num_dof)
        self.last_ddq = np.zeros(self.num_dof)
        self.last_torque = np.zeros(self.num_ctrl)
        self.use_safe = True
        
    def _init_qp(self):

        if self.use_safe and not self.use_wbc:
            
            self.weights_functions = [
                (self.formulate_angular_acceleration_task, 1),
                (self.formulate_linear_acceleration_task, 10),
                # (self.formulate_contact_force_task, 10),
                (self.formulate_joint_acceleration_task, 10),
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
                (self.formulate_safety_task, 0, 6 ),
                # (self.formulate_contact_point_acceleration_task, 3 * self.num_contacts, 0),
                # (self.formulate_zmp_task, 0, self.zmp_edges.shape[0]),
            ]
        
        elif self.use_safe and self.use_wbc:
            self.weights_functions = [
                (self.formulate_angular_acceleration_task, 1),
                (self.formulate_linear_acceleration_task, 1),
                # (self.formulate_contact_force_task, 10),
                (self.formulate_joint_acceleration_task, 100),
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
                (self.formulate_safety_task, 0, 6 ),
                (self.formulate_contact_point_acceleration_task, 3 * self.num_contacts, 0),
                # (self.formulate_zmp_task, 0, self.zmp_edges.shape[0]),
            ]
        
        elif self.use_wbc and not self.use_safe:
            self.weights_functions = [
                (self.formulate_angular_acceleration_task, 1),
                (self.formulate_linear_acceleration_task, 1),
                # (self.formulate_contact_force_task, 10),
                (self.formulate_joint_acceleration_task, 100),
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
            
        self.num_equality_constraints = sum([constraint[1] for constraint in self.constraints_functions])
        self.num_inequality_constraints = sum([constraint[2] for constraint in self.constraints_functions])
        
        self.qp = proxsuite.proxqp.dense.QP(self.num_decision_vars, self.num_equality_constraints, self.num_inequality_constraints)

        # self.qp_init = False
        self.prev_sol = np.zeros((self.num_decision_vars,))
        # self.qp.settings.verbose = True                # Print solver info each iteration
        self.qp.settings.max_iter = 1000               # Maximum number of iterations
        self.qp.settings.eps_abs = 1e-5                # Absolute tolerance
        self.qp.settings.eps_rel = 1e-5                # Relative tolerance
        self.qp.settings.compute_timings = True        # Track solve timing (optional)
        
        
    def formulate_floating_base_eom_task(self):
        '''
        Formulate the floating base equations of motion task.'''

        s = np.zeros((self.num_ctrl, self.num_dof))
        s[:, 6:] = np.eye(self.num_ctrl)
        
        a = np.zeros((self.num_dof, self.num_decision_vars))
        a[:, :self.num_dof] = self.M
        a[:, self.num_dof: self.num_dof + 3 * self.num_contacts] = -self.J_c.T
        a[:, self.num_dof + 3 * self.num_contacts: self.num_dof + 3 * self.num_contacts + self.num_ctrl] = -s.T
        
        
        # a = np.hstack([
        #     self.M, 
        #     -self.J_c.T, 
        #     -s.T,
        # ])

        b = -self.nle
        
        return a, b, np.empty((0, self.num_decision_vars)), np.empty(0)
    
    def formulate_torque_limits_task(self):
        '''
        Formulate the torque limits constraint.
        '''
        d = np.zeros((2 * self.num_ctrl, self.num_decision_vars))
    
        identity_block = np.eye(self.num_ctrl)
        start_col = self.num_dof + 3 * self.num_contacts
        
        d[:self.num_ctrl, start_col:start_col + self.num_ctrl] = identity_block
        d[self.num_ctrl:, start_col:start_col + self.num_ctrl] = -identity_block

        f = np.zeros(2 * self.num_ctrl)
        f[:self.num_ctrl] = self.torque_limits
        f[self.num_ctrl:] = self.torque_limits
        return np.empty((0, self.num_decision_vars)), np.empty(0), d, f
    
    def formulate_acceleration_limits_task(self):
        '''
        Formulate the acceleration limits constraint.
        '''
        d = np.zeros((2 * self.num_dof, self.num_decision_vars))
    
        identity_block = np.eye(self.num_dof)
        start_col = 0
        
        d[:self.num_dof, start_col:start_col + self.num_dof] = identity_block
        d[self.num_dof:, start_col:start_col + self.num_dof] = -identity_block

        f = np.zeros(2 * self.num_dof)
        f[:self.num_dof] = self.acceleration_limits
        f[self.num_dof:] = self.acceleration_limits
        return np.empty((0, self.num_decision_vars)), np.empty(0), d, f
    
    def formulate_friction_cone_task(self):
        '''
        Formulate the friction cone constraint.
        '''
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
                a[3 * j : 3 * (j + 1), self.num_dof + 3 * i : self.num_dof + 3 * (i + 1)] = np.eye(3)
                j += 1
        
        # Initialize vector 'b' with zeros
        b = np.zeros(a.shape[0])

        # Define friction pyramid matrix
        friction_pyramid = np.array([
            [ 0,  0, -1],
            [ 1,  0, -self.friction_coeff],
            [-1,  0, -self.friction_coeff],
            [ 0,  1, -self.friction_coeff],
            [ 0, -1, -self.friction_coeff]
        ])

        # Initialize matrix 'd' with zeros
        d = np.zeros((5 * num_contacts + 3 * (num_three_dof_contacts - num_contacts), self.num_decision_vars))
        
        # Populate matrix 'd' based on contactFlag_
        j = 0
        for i in range(num_three_dof_contacts):
            if contact_flag[i]:  
                d[5 * j : 5 * (j + 1), self.num_dof + 3 * i : self.num_dof + 3 * (i + 1)] = friction_pyramid
                j += 1
        
        # Initialize vector 'f' with zeros
        f = np.zeros(d.shape[0])

        return a, b, d, f

    def formulate_contact_force_task(self):
        '''
        Formulate the contact force task.
        '''
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
        '''
        Formulate the contact point task.
        '''
        # Initialize matrix 'a' with zeros
        a = np.zeros((3 * self.num_contacts, self.num_decision_vars))
        b = np.zeros(a.shape[0])
        for j in range(self.num_contacts):
            a[3 * j : 3 * (j + 1),  : self.num_dof] = self.J_c[3 * j : 3 * (j + 1), : self.num_dof]
            b[3 * j : 3 * (j + 1)] = - self.dJ_c[3 * j : 3 * (j + 1), : self.num_dof] @ self.qvel_fbk
  
        if self.time_step < 300: # supress first 300 steps after every reset
            a *= 0.0
            b *= 0.0
  
        return a, b, np.empty((0, self.num_decision_vars)), np.empty(0)

    def formulate_zmp_task(self):
        '''
        Formulate the ZMP task.
        '''
        
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
            S_z[c * 3 + 2] = 1.0                # f_z term
        
        # Build inequality for each edge
        for j in range(num_edges):
            normal_x, normal_y, offset = self.zmp_edges[j]
            S_proj = normal_x * S_x + normal_y * S_y + offset * S_z
            d[j, self.num_dof : self.num_dof + 3 * self.num_contacts] = S_proj.flatten()
            f[j] = 0.0
            
        if self.time_step < 100: # supress first 100 steps after every reset
            d *= 0.0
            f *= 0.0 
            
        return np.empty((0, self.num_decision_vars)), np.empty(0), d, f

    def formulate_angular_acceleration_task(self):
        '''
        Formulate the base acceleration weight.
        '''
        # Initialize matrices and vectors
        a = np.zeros((3, self.num_decision_vars))
        b = np.zeros(3)

        frame_name = "pelvis_link_1"
        J_frame = self.robot_kinematics.get_jacobian(frame_name)
        dJ_frame = self.robot_kinematics.get_jacobian_dot(frame_name)
        
        a[:, :self.num_dof] = J_frame[3:, :self.num_dof]

        v =  J_frame[3:, :] @ self.qvel_fbk
        rot = self.robot_kinematics.get_frame_transformation(frame_name)[:3, :3]
              
        target_v = self.base_velocity_des[-3:]
        target_a = self.base_acceleration_des[-3:]
        target_euler = self.base_pose_des[-3:]

        target_rot = R.from_euler('xyz', target_euler).as_matrix()
        
        R_error = target_rot @ rot.T
        error = np.array([
            R_error[2, 1] - R_error[1, 2],
            R_error[0, 2] - R_error[2, 0],
            R_error[1, 0] - R_error[0, 1]
        ]) * 0.5

        # Compute b vector
        b = (target_a 
            + self.base_angular_kp * error 
            + self.base_angular_kd * (target_v - v) 
            - dJ_frame[3:6, :self.num_dof] @ self.qvel_fbk)

        return a, b, np.empty((0, 0)), np.empty(0)
    
    def formulate_linear_acceleration_task(self):
        '''
        Formulate the base linear acceleration task.
        '''
        # Initialize matrices and vectors
        a = np.zeros((3, self.num_decision_vars))
        b = np.zeros(3)

        frame_name = "pelvis_link_1"
        J_frame = self.robot_kinematics.get_jacobian(frame_name)
        dJ_frame = self.robot_kinematics.get_jacobian_dot(frame_name)
        
        a[:, :self.num_dof] = J_frame[:3, :self.num_dof]
        
        p = self.robot_kinematics.get_frame_transformation(frame_name)[:3, 3]
        v =  J_frame[:3, :] @ self.qvel_fbk
        
        target_p = self.base_pose_des[:3]
        target_v = self.base_velocity_des[:3]
        target_a = self.base_acceleration_des[:3]
        
        b = (target_a 
            + self.base_linear_kp * (target_p - p) 
            + self.base_linear_kd * (target_v - v) 
            - dJ_frame[:3, :self.num_dof] @ self.qvel_fbk)
        
        # a = a[:2, :]
        # b = b[:2]
        
        return a, b, np.empty((0, 0)), np.empty(0)
    
    def formulate_joint_acceleration_task(self):
        '''
        Formulate the joint acceleration task.
        '''
        # Initialize matrices and vectors
        a = np.zeros((self.num_dof, self.num_decision_vars))
        b = np.zeros(self.num_dof)

        # Extract relevant block from base_j
        weights = np.ones(self.num_dof)
        weights[6:-17] *= 10.0
        a[:, :self.num_dof] = np.diag(weights)

        # Compute b vector
        b = self.ref_ddq
        
        return a, b, np.empty((0, 0)), np.empty(0)
    
    def formulate_delta_joint_acceleration_task(self):
        '''
        Formulate the joint acceleration task.
        '''
        # Initialize matrices and vectors
        a = np.zeros((self.num_dof, self.num_decision_vars))
        b = np.zeros(self.num_dof)

        # Extract relevant block from base_j
        weights = np.ones(self.num_dof)
        weights[6:-17] *= 10.0
        a[:, :self.num_dof] = np.diag(weights)

        # Compute b vector
        b = self.last_ddq
        
        return a, b, np.empty((0, 0)), np.empty(0)
    
    def formulate_joint_torque_task(self):
        '''
        Formulate the joint torque task.
        '''
        # Initialize matrices and vectors
        a = np.zeros((self.num_ctrl, self.num_decision_vars))
        b = np.zeros(self.num_ctrl)

        # Extract relevant block from base_j
        a[:, self.num_dof + 3 * self.num_contacts: self.num_dof + 3 * self.num_contacts + self.num_ctrl] = np.eye(self.num_ctrl)

        # Compute b vector
        b = np.zeros(self.num_ctrl)
        
        return a, b, np.empty((0, 0)), np.empty(0)
    
    def formulate_delta_joint_torque_task(self):
        '''
        Formulate the joint torque task.
        '''
        # Initialize matrices and vectors
        a = np.zeros((self.num_ctrl, self.num_decision_vars))
        b = np.zeros(self.num_ctrl)

        # Extract relevant block from base_j
        a[:, self.num_dof + 3 * self.num_contacts: self.num_dof + 3 * self.num_contacts + self.num_ctrl] = np.eye(self.num_ctrl)

        # Compute b vector
        b = self.last_torque
        
        return a, b, np.empty((0, 0)), np.empty(0)

    def eta_fn(self, phi):
            eta_values = np.where(
                            (self.safety_index.phi_mask.reshape(-1,1) > 0) & (phi.reshape(-1,1) >= 0),
                            np.ones((self.safety_index.num_constraint, 1)) * self.eta_default,
                            # np.ones((self.safety_index.num_constraint, 1)) * self.eta_default * phi.reshape(-1, 1),
                            np.ones((self.safety_index.num_constraint, 1)) * -np.inf
                        )
            return eta_values.reshape(-1, 1)

    def formulate_safety_task(self):
        '''
        Formulate the safety task.
        '''
        use_slack = False
        a = np.empty((0, self.num_decision_vars))
        b = np.empty(0)
        if not use_slack:
            # Initialize matrices and vectors
            d = np.zeros((self.phi.shape[0], self.num_decision_vars))
            f = np.zeros(self.phi.shape[0])
            # print(self.phi[self.safety_index.phi_mask].max())
            
            d[:, :self.num_dof] = self.Lg

            # Compute f vector
            f = -self.Lf.reshape(-1,) - self.eta_fn(self.phi).reshape(-1,) 
            
        else:
            d = np.zeros((self.phi.shape[0] + self.num_slack, self.num_decision_vars))
            f = np.zeros(self.phi.shape[0] + self.num_slack)
            
            d[:self.phi.shape[0], :self.num_dof] = self.Lg
            f[:self.phi.shape[0]] = -self.Lf.reshape(-1,) - self.eta_fn(self.phi).reshape(-1,)
            
            d[-self.num_slack:, -self.num_slack:] =  - np.eye(self.num_slack)
            f[-self.num_slack:] = np.inf * np.ones(self.num_slack)

        return a, b,  d,  f

    def formulate_slack_variable_task(self):
        a = np.zeros((self.num_slack, self.num_decision_vars))
        b = np.zeros(self.num_slack)
        a[:,-self.num_slack:] = 1e3 * np.eye(self.num_slack)
        
        return a, b, np.empty((0, 0)), np.empty(0)

    def formulate_constraints(self):
        '''
        Formulate the constraints for the optimization problem.'''
        
        constraints = [fun() for fun, _, _ in self.constraints_functions]

        # Equality constraints aX = b
        a = np.vstack([constraint[0] for constraint in constraints])  # (35, 76)
        b = np.hstack([constraint[1] for constraint in constraints])  # (35,)
        # # Inequality constraints dX <= f
        d = np.vstack([constraint[2] for constraint in constraints])  # (78, 76)
        f = np.hstack([constraint[3] for constraint in constraints])  # (78,)
    
        # Combine all constraints
        A = np.vstack([
            a, 
            d
        ])                                      # A: (113, 76)
        lbA = np.hstack([
            b, 
            -np.inf * np.ones(f.shape[0])
        ])                                      # lbA: (113,)
        ubA = np.hstack([
            b, 
            f
        ])                                      # ubA: (113,)

        return A, lbA, ubA, a, d, b, f
    
    def formulate_weights(self):
        '''
        Formulate the weights for the optimization problem.
        '''
        weights = [(fun(), weight) for fun, weight in self.weights_functions]
        
        a = np.vstack([weight[0][0] * weight[1]  for weight in weights])
        b = np.hstack([weight[0][1] * weight[1] for weight in weights])
        
        P = a.T @ a
        q = -a.T @ b
        
        return P, q
    
    def calculate_zmp(self):
        '''
        Calculate the Zero Moment Point (ZMP) based on the contact forces.
        '''
        # Get the contact forces
        contact_forces = self.prev_sol[self.num_dof:self.num_dof + 3 * self.num_contacts].reshape(-1, 3)
        
        # Get the positions of the contact points
        contact_positions = np.array([self.robot_kinematics.get_frame_transformation(frame_name)[:3, 3] for frame_name in self.contact_frames])
        
        # Calculate the ZMP
        zmp = np.sum(contact_positions[:, :2] * contact_forces[:, 2][:, np.newaxis], axis=0) / np.sum(contact_forces[:, 2])
        
        return np.concatenate((zmp, np.zeros(1)))  # Add a zero for the z-coordinate of ZMP
    
    def solve_wbc(self):
        self._init_qp()

        self.M, self.nle, self.J_c, self.dJ_c, self.frame_c = self.robot_kinematics.get_dynamics(self.qpos_fbk, self.qvel_fbk, self.contact_frames)
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

            
            sol_acc = self.qp.results.x[:self.num_dof].flatten()
            sol_contact_force = self.qp.results.x[self.num_dof:self.num_dof + 3 * self.num_contacts].flatten()
            sol_torque = self.qp.results.x[self.num_dof + 3 * self.num_contacts:self.num_dof + 3 * self.num_contacts + self.num_ctrl].flatten()
            # print("ZMP:", np.round(zmp, 3))
            
            constarint_violation = self.Lf + self.Lg @ sol_acc.reshape(-1, 1) + self.eta_fn(self.phi.reshape(-1, 1))
            constarint_violation[constarint_violation < 0] = 0
            # print(np.round(sol_acc, 3))
            
            return sol_acc, sol_contact_force, sol_torque 
        
        else:
            print(f"{self.time_step} Solver failed with status: {status}")

            return None, None, None

    
    def act(self, agent_feedback: dict, task_info: dict):
        info = {}
        self.time_step = task_info["episode_length"]
        self.qpos_fbk = agent_feedback["qpos_fbk"]
        self.qvel_fbk = agent_feedback["qvel_fbk"]
        self.phi, self.Lg, self.Lf, self.phi0, self.phi0dot = self.safety_index.phi(agent_feedback["state"], task_info)

        if task_info["ik_success"] is False:
            # If IK failed, skip the WBC and return None
            info["ik_success"] = False
            sol_torque = None
        else:
            # If IK succeeded, proceed with WBC
            info["ik_success"] = True
            ddq_ctrl = task_info["ref_ddq"]
            self.ref_ddq[-17:] = ddq_ctrl[-17:]
            
            self.use_safe = True 
            self.use_wbc = True
            sol_acc, sol_contact_force, sol_torque = self.solve_wbc()
        
            if sol_acc is None:   
                # If WBC failed, return None
                info["qp_success"] = False
                sol_torque = None
            else:
                # If WBC succeeded, return the solution
                info["qp_success"] = True
                info["sol_acc"] = sol_acc
                info["sol_contact_force"] = sol_contact_force
                info["sol_torque"] = sol_torque
                info["zmp"] = self.calculate_zmp()
                info["phi"] = self.phi
        
                self.last_ddq = sol_acc
                self.last_torque = sol_torque
                
        return sol_torque, info
