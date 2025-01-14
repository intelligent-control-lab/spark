import casadi          
import numpy as np
import pinocchio as pin
from pinocchio import casadi as cpin               
import os

from .weighted_moving_filter import WeightedMovingFilter
from spark_robot.base.base_robot_kinematics import RobotKinematics
from spark_robot.base.base_robot_config import RobotConfig
from spark_robot import SPARK_ROBOT_RESOURCE_DIR
from spark_utils import pos_quat_to_transformation, rpy2quat
# ?not circular?

class G1BasicKinematics(RobotKinematics):
    def __init__(self, robot_cfg: RobotConfig, **kwargs) -> None:
        
        super().__init__(robot_cfg)
        self.robot_cfg = robot_cfg
        np.set_printoptions(precision=5, suppress=True, linewidth=200)
        self.robot = pin.RobotWrapper.BuildFromMJCF(
            os.path.join(SPARK_ROBOT_RESOURCE_DIR, self.robot_cfg.mjcf_path)
        )
  
        self.mixed_jointsToLockIDs = self.robot_cfg.joint_to_lock
        self.add_extra_frames()
        self.reduced_robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=self.mixed_jointsToLockIDs,
            reference_configuration=np.array([0.0] * self.robot.model.nq),
        )
        
        
        print(self.reduced_robot.model.nq)

        for i in range(self.reduced_robot.model.nframes):
            frame = self.reduced_robot.model.frames[i]
            frame_id = self.reduced_robot.model.getFrameId(frame.name)
            # print(f"Frame ID: {frame_id}, Name: {frame.name}")
        
        # Creating Casadi models and data for symbolic computing
        self.cmodel = cpin.Model(self.reduced_robot.model)
        self.cdata = self.cmodel.createData()

        # Creating symbolic variables
        self.cq = casadi.SX.sym("q", self.reduced_robot.model.nq, 1) 
        self.cTf_l = casadi.SX.sym("tf_l", 4, 4)
        self.cTf_r = casadi.SX.sym("tf_r", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)
        
        # Get the hand joint ID and define the error function
        self.L_hand_id = self.reduced_robot.model.getFrameId("L_ee")
        self.R_hand_id = self.reduced_robot.model.getFrameId("R_ee")

        self.translational_error = casadi.Function(
            "translational_error",
            [self.cq, self.cTf_l, self.cTf_r],
            [
                casadi.vertcat(
                    self.cdata.oMf[self.L_hand_id].translation - self.cTf_l[:3,3],
                    self.cdata.oMf[self.R_hand_id].translation - self.cTf_r[:3,3]
                )
            ],
        )
        self.rotational_error = casadi.Function(
            "rotational_error",
            [self.cq, self.cTf_l, self.cTf_r],
            [
                casadi.vertcat(
                    cpin.log3(self.cdata.oMf[self.L_hand_id].rotation @ self.cTf_l[:3,:3].T),
                    cpin.log3(self.cdata.oMf[self.R_hand_id].rotation @ self.cTf_r[:3,:3].T)
                )
            ],
        )
        self._FK = self.create_fk_function()

        # Defining the optimization problem
        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.reduced_robot.model.nq)
        self.var_q_last = self.opti.parameter(self.reduced_robot.model.nq)   # for smooth
        self.param_tf_l = self.opti.parameter(4, 4)
        self.param_tf_r = self.opti.parameter(4, 4)
        self.translational_cost = casadi.sumsqr(self.translational_error(self.var_q, self.param_tf_l, self.param_tf_r))
        self.rotation_cost = casadi.sumsqr(self.rotational_error(self.var_q, self.param_tf_l, self.param_tf_r))
        self.regularization_cost = casadi.sumsqr(self.var_q)
        self.smooth_cost = casadi.sumsqr(self.var_q - self.var_q_last)

        # Setting optimization constraints and goals
        self.opti.subject_to(self.opti.bounded(
            self.reduced_robot.model.lowerPositionLimit,
            self.var_q,
            self.reduced_robot.model.upperPositionLimit)
        )
        self.opti.minimize(50 * self.translational_cost + self.rotation_cost + 0.02 * self.regularization_cost + 0.1 * self.smooth_cost)

        ##### IPOPT #####
        opts = {
            'ipopt':{
                'print_level': 0,
                'max_iter': 20,
                'tol': 1e-4,
            },
            'print_time':False,# print or not
            'calc_lam_p':False # https://github.com/casadi/casadi/wiki/FAQ:-Why-am-I-getting-%22NaN-detected%22in-my-optimization%3F
        }
        self.opti.solver("ipopt", opts)

        self.init_data = np.zeros(self.reduced_robot.model.nq)
        self.smooth_filter = WeightedMovingFilter(np.array([0.4, 0.3, 0.2, 0.1]), len(self.robot_cfg.DoFs))
          
    def add_extra_frames(self):
        self.robot.model.addFrame(
            pin.Frame('L_ee',
                      self.robot.model.getJointId('left_wrist_yaw_joint'),
                      pin.SE3(np.eye(3),
                              np.array([0.05,0,0]).T),
                      pin.FrameType.OP_FRAME)
        )
        
        self.robot.model.addFrame(
            pin.Frame('R_ee',
                      self.robot.model.getJointId('right_wrist_yaw_joint'),
                      pin.SE3(np.eye(3),
                              np.array([0.05,0,0]).T),
                      pin.FrameType.OP_FRAME)
        )
        
        self.robot.model.addFrame(
            pin.Frame('torso_link_1',
                      self.robot.model.getJointId('waist_pitch_joint'),
                      pin.SE3(np.eye(3),
                              np.array([0.0, 0.0, 0.1]).T),
                      pin.FrameType.OP_FRAME)
        )
        
        self.robot.model.addFrame(
            pin.Frame('torso_link_2',
                      self.robot.model.getJointId('waist_pitch_joint'),
                      pin.SE3(np.eye(3),
                              np.array([0.0, 0.0, 0.2]).T),
                      pin.FrameType.OP_FRAME)
        )
        
        self.robot.model.addFrame(
            pin.Frame('torso_link_3',
                      self.robot.model.getJointId('waist_pitch_joint'),
                      pin.SE3(np.eye(3),
                              np.array([0.0, 0.0, 0.4]).T),
                      pin.FrameType.OP_FRAME)
        )
        
        self.robot.model.addFrame(
            pin.Frame('pelvis_link_1',
                      self.robot.model.getJointId('waist_yaw_joint'),
                      pin.SE3(np.eye(3),
                              np.array([0.0, 0.0, 0.0]).T),
                      pin.FrameType.OP_FRAME)
        )
        
        self.robot.model.addFrame(
            pin.Frame('pelvis_link_2',
                      self.robot.model.getJointId('waist_yaw_joint'),
                      pin.SE3(np.eye(3),
                              np.array([0.0, 0.15, 0.0]).T),
                      pin.FrameType.OP_FRAME)
        )
        
        self.robot.model.addFrame(
            pin.Frame('pelvis_link_3',
                      self.robot.model.getJointId('waist_yaw_joint'),
                      pin.SE3(np.eye(3),
                              np.array([0.0, -0.15, 0.0]).T),
                      pin.FrameType.OP_FRAME)
        )
           
    def create_fk_function(self):
        # Prepare a list to hold the transformation matrices
        transforms = []

        # Create the transformation matrices for each part
        for frame in self.robot_cfg.Frames:
            # Get the transformation for the current part
            id = self.reduced_robot.model.getFrameId(frame.name)
            rotation = self.cdata.oMf[id].rotation
            translation = self.cdata.oMf[id].translation
            
            # Create the transformation matrix (4x4)
            transform = casadi.vertcat(
                casadi.horzcat(rotation, translation),  # Reshape for proper matrix formation
                np.array([[0.0, 0.0, 0.0, 1.0]])
            )
            transforms.append(transform)
            
        full_transform = casadi.vertcat(*transforms)

        # Create the output function
        return casadi.Function("FK", [self.cq], [full_transform]) 

    def update_base_frame(self, trans_world2base, dof):
        try:
            trans_world2base_new = pos_quat_to_transformation(
                                np.array([dof[self.robot_cfg.DoFs.LinearX], dof[self.robot_cfg.DoFs.LinearY], trans_world2base[2,3]]), 
                                np.array(rpy2quat([0.0, 0.0, dof[self.robot_cfg.DoFs.RotYaw]])))
            return trans_world2base_new
        except:
            return trans_world2base

    def forward_kinematics(self, dof):

        frames = self._FK(dof[:self.reduced_robot.model.nq])
        
        # Convert the entire frames matrix to a NumPy array once
        frames_full = frames.full()
        
        # Reshape the array to (num_joints, 4, 4)
        frames_full = frames_full.reshape(-1, 4, 4)
        
        return frames_full
    
    def inverse_kinematics(self, T , current_lr_arm_motor_q = None, current_lr_arm_motor_dq = None):
        left_wrist, right_wrist = T[0], T[1]
        if current_lr_arm_motor_q is not None:
            self.init_data = current_lr_arm_motor_q
        self.opti.set_initial(self.var_q, self.init_data)

        self.opti.set_value(self.param_tf_l, left_wrist)
        self.opti.set_value(self.param_tf_r, right_wrist)
        self.opti.set_value(self.var_q_last, self.init_data) # for smooth

        try:
            sol = self.opti.solve()
            # sol = self.opti.solve_limited()

            sol_q = self.opti.value(self.var_q)
            # self.smooth_filter.add_data(sol_q)
            # sol_q = self.smooth_filter.filtered_data

            if current_lr_arm_motor_dq is not None:
                v = current_lr_arm_motor_dq * 0.0
            else:
                v = (sol_q - self.init_data) * 0.0

            self.init_data = sol_q

            sol_tauff = pin.rnea(self.reduced_robot.model, self.reduced_robot.data, sol_q, v, np.zeros(self.reduced_robot.model.nv))
            sol_tauff = np.concatenate([sol_tauff, np.zeros(len(self.robot_cfg.DoFs) - sol_tauff.shape[0])], axis=0)
            
            info = {"sol_tauff": sol_tauff}

            dof = np.zeros(self.num_dof)
            dof[:len(sol_q)] = sol_q

            return dof, info
        
        except Exception as e:
            print(f"ERROR in convergence, plotting debug info.{e}")

            sol_q = self.opti.debug.value(self.var_q)
            # self.smooth_filter.add_data(sol_q)
            # sol_q = self.smooth_filter.filtered_data

            if current_lr_arm_motor_dq is not None:
                v = current_lr_arm_motor_dq * 0.0
            else:
                v = (sol_q - self.init_data) * 0.0

            self.init_data = sol_q

            sol_tauff = pin.rnea(self.reduced_robot.model, self.reduced_robot.data, sol_q, v, np.zeros(self.reduced_robot.model.nv))
            sol_tauff = np.concatenate([sol_tauff, np.zeros(len(self.robot_cfg.DoFs) - sol_tauff.shape[0])], axis=0)

            print(f"sol_q:{sol_q} \nmotorstate: \n{current_lr_arm_motor_q} \nleft_pose: \n{left_wrist} \nright_pose: \n{right_wrist}")

            info = {"sol_tauff": sol_tauff * 0.0}

            dof = np.zeros(self.num_dof)
            dof[:len(current_lr_arm_motor_q)] = current_lr_arm_motor_q

            return dof, info

    def compute_frame_jacobian(self, q, frame_id):
        frame_jacobian = pin.computeFrameJacobian(self.reduced_robot.model, self.reduced_robot.data, q, frame_id)
        return frame_jacobian

if __name__ == "__main__":
    
    arm_ik = G1BasicKinematics()
