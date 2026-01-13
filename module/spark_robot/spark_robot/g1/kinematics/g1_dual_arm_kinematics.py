import casadi          
import numpy as np
import pinocchio as pin
from pinocchio import casadi as cpin               
import os

from spark_robot.base.base_robot_kinematics import RobotKinematics
from spark_robot.base.base_robot_config import RobotConfig
from spark_robot import SPARK_ROBOT_RESOURCE_DIR
from spark_utils import pos_quat_to_transformation, rpy2quat, transformation_to_pos_quat, quat2rpy
from scipy.spatial.transform import Rotation as R

class G1DualArmBaseKinematics(RobotKinematics):
    def __init__(self, robot_cfg: RobotConfig, **kwargs) -> None:
        super().__init__(robot_cfg)
        self.robot_cfg = robot_cfg
        self.mixed_jointsToLockIDs = self.robot_cfg.joint_to_lock
        self.kinematics_model_path = 'g1/g1_29dof_dual_arm.xml'
        self._init_fixed_base_kinematics()
        self._init_whole_body_kinematics()
        self._init_collision_model()

    def _init_fixed_base_kinematics(self):
        self.fixed_base_robot = pin.RobotWrapper.BuildFromMJCF(
            os.path.join(SPARK_ROBOT_RESOURCE_DIR, self.kinematics_model_path)
        )
        self.add_extra_frames(self.fixed_base_robot.model)
        self.reduced_fixed_base_robot = self.fixed_base_robot.buildReducedRobot(
            list_of_joints_to_lock=self.mixed_jointsToLockIDs,
            reference_configuration=np.array([0.0] * self.fixed_base_robot.nq),
        )
        self.reduced_fixed_base_model = self.reduced_fixed_base_robot.model
        self.reduced_fixed_base_data = self.reduced_fixed_base_robot.data

        # Creating Casadi models and data for symbolic computing
        self.cmodel = cpin.Model(self.reduced_fixed_base_model)
        self.cdata = self.cmodel.createData()

        # Creating symbolic variables
        self.cq = casadi.SX.sym("q", self.reduced_fixed_base_model.nq, 1) 
        self.cTf_l = casadi.SX.sym("tf_l", 4, 4)
        self.cTf_r = casadi.SX.sym("tf_r", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)
        
        # Get the hand joint ID and define the error function
        self.L_hand_id = self.reduced_fixed_base_model.getFrameId("L_ee")
        self.R_hand_id = self.reduced_fixed_base_model.getFrameId("R_ee")

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

        # Defining the optimization problem
        self.opti = casadi.Opti()
        self.var_q = self.opti.variable(self.reduced_fixed_base_model.nq)
        self.var_q_last = self.opti.parameter(self.reduced_fixed_base_model.nq)   # for smooth
        self.param_tf_l = self.opti.parameter(4, 4)
        self.param_tf_r = self.opti.parameter(4, 4)
        self.translational_cost = casadi.sumsqr(self.translational_error(self.var_q, self.param_tf_l, self.param_tf_r))
        self.rotation_cost = casadi.sumsqr(self.rotational_error(self.var_q, self.param_tf_l, self.param_tf_r))
        self.regularization_cost = casadi.sumsqr(self.var_q)
        self.smooth_cost = casadi.sumsqr(self.var_q - self.var_q_last)

        # Setting optimization constraints and goals
        self.opti.subject_to(self.opti.bounded(
            self.reduced_fixed_base_model.lowerPositionLimit,
            self.var_q,
            self.reduced_fixed_base_model.upperPositionLimit)
        )
        self.opti.minimize(10 * self.translational_cost + 10.0*self.rotation_cost + 0.01 * self.regularization_cost + 0.0 * self.smooth_cost)
        # self.opti.minimize(50 * self.translational_cost + self.rotation_cost + 0.02 * self.regularization_cost + 0.1 * self.smooth_cost)


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

        self.init_data = np.zeros(self.reduced_fixed_base_model.nq)

    def _init_whole_body_kinematics(self):
        print(f"Initializing {self.__class__.__name__}")
        self.robot = pin.RobotWrapper.BuildFromMJCF(
            os.path.join(SPARK_ROBOT_RESOURCE_DIR, self.kinematics_model_path)
        )
        
        self.reduced_robot = self.robot.buildReducedRobot(
            list_of_joints_to_lock=self.mixed_jointsToLockIDs,
            reference_configuration=np.array([0.0] * self.robot.nq),
        )

        self.model = self.reduced_robot.model
        self.add_extra_frames(self.model)
        self.data = pin.Data(self.model)
        
        self.pin_frame_dict = {}
        for frame in self.robot_cfg.Frames:
            for j in range(self.model.nframes):
                pin_frame = self.model.frames[j]
                pin_frame_id = self.model.getFrameId(pin_frame.name)
                if frame.name == pin_frame.name:
                    self.pin_frame_dict[frame.name] = pin_frame_id 
            print(f"Frame {frame.name} has ID {self.pin_frame_dict[frame.name]}")

    def _init_collision_model(self):
        self.collision_model = pin.buildGeomFromMJCF(
            self.model, os.path.join(SPARK_ROBOT_RESOURCE_DIR, self.kinematics_model_path), pin.GeometryType.COLLISION
        )
        self.visual_model    = pin.buildGeomFromMJCF(
            self.model, os.path.join(SPARK_ROBOT_RESOURCE_DIR, self.kinematics_model_path), pin.GeometryType.VISUAL
        )
        self.collision_model.addAllCollisionPairs()
        self.remove_adjacent_collision_pairs(self.model, self.collision_model)
        self.collision_data = self.collision_model.createData()
        return
    
    def collision_free(self, full_q):
        pin.computeCollisions(self.model, self.data, self.collision_model, self.collision_data, full_q, True)
        for pair, result in zip(self.collision_model.collisionPairs, self.collision_data.collisionResults):
            if result.isCollision():
                geom1 = self.collision_model.geometryObjects[pair.first]
                geom2 = self.collision_model.geometryObjects[pair.second]
                print(f"Collision detected between: {geom1.name} and {geom2.name}")
        return not any(r.isCollision() for r in self.collision_data.collisionResults)
        
    def remove_adjacent_collision_pairs(self, model, collision_model):
        pairs_to_remove = []
        remove_name_elements = ["coupler", "spring", "follower", "driver", "pad", "gripper", "2f85"]
        for i, pair in enumerate(collision_model.collisionPairs):
            geom1 = collision_model.geometryObjects[pair.first]
            geom2 = collision_model.geometryObjects[pair.second]
            joint1 = geom1.parentJoint
            joint2 = geom2.parentJoint
            if model.parents[joint1] == joint2 or model.parents[joint2] == joint1:
                pairs_to_remove.append(i)
                continue
            for name in remove_name_elements:
                if(name in geom1.name or name in geom2.name):
                    pairs_to_remove.append(i)
                    break
        for i in reversed(pairs_to_remove):
            collision_model.removeCollisionPair(collision_model.collisionPairs[i])
           

    def add_extra_frames(self, model):
       model.addFrame(
            pin.Frame('L_ee',
                     model.getJointId('left_wrist_yaw_joint'),
                      pin.SE3(np.eye(3),
                              np.array([0.11, -0.025, -0.0]).T),
                      pin.FrameType.OP_FRAME)
        )
        
       model.addFrame(
            pin.Frame('R_ee',
                     model.getJointId('right_wrist_yaw_joint'),
                      pin.SE3(np.eye(3),
                              np.array([0.11, 0.025, -0.0]).T),
                      pin.FrameType.OP_FRAME)
        )
        
           
    def update_base_frame(self, trans_world2base, dof):
        try:
            R = trans_world2base[:3, :3]
            current_yaw = np.arctan2(R[1, 0], R[0, 0])

            # Compute yaw difference
            delta_yaw = dof[self.robot_cfg.DoFs.RotYaw] - current_yaw

            # Create new yaw rotation matrix
            Rz_new = np.array([
                [np.cos(delta_yaw), -np.sin(delta_yaw), 0],
                [np.sin(delta_yaw),  np.cos(delta_yaw), 0],
                [0, 0, 1]
            ])

            # Compute new rotation matrix
            R_new = Rz_new @ R
            
            trans_world2base[:3, :3] = R_new
            trans_world2base[:2, 3] = dof[self.robot_cfg.DoFs.LinearX], dof[self.robot_cfg.DoFs.LinearY]
        except:
            return trans_world2base
                
        return trans_world2base

    def forward_kinematics(self, dof):   
        # in robot base frame
        self.pre_computation(dof)
        frames = self.get_forward_kinematics()
        return frames
    
    def inverse_kinematics(self, T , current_lr_arm_motor_q = None, current_lr_arm_motor_dq = None):
        
        right_wrist, left_wrist  = T[0], T[1]
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
            v = np.zeros(self.reduced_fixed_base_model.nv)
            self.init_data = sol_q

            sol_tauff = pin.rnea(self.reduced_fixed_base_model, self.reduced_fixed_base_data, sol_q, v, np.zeros(self.reduced_fixed_base_model.nv))
            sol_tauff = np.concatenate([sol_tauff, np.zeros(len(self.robot_cfg.DoFs) - sol_tauff.shape[0])], axis=0)
            
            info = {"sol_tauff": sol_tauff, "success": True}
            return sol_q, info
        
        except Exception as e:
            print(f"ERROR in convergence, plotting debug info.{e}")

            sol_q = self.opti.debug.value(self.var_q)

            if current_lr_arm_motor_dq is not None:
                v = current_lr_arm_motor_dq * 0.0
            else:
                v = (sol_q - self.init_data) * 0.0

            self.init_data = sol_q

            sol_tauff = pin.rnea(self.reduced_fixed_base_model, self.reduced_fixed_base_data, sol_q, v, np.zeros(self.reduced_fixed_base_model.nv))
            
            sol_tauff = np.concatenate([sol_tauff, np.zeros(len(self.robot_cfg.DoFs) - sol_tauff.shape[0])], axis=0)

            print(f"sol_q:{sol_q} \nmotorstate: \n{current_lr_arm_motor_q} \nleft_pose: \n{left_wrist} \nright_pose: \n{right_wrist}")

            info = {"sol_tauff": sol_tauff * 0.0, "success": False}
            
            raise e
      
    def pre_computation(self, dof_pos, dof_vel = None):
        q = dof_pos
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        pin.computeJointJacobians(self.model, self.data)
        pin.updateGlobalPlacements(self.model, self.data)
        if dof_vel is not None:
            dq = dof_vel
            pin.computeJointJacobiansTimeVariation(self.model, self.data, q, dq)
        return
    
    def get_jacobian(self, frame_name):
        """ Compute the Jacobian of a given frame in the robot base frame """
        return pin.getFrameJacobian(self.model, self.data, self.model.getFrameId(frame_name), pin.LOCAL_WORLD_ALIGNED)
    
    def get_jacobian_dot(self, frame_name):
        """ Compute the Jacobian of a given frame in the robot base frame """
        return pin.getFrameJacobianTimeVariation(self.model, self.data, self.model.getFrameId(frame_name), pin.LOCAL_WORLD_ALIGNED)
    
    def get_forward_kinematics(self):
        frames = np.zeros((len(self.robot_cfg.Frames), 4, 4))
        for frame in self.robot_cfg.Frames:
            pin_frame = self.pin_frame_dict[frame.name]
            frames[frame] = self.data.oMf[pin_frame].homogeneous
        return frames

if __name__ == "__main__":
    
    arm_ik = G1FixedBaseKinematics()
