import casadi          
import numpy as np
import pinocchio as pin
from pinocchio import casadi as cpin               
import os

from spark_robot.base.base_robot_kinematics import RobotKinematics
from spark_robot.r1lite.kinematics.r1lite_upper_kinematics import R1LiteUpperKinematics
from spark_robot.base.base_robot_config import RobotConfig
from spark_robot import SPARK_ROBOT_RESOURCE_DIR
from spark_utils import pos_quat_to_transformation, rpy2quat, transformation_to_pos_quat, quat2rpy
from scipy.spatial.transform import Rotation as R

class R1LiteMobileKinematics(R1LiteUpperKinematics):
    def __init__(self, robot_cfg: RobotConfig, **kwargs) -> None:
        super().__init__(robot_cfg, **kwargs)

    def _init_whole_body_kinematics(self):
        print(f"Initializing {self.__class__.__name__}")
        jointComposite = pin.JointModelComposite(3);
        jointComposite.addJoint(pin.JointModelPX());
        jointComposite.addJoint(pin.JointModelPY());
        jointComposite.addJoint(pin.JointModelRZ());
        
        self.robot = pin.RobotWrapper.BuildFromMJCF(
            os.path.join(SPARK_ROBOT_RESOURCE_DIR, self.kinematics_model_path), jointComposite
        )
        
        self.model = self.robot.model
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
            
            dof = np.zeros(len(self.robot_cfg.DoFs))
            dof[:len(sol_q)] = sol_q
            
            return dof, info
        
        except Exception as e:
            print(f"ERROR in convergence, plotting debug info.{e}")

            sol_q = self.opti.debug.value(self.var_q)

            if current_lr_arm_motor_dq is not None:
                v = current_lr_arm_motor_dq * 0.0
            else:
                v = (sol_q - self.init_data) * 0.0

            self.init_data = sol_q

            sol_tauff = pin.rnea(self.reduced_fixed_base_model, self.reduced_fixed_base_data, sol_q, v, np.zeros(self.reduced_fixed_base_model.nv))
            import ipdb; ipdb.set_trace()
            sol_tauff = np.concatenate([sol_tauff, np.zeros(len(self.robot_cfg.DoFs) - sol_tauff.shape[0])], axis=0)

            print(f"sol_q:{sol_q} \nmotorstate: \n{current_lr_arm_motor_q} \nleft_pose: \n{left_wrist} \nright_pose: \n{right_wrist}")

            info = {"sol_tauff": sol_tauff * 0.0, "success": False}
            
            raise e
    def pre_computation(self, dof, ddof = None):
        q = np.zeros(self.model.nq)
        q[:3] = dof[-3:]
        q[3:] = dof[:-3]
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)
        pin.computeJointJacobians(self.model, self.data)
        pin.updateGlobalPlacements(self.model, self.data)
        if ddof is not None:
            dq = np.zeros(self.model.nv)
            dq[:3] = ddof[-3:]
            dq[3:] = ddof[:-3]
            pin.computeJointJacobiansTimeVariation(self.model, self.data, q, dq)
        return
    
    def forward_kinematics(self, dof):   
        # in robot base frame
        self.pre_computation(dof)
        frames = self.get_forward_kinematics()
        base_id = self.model.getFrameId("robot")
        frames = np.linalg.inv(self.data.oMf[base_id].homogeneous) @ frames
        return frames
    
    def get_jacobian(self, frame_id):
        pin_frame_id = self.pin_frame_dict[frame_id]
        pin_J = pin.getFrameJacobian(self.model, self.data, pin_frame_id, pin.LOCAL_WORLD_ALIGNED)
        J = np.zeros_like(pin_J)
        J[:, :-3] = pin_J[:, 3:]
        J[:, -3:] = pin_J[:, :3]
        return J
        
    def get_jacobian_dot(self, frame_id):
        pin_frame_id = self.pin_frame_dict[frame_id]
        pin_dJ = pin.getFrameJacobianTimeVariation(self.model, self.data, pin_frame_id, pin.LOCAL_WORLD_ALIGNED) 
        dJ = np.zeros_like(pin_dJ)
        dJ[:, :-3] = pin_dJ[:, 3:]
        dJ[:, -3:] = pin_dJ[:, :3]
        return dJ

if __name__ == "__main__":
    
    arm_ik = R1LiteMobileKinematics()
