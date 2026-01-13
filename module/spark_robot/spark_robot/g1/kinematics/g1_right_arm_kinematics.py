import casadi          
import numpy as np
import pinocchio as pin
from pinocchio import casadi as cpin               
import os

from spark_robot.g1.kinematics.g1_fixed_base_kinematics import G1FixedBaseKinematics
from spark_robot.base.base_robot_config import RobotConfig
from spark_robot import SPARK_ROBOT_RESOURCE_DIR

class G1RightArmKinematics(G1FixedBaseKinematics):
    def __init__(self, robot_cfg: RobotConfig, **kwargs) -> None:
        super().__init__(robot_cfg, **kwargs)

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
        self.cTf_r = casadi.SX.sym("tf_r", 4, 4)
        cpin.framesForwardKinematics(self.cmodel, self.cdata, self.cq)
        
        # Get the hand joint ID and define the error function
        self.R_hand_id = self.reduced_fixed_base_model.getFrameId("R_ee")

        self.translational_error = casadi.Function(
            "translational_error",
            [self.cq, self.cTf_r],
            [
                casadi.vertcat(
                    self.cdata.oMf[self.R_hand_id].translation - self.cTf_r[:3,3]
                )
            ],
        )
        self.rotational_error = casadi.Function(
            "rotational_error",
            [self.cq, self.cTf_r],
            [
                casadi.vertcat(
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
        self.translational_cost = casadi.sumsqr(self.translational_error(self.var_q, self.param_tf_r))
        self.rotation_cost = casadi.sumsqr(self.rotational_error(self.var_q, self.param_tf_r))
        self.regularization_cost = casadi.sumsqr(self.var_q)
        self.smooth_cost = casadi.sumsqr(self.var_q - self.var_q_last)

        # Setting optimization constraints and goals
        self.opti.subject_to(self.opti.bounded(
            self.reduced_fixed_base_model.lowerPositionLimit,
            self.var_q,
            self.reduced_fixed_base_model.upperPositionLimit)
        )
        self.opti.minimize(10 * self.translational_cost + 0.0*self.rotation_cost + 0.0 * self.regularization_cost + 0.0 * self.smooth_cost)

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
  
    def add_extra_frames(self, model):   
        model.addFrame(
            pin.Frame('R_ee',
                     model.getJointId('right_wrist_yaw_joint'),
                      pin.SE3(np.eye(3),
                              np.array([0.05,0,0]).T),
                      pin.FrameType.OP_FRAME)
        )
      
    def inverse_kinematics(self, T , current_lr_arm_motor_q = None, current_lr_arm_motor_dq = None):
        right_wrist = T[0]
        if current_lr_arm_motor_q is not None:
            self.init_data = current_lr_arm_motor_q
        self.opti.set_initial(self.var_q, self.init_data)

        self.opti.set_value(self.param_tf_r, right_wrist)
        self.opti.set_value(self.var_q_last, self.init_data) # for smooth

        try:
            sol = self.opti.solve()
            sol_q = self.opti.value(self.var_q)
            # self.smooth_filter.add_data(sol_q)
            # sol_q = self.smooth_filter.filtered_data

            if current_lr_arm_motor_dq is not None:
                v = current_lr_arm_motor_dq * 0.0
            else:
                v = (sol_q - self.init_data) * 0.0

            self.init_data = sol_q

            sol_tauff = pin.rnea(self.reduced_fixed_base_model, self.reduced_fixed_base_data, sol_q, v, np.zeros(self.reduced_fixed_base_model.nv))
            sol_tauff = np.concatenate([sol_tauff, np.zeros(len(self.robot_cfg.DoFs) - sol_tauff.shape[0])], axis=0)
            
            info = {"sol_tauff": sol_tauff, "success": True}

            dof = np.zeros(self.num_dof)
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
            sol_tauff = np.concatenate([sol_tauff, np.zeros(len(self.robot_cfg.DoFs) - sol_tauff.shape[0])], axis=0)

            print(f"sol_q:{sol_q} \nmotorstate: \n{current_lr_arm_motor_q} \nright_pose: \n{right_wrist}")

            info = {"sol_tauff": sol_tauff * 0.0, "success": False}

            dof = np.zeros(self.num_dof)
            # dof[:len(sol_q)] = current_lr_arm_motor_q
            dof[:len(sol_q)] = self.init_data
            
            return dof, info

if __name__ == "__main__":
    
    arm_ik = G1RightArmKinematics()
