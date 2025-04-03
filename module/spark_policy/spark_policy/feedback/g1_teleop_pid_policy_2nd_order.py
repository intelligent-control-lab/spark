from spark_policy.base.base_policy import BasePolicy
from spark_agent import BaseAgent
from spark_robot import RobotKinematics, RobotConfig
import numpy as np
from scipy.spatial.transform import Rotation as R

class G1TeleopPIDPolicy2(BasePolicy):
    def __init__(self, robot_cfg: RobotConfig, robot_kinematics: RobotKinematics) -> None:
        super().__init__(robot_cfg, robot_kinematics)
        self.pos_K_p = 5.0 * np.ones(len(self.robot_cfg.DoFs))
        self.pos_K_d = 1.0 * np.ones(len(self.robot_cfg.DoFs))

    def tracking_pos_with_acc(self, 
                              desired_dof_pos,
                              dof_pos,
                              dof_vel):
        nominal_dof_vel = self.pos_K_p * (desired_dof_pos - dof_pos) - self.pos_K_d * dof_vel
        nominal_dof_acc = (nominal_dof_vel - dof_vel)
        
        for control_id in self.robot_cfg.Control:
            nominal_dof_acc[control_id] = np.clip(nominal_dof_acc[control_id], -self.robot_cfg.ControlLimit[control_id], self.robot_cfg.ControlLimit[control_id])
        return nominal_dof_acc

    def act(self, agent_feedback: dict, task_info: dict, dt=0.0001):
        dof_pos_cmd = agent_feedback["dof_pos_cmd"]
        dof_vel_cmd = agent_feedback["dof_vel_cmd"]
        
        goal_teleop = task_info["goal_teleop"]
        info = {}
        
        try:
            desired_dof_pos, info = self.robot_kinematics.inverse_kinematics([goal_teleop["left"], goal_teleop["right"]])
            dof_control = self.tracking_pos_with_acc(desired_dof_pos, dof_pos_cmd, dof_vel_cmd)
            info["ik_success"] = True
        
        except Exception as e:
            print("inverse_kinematics error", e)
        
            dof_control = np.zeros_like(dof_pos_cmd)
            info["ik_success"] = False
            raise e
        
        return dof_control, info  # Now it returns acceleration commands