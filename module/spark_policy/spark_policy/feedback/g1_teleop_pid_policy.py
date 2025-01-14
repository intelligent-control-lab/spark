from spark_policy.base.base_policy import BasePolicy
from spark_agent import BaseAgent
from spark_robot import RobotKinematics, RobotConfig
import numpy as np

class G1TeleopPIDPolicy(BasePolicy):
    def __init__(self, robot_cfg: RobotConfig, robot_kinematics: RobotKinematics) -> None:
        super().__init__(robot_cfg, robot_kinematics)
        self.pos_K_p = np.ones(len(self.robot_cfg.DoFs))
        self.pos_K_d = np.zeros(len(self.robot_cfg.DoFs))
    
    def tracking_pos_with_vel(self, 
                              desired_dof_pos,
                              dof_pos,
                              dof_vel):
        
        nominal_dof_vel = self.pos_K_p * (desired_dof_pos - dof_pos) - self.pos_K_d * dof_vel
        
        return nominal_dof_vel

    def act(self, agent_feedback: dict, task_info: dict):
        
        dof_pos_cmd = agent_feedback["dof_pos_cmd"]
        dof_vel_cmd = agent_feedback["dof_vel_cmd"]
        
        goal_teleop = task_info["goal_teleop"]
        
        desired_dof_pos, _ = self.robot_kinematics.inverse_kinematics([goal_teleop["left"], goal_teleop["right"]])
        
        dof_control = self.tracking_pos_with_vel(desired_dof_pos, dof_pos_cmd, dof_vel_cmd)

        info = {}

        return dof_control, info