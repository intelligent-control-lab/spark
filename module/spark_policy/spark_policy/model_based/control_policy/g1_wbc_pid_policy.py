from spark_policy.base.base_policy import BasePolicy
from spark_robot import RobotKinematics, RobotConfig
import numpy as np

class G1WBCPIDPolicy(BasePolicy):
    def __init__(self, robot_cfg: RobotConfig, robot_kinematics: RobotKinematics) -> None:
        super().__init__(robot_cfg, robot_kinematics)

    def act(self, agent_feedback: dict, task_info: dict):
        info = {}
        dof_pos_cmd = agent_feedback["dof_pos_cmd"]
        dof_vel_cmd = agent_feedback["dof_vel_cmd"]
        dof_pos_fbk = agent_feedback["dof_pos_fbk"]
        dof_vel_fbk = agent_feedback["dof_vel_fbk"]
        goal_teleop = task_info["goal_teleop"]
        robot_base_frame = agent_feedback["robot_base_frame"]
        try:
            dof_pos_target, _ = self.robot_kinematics.inverse_kinematics([np.linalg.inv(robot_base_frame) @ goal_teleop["right"], 
                                                                          np.linalg.inv(robot_base_frame) @ goal_teleop["left"]])
            dof_vel_target = np.zeros_like(dof_vel_fbk)
            dof_pos_current = dof_pos_fbk
            dof_vel_current = dof_vel_fbk
            K_p = 100.0 * np.ones(len(self.robot_cfg.Control))
            K_d = 5.0 * np.ones(len(self.robot_cfg.Control))
            control = K_p * (dof_pos_target[-29:] - dof_pos_current[-29:]) - K_d * dof_vel_current[-29:]
            info["ik_success"] = True
        
        except Exception as e:
            print("inverse_kinematics error", e)
        
            control = np.zeros_like(dof_pos_fbk)
            info["ik_success"] = False
            
        for control_id in self.robot_cfg.Control:
            control[control_id] = np.clip(control[control_id], -self.robot_cfg.ControlLimit[control_id], self.robot_cfg.ControlLimit[control_id])

        return control, info