from spark_utils import initialize_class

from .spark_algo_config import SparkAlgoConfig
from spark_robot import RobotConfig, RobotKinematics
import numpy as np
class SparkAlgoWrapper:
    
    def __init__(self, cfg : SparkAlgoConfig, robot_cfg : RobotConfig, robot_kinematics : RobotKinematics):
        
        self.cfg = cfg
        self.robot_cfg = robot_cfg
        self.safe_controller = initialize_class(
            self.cfg.safe_controller,
            robot_cfg = robot_cfg,
            robot_kinematics = robot_kinematics
        )
        
        self.policy = initialize_class(self.cfg.policy, 
                                        robot_cfg=robot_cfg,
                                        robot_kinematics=robot_kinematics)
    
    def act(self, agent_feedback, task_info):
        
        u_ref, action_info = self.policy.act(
            agent_feedback = agent_feedback,
            task_info = task_info
        )
        # compute safe control
        u_safe, safe_control_info = self.safe_controller.safe_control(
            x = agent_feedback["state"],
            u_ref = u_ref,
            agent_feedback = agent_feedback,
            task_info = task_info,
            action_info = action_info
        )
        if np.isnan(u_safe).any() or np.isinf(u_safe).any():
            print("u_safe contains nan or inf", u_safe)
            u_safe = np.zeros_like(u_safe)
            
            import ipdb;ipdb.set_trace()
            
        for control_id in self.robot_cfg.Control:
            u_safe[control_id] = np.clip(u_safe[control_id], -self.robot_cfg.ControlLimit[control_id], self.robot_cfg.ControlLimit[control_id])

        action_info['u_ref'] = u_ref
        action_info['u_safe'] = u_safe
        return u_safe, {**action_info, **safe_control_info}