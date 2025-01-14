from spark_utils import initialize_class

from .spark_algo_config import SparkAlgoConfig
from spark_safe.safe_controller import BaseSafeController
from spark_policy import BasePolicy
from spark_robot import RobotConfig, RobotKinematics
import numpy as np
class SparkAlgoWrapper:
    
    def __init__(self, cfg : SparkAlgoConfig, robot_cfg : RobotConfig, robot_kinematics : RobotKinematics):
        self.cfg = cfg
        
        # self.safe_controller : BaseSafeController = initialize_class(
        #     self.cfg.safe_controller, robot_cfg=robot_cfg, robot_kinematics=robot_kinematics
        # )
        
        self.safe_controller : BaseSafeController = BaseSafeController(
            cfg = self.cfg.safe_controller,
            robot_cfg = robot_cfg,
            robot_kinematics = robot_kinematics
        )
        
        self.policy : BasePolicy = initialize_class(self.cfg.policy, 
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
        
        return u_safe, {**action_info, **safe_control_info}