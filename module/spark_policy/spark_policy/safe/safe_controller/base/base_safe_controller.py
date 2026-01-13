from .base_safe_controller_config import SafeControllerConfig
from spark_robot import RobotConfig, RobotKinematics
from spark_policy.safe.safe_algo import BaseSafetyIndex
from spark_policy.safe.safe_algo import BaseSafeAlgorithm
import numpy as np
from spark_utils import initialize_class

class BaseSafeController:
    
    def __init__(self,
                 robot_cfg: RobotConfig = None,
                 robot_kinematics: RobotKinematics = None, 
                 **kwargs
                 ) -> None:
        
        safety_index_cfg = kwargs.get('safety_index', None)
        safe_algo_cfg = kwargs.get('safe_algo', None)

        if robot_cfg is not None:
            self.robot_cfg : RobotConfig = robot_cfg
            print("BaseSafeController: using provided robot_cfg")
        else:
            self.robot_cfg : RobotConfig = initialize_class(self.cfg.robot.cfg)
            print("BaseSafeController: initializing robot_cfg")
        
        if robot_kinematics is not None:
            self.robot_kinematics : RobotKinematics = robot_kinematics
            print("BaseSafeController: using provided robot_kinematics")
        else:
            self.robot_kinematics : RobotKinematics = initialize_class(self.cfg.robot.kinematics, robot_cfg=self.robot_cfg)
            print("BaseSafeController: initializing robot_kinematics")
            
        self.safety_index : BaseSafetyIndex = initialize_class(safety_index_cfg, robot_kinematics=self.robot_kinematics)
        self.safe_algo : BaseSafeAlgorithm = initialize_class(safe_algo_cfg, safety_index=self.safety_index, robot_kinematics=self.robot_kinematics)
    
    def safe_control(self,
                     x             : np.ndarray,
                     u_ref         : np.ndarray,
                     agent_feedback: dict,
                     task_info     : dict,
                     action_info   : dict
                     ):
        
        return self.safe_algo.safe_control(x, u_ref, agent_feedback, task_info, action_info)
