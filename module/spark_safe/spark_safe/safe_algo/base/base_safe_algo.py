from abc import ABC, abstractmethod
from spark_robot import RobotKinematics
import numpy as np

class BaseSafeAlgorithm(ABC):
    def __init__(self, robot_kinematics: RobotKinematics, **kwargs):
        super().__init__()
        
        self.robot_kinematics = robot_kinematics
        self.robot_cfg        = robot_kinematics.robot_cfg
        self.num_dof          = len(self.robot_cfg.DoFs)
        self.num_control      = len(self.robot_cfg.Control)
    
    @abstractmethod
    def safe_control(self,
                     x             : np.ndarray,
                     u_ref         : np.ndarray,
                     agent_feedback: dict,
                     task_info     : dict,
                     action_info   : dict):
        pass
