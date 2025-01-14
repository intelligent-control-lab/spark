from abc import abstractmethod
import numpy as np

from spark_safe.safe_algo.base.base_safe_algo import BaseSafeAlgorithm
from spark_safe.safe_algo.value_based.base.base_safety_index import BaseSafetyIndex
from spark_robot import RobotKinematics

class ValueBasedSafeAlgorithm(BaseSafeAlgorithm):
    def __init__(self, safety_index: BaseSafetyIndex, **kwargs):
        
        super().__init__(**kwargs)
        
        self.safety_index = safety_index

        self.control_max    = np.array([self.robot_cfg.ControlLimit[i] for i in self.robot_cfg.Control])
        self.control_min    = np.array([-self.robot_cfg.ControlLimit[i] for i in self.robot_cfg.Control])
        
    # @abstractmethod
    # def kappa(self, x):
    #     '''
    #         K-class function for value based safe algorithms

    #         control constraint: phi_dot <= -k(phi)

    #         x: R^n
    #         k: R^n -> R^n
    #     '''
    #     pass
    
    @abstractmethod
    def safe_control(self,
                     x             : np.ndarray,
                     u_ref         : np.ndarray,
                     agent_feedback: dict,
                     task_info     : dict,
                     action_info   : dict
                     ) : 

        pass

