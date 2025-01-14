from abc import ABC, abstractmethod
from spark_robot import RobotKinematics
import numpy as np

class BaseSafetyIndex(ABC):
    '''
        Base class for safety index for articulated robots and multiple safety constraints.
    '''
    
    def __init__(self,
                 robot_kinematics: RobotKinematics
                 ) -> None:
        super().__init__()
        
        self.robot_kinematics = robot_kinematics
        self.robot_cfg = robot_kinematics.robot_cfg
        self.num_dof   = len(self.robot_cfg.DoFs)
        self.num_state = self.robot_cfg.num_state
        self.num_control = len(self.robot_cfg.Control)

        # to be computed in derived class
        self.num_constraint = None
        self.phi_mask = None
    
    @abstractmethod
    def phi(self):
        '''
            [num_constraint,] Safety index function.     
        '''
        pass
    
    @abstractmethod
    def grad_phi(self):
        '''
            [num_constraint, num_dof] gradient of safety index function w.r.t. dof.
        '''
        pass

    def decode_constraint_info(self) -> dict:
        '''
            Recover constraint info from phi vector.
        '''
        raise NotImplementedError("decode_constraint_info not implemented")
        