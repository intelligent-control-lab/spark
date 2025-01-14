from abc import ABC, abstractmethod
import numpy as np
from spark_robot.base.base_robot_config import RobotConfig

class RobotKinematics(ABC):
    """
    An abstract base class for defining robot kinematics.
    """

    def __init__(self, robot_cfg: RobotConfig) -> None:
        super().__init__()
        
        self.robot_cfg = robot_cfg
        self.num_dof   = len(self.robot_cfg.DoFs)
        self.num_control = len(self.robot_cfg.Control)

    @abstractmethod
    def update_base_frame(self, trans_world2base: np.ndarray, dof: np.ndarray) -> np.ndarray:
        """
        Compute the base frame of the robot in the world frame.
        Dof may update the base frame. For example, the calcualtion of grad_phi.

        Input:
            trans_world2base: [4, 4], SE(3) of robot base frame in world frame
            dof: 1D array, current dof values defined by RobotConfig.DoFs
            
        Output:
            T: [4, 4], SE(3) of robot base frame in world frame
        
        """
        pass

    @abstractmethod
    def forward_kinematics(self, dof: np.ndarray) -> np.ndarray:
        """
        Compute the forward kinematics for the robot.

        Input:
            dof: 1D array, current dof values defined by RobotConfig.DoFs
            
        Output:
            T: [num_frames, 4, 4], SE(3) of RobotConfig.Frames in robot base frame
        
        """
        pass

    @abstractmethod
    def inverse_kinematics(self, T: np.ndarray) -> np.ndarray:
        """
        Compute the inverse kinematics for the robot.

        Input:
            T: [num_frames, 4, 4], SE(3) of frames to solve for in robot base frame
            
        Output:
            dof: 1D array, dof values defined by RobotConfig.DoFs
            info: dict with additional information
        
        """
        pass
