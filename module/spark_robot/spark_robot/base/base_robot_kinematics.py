from abc import ABC, abstractmethod
import numpy as np
from spark_robot.base.base_robot_config import RobotConfig
import json
import os
from spark_robot import SPARK_ROBOT_ROOT

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

    def load_collision_spheres(self):
        """
        Returns:
            Dict[str, List[Dict]] mapping link_name -> list of spheres
        """
        path = os.path.join(SPARK_ROBOT_ROOT, "spark_robot", self.robot_cfg.collision_spheres_json_path)
        if path is None:
            return {}

        with open(path, "r") as f:
            data = json.load(f)

        out = {}
        for key, levels in data.items():
            link = key.split("::")[0]
            finest = max(levels.keys(), key=lambda x: int(x))
            spheres = []
            for cluster in levels[finest].values():
                spheres.extend(cluster.get("spheres", []))
            out[link] = spheres

        return out
