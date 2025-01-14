from abc import ABC, abstractmethod
import numpy as np
from spark_robot import RobotConfig

class BaseAgent(ABC):
    """
    An abstract base class for defining robot agents.
    """

    def __init__(self, robot_cfg: RobotConfig) -> None:
        super().__init__()
        self.robot_cfg = robot_cfg
        self.num_dof = len(self.robot_cfg.DoFs)
        self.num_control = len(self.robot_cfg.Control)
        self.num_obstacle_agent = 0

        self.dof_pos_cmd = None
        self.dof_vel_cmd = None
        self.dof_acc_cmd = None

        self.dof_pos_fbk = None
        self.dof_vel_fbk = None
        self.dof_acc_fbk = None

    @abstractmethod
    def reset(self) -> None:
        """
        Reset the robot agent.
        """
        pass

    @abstractmethod
    def compose_state(self) -> np.ndarray:
        """
        Compose the state of the robot agent that matches the robot config.
        """
        pass

    @abstractmethod
    def send_control(self, control: np.ndarray) -> None:
        """
        Execute control on the robot agent.
        """
        pass

    @abstractmethod
    def post_control_processing(self, **kwargs) -> None:
        """
        Processes following commands: sim step, state refresh, render, etc.
        """
        pass

    def step(self, control: np.ndarray, **kwargs) -> None:
        """
        Step the robot agent with a given action.
        Action should be defined in RobotConfig
        """

        self.send_control(control, **kwargs)
        self.post_control_processing(**kwargs)

    @abstractmethod
    def get_feedback(self) -> None:
        """
        Get the current state of the robot agent.
        """
        pass
