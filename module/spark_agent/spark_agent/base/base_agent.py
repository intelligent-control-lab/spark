from abc import ABC, abstractmethod
import numpy as np
from spark_robot import RobotConfig

class BaseAgent(ABC):
    """
    Abstract base class for defining robot agents.
    Provides a structure for handling robot control, feedback, and state updates.
    """

    def __init__(self, robot_cfg: RobotConfig) -> None:
        """
        Initialize the robot agent with the given configuration.
        
        Args:
            robot_cfg (RobotConfig): Configuration object for the robot.
        """
        super().__init__()
        self.robot_cfg = robot_cfg
        
        # Degrees of freedom (DoFs) and control parameters
        self.num_dof = len(self.robot_cfg.DoFs)
        self.num_control = len(self.robot_cfg.Control)
        self.num_obstacle_agent = 0  # Number of obstacle agents (default: 0)

        # Command variables (desired values for DoFs)
        self.dof_pos_cmd = None  # Position command
        self.dof_vel_cmd = None  # Velocity command
        self.dof_acc_cmd = None  # Acceleration command

        # Feedback variables (actual sensor readings)
        self.dof_pos_fbk = None  # Position feedback
        self.dof_vel_fbk = None  # Velocity feedback
        self.dof_acc_fbk = None  # Acceleration feedback
        
        # Current DoF state (latest computed values)
        self.dof_pos = None  # Current position
        self.dof_vel = None  # Current velocity
        self.dof_acc = None  # Current acceleration

    def reset(self) -> None:
        """
        Reset the robot agent's command, feedback, and state variables.
        """
        self.dof_pos_cmd = None
        self.dof_vel_cmd = None
        self.dof_acc_cmd = None

        self.dof_pos_fbk = None
        self.dof_vel_fbk = None
        self.dof_acc_fbk = None
        
        self.dof_pos = None
        self.dof_vel = None
        self.dof_acc = None

    @abstractmethod
    def send_control(self, control: np.ndarray) -> None:
        """
        Execute the given control input on the robot.
        
        Args:
            control (np.ndarray): Control input array.
        """
        pass

    def _post_control_processing(self, **kwargs) -> None:
        """
        Perform any necessary post-control operations, such as:
        - Simulation step
        - State refresh
        - Rendering
        
        Args:
            **kwargs: Additional arguments for processing.
        """
        pass

    def step(self, control: np.ndarray, **kwargs) -> None:
        """
        Perform a single step of the robot agent by sending control input and executing post-processing.
        
        Args:
            control (np.ndarray): Control input array.
            **kwargs: Additional arguments for processing.
        """
        self.send_control(control, **kwargs)
        self._post_control_processing(**kwargs)

    @abstractmethod
    def get_feedback(self) -> None:
        """
        Retrieve the current state feedback of the robot agent.
        """
        pass

    def compose_cmd_state(self) -> np.ndarray:
        """
        Compose the state representation of the robot agent based on the robot configuration.
        This method should be implemented in subclasses.
        
        Returns:
            np.ndarray: State representation of the robot agent.
        """
        pass