from abc import ABC, abstractmethod
from enum import IntEnum, Enum
from spark_agent import BaseAgent
from spark_robot import RobotConfig, RobotKinematics
import inspect
import numpy as np

class BasePolicy(ABC):
    def __init__(self, robot_cfg: RobotConfig, robot_kinematics: RobotKinematics) -> None:
        self.robot_cfg : RobotConfig = robot_cfg
        self.robot_kinematics : RobotKinematics = robot_kinematics
        self.num_dof = len(self.robot_cfg.DoFs)
        self.num_control = len(self.robot_cfg.Control)

    @abstractmethod
    def act(self, agent_feedback: dict, task_info: dict) -> None:
        pass