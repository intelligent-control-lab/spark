from __future__ import annotations

from typing import TYPE_CHECKING

from .base_task_config import BaseTaskConfig
from spark_utils import class_to_dict
from spark_robot import RobotConfig, RobotKinematics
from abc import ABC, abstractmethod

if TYPE_CHECKING:
    from spark_agent import BaseAgent


class BaseTask(ABC):
    def __init__(
        self, robot_cfg: RobotKinematics, robot_kinematics: RobotKinematics, agent: BaseAgent
    ):

        self.robot_cfg: RobotConfig = robot_cfg
        self.robot_kinematics: RobotKinematics = robot_kinematics
        self.agent: BaseAgent = agent
        self.agent_feedback = None
        self.num_obstacle_agent = self.agent.num_obstacle_agent

    @abstractmethod
    def reset(self, feedback):
        """
        Reset the task environment.
        """
        pass

    @abstractmethod
    def get_reset_info(self):
        """
        Generate reset information for agent and environment objects.
        """
        pass

    @abstractmethod
    def step(self, feedback):
        """
        Execute a step in the task environment.
        """
        pass

    @abstractmethod
    def get_info(self, feedback) -> dict:
        """
        Get the state from the agent and the environment.
        Output the task information to the policy in the form of a dictionary.
        """
        pass
