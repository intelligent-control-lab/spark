"""User-facing executable policy interface."""

from abc import ABC, abstractmethod
from typing import Any

import numpy as np

from .context import ResetContext
from spark_robot import RobotConfig, RobotKinematics


class Policy(ABC):
    """A complete policy consumable by :mod:`spark_pipeline`."""

    def reset(self, context: ResetContext | None = None) -> None:
        """Reset episodic state. Stateless policies may use this no-op."""

    @abstractmethod
    def act(
        self,
        agent_feedback: dict,
        task_info: dict,
    ) -> tuple[np.ndarray, dict[str, Any]]: ...


class BasePolicy(Policy):
    """Robot-aware policy base retained as part of the new core API."""

    def __init__(
        self,
        robot_cfg: RobotConfig,
        robot_kinematics: RobotKinematics | None,
    ) -> None:
        self.robot_cfg = robot_cfg
        self.robot_kinematics = robot_kinematics
        self.num_dof = len(robot_cfg.DoFs)
        self.num_control = len(robot_cfg.Control)
