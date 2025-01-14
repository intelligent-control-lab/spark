import numpy as np
from spark_agent.base.base_agent import BaseAgent
from abc import ABC, abstractmethod

class SimulationAgent(BaseAgent):

    def __init__(self, robot_cfg):
        super().__init__(robot_cfg)

    def send_control(self, command: np.ndarray, use_sim_dynamics: bool = False, **kwargs) -> None:
        if use_sim_dynamics:
            self._send_control_sim_dynamics(command, **kwargs)
        else:
            self._send_control_modeled_dynamics(command, **kwargs)

    def _send_control_sim_dynamics(self, command: np.ndarray, **kwargs) -> None:
        raise NotImplementedError("send_vel_command_sim_dynamics not implemented")

    def _send_control_modeled_dynamics(self, command: np.ndarray, **kwargs) -> None:
        raise NotImplementedError("send_vel_command_modeled_dynamics not implemented")


    