from __future__ import annotations

import numpy as np

from spark_agent.base.base_agent import BaseAgent
from spark_agent.dynamics import DynamicsExecutor


class DynamicsModelAgent(BaseAgent):
    """Simulator-independent agent backed only by robot-config dynamics."""

    def __init__(
        self,
        robot_cfg,
        *,
        dt: float = 0.02,
        integrator: str | None = None,
        substeps: int = 1,
        seed: int | None = None,
        state_dof_names=None,
        control_names=None,
    ) -> None:
        super().__init__(robot_cfg)
        self.dynamics_model = robot_cfg.create_dynamics_model(
            state_dof_names=state_dof_names,
            control_names=control_names,
        )
        self.dynamics_executor = DynamicsExecutor(
            self.dynamics_model,
            dt=dt,
            integrator=integrator,
            substeps=substeps,
            seed=seed,
        )
        self.last_control = np.zeros(self.dynamics_model.control_dim, dtype=float)

    def _default_state(self) -> np.ndarray:
        dof_pos = np.array(
            [self.robot_cfg.DefaultDoFVal[dof] for dof in self.robot_cfg.DoFs],
            dtype=float,
        )
        velocity_dim = (
            self.dynamics_model.state_dim - self.dynamics_model.position_dim
            if self.dynamics_model.full_state_only
            else self.num_dof
        )
        return self.dynamics_model.extract_state(
            dof_pos,
            np.zeros(velocity_dim, dtype=float),
        )

    def reset(self, agent_reset_info=None, **kwargs) -> None:
        super().reset()
        reset_info = dict(agent_reset_info or {})
        initial_state = reset_info.get(
            "state", reset_info.get("reset_state", self._default_state())
        )
        self.dynamics_executor.reset(
            initial_state,
            time=float(reset_info.get("time", 0.0)),
            seed=reset_info.get("seed"),
        )
        self.last_control = np.zeros(self.dynamics_model.control_dim, dtype=float)
        self._update_dof_views()

    def send_control(self, control: np.ndarray, **kwargs) -> None:
        action_info = kwargs.get("action_info") or {}
        self.last_control = np.asarray(control, dtype=float).reshape(
            self.dynamics_model.control_dim
        )
        self.dynamics_executor.step(
            self.last_control,
            parameters=action_info.get("dynamics_parameters"),
            exogenous=action_info.get("dynamics_exogenous"),
        )
        self._update_dof_views()

    def _update_dof_views(self) -> None:
        state = self.dynamics_executor.state
        full_state = self.dynamics_model.expand_state(state)
        self.dof_pos_cmd = self.robot_cfg.decompose_state_to_dof_pos(full_state)
        self.dof_vel_cmd = self.robot_cfg.decompose_state_to_dof_vel(full_state)
        self.dof_pos_fbk = np.asarray(self.dof_pos_cmd, dtype=float).copy()
        self.dof_vel_fbk = np.asarray(self.dof_vel_cmd, dtype=float).copy()
        self.dof_acc_cmd = np.zeros(self.num_dof, dtype=float)
        self.dof_acc_fbk = self.dof_acc_cmd.copy()

    def get_feedback(self):
        return {
            "state": self.dynamics_executor.state.copy(),
            "time": self.dynamics_executor.time,
            "step_index": self.dynamics_executor.step_index,
            "dof_pos_fbk": self.dof_pos_fbk.copy(),
            "dof_vel_fbk": self.dof_vel_fbk.copy(),
            "dof_pos_cmd": self.dof_pos_cmd.copy(),
            "dof_vel_cmd": self.dof_vel_cmd.copy(),
            "control": self.last_control.copy(),
        }

    def _compose_cmd_state(self):
        return self.dynamics_executor.state.copy()
