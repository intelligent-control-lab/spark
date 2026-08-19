from __future__ import annotations

import numpy as np

from spark_task.base.base_task import BaseTask


class DynamicsTask(BaseTask):
    """Minimal task lifecycle for pure numerical dynamics experiments."""

    def __init__(
        self,
        robot_cfg,
        robot_kinematics,
        agent,
        *,
        initial_state=None,
        max_steps: int | None = None,
        seed: int | None = None,
    ) -> None:
        super().__init__(robot_cfg, robot_kinematics, agent)
        if initial_state is None:
            initial_state = [robot_cfg.DefaultDoFVal[dof] for dof in robot_cfg.DoFs]
            if int(robot_cfg.dynamics_order) == 2:
                model = getattr(agent, "dynamics_model", None)
                velocity_dim = (
                    model.state_dim - model.position_dim
                    if model is not None and model.full_state_only
                    else robot_cfg.num_dof
                )
                initial_state = robot_cfg.compose_state_from_dof(
                    np.asarray(initial_state, dtype=float),
                    np.zeros(velocity_dim, dtype=float),
                )
        state_dim = getattr(agent, "dynamics_model", None)
        state_dim = robot_cfg.num_state if state_dim is None else state_dim.state_dim
        self.initial_state = np.asarray(initial_state, dtype=float).reshape(state_dim)
        self.max_steps = None if max_steps is None else int(max_steps)
        self.seed = seed
        self.step_count = 0
        self.agent_feedback = None

    def reset(self, feedback):
        self.step_count = 0
        self.agent_feedback = feedback

    def get_reset_info(self):
        return {"state": self.initial_state.copy(), "seed": self.seed}

    def step(self, feedback):
        self.step_count += 1
        self.agent_feedback = feedback

    def get_info(self):
        return {
            "done": self.max_steps is not None and self.step_count >= self.max_steps,
            "step_count": self.step_count,
        }
