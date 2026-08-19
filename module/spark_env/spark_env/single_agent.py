from spark_agent import BaseAgent
from spark_task import BaseTask
from spark_utils import initialize_class

from .core import Environment, EnvironmentOutput, SparkEnvConfig


class SparkEnvironment(Environment):
    """Coordinate the existing single-agent, single-task lifecycle."""

    def __init__(self, cfg: SparkEnvConfig, robot_cfg, robot_kinematics=None):
        self.cfg = cfg
        self.agent: BaseAgent = initialize_class(cfg.agent, robot_cfg=robot_cfg)
        self.task: BaseTask = initialize_class(
            cfg.task,
            robot_cfg=robot_cfg,
            robot_kinematics=robot_kinematics,
            agent=self.agent,
        )

    def reset_output(self) -> EnvironmentOutput:
        reset_info = self.task.get_reset_info()
        self.agent.reset(reset_info)
        agent_feedback = self.agent.get_feedback()
        self.task.reset(agent_feedback)
        task_info = self.task.get_info()
        return EnvironmentOutput(
            agent_feedback=agent_feedback,
            task_info=task_info,
            done=bool(task_info.get("done", False)),
        )

    def transition_task_output(self) -> EnvironmentOutput:
        """Resample task state while preserving simulator and policy state."""
        # Advance the deterministic task seed exactly as a full reset would,
        # but intentionally ignore the returned agent reset command.
        self.task.get_reset_info()
        agent_feedback = self.agent.get_feedback()
        self.task.reset(agent_feedback)
        task_info = self.task.get_info()
        return EnvironmentOutput(
            agent_feedback=agent_feedback,
            task_info=task_info,
            done=bool(task_info.get("done", False)),
            diagnostics={"physical_reset": False, "task_transition": True},
        )

    def step_output(self, action, action_info=None) -> EnvironmentOutput:
        self.agent.step(action, action_info=action_info)
        agent_feedback = self.agent.get_feedback()
        self.task.step(agent_feedback)
        task_info = self.task.get_info()
        return EnvironmentOutput(
            agent_feedback=agent_feedback,
            task_info=task_info,
            done=bool(task_info.get("done", False)),
        )


SparkEnvWrapper = SparkEnvironment
