from abc import ABC, abstractmethod

from .types import EnvironmentOutput


class Environment(ABC):
    @abstractmethod
    def reset_output(self) -> EnvironmentOutput: ...

    @abstractmethod
    def step_output(self, action, action_info=None) -> EnvironmentOutput: ...

    def reset(self):
        return self.reset_output().as_legacy_tuple()

    def transition_task_output(self) -> EnvironmentOutput:
        """Start a new task episode without resetting the physical agent."""
        return self.reset_output()

    def transition_task(self):
        return self.transition_task_output().as_legacy_tuple()

    def step(self, action, action_info=None):
        return self.step_output(action, action_info).as_legacy_tuple()
