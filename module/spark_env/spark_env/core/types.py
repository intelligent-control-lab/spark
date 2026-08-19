from dataclasses import dataclass, field
from typing import Any, Mapping


@dataclass
class EnvironmentOutput:
    agent_feedback: dict
    task_info: dict
    done: bool = False
    diagnostics: Mapping[str, Any] = field(default_factory=dict)

    def as_legacy_tuple(self):
        return self.agent_feedback, self.task_info
