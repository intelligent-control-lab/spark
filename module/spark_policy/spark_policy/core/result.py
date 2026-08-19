"""Results returned by composable policy components."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any, Generic, Mapping, TypeVar

T = TypeVar("T")


class ExecutionStatus(Enum):
    OK = auto()
    DEGRADED = auto()
    FAILED = auto()
    INFEASIBLE = auto()


@dataclass
class ComponentResult(Generic[T]):
    value: T
    status: ExecutionStatus = ExecutionStatus.OK
    diagnostics: Mapping[str, Any] = field(default_factory=dict)
    timestamp: float | None = None
