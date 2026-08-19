"""Declarative properties for composable policy components."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import FrozenSet


class ComponentRole(Enum):
    ESTIMATOR = auto()
    PLANNER = auto()
    CONTROLLER = auto()
    SAFETY_MONITOR = auto()
    SAFETY_FILTER = auto()
    COMPOSITE_POLICY = auto()


class PortKind(Enum):
    OBSERVATION = auto()
    ESTIMATED_STATE = auto()
    PARAMETER_ESTIMATE = auto()
    GOAL = auto()
    REFERENCE = auto()
    ENVIRONMENT_MODEL = auto()
    PATH = auto()
    STATE_TRAJECTORY = auto()
    CONTROL_TRAJECTORY = auto()
    CONTROL_COMMAND = auto()
    SAFETY_ASSESSMENT = auto()


class HorizonKind(Enum):
    INSTANTANEOUS = auto()
    SHORT_HORIZON = auto()
    LONG_HORIZON = auto()
    VARIABLE = auto()


class ImplementationKind(Enum):
    ANALYTIC = auto()
    MODEL_BASED = auto()
    LEARNED = auto()
    OPTIMIZATION = auto()
    EXTERNAL = auto()
    HYBRID = auto()


class ExecutionKind(Enum):
    FEEDBACK = auto()
    FEEDFORWARD = auto()
    HYBRID = auto()


class PurposeKind(Enum):
    NOMINAL = auto()
    SAFETY = auto()
    SUPPORT = auto()


class Statefulness(Enum):
    STATELESS = auto()
    EPISODIC = auto()
    PERSISTENT = auto()


@dataclass(frozen=True)
class ComponentSpec:
    """Machine-readable component capabilities, independent of package layout."""

    role: ComponentRole
    inputs: FrozenSet[PortKind] = field(default_factory=frozenset)
    outputs: FrozenSet[PortKind] = field(default_factory=frozenset)
    horizon: HorizonKind = HorizonKind.INSTANTANEOUS
    implementation: FrozenSet[ImplementationKind] = field(default_factory=frozenset)
    execution: ExecutionKind = ExecutionKind.FEEDBACK
    purpose: FrozenSet[PurposeKind] = field(
        default_factory=lambda: frozenset({PurposeKind.NOMINAL})
    )
    statefulness: Statefulness = Statefulness.STATELESS
