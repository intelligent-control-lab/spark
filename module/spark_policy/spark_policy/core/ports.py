"""Typed values exchanged between policy components.

The legacy dictionary API remains supported.  These types provide a gradual
migration path and a stable vocabulary for newly composed algorithms.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum, auto
from typing import Any, Mapping, Sequence

import numpy as np


class ControlMode(Enum):
    POSITION = auto()
    VELOCITY = auto()
    ACCELERATION = auto()
    TORQUE = auto()
    GENERIC = auto()


@dataclass(frozen=True)
class JointReference:
    position: np.ndarray
    velocity: np.ndarray | None = None
    acceleration: np.ndarray | None = None
    mask: np.ndarray | None = None


@dataclass(frozen=True)
class CartesianReference:
    frames: Mapping[str, np.ndarray]
    linear_velocity: Mapping[str, np.ndarray] | None = None
    angular_velocity: Mapping[str, np.ndarray] | None = None


@dataclass(frozen=True)
class TrajectoryReference:
    states: np.ndarray | None = None
    controls: np.ndarray | None = None
    dt: float | None = None
    frame: str | None = None


@dataclass(frozen=True)
class StateEstimate:
    state: np.ndarray
    covariance: np.ndarray | None = None
    timestamp: float | None = None
    frame: str | None = None


@dataclass(frozen=True)
class ParameterEstimate:
    parameters: np.ndarray
    information: np.ndarray | None = None
    timestamp: float | None = None


@dataclass(frozen=True)
class ControlCommand:
    values: np.ndarray
    mode: ControlMode = ControlMode.GENERIC
    duration: float | None = None


@dataclass(frozen=True)
class SafetyAssessment:
    safe: bool
    margins: np.ndarray | None = None
    active_constraints: Sequence[str] = field(default_factory=tuple)
    severity: float | None = None
    diagnostics: Mapping[str, Any] = field(default_factory=dict)
