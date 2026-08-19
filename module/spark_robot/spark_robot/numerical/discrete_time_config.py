from __future__ import annotations

from enum import IntEnum
from typing import Any, Callable, Mapping

import numpy as np

from spark_robot.base.base_robot_config import RobotConfig
from spark_robot.base.dynamics_model import DynamicsStepContext, RobotDynamicsModel


Transition = Callable[[np.ndarray, np.ndarray, DynamicsStepContext, Mapping[str, Any]], np.ndarray]


class DiscreteTimeDynamicsModel(RobotDynamicsModel):
    """A native discrete-time model selected by a robot configuration."""

    default_integrator = "native"
    time_domain = "discrete"

    def step(self, state, control, dt, integrator="native", context=None):
        mode = str(integrator or "native").lower()
        if mode not in ("native", "direct", "discrete"):
            raise ValueError(
                f'{self.variant} is a native discrete model; integrator must be "native", not "{integrator}".'
            )
        state = np.asarray(state, dtype=float).reshape(self.state_dim)
        control = np.asarray(control, dtype=float).reshape(self.control_dim)
        context = context or DynamicsStepContext()
        return np.asarray(
            self.robot_cfg.transition(state, control, context, self.parameters),
            dtype=float,
        ).reshape(self.state_dim)

    def derivative(self, state, control, context=None):
        raise TypeError(
            f"{self.variant} defines a state transition, not a continuous-time derivative."
        )

    @property
    def is_linear(self):
        return bool(getattr(self.robot_cfg, "dynamics_is_linear", False))

    def discrete_matrices(self, dt=1.0, discretization="native"):
        if not self.is_linear:
            raise ValueError(
                f"{self.variant} is nonlinear; no constant discrete matrices are available."
            )
        return np.asarray(self.parameters["A"], dtype=float), np.asarray(
            self.parameters["B"], dtype=float
        )


class DiscreteTimeDynamicsConfig(RobotConfig):
    """Robot-config adapter for pure numerical discrete-time systems.

    It intentionally has no hardware, kinematics, collision, or simulator
    mapping.  The DoF and Control enums name numerical state/control axes so it
    can use the same configuration and agent lifecycle as physical robots.
    """

    dynamics_variant = "discrete_time"
    dynamics_order = 1
    dynamics_is_linear = False
    dynamics_full_state_only = True
    agent_class_names = {"dynamics": "DynamicsModelAgent"}
    model_only = True

    def __init__(
        self,
        state_dim: int,
        control_dim: int,
        transition: Transition,
        *,
        dynamics_variant: str | None = None,
        parameters: Mapping[str, Any] | None = None,
        state_names: list[str] | tuple[str, ...] | None = None,
        control_names: list[str] | tuple[str, ...] | None = None,
    ):
        if state_dim <= 0 or control_dim <= 0:
            raise ValueError("state_dim and control_dim must be positive.")
        state_names = tuple(state_names or (f"x{i}" for i in range(state_dim)))
        control_names = tuple(control_names or (f"u{i}" for i in range(control_dim)))
        if len(state_names) != state_dim or len(control_names) != control_dim:
            raise ValueError("State/control name counts must match their dimensions.")

        self._DoFs = IntEnum("NumericalDoFs", {name: i for i, name in enumerate(state_names)})
        self._Control = IntEnum(
            "NumericalControl", {name: i for i, name in enumerate(control_names)}
        )
        self._Frames = IntEnum("NumericalFrames", {"Origin": 0})
        self._empty_enum = IntEnum("EmptyNumericalMapping", {})
        self._transition = transition
        self.dynamics_parameters = dict(parameters or {})
        if dynamics_variant is not None:
            self.dynamics_variant = str(dynamics_variant)
        super().__init__()

    @property
    def NumTotalMotors(self):
        return 0

    @property
    def RealMotors(self):
        return self._empty_enum

    @property
    def RealMotorPosLimit(self):
        return {}

    @property
    def NormalMotor(self):
        return []

    @property
    def WeakMotor(self):
        return []

    @property
    def DelicateMotor(self):
        return []

    @property
    def DoFs(self):
        return self._DoFs

    @property
    def DefaultDoFVal(self):
        return {dof: 0.0 for dof in self.DoFs}

    @property
    def Control(self):
        return self._Control

    @property
    def ControlLimit(self):
        return {control: np.inf for control in self.Control}

    @property
    def NormalControl(self):
        return list(self.Control)

    @property
    def WeakControl(self):
        return []

    @property
    def DelicateControl(self):
        return []

    @property
    def num_state(self):
        return len(self.DoFs)

    def compose_state_from_dof(self, dof_pos, dof_vel=None):
        return np.asarray(dof_pos, dtype=float).reshape(self.num_state)

    def decompose_state_to_dof_pos(self, state):
        return np.asarray(state, dtype=float).reshape(self.num_state)

    def decompose_state_to_dof_vel(self, state):
        return np.zeros(self.num_state, dtype=float)

    def dynamics_f(self, state):
        raise TypeError("A native discrete-time config does not define dynamics_f.")

    def dynamics_g(self, state):
        raise TypeError("A native discrete-time config does not define dynamics_g.")

    def transition(self, state, control, context, parameters):
        return self._transition(state, control, context, parameters)

    def create_dynamics_model(self, state_dof_names=None, control_names=None):
        if state_dof_names is not None or control_names is not None:
            raise ValueError("Reduced views are not supported for numerical discrete-time configs.")
        return DiscreteTimeDynamicsModel(self)

    @property
    def MujocoDoFs(self):
        return self._empty_enum

    @property
    def MujocoMotors(self):
        return self._empty_enum

    @property
    def MujocoDoF_to_DoF(self):
        return {}

    @property
    def DoF_to_MujocoDoF(self):
        return {}

    @property
    def MujocoMotor_to_Control(self):
        return {}

    @property
    def RealMotor_to_Control(self):
        return {}

    @property
    def Frames(self):
        return self._Frames

    @property
    def CollisionVol(self):
        return {}

    @property
    def AdjacentCollisionVolPairs(self):
        return []

    @property
    def SelfCollisionVolIgnored(self):
        return []

    @property
    def EnvCollisionVolIgnored(self):
        return []


class LinearDiscreteDynamicsConfig(DiscreteTimeDynamicsConfig):
    """Config-selected ``x[k+1] = A x[k] + B u[k] + disturbance`` model."""

    dynamics_variant = "linear_discrete"
    dynamics_is_linear = True

    def __init__(
        self,
        A,
        B,
        *,
        disturbance: Transition | None = None,
        dynamics_variant: str | None = None,
        state_names=None,
        control_names=None,
    ):
        A = np.asarray(A, dtype=float)
        B = np.asarray(B, dtype=float)
        if A.ndim != 2 or A.shape[0] != A.shape[1]:
            raise ValueError("A must be square.")
        if B.ndim != 2 or B.shape[0] != A.shape[0]:
            raise ValueError("B must have the same row count as A.")
        self._disturbance = disturbance
        super().__init__(
            A.shape[0],
            B.shape[1],
            self._linear_transition,
            dynamics_variant=dynamics_variant or self.dynamics_variant,
            parameters={"A": A.copy(), "B": B.copy()},
            state_names=state_names,
            control_names=control_names,
        )

    def _linear_transition(self, state, control, context, parameters):
        result = np.asarray(parameters["A"]) @ state + np.asarray(parameters["B"]) @ control
        if self._disturbance is not None:
            result = result + np.asarray(
                self._disturbance(state, control, context, parameters), dtype=float
            ).reshape(self.num_state)
        return result
