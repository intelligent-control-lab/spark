"""Metadata catalog for legacy and role-based policy classes."""

from __future__ import annotations

from .metadata import (
    ComponentRole,
    ComponentSpec,
    ExecutionKind,
    HorizonKind,
    ImplementationKind,
    PortKind,
    PurposeKind,
    Statefulness,
)

_SPECS: dict[str, ComponentSpec] = {}


def register_component_spec(name: str, spec: ComponentSpec) -> None:
    _SPECS[name] = spec


def get_component_spec(component_or_name) -> ComponentSpec | None:
    if isinstance(component_or_name, str):
        name = component_or_name
    elif isinstance(component_or_name, type):
        name = component_or_name.__name__
    else:
        explicit = getattr(component_or_name, "spec", None)
        if isinstance(explicit, ComponentSpec):
            return explicit
        name = type(component_or_name).__name__
    return _SPECS.get(name)


def component_specs() -> dict[str, ComponentSpec]:
    return dict(_SPECS)


def _spec(
    role,
    inputs,
    outputs,
    *,
    horizon=HorizonKind.INSTANTANEOUS,
    implementation=(ImplementationKind.MODEL_BASED,),
    execution=ExecutionKind.FEEDBACK,
    purpose=(PurposeKind.NOMINAL,),
    statefulness=Statefulness.STATELESS,
):
    return ComponentSpec(
        role=role,
        inputs=frozenset(inputs),
        outputs=frozenset(outputs),
        horizon=horizon,
        implementation=frozenset(implementation),
        execution=execution,
        purpose=frozenset(purpose),
        statefulness=statefulness,
    )


_STATE_REFERENCE = (PortKind.ESTIMATED_STATE, PortKind.REFERENCE)
_CONTROL = (PortKind.CONTROL_COMMAND,)

for _name in (
    "BasePIDPolicy",
    "BenchmarkPIDPolicy",
    "TeleopPIDPolicy",
    "UnitreeG1WBCPIDPolicy",
    "JointPIDController",
    "CartesianPIDController",
    "IKJointPIDController",
    "MultiTargetPIDPolicy",
):
    register_component_spec(
        _name,
        _spec(ComponentRole.CONTROLLER, _STATE_REFERENCE, _CONTROL),
    )

register_component_spec(
    "LeggedLocomotionTrackingPolicy",
    _spec(
        ComponentRole.CONTROLLER,
        _STATE_REFERENCE,
        _CONTROL,
        implementation=(ImplementationKind.ANALYTIC,),
        execution=ExecutionKind.FEEDBACK,
        statefulness=Statefulness.EPISODIC,
    ),
)

for _name in ("LQRPolicy", "FiniteHorizonLQRPolicy"):
    register_component_spec(
        _name,
        _spec(
            ComponentRole.CONTROLLER,
            _STATE_REFERENCE,
            _CONTROL,
            horizon=HorizonKind.SHORT_HORIZON
            if _name.startswith("Finite")
            else HorizonKind.LONG_HORIZON,
        ),
    )

for _name in (
    "KalmanFilterEstimator",
    "SteadyStateKalmanFilterEstimator",
    "ExtendedKalmanFilterEstimator",
    "UnscentedKalmanFilterEstimator",
):
    register_component_spec(
        _name,
        _spec(
            ComponentRole.ESTIMATOR,
            (PortKind.OBSERVATION,),
            (PortKind.ESTIMATED_STATE,),
            implementation=(ImplementationKind.ANALYTIC, ImplementationKind.MODEL_BASED),
            purpose=(PurposeKind.SUPPORT,),
            statefulness=Statefulness.PERSISTENT,
        ),
    )

for _name in ("RecursiveLeastSquaresEstimator", "GradientParameterEstimator"):
    register_component_spec(
        _name,
        _spec(
            ComponentRole.ESTIMATOR,
            (PortKind.OBSERVATION,),
            (PortKind.PARAMETER_ESTIMATE,),
            implementation=(ImplementationKind.ANALYTIC, ImplementationKind.MODEL_BASED),
            purpose=(PurposeKind.SUPPORT,),
            statefulness=Statefulness.PERSISTENT,
        ),
    )

for _name in ("MRACPolicy", "ModelReferenceAdaptiveController"):
    register_component_spec(
        _name,
        _spec(
            ComponentRole.CONTROLLER,
            _STATE_REFERENCE,
            _CONTROL,
            implementation=(ImplementationKind.ANALYTIC, ImplementationKind.MODEL_BASED),
            execution=ExecutionKind.FEEDBACK,
            statefulness=Statefulness.PERSISTENT,
        ),
    )

for _name in (
    "FrequencyILCPolicy",
    "TimeDomainILCPolicy",
    "FrequencyDomainILC",
    "TimeDomainILC",
):
    register_component_spec(
        _name,
        _spec(
            ComponentRole.CONTROLLER,
            (PortKind.REFERENCE, PortKind.CONTROL_TRAJECTORY),
            (PortKind.CONTROL_TRAJECTORY,),
            horizon=HorizonKind.LONG_HORIZON,
            implementation=(ImplementationKind.MODEL_BASED, ImplementationKind.LEARNED),
            execution=ExecutionKind.FEEDFORWARD,
            statefulness=Statefulness.EPISODIC,
        ),
    )

for _name in ("LinearMPCPolicy", "InputConstrainedMPCPolicy", "ConstrainedMPCPolicy"):
    register_component_spec(
        _name,
        _spec(
            ComponentRole.CONTROLLER,
            _STATE_REFERENCE,
            _CONTROL,
            horizon=HorizonKind.SHORT_HORIZON,
            implementation=(ImplementationKind.MODEL_BASED, ImplementationKind.OPTIMIZATION),
            statefulness=Statefulness.EPISODIC,
        ),
    )

for _name in ("RRTConnect", "RRTConnectPolicy"):
    register_component_spec(
        _name,
        _spec(
            ComponentRole.PLANNER,
            (PortKind.ESTIMATED_STATE, PortKind.GOAL, PortKind.ENVIRONMENT_MODEL),
            (PortKind.PATH,),
            horizon=HorizonKind.LONG_HORIZON,
            execution=ExecutionKind.FEEDFORWARD,
            statefulness=Statefulness.EPISODIC,
        ),
    )

for _name in ("ILQRPolicy", "TrajOptPolicy"):
    register_component_spec(
        _name,
        _spec(
            ComponentRole.COMPOSITE_POLICY,
            (PortKind.ESTIMATED_STATE, PortKind.GOAL),
            (PortKind.CONTROL_COMMAND, PortKind.STATE_TRAJECTORY, PortKind.CONTROL_TRAJECTORY),
            horizon=HorizonKind.LONG_HORIZON,
            implementation=(ImplementationKind.MODEL_BASED, ImplementationKind.OPTIMIZATION),
            execution=ExecutionKind.HYBRID,
            statefulness=Statefulness.EPISODIC,
        ),
    )

register_component_spec(
    "TrajTrackingPolicy",
    _spec(
        ComponentRole.CONTROLLER,
        (PortKind.ESTIMATED_STATE, PortKind.STATE_TRAJECTORY),
        _CONTROL,
        horizon=HorizonKind.SHORT_HORIZON,
        statefulness=Statefulness.EPISODIC,
    ),
)

register_component_spec(
    "UnitreeG1WBCPolicy",
    _spec(
        ComponentRole.CONTROLLER,
        _STATE_REFERENCE + (PortKind.ENVIRONMENT_MODEL,),
        _CONTROL,
        implementation=(ImplementationKind.MODEL_BASED, ImplementationKind.OPTIMIZATION),
        purpose=(PurposeKind.NOMINAL, PurposeKind.SAFETY),
        statefulness=Statefulness.EPISODIC,
    ),
)

for _name in (
    "UnitreeG1SportPolicy",
    "UnitreeG1WBTPolicy",
    "UnitreeG1BatchedWBTPolicy",
    "UnitreeG1SonicPolicy",
    "UnitreeG1BatchedSonicPolicy",
):
    register_component_spec(
        _name,
        _spec(
            ComponentRole.CONTROLLER,
            (PortKind.OBSERVATION, PortKind.GOAL),
            _CONTROL,
            implementation=(ImplementationKind.LEARNED, ImplementationKind.HYBRID),
            execution=ExecutionKind.FEEDBACK,
            statefulness=Statefulness.EPISODIC,
        ),
    )

for _name in (
    "UnitreeG1WBTSafePolicy",
    "UnitreeG1SportSafePolicy",
    "UnitreeG1SonicSafePolicy",
):
    register_component_spec(
        _name,
        _spec(
            ComponentRole.COMPOSITE_POLICY,
            (PortKind.OBSERVATION, PortKind.GOAL),
            _CONTROL,
            implementation=(ImplementationKind.LEARNED, ImplementationKind.HYBRID),
            execution=ExecutionKind.FEEDBACK,
            purpose=(PurposeKind.NOMINAL, PurposeKind.SAFETY),
            statefulness=Statefulness.EPISODIC,
        ),
    )

register_component_spec(
    "UnitreeG1WBCComposedPolicy",
    _spec(
        ComponentRole.COMPOSITE_POLICY,
        (PortKind.OBSERVATION, PortKind.GOAL),
        _CONTROL,
        implementation=(ImplementationKind.MODEL_BASED, ImplementationKind.OPTIMIZATION),
        execution=ExecutionKind.FEEDBACK,
        statefulness=Statefulness.EPISODIC,
    ),
)

for _name in (
    "BaseSafetyIndex",
    "BasicCollisionSafetyIndex",
    "FirstOrderCollisionSafetyIndex",
    "FirstOrderCollisionSafetyIndexApprox",
    "SecondOrderCollisionSafetyIndex",
    "SecondOrderCollisionSafetyIndexApprox",
    "SecondOrderNNCollisionSafetyIndex",
):
    register_component_spec(
        _name,
        _spec(
            ComponentRole.SAFETY_MONITOR,
            (PortKind.ESTIMATED_STATE, PortKind.ENVIRONMENT_MODEL),
            (PortKind.SAFETY_ASSESSMENT,),
            purpose=(PurposeKind.SAFETY,),
        ),
    )

for _name in (
    "BaseSafeAlgorithm",
    "ByPassSafeControl",
    "ValueBasedSafeAlgorithm",
    "BasicControlBarrierFunction",
    "RelaxedControlBarrierFunction",
    "BasicPotentialFieldMethod",
    "BasicSlidingModeAlgorithm",
    "BasicSafeSetAlgorithm",
    "RelaxedSafeSetAlgorithm",
    "BasicSublevelSafeSetAlgorithm",
    "RelaxedSublevelSafeSetAlgorithm",
):
    register_component_spec(
        _name,
        _spec(
            ComponentRole.SAFETY_FILTER,
            (PortKind.ESTIMATED_STATE, PortKind.CONTROL_COMMAND, PortKind.SAFETY_ASSESSMENT),
            _CONTROL,
            implementation=(ImplementationKind.MODEL_BASED, ImplementationKind.OPTIMIZATION),
            purpose=(PurposeKind.SAFETY,),
        ),
    )
