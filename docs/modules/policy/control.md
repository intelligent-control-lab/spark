# Control

The `control` package follows the source hierarchy.

## PID

Joint, Cartesian, IK-joint, multi-target, teleoperation, and benchmark variants
share the standard form

```{math}
u=K_Pe+K_I\int e\,dt+K_D\dot e.
```

## Trajectory

Playback returns stored commands; tracking follows time-indexed references.

## Linear quadratic

LQR, finite-horizon LQR, and MPC operate on linear discrete dynamics and
quadratic state/control costs. Constrained variants enforce configured bounds.

## Adaptive and iterative learning

MRAC updates parameters from reference-model error. Time- and frequency-domain
ILC update commands across repeated trials.

## Whole body

Unitree G1 implementations include Sport, WBC and WBC PID, WBT and batched
WBT, deployment-backed SONIC, and planner-free native batched SONIC. The
native adapter requires an upstream full-body reference; see
{doc}`native_sonic`.
