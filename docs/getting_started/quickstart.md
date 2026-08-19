# Quick start

## Step a configured robot model

The core profile is enough to construct a robot-selected dynamics model and
advance it without a simulator:

```python
import numpy as np
from spark_robot import KinovaGen3SingleArmDynamic2Config

robot = KinovaGen3SingleArmDynamic2Config()
model = robot.create_dynamics_model()
state = np.zeros(model.state_dim)
control = np.full(model.control_dim, 0.25)
next_state = model.step(state, control, dt=0.02, integrator="RK4")

print(model.variant, model.dimensions)
print(next_state)
```

The configuration owns the state and control dimensions and selects the
double-integrator equation. A different compatible robot configuration can use
the same model API without moving robot-specific equations into the caller.

## Continue into simulation

Install the `mujoco` profile before running the interactive and benchmark entry
points under `example/`. Start with {doc}`../tutorials/teleoperation` for
viewer controls or {doc}`../tutorials/autonomous_evaluation` for task, policy,
and safety composition. Backend availability by robot family is recorded in
{doc}`../reference/backend_support`.

## What happens in a run?

The selected pipeline constructs a robot model, an execution agent, a task,
and one or more policies. Each control step reads observations, evaluates the
policy composition, applies safety constraints, and sends the resulting action
to the simulator or robot.

Read {doc}`concepts` for the core vocabulary or see
{doc}`../architecture/overview` for the complete component relationship.
