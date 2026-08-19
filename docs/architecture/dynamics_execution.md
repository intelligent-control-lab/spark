# Dynamics execution

SPARK separates four related decisions that are easy to conflate: the
dynamics model, dynamics order, dynamics backend, and numerical integrator.

## Selection layers

| Layer | Question it answers | Selected by | Examples |
|---|---|---|---|
| Dynamics model (variant) | What state, control, and equations describe the plant? | The selected `RobotConfig` and its `dynamics_variant` | `single_integrator`, `double_integrator`, `unicycle`, `bicycle` |
| Dynamics order | Does control act on the first or second state derivative? | `RobotConfig.dynamics_order`; public class names use `Dynamic1` or `Dynamic2` | order 1: velocity control; order 2: acceleration or force control |
| Dynamics backend | Which runtime is authoritative for the next state and feedback? | `dynamics_backend`, normally through `--dynamics-backend` | `simulator` or `model` |
| Model integrator | How is one continuous SPARK model step discretized? | `model_integrator` or `RobotDynamicsModel.step(..., integrator=...)` | Euler, RK4, ZOH/direct |

These dimensions are independent. For example,
`AgiBotG1MobileBaseUnicycleDynamic1Config` selects a unicycle mobility equation
of order 1; `AgiBotG1MobileBaseBicycleDynamic2Config` selects a bicycle
equation of order 2. A future configuration could choose another mobility
equation and order without changing the meaning of `dynamics_backend`.

## Dynamics model and order

The robot configuration is the only source of the dynamics equation and its
state/control layout. The usual order contract is:

| Order | Typical state | Typical control | Public name suffix |
|---:|---|---|---|
| 1 | position/configuration, `q` | velocity, `q_dot` | `Dynamic1Config` |
| 2 | position and velocity, `(q, q_dot)` | acceleration or generalized force | `Dynamic2Config` |

Order does not identify the complete equation. A first-order joint integrator
and a first-order unicycle are both order 1, but they have different state and
control geometry. Inspect the viewer's **Dynamics model**, **Dynamics order**,
and **Dimensions** rows together, or inspect the selected robot configuration.
Specialized models may expose a conventional coordinate order instead of the
generic `(q, q_dot)` block order. In particular, the AgiBot dynamic-bicycle
view uses the course convention `(X, Y, v_x_body, v_y_body, yaw_rate, yaw)` and
converts to/from the physical robot's world-frame velocity feedback at the
model boundary.

## Dynamics backend

| `dynamics_backend` | State authority | Simulator's role | Appropriate use |
|---|---|---|---|
| `simulator` | MuJoCo or Isaac/PhysX physics | Integrates the physical articulation and reports measured state | Contact, actuator, gravity, and physical-stability evaluation |
| `model` | The robot-selected SPARK analytical model | Displays the state produced by `DynamicsExecutor` | Deterministic model-based control, planning, and dynamics studies |

`--dynamics-backend` selects ownership only. It does not select a robot
dynamics variant, change order 1 to order 2, or choose Euler versus RK4.
`use_sim_dynamics` remains accepted as a compatibility alias; new
configurations should use `dynamics_backend`.

MuJoCo also accepts an explicit `projected` diagnostic mode. Release
validation does not use it because projection writes a target into simulator
state and therefore cannot demonstrate physical stability.

## Model integrator

The integrator matters only when a SPARK model owns continuous state
propagation. It is separate from the MuJoCo or PhysX engine integrator used by
`dynamics_backend="simulator"`.

| Integrator | Runtime behavior | Cost per model substep | Recommended use |
|---|---|---:|---|
| Euler (`Euler`, `ForwardEuler`, `FE`) | `x_next = x + dt * f(x, u)` | One derivative evaluation | Default; first-order integrators and small control periods |
| RK4 | Classical fourth-order Runge--Kutta with the command held over the step | Four derivative evaluations | Smooth nonlinear models when integration error matters |
| ZOH/direct | Exact matrix-exponential step for declared linear models; adaptive `solve_ivp` step for nonlinear models | Model dependent | Exact linear discretization or deliberate high-accuracy studies |

Euler is the default. For a first-order single integrator,
`q_dot = u`, Euler, RK4, and linear ZOH produce the same exact update for a
command held over the control period. A higher-cost integrator therefore does
not improve FANUC `Dynamic1` model teleoperation. Native discrete models own
their transition directly and require one executor substep.

The shared agents support `model_integrator` and `model_substeps` internally.
Public robot runners currently do not expose universal
`--model-integrator`/`--model-substeps` flags, so normal examples use the
qualified Euler default. A custom pipeline may configure them explicitly:

```python
cfg.env.agent.dynamics_backend = "model"
cfg.env.agent.model_integrator = "RK4"
cfg.env.agent.model_substeps = 2
```

Do not encode the integrator into `--dynamics-backend`. If these advanced
settings become public CLI options, they should remain separate and only
affect model-owned execution.

## Timing and substeps

`SimulatorDynamicsSpec.physics_dt` is the physics step and
`control_decimation` is the number of physics steps executed for each policy
command:

```text
control period = physics_dt * control_decimation
control frequency = 1 / control period
```

For model-owned execution, `DynamicsExecutor` advances once per control period;
`model_substeps` optionally divides that period for numerical integration. For
simulator-owned execution, physics decimation instead determines how many
MuJoCo or PhysX steps realize the held command. Changing timing can affect
physical stability and model accuracy, so robot runners inherit qualified
configuration defaults unless the user explicitly overrides them.

## Viewer terminology

The MuJoCo overlay and Isaac **SPARK Simulation Information** window report the
propagation owner directly:

| Viewer value | Meaning |
|---|---|
| `MuJoCo physics` | `dynamics_backend="simulator"`; MuJoCo and its configured engine integrator own physical propagation |
| `Isaac PhysX` | `dynamics_backend="simulator"`; Isaac PhysX owns physical propagation and articulation feedback |
| `SPARK model (Euler)` | `dynamics_backend="model"`; the selected analytical model is advanced by Euler and MuJoCo is the visualizer |
| `SPARK model (RK4)` or `SPARK model (ZOH)` | Model-owned propagation with an explicitly configured advanced integrator |

The separate **Dynamics model** and **Dynamics order** rows identify the
equation being propagated. The parenthesized integrator identifies only the
numerical stepping method. In Isaac, **SPARK Simulation Information** is the
window caption, not a metadata row; the first row is **Robot config**.

## Runtime responsibilities

The implementation follows these ownership rules:

- `spark_robot` owns every dynamics equation and variant. A selected
  `RobotConfig` creates the corresponding dynamics model.
- `RobotDynamicsModel.step` implements continuous-model integration (Euler,
  RK4, and ZOH/adaptive integration). Native discrete configurations implement
  their transition in `DiscreteTimeDynamicsModel.step`.
- `spark_agent.dynamics.DynamicsExecutor` owns mutable episode state, time,
  integration settings, random state, exogenous inputs, and online parameter
  updates. It receives a model; it never selects one.
- `DynamicsModelAgent` exposes the executor through the normal SPARK agent
  lifecycle for simulator-independent numerical experiments.
- MuJoCo and the generic Isaac articulation adapter use the same executor when
  converting commands to configured model states. With
  `dynamics_backend="model"`, the simulator only visualizes that state. With
  `dynamics_backend="simulator"`, the configured state is a command target and
  the physics engine owns feedback state.
- `SparkEnvironment` provides the common reset/step lifecycle. Application
  code owns experiment-specific rollout collection and stopping rules.

Every robot configuration exposes a `SimulatorDynamicsSpec`. Its qualified
defaults are a 2 ms physics step, ten physics steps per control command, and a
2 second zero-command hold gate. Simulator mode must keep state ownership in
the simulator and leave projection disabled. Robot-specific actuator gains or
command translation may be declared by an embodiment adapter, but robot names
and actuator layouts do not belong in the generic simulator base.

A robot family may override those generic defaults after backend qualification.
Unitree G1, AgiBot G1, Galaxea R1 Lite, and KUKA iiwa 14 use four 5 ms physics
steps per command. This keeps simulation aligned with their 20 ms, 50 Hz
control contract while retaining finer contact and drive integration. Public
benchmark and teleoperation runners inherit the selected configuration unless
the user explicitly supplies timing overrides.

Runtime model changes do not require rebuilding the robot configuration. Pass
`dynamics_parameters` or `dynamics_exogenous` in `action_info`; the executor
updates its model instance and supplies the values through
`DynamicsStepContext`.
