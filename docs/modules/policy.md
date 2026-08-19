# Policy

`spark_policy` exposes a uniform `act(agent_feedback, task_info)` interface.
Policies may be used directly when they produce the robot's complete command,
or explicitly composed when one policy consumes another policy's result.

## Dependency and composition graph

```{mermaid}
flowchart LR
  Task[task_info + feedback]
  Task --> Direct[Direct control policies]
  Task --> Plan[Planning policies]
  Task --> Est[Estimation policies]
  Est --> Direct
  Plan --> Track[Trajectory playback / tracking]
  Track --> Nominal[Nominal action]
  Direct --> Nominal
  Nominal --> SF[SafetyFilteredPolicy]
  Monitor[Collision safety index] --> Filter[Safety filter]
  Filter --> SF
  SF --> Action[Robot action]

  WBC[UnitreeG1WBCPolicy] --> WBCComp[UnitreeG1WBCComposedPolicy]
  WBT[UnitreeG1WBTPolicy] --> WBTSafe[UnitreeG1WBTSafePolicy]
  Sonic[UnitreeG1SonicPolicy] --> SonicSafe[UnitreeG1SonicSafePolicy]
  Filter --> WBTSafe
  Filter --> SonicSafe
  WBCComp --> Action
  WBTSafe --> Action
  SonicSafe --> Action
```

Composition is configuration-driven, not implicit. A direct policy such as
PID, LQR, MPC, iLQR, Sport, WBC, WBT, or SONIC can own the action. Planner
output can feed a tracking policy. `SafetyFilteredPolicy` wraps a nominal
policy and a configured safety controller. The Unitree safe compositions add
the embodiment-specific mapping required around WBT or SONIC.

## PID control

The PID family contains joint, Cartesian, inverse-kinematics joint,
multi-target, teleoperation, and benchmark variants. The common command is

```{math}
u(t)=K_P e(t)+K_I\int_0^t e(\tau)d\tau+K_D\dot e(t),
\qquad e(t)=r(t)-y(t).
```

Cartesian policies form translation and orientation error at selected frames;
IK variants first map desired end-effector poses to joint targets. Configuration
controls gains, controlled frames/coordinates, integral limiting, and command
clipping.

## Trajectory control and optimization

`playback` replays stored actions. `tracking` consumes a target trajectory and
applies the configured tracking controller. `TrajOptPolicy` minimizes a
finite-horizon objective containing pose error, nominal-posture cost, velocity
and acceleration smoothness, and a soft initial-state anchor:

```{math}
J=\sum_{k=0}^{H-1}\left(
w_p\lVert e_p(q_k)\rVert^2+w_R\lVert e_R(q_k)\rVert^2
+\lambda_n\lVert q_k-q_{nom}\rVert^2\right)
+\lambda_v\sum_k\lVert q_{k+1}-q_k\rVert^2
+\lambda_a\sum_k\lVert q_{k+2}-2q_{k+1}+q_k\rVert^2.
```

The implementation uses CasADi/Ipopt, joint bounds, and configurable horizon,
weights, time step, and solver tolerances.

## LQR and MPC

For linear discrete dynamics `x[k+1]=Ax[k]+Bu[k]`, LQR minimizes

```{math}
J=\sum_{k=0}^{\infty}(x_k^\mathsf{T}Qx_k+u_k^\mathsf{T}Ru_k),
\qquad u_k=-Kx_k,
```

with `K` obtained from the Riccati equation. Finite-horizon LQR performs the
backward Riccati recursion. Linear MPC repeatedly solves the finite-horizon
problem from the measured state; constrained variants add state/input bounds.
The [MIT Underactuated LQR chapter](https://underactuated.mit.edu/lqr.html)
provides a detailed derivation.

## iLQR

`ILQRPolicy` currently plans once for the AgiBot unicycle model with reduced
state `[x,y,yaw]` and controls `[linear velocity,yaw rate]`. It alternates a
backward local quadratic approximation with a forward rollout and line search,
using running matrices `Q`, `R`, terminal matrix `S`, regularization, and a
configurable horizon.

## Motion planning

`RRTConnectPolicy` grows two rapidly exploring trees from start and goal,
steers through joint space, checks interpolated edges with the robot collision
model, and joins the trees when a valid connection is found. Parameters include
planning joints, step size, connection resolution, goal tolerance, sampling
bounds, and iteration limit. See the original
[RRT-Connect paper](https://www.clear.rice.edu/comp450/papers/kuffner_lavalle_00.pdf).

## Adaptive and iterative control

`MRACPolicy`/`ModelReferenceAdaptiveController` track a reference model while
updating controller parameters from tracking error. The ILC submodule provides
time-domain and frequency-domain iterative learning controllers that update a
repeated-trial command using the previous trial's error.

## Estimation

The estimation package provides linear, steady-state, extended, and unscented
Kalman filters plus gradient and recursive least-squares parameter estimators.
For a linear model, the correction step is

```{math}
K_k=P_k^-C^\mathsf{T}(CP_k^-C^\mathsf{T}+R)^{-1},\qquad
\hat x_k=\hat x_k^-+K_k(y_k-C\hat x_k^-).
```

## Unitree whole-body policies

`sport` maps planar commands to the Unitree locomotion interface. `wbc` and its
PID component coordinate whole-body targets. `wbt` and `sonic` integrate their
respective learned/runtime controllers, with batched variants for vectorized
execution. SONIC also provides a
{doc}`planner-free CUDA adapter <policy/native_sonic>` for reference-driven,
large-batch training. Their safe composed policies retain the original
controller as the nominal behavior and apply explicitly configured collision
constraints.

```{toctree}
:maxdepth: 1

policy/overview
policy/core
policy/control
policy/native_sonic
policy/planning
policy/estimation
policy/safety
policy/composed
```
