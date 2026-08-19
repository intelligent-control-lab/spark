# SPARK policy and runtime architecture

## Boundaries

- `spark_robot` describes robot state, control, kinematics, dynamics, limits,
  and collision geometry.
- `spark_agent` executes actions and returns observations for a simulated or
  physical endpoint.
- `spark_task` supplies goals, references, environment information, task
  progress, and termination state.
- `spark_env` coordinates the reset/step lifecycle of one agent and one task.
- `spark_policy` converts agent feedback and task information into actions.
- `spark_pipeline` owns the application run loop, visualization, evaluation,
  and recording.

The packages remain peers. In particular, `spark_agent` and `spark_task` are
not owned by `spark_env`; an environment composes instances of them.

## Policy structure

The primary package classification is the module's workflow role:

- `estimation`: observations to estimated state;
- `planning`: goals and state to paths or trajectories;
- `control`: state and references to commands;
- `safety.monitoring`: state/environment to safety assessments;
- `safety.filtering`: nominal commands to modified safe commands.

Every concrete public policy package uses `policy.py` as its implementation
entry point and `config.py` as its configuration entry point. This convention
also applies to estimation and safety policies; their role and result type are
declared through metadata rather than encoded in the filename.

`composed_policy` contains grounded, user-selectable policies that join multiple
roles. Each composed policy owns its workflow directly and depends only on the
single-role component policies it selects. There is no shared composition
template layer. `spark_pipeline` remains the only layer called a pipeline.

Single-role variants stay beside their base method. Benchmark and teleoperation
PID therefore live under `control.pid`, while nominal Unitree WBC, WBT, and
SONIC policies live under `control.whole_body.unitree_g1`. Their safety-aware
combinations live under `composed_policy.unitree_g1`.

Within these roles, reusable estimation and learning-control implementations
are organized as follows:

- `estimation.filtering`: Kalman, steady-state Kalman, EKF, and UKF;
- `estimation.parameter`: recursive least-squares and gradient identification;
- `control.adaptive`: model-reference adaptive control;
- `control.iterative_learning`: frequency- and time-domain ILC;
- `utils.rollout`: pure numerical model/transition rollouts used for analysis.

Numerical rollouts are deliberately separate from `spark_env.Environment.step`.
An environment step executes a complete agent/task lifecycle and may have
simulation or hardware side effects; a model rollout evaluates a pure reduced
dynamics model with an explicitly selected integrator.

Implementation properties such as model-based/learned, feedback/feedforward,
horizon, statefulness, and nominal/safety purpose are represented by
`ComponentSpec`, not by parallel folder trees.

Safety monitoring and filtering policies are grounded directly by composed
policies. There is no canonical safety-stack/controller wrapper between them.
The retired `BaseSafeController` implementation is not part of the repository
or installed packages.

## PID references

Tasks may provide joint or Cartesian references. Generic IK remains a robot
kinematics capability in `spark_robot`. `spark_policy.control.pid` provides:

- direct joint-space PID;
- direct Cartesian-space PID;
- Cartesian-to-IK-to-joint-PID composition.

The task defines the target. The selected policy defines how it is tracked.

## Compatibility boundary

The former `model_based`, `model_free`, `safe`, and `spark_algo` trees are not
part of the repository or active import graph. `SparkEnvWrapper` remains a
tested environment alias for one compatibility cycle and is unrelated to
policy implementation selection.
