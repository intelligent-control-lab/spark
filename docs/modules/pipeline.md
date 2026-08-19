# Pipeline

`spark_pipeline` turns configurations into complete applications.

`base`
: `BasePipeline` performs construction and the core environment-policy loop.
  `BaseGoalPipeline` adds reset/transition behavior, logging, visualization,
  performance profiling, and safety diagnostics.

`teleop`
: Configures teleoperation tasks and robot/policy combinations, including
  Unitree G1 PID, WBT, and SONIC workflows.

`autonomy`
: Configures benchmark goals, reproducible test cases, and MuJoCo or Isaac
  execution.

`simulation`
: Connects policies and tasks to direct dynamics-model experiments.

Pipeline configuration selects:

| Section | Selects |
|---|---|
| `robot.cfg` | Robot coordinates, dynamics, limits, collision model |
| `robot.kinematics` | FK, Jacobian, and IK implementation |
| `env.agent` | MuJoCo, Isaac, dynamics, or real backend and its timing |
| `env.task` | Task class and task-specific parameters |
| `policy` | Direct/composed policy class and parameters |
| runtime fields | step/reset limits, visualization, logging, pacing |

See {doc}`../architecture/pipeline_workflow` for the execution graph.

```{toctree}
:maxdepth: 1

pipeline/overview
pipeline/configuration
pipeline/base_goal
pipeline/teleoperation
pipeline/benchmark
pipeline/runtime_utilities
```
