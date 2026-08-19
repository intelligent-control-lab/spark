# Modules

SPARK is divided into six user-facing packages.

| Package | Responsibility | Main inputs and outputs |
|---|---|---|
| {doc}`robot` | Embodiment, dynamics, kinematics, limits, collision model | state and control coordinates, frames, Jacobians |
| {doc}`agent` | Simulator and physical-robot execution backends | action → feedback |
| {doc}`task` | Goals, obstacles, trajectories, completion | feedback → `task_info` |
| {doc}`policy` | Planning, estimation, tracking, and control | feedback + task → action |
| {doc}`env` | Agent–task coordination and transitions | action → feedback + task information |
| {doc}`pipeline` | Configuration, runtime loop, visualization, logging | complete experiment/application |

The separation is contractual: robot configurations define dimensions;
agents execute those dimensions; tasks describe intent; policies decide
actions; pipelines choose and connect concrete implementations.

```{toctree}
:maxdepth: 2

robot
agent
task
policy
env
pipeline
```
