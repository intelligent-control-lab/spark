# Architecture overview

SPARK organizes a robot application around the robot, agent, environment,
task, and policy interfaces shown in the project architecture figure.

```{image} ../img/spark_sys_framework.png
:alt: SPARK architecture showing the relationship among robot, agent, environment, task, and policy
:align: center
:width: 100%
```

Robot
: Defines the controlled coordinates, limits, dynamics, kinematics, frames,
  collision volumes, and compatible execution agents.

Agent
: Adapts a simulator or physical robot to the common feedback/action contract.

Environment
: Owns the agent and task, applies actions, advances execution, and returns
  `agent_feedback` together with `task_info`.

Task
: Produces goals, obstacles, episode state, and termination information.

Policy
: Maps feedback and task information to an action. A policy can be a direct
  controller or an explicit composition of planning, control, estimation, and
  safety components.

See {doc}`pipeline_workflow` for the runtime sequence and {doc}`../modules/robot`
for the first implementation-level module guide.
