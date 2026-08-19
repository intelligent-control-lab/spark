# Env

`spark_env` defines the environment contract that connects an agent with a
task without merging their responsibilities.

## Environment

`Environment` is the abstract interface for reset and step operations. It
defines the transition boundary used by pipelines.

## SparkEnvironment

`SparkEnvironment` is the single-agent implementation. `reset()` initializes
the agent and task and returns `(agent_feedback, task_info)`. Then
`step(action, action_info)` applies the action through the agent, obtains new
feedback, updates the task, and returns the next pair.

```{math}
(y_{k+1},\tau_{k+1})=
\operatorname{SparkEnvironment.step}(u_k,i_k).
```

The environment does not choose the policy and does not contain robot-specific
control logic.

```{toctree}
:maxdepth: 1

env/overview
env/interface
env/spark_environment
env/episode_data
```
