# Pipeline workflow

`BasePipeline` constructs the configured robot, kinematics object, environment,
and policy. `BaseGoalPipeline` adds reset handling, logging, visualization,
timing, and repeated episodes.

```{mermaid}
flowchart TD
  C[Load pipeline configuration] --> R[Create RobotConfig and RobotKinematics]
  R --> E[Create SparkEnvironment: Agent + Task]
  E --> P[Create direct or composed Policy]
  P --> Z[environment.reset]
  Z --> A[policy.act feedback, task_info]
  A --> S[environment.step action, action_info]
  S --> D{task_info.done?}
  D -- no --> O[process logs, visualization, timing]
  O --> A
  D -- yes, reset allowed --> T[reset or transition task]
  T --> A
  D -- yes, finished --> X[save results and close agent]
```

At control step *k*, the essential recurrence is:

```{math}
(y_{k+1},\,\tau_{k+1}) = \operatorname{EnvironmentStep}(u_k, i_k),
\qquad
(u_{k+1},\,i_{k+1}) = \pi(y_{k+1},\tau_{k+1}),
```

where `y` is agent feedback, `τ` is task information, `u` is the command, and
`i` is policy diagnostic information. The policy is not assumed to contain a
particular internal layering; its composition is declared by its concrete
class and configuration.
