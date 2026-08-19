# Data and Control Flow

At each step, the environment returns agent feedback and task information. The
policy maps them to an action and diagnostics, and the environment applies the
action through its agent.

```{math}
(y_{k+1},\tau_{k+1})=E(u_k,i_k),\qquad
(u_{k+1},i_{k+1})=\pi(y_{k+1},\tau_{k+1}).
```

`agent_feedback` contains measured robot/backend state. `task_info` contains
goals, obstacles, transforms, and completion state. `action_info` carries
policy plans, safety values, and visualization diagnostics.
