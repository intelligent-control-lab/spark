# Data Contracts

| Contract | Producer | Consumer | Typical content |
|---|---|---|---|
| `agent_feedback` | Agent | Task and policy | joint state, frames, backend state |
| `task_info` | Task | Policy | goals, obstacles, completion state |
| `action` | Policy | Agent | robot command vector |
| `action_info` | Policy | Pipeline and agent | plans, safety data, diagnostics |
