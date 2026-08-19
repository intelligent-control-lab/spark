# Policy Module Overview

Policies implement `act(agent_feedback, task_info)` and return an action plus
diagnostics. They may directly own the complete robot command or be connected
through an explicit composed policy.

```{mermaid}
flowchart LR
  Input[Feedback + task information] --> Control[Control or planning policy]
  Planning[Planning] --> Tracking[Tracking control]
  Estimation[Estimation] --> Control
  Tracking --> Nominal[Nominal action]
  Control --> Nominal
  Nominal --> Composition[Optional composed policy]
  Safety[Safety monitoring + filtering] --> Composition
  Composition --> Action[Robot action]
  Nominal --> Action
```
