# Policy package migration

The workflow-oriented paths are canonical. The former hierarchy has been
removed from the repository; this page records import migrations for users
updating older applications.

| Legacy path | Workflow-oriented path |
| --- | --- |
| `spark_policy.model_based.control_policy.base_pid_policy` | `spark_policy.control.pid` |
| `spark_policy.model_based.control_policy.lqr_policy` | `spark_policy.control.linear_quadratic` |
| `spark_policy.model_based.control_policy.traj_tracking_policy` | `spark_policy.control.trajectory` |
| `spark_policy.model_based.control_policy.unitree_g1_wbc_policy` | `spark_policy.control.whole_body` |
| `spark_policy.model_based.planning_policy.rrt_connect_policy` | `spark_policy.planning.motion` |
| `spark_policy.model_based.control_policy.ilqr_policy` | `spark_policy.planning.trajectory` |
| `spark_policy.safe.safe_algo.value_based.base` | `spark_policy.safety.monitoring` |
| `spark_policy.safe.safe_algo` control modifiers | `spark_policy.safety.filtering` |
| `spark_algo.SparkAlgoWrapper` | `spark_policy.composed_policy.safety_filtered.SafetyFilteredPolicy` |
| `spark_algo.WBCAlgoWrapper` | `spark_policy.composed_policy.unitree_g1.wbc.UnitreeG1WBCComposedPolicy` |
| `spark_algo.WBTAlgoWrapper` | `spark_policy.composed_policy.unitree_g1.wbt_safe.UnitreeG1WBTSafePolicy` |
| `spark_algo.SonicAlgoWrapper` | `spark_policy.composed_policy.unitree_g1.sonic_safe.UnitreeG1SonicSafePolicy` |
| `spark_env.SparkEnvWrapper` | `spark_env.SparkEnvironment` |

WBT and SONIC cascade orchestration is implemented directly by grounded
policies in `spark_policy.composed_policy.unitree_g1`; their nominal policies
live under `spark_policy.control.whole_body.unitree_g1`. Historical source
trees and rollback copies are intentionally not shipped.

## New PID building blocks

```python
from spark_policy.control.pid import (
    CartesianPIDController,
    IKJointPIDController,
    JointPIDController,
)
```

Generic IK stays in `spark_robot`. `IKJointPIDController` composes that robot
capability with reusable joint PID feedback.
