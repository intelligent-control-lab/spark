# SPARK Naming Conventions

This document records the naming rules for robot-specific modules in SPARK. Prefer explicit vendor and model names so different robots with similar model names remain distinguishable.

## Robot Package Folders

Use:

```text
<vendor>_<model>
```

Examples:

| Robot | Package folder |
| --- | --- |
| Unitree G1 | `unitree_g1` |
| AgiBot G1 | `agibot_g1` |
| Galaxea R1 Lite | `galaxea_r1lite` |
| FANUC LR Mate 200iD | `fanuc_lrmate200id` |
| KUKA IIWA14 | `kuka_iiwa14` |
| Kinova Gen3 | `kinova_gen3` |

Do not use ambiguous package folders such as `g1`, `gen3`, or `iiwa14` for new robot modules.

## Robot Config Files

Use:

```text
<vendor>_<model>_<mode>_<type>_<dynamic_order>_config.py
```

Where:

| Field | Meaning | Examples |
| --- | --- | --- |
| `vendor` | Robot vendor or owner | `unitree`, `agibot`, `galaxea`, `fanuc`, `kuka`, `kinova` |
| `model` | Robot model | `g1`, `r1lite`, `lrmate200id`, `iiwa14`, `gen3` |
| `mode` | Controlled robot subset or platform mode | `single_arm`, `dual_arm`, `right_arm`, `fixed_base`, `mobile_base`, `sport_mode`, `whole_body` |
| `type` | Dynamics or config purpose | `dynamic`, `collision` |
| `dynamic_order` | Dynamic model order | `1`, `2` |

Examples:

```text
unitree_g1_dual_arm_dynamic_1_config.py
galaxea_r1lite_dual_arm_dynamic_1_config.py
fanuc_lrmate200id_dual_arm_dynamic_1_config.py
kuka_iiwa14_dual_arm_dynamic_1_config.py
kinova_gen3_dual_arm_dynamic_1_config.py
agibot_g1_fixed_base_dynamic_1_config.py
agibot_g1_dual_arm_dynamic_1_config.py
agibot_g1_right_arm_dynamic_1_config.py
```

Collision config files insert `collision` before `config`:

```text
kinova_gen3_single_arm_dynamic_1_collision_config.py
```

## Kinematics Files

Use:

```text
<vendor>_<model>_<mode>_kinematics.py
```

Examples:

```text
unitree_g1_dual_arm_kinematics.py
galaxea_r1lite_dual_arm_kinematics.py
fanuc_lrmate200id_dual_arm_kinematics.py
kuka_iiwa14_dual_arm_kinematics.py
kinova_gen3_dual_arm_kinematics.py
agibot_g1_fixed_base_kinematics.py
agibot_g1_dual_arm_kinematics.py
agibot_g1_right_arm_kinematics.py
```

## Agent Files

Use:

```text
<vendor>_<model>_<mode>_agent.py
```

Each public robot agent file should define one public agent class that inherits from `MujocoAgent` or a private shared base class that inherits from `MujocoAgent`.

Examples:

```text
unitree_g1_dual_arm_mujoco_agent.py
galaxea_r1lite_fixed_base_agent.py
galaxea_r1lite_right_arm_agent.py
fanuc_lrmate200id_single_arm_agent.py
kuka_iiwa14_single_arm_agent.py
kinova_gen3_dual_arm_agent.py
agibot_g1_fixed_base_agent.py
agibot_g1_dual_arm_agent.py
agibot_g1_right_arm_agent.py
```

If multiple agents share substantial MuJoCo logic, keep the shared code in a private base implementation such as `kinova_gen3_base_agent.py`, and expose concrete agents from separate files such as `kinova_gen3_single_arm_agent.py` and `kinova_gen3_dual_arm_agent.py`.

## Class Names

Use PascalCase with the same vendor, model, and mode terms:

```text
<Vendor><Model><Mode><Type><DynamicOrder>Config
<Vendor><Model><Mode>Kinematics
<Vendor><Model><Mode>Agent
```

Examples:

| File | Class |
| --- | --- |
| `unitree_g1_dual_arm_dynamic_1_config.py` | `UnitreeG1DualArmDynamic1Config` |
| `unitree_g1_dual_arm_kinematics.py` | `UnitreeG1DualArmKinematics` |
| `unitree_g1_dual_arm_mujoco_agent.py` | `UnitreeG1DualArmMujocoAgent` |
| `galaxea_r1lite_dual_arm_dynamic_1_config.py` | `GalaxeaR1LiteDualArmDynamic1Config` |
| `fanuc_lrmate200id_dual_arm_dynamic_1_config.py` | `FanucLRMate200iDDualArmDynamic1Config` |
| `kuka_iiwa14_dual_arm_dynamic_1_config.py` | `KukaIIWA14DualArmDynamic1Config` |
| `kinova_gen3_dual_arm_dynamic_1_config.py` | `KinovaGen3DualArmDynamic1Config` |
| `agibot_g1_fixed_base_dynamic_1_config.py` | `AgiBotG1FixedBaseDynamic1Config` |
| `agibot_g1_dual_arm_dynamic_1_config.py` | `AgiBotG1DualArmDynamic1Config` |
| `agibot_g1_right_arm_dynamic_1_config.py` | `AgiBotG1RightArmDynamic1Config` |

Use `Fanuc` in Python class names, not all-caps `FANUC`, to follow normal PascalCase style. Keep official capitalization in user-facing text where useful.

## AgiBot G1 vs Unitree G1

AgiBot G1 and Unitree G1 must be treated as separate vendor modules, even though both use the model token `g1`.

Use:

```text
module/spark_robot/spark_robot/unitree_g1/
module/spark_robot/spark_robot/agibot_g1/
module/spark_agent/spark_agent/simulation/mujoco/unitree_g1/
module/spark_agent/spark_agent/simulation/mujoco/agibot_g1/
```

Do not use mixed names such as `agibot_unitree_g1`. If a file, class, or registry key refers to AgiBot, it should start with `agibot_g1` or `AgiBotG1`; if it refers to Unitree, it should start with `unitree_g1` or `UnitreeG1`.

For AgiBot G1, use these public platform/subset names:

| Variant | Config class | Agent class |
| --- | --- | --- |
| Fixed base, torso lift + dual arms | `AgiBotG1FixedBaseDynamic1Config` | `AgiBotG1FixedBaseAgent` |
| Fixed base, arms only | `AgiBotG1DualArmDynamic1Config` | `AgiBotG1DualArmAgent` |
| Fixed base, right arm only | `AgiBotG1RightArmDynamic1Config` | `AgiBotG1RightArmAgent` |
| Mobile base, single integrator | `AgiBotG1MobileBaseDynamic1Config` | `AgiBotG1MobileBaseAgent` |
| Mobile base, double integrator | `AgiBotG1MobileBaseDynamic2Config` | `AgiBotG1MobileBaseAgent` |
| Mobile base, order-1 unicycle | `AgiBotG1MobileBaseUnicycleDynamic1Config` | `AgiBotG1MobileBaseAgent` |
| Mobile base, order-2 bicycle | `AgiBotG1MobileBaseBicycleDynamic2Config` | `AgiBotG1MobileBaseAgent` |

`AgiBotG1FixedBase` is the torso-lift dual-arm fixed-base variant. Do not add separate public torso-dual fixed-base or dual-arm mobile-base names.

## Examples, Pipelines, and Registry Keys

Robot-specific examples and pipeline config files should use the same explicit prefix:

```text
example/unitree_g1/run_unitree_g1_benchmark.py
pipeline/spark_pipeline/autonomy/unitree_g1_benchmark_pipeline_config.py
pipeline/spark_pipeline/teleop/unitree_g1_teleop_pipeline_config.py
```

Benchmark test-case keys should also include the vendor:

```text
UnitreeG1FixedBase_D1_AG_DO_v0
KinovaGen3SingleArm_D1_AG_SO_v0
GalaxeaR1LiteFixedBase_D1_AG_SO_v0
AgiBotG1FixedBase_D1_AG_SO_v0
AgiBotG1DualArm_D1_AG_SO_v0
AgiBotG1RightArm_D1_AG_SO_v0
```

## Other Module Naming Guidance

Apply the same explicit vendor prefix when a module is robot-specific:

| Module type | Recommendation |
| --- | --- |
| Task files | Keep generic task files generic, for example `teleop_task.py` and `benchmark_task.py`. Use `<vendor>_<model>_<purpose>_task.py` only for robot-specific tasks. |
| Policy files | Generic policies should stay generic. Robot-specific policies should use explicit names, for example `unitree_g1_wbc_policy.py`. |
| Controller files | Robot-specific controllers should include the vendor and model. Generic controller interfaces should stay generic. |
| Evaluator files | Prefer task or benchmark names, for example `<benchmark>_evaluator.py`, unless the evaluator is robot-specific. |
| Environment files | Keep generic environment wrappers generic. Use explicit robot prefixes only for robot-bound environments. |
| Experiment scripts | Use `run_<vendor>_<model>_<mode_or_task>.py`. |
| Registry keys | Include vendor and model for robot-specific keys. |
| Robot asset folders | Asset folders may keep upstream names such as `g1`, `gen3`, or `iiwa14` when XML include paths and mesh references depend on them. Rename asset folders only as a dedicated migration with XML path validation. |
| Hardware protocol types | SDK or hardware protocol names such as `G1_29_*` may keep the upstream model identifier when the name mirrors a vendor interface. |
