# SPARK in action

These curated animations are the same demonstrations embedded in the project
README. The numerical support matrix remains the exhaustive backend and robot
configuration validation record.

## Whole-body Unitree G1

::::{grid} 1 2 3 3
:gutter: 3

:::{grid-item-card} WBT safe teleoperation
```{image} ../media/demos/robot_matrix/unitree_g1__whole_body_wbt__mujoco_teleop.gif
:alt: Unitree G1 whole-body WBT safe teleoperation
:class: spark-demo
```
:::

:::{grid-item-card} Sport safe teleoperation
```{image} ../media/demos/robot_matrix/unitree_g1__whole_body_sport__mujoco_teleop.gif
:alt: Unitree G1 whole-body Sport safe teleoperation
:class: spark-demo
```
:::

:::{grid-item-card} SONIC parallel benchmark
```{image} ../media/demos/robot_matrix/unitree_g1__whole_body_sonic__isaac_parallel.gif
:alt: Unitree G1 whole-body SONIC parallel Isaac benchmark
:class: spark-demo
```
:::

::::

## Robot portfolio

::::{grid} 1 2 2 2
:gutter: 3

:::{grid-item-card} Kinova Gen3
```{image} ../media/demos/robot_matrix/kinova_gen3__single_arm__mujoco_teleop.gif
:alt: Kinova Gen3 single-arm safe teleoperation
:class: spark-demo
```
:::

:::{grid-item-card} KUKA iiwa 14
```{image} ../media/demos/robot_matrix/kuka_iiwa14__dual_arm__mujoco_teleop.gif
:alt: KUKA iiwa 14 dual-arm safe teleoperation
:class: spark-demo
```
:::

:::{grid-item-card} FANUC LR Mate 200iD
```{image} ../media/demos/robot_matrix/fanuc_lrmate200id__single_arm__mujoco_teleop.gif
:alt: FANUC LR Mate 200iD single-arm safe teleoperation
:class: spark-demo
```
:::

:::{grid-item-card} Galaxea R1 Lite
```{image} ../media/demos/robot_matrix/galaxea_r1lite__mobile_base__mujoco_teleop.gif
:alt: Galaxea R1 Lite mobile-base safe teleoperation
:class: spark-demo
```
:::

:::{grid-item-card} AgiBot G1
```{image} ../media/demos/robot_matrix/agibot_g1__mobile_base__mujoco_teleop.gif
:alt: AgiBot G1 mobile-base safe teleoperation
:class: spark-demo
```
:::

::::

```{note}
The release intentionally checks in only these eight README demonstrations to
keep clone and push sizes practical. Use
`tools/generate_robot_demo_gifs.py` to generate the complete visual matrix in
the ignored `build/media/robot_matrix/` directory when needed.
```
