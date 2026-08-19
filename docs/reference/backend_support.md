# Backend support matrix

Backend installation and robot support are separate. An installed simulator
can run only robot configurations that declare an adapter for that backend.

| Robot family | Direct dynamics | CPU kinematics | MuJoCo | Isaac | Released hardware adapter |
|---|---:|---:|---:|---:|---:|
| Unitree G1 | Yes | Yes | Yes | Single + tensor | Yes |
| AgiBot G1 | Yes | Yes | Yes | Single + tensor | No |
| Galaxea R1 Lite | Yes | Yes | Yes | Single + tensor | No |
| Kinova Gen3 | Yes | Yes | Yes | Single + tensor | No |
| KUKA iiwa 14 | Yes | Yes | Yes | Single + tensor | No |
| FANUC LR Mate 200iD | Yes | Yes | Yes | Single + tensor | No |
| Numerical configurations | Yes | Not applicable | No | No | No |

`Yes` means a public configuration-to-agent mapping exists. `Single + tensor`
means both a scalar articulation and CUDA-resident cloned articulations are
available. One declarative implementation covers all 35 executable AgiBot,
Galaxea, Kinova, KUKA, and FANUC configurations, including first-order,
second-order, collision-volume, single-arm, dual-arm, fixed-base, and planar
mobile-base variants. Each family exposes separate public Isaac agent classes
in its own folder; these thin adapters are extension points and retain the
common tensor route for batched construction. Unitree uses its specialized
adapters for floating-base and learned-policy workflows. Hardware support
still requires the matching optional SDK or host middleware and target-robot
validation.

## Reproducible release matrix

The release matrix applies bounded scripted joint teleoperation and a fixed
goal-reaching benchmark. It tests both `dynamics_backend=simulator` and
`dynamics_backend=model`, one and four environments, and verifies finite
feedback, actual motion, goal error, and clone isolation:

```bash
python tools/run_robot_support_matrix.py \
  --backend mujoco isaac \
  --report docs/reference/robot_support_matrix.json
```

The committed combined report is `docs/reference/robot_support_matrix.json`;
its backend reports are `docs/reference/robot_support_matrix_mujoco.json` and
`docs/reference/robot_support_matrix_isaac.json`. MuJoCo uses independent
worlds for the four-environment case. Isaac uses the CUDA tensor adapter and
isolates each robot configuration in a fresh Kit process to prevent native
PhysX replication state from crossing asset boundaries. A child that exits
before writing a report is retried twice with a short backoff; case failures
are never retried or hidden. Isaac disables multi-GPU rendering by default
because one CUDA device owns the cloned physics scene.

The documentation showcase is deliberately smaller than this numerical
matrix. `tools/generate_robot_demo_gifs.py` records 21 representative robot
profiles in MuJoCo and Isaac safe teleoperation, single-environment
benchmarking, and four-environment Isaac benchmarking. Those 105 recordings
use matched scene styling and fixed profile-specific camera specifications and
show goals, obstacles, paths, collision volumes, and clone-local resets. The
complete matrix is generated locally under `build/media/`; the release keeps
only the eight representative animations embedded in the main README.

For one inspectable case, use:

```bash
python example/run_robot_matrix_case.py \
  --robot-config KinovaGen3DualArmDynamic1Config \
  --backend isaac --num-envs 4 --mode benchmark \
  --dynamics-backend simulator
```

The case runner selects `cpu` when `--num-envs 1` and `cuda:0` when the count
is greater than one unless `--device` is supplied explicitly. For example, an
AgiBot 32-clone CUDA conformance benchmark is:

```bash
python example/run_robot_matrix_case.py \
  --robot-config AgiBotG1MobileBaseDynamic1Config \
  --backend isaac --num-envs 32 --mode benchmark \
  --test-case arm_goal_static_v1 --dynamics-backend simulator
```

The shared conformance runner opens its viewer by default. Add `--headless`
for unattended throughput runs. Each reset is capped at 1000 control steps;
`--duration` may select a shorter episode. Parallel viewers place the goal,
and obstacles around every cloned robot. Detailed collision volumes default to
clone zero to keep the interactive Kit UI responsive; use
`--max-visualized-envs N` only when deliberately increasing that overlay cost.
Every clone samples a distinct seeded goal and obstacle layout, and both are
resampled after each reset. In the tensor benchmark, faster clones keep
resetting instead of freezing while slower clones finish; the finite run ends
after every clone reaches the requested quota. Pass `--num-resets -1` to
continue until the viewer is closed or the process is interrupted; finite
reports retain ten completed episodes per clone by default.

The matrix `teleop` mode is a bounded joint-command conformance test. Its
`control_limit` filter verifies the robot's declared actuation bounds, but it
is not the Cartesian collision-avoidance policy used by the interactive safe
teleoperation pipelines. Those pipelines are single-environment workflows and
now select either MuJoCo or Isaac from the same robot capability declaration.
For example:

```bash
# FANUC, simulator-owned dynamics, MuJoCo, single-env collision-safe teleop
python example/fanuc_lrmate200id/run_fanuc_lrmate200id_teleop.py \
  --backend mujoco \
  --robot-config FanucLRMate200iDSingleArmDynamic1CollisionConfig \
  --dynamics-backend simulator --safe-algo rssa --safety-index si1

# Kinova dual arm, simulator-owned dynamics, Isaac, single-env safe teleop
python example/kinova_gen3/run_kinova_gen3_teleop.py \
  --backend isaac \
  --robot-config KinovaGen3DualArmDynamic1CollisionConfig \
  --dynamics-backend simulator --safe-algo rssa --safety-index si1

# KUKA dual arm, simulator-owned dynamics, 16 cloned Isaac environments
python example/kuka_iiwa14/run_kuka_iiwa14_benchmark.py \
  --backend isaac --num-envs 16 \
  --robot-config KukaIIWA14DualArmDynamic1CollisionConfig \
  --test-case arm_goal_static_v1 --profile-frequency
```

Replace the family script and `--robot-config` for AgiBot, Galaxea, KUKA, or
Kinova safe teleoperation. In the AgiBot entry point, passing `--headless
--enable-keyboard-control false --max-num-steps 100` gives a finite headless
smoke test. The matrix remains a plant conformance test. The robot-local
benchmark launchers provide semantic Cartesian tasks and tensor safety-filter
paths with independent goals, obstacles, counters, and indexed resets for
every Isaac clone. Fixed-base Kinova, KUKA, and FANUC launchers expose
single/dual-arm v0 and v1 cases; the mobile-manipulator launchers additionally
expose their whole-body cases.

All ordinary MuJoCo joint-control agents use the same simulator-dynamics state
contract: simulator-owned execution integrates velocity or acceleration from
measured feedback, while model-owned execution integrates from the analytical
command state. This prevents target windup without coupling the shared backend
to a robot family. The Unitree whole-body learned-policy adapter retains its
separate torque/target contract. Binary gripper goals and persistent keyboard
overrides are likewise resolved by the shared MuJoCo layer; embodiment code is
responsible only for mapping that goal to its actuator geometry.

The Galaxea source tree contains an experimental ROS adapter, but it is not a
released configuration backend until its feedback, limits, reset behavior,
and hardware safety procedure are validated. Its presence is therefore not
reported as hardware support in this matrix.

The AgiBot UnicycleDynamic1 and BicycleDynamic2 configurations retain the full
mobile robot and support both simulator backends. The suffix records the
uniform dynamics order while unicycle/bicycle records the planar mobility
equation. Their analytical dynamics are reduced planning/control views.
Simulator-owned execution uses the common physical planar-drive contract and
warns about that distinction; model-owned execution follows the selected
analytical equations.

## Asset provenance

- FANUC uses the ROS-Industrial `fanuc` Noetic LR Mate 200iD/7L description;
  the retained BSD-3-Clause license and adaptation notes are stored beside the
  asset.
- Kinova's Gen3 arm chain and meshes match the official `ros_kortex` Noetic
  description after localizing mesh URLs; SPARK then attaches its bundled
  lightweight articulated display gripper and retains Kinova's BSD-3-Clause
  notice beside the asset.
- Galaxea is adapted from Galaxea Dynamics' public R1 Lite 2025 description.
  The upstream repository did not publish a license file when qualified, so
  its local provenance note records that unresolved redistribution condition.
- The bundled AgiBot geometry matches the contributor-supplied
  `A2D_Omnipicker` archive after SPARK's importer-safe inertia addition. No
  public upstream URL or redistribution license was found, so SPARK records
  that caveat and does not represent the archive as an official release.

See each `module/spark_robot/resources/<robot>/README.md` for exact source,
revision, transformations, and licensing notes.
