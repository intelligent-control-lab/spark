# Robot

`spark_robot` contains the robot contract, kinematics implementations,
dynamics views, numerical integration helpers, model loading, and
embodiment-specific configurations.

## Common contract

Every `RobotConfig` provides `DoFs`, `Control`, `ControlLimit`, frame names,
collision volumes, default positions, and control-affine dynamics

```{math}
\dot{x}=f(x)+g(x)u.
```

First-order configurations use `x=q` and velocity-like control `u`; their
state dimension equals the DoF count. Second-order configurations use
`x=[q^\mathsf{T},\dot q^\mathsf{T}]^\mathsf{T}` and acceleration-like control;
their state dimension is normally twice the DoF count. Whole-body floating
base models can have a non-Euclidean state layout, so use the dimensions
reported by the selected configuration rather than assuming `2 × DoFs`.

## Supported robot families

| Robot | Configuration | DoFs | Controls | State dimensions |
|---|---|---:|---:|---:|
| Unitree G1 | right arm | 7 | 7 | 7 / 14 |
| Unitree G1 | dual arm | 14 | 14 | 14; hand variant also supports 28 |
| Unitree G1 | fixed base: dual arms + waist | 17 | 17 | 17 / 34 |
| Unitree G1 | mobile base: fixed base + planar base | 20 | 20 | 20 / 40 |
| Unitree G1 | floating whole body | 36 | 29 | 36 / 71 |
| Unitree G1 | whole body with articulated hands | 50 or 36 by dynamics variant | 29 | 50 or 71 |
| AgiBot G1 | right arm / dual arm / fixed base / mobile base | 7 / 14 / 15 / 19 | same | same; mobile Dynamic2 is 38 |
| AgiBot G1 | mobile UnicycleDynamic1 / BicycleDynamic2 views | 19 | 19 | reduced defaults 3 / 6 |
| KUKA iiwa 14 | single / dual arm | 7 / 14 | 7 / 14 | single 7 or 14; dual 14 |
| Kinova Gen3 | single / dual arm | 7 / 14 | 7 / 14 | single 7 or 14; dual 14 |
| FANUC LR Mate 200iD | single / dual arm | 6 / 12 | 6 / 12 | single 6 or 12; dual 12 |
| Galaxea R1 Lite | right arm / dual arm / fixed base / mobile base | 6 / 12 / 15 / 18 | same | fixed base 15 or 30; others first order |

The `/` notation denotes available first- and second-order variants, not a
runtime-variable dimension. `CollisionConfig` variants attach sphere-based
collision volumes while retaining the corresponding control dimensions.

## Submodules

`base`
: Defines `RobotConfig`, `RobotKinematics`, and `RobotDynamicsModel`. A reduced
  dynamics view can select named state and control axes without changing the
  configuration's governing dynamics.

`kinematics`
: Provides model loading, bounded least-squares inverse kinematics, cuRobo
  integration, and the common forward-kinematics/Jacobian interface.

`numerical`
: Wraps continuous robot dynamics with discrete Euler or RK4 integration for
  simulation and controller examples.

Robot-family packages
: Each family has `config/` and `kinematics/` submodules. Unitree also exposes
  gripper actuation and filtering utilities.

## Selecting a configuration

Choose the smallest configuration that contains every coordinate used by the
task and policy. Choose Dynamic1 for velocity control and Dynamic2 for
acceleration-aware policies. The agent backend must support the selected
configuration. Every executable configuration declares its backend-to-agent
mapping. Unitree G1 declares MuJoCo, Isaac, and real mappings. Kinova Gen3 and
KUKA iiwa 14, FANUC LR Mate 200iD, Galaxea R1 Lite, and executable AgiBot G1
configurations declare MuJoCo plus scalar/tensor Isaac adapters. AgiBot
exposes embodiment-named Isaac wrappers over that generic machinery; its
unicycle and bicycle analytical views retain the full physical mobile
embodiment. Numerical-only configurations declare only the direct dynamics
backend.

```{toctree}
:maxdepth: 1

robot/overview
robot/unitree_g1
robot/agibot_g1
robot/galaxea_r1lite
robot/kinova_gen3
robot/kuka_iiwa14
robot/fanuc_lrmate200id
```
