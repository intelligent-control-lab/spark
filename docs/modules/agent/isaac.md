# Isaac Agent

The Isaac backend owns the simulation application lifecycle, imports or loads
robot assets, resolves articulation joints by name, applies configured control
modes, and provides NumPy or tensor feedback. Rendering can run inline or
through the renderer process, while headless execution omits presentation work.

Add `--show-simulation-info` to an interactive command to open a floating
**SPARK Simulation Information** window. The default viewport does not open
the window. It reports the selected robot configuration, dynamics model and
order, state/control dimensions, propagation owner, PhysX self-contact state,
environment count, device, control period, and live simulation time. The same
window is available for scalar, cloned tensor, and process-isolated renderers.
It is presentation metadata only: closing it does not change the simulation,
and `--headless` does not construct it. Custom agent integrations can opt in
with `viewer_show_simulation_info=True`.

SPARK launches the interactive window as a presentation viewport rather than
an Isaac scene-editing workspace. Stage, Property, Content, and other editor
sidebars are hidden after Kit finishes its lazy layout updates, while the
native application menu and the SPARK information window remain available.

Two adapter contracts are released:

- `ConfiguredIsaacAgent` consumes `RobotConfig.isaac_articulation` metadata
  and supports one physical articulation environment.
- `ConfiguredIsaacTensorAgent` consumes the same metadata, imports one source
  articulation, clones it, and keeps state and control tensors on CUDA. It is
  selected automatically when `num_envs > 1`.
- AgiBot G1, Galaxea R1 Lite, Kinova Gen3, KUKA iiwa 14, and FANUC LR Mate
  expose family/embodiment wrapper classes below their own
  `simulation/isaac/<robot>/` directories. These preserve clear public
  extension points while delegating to the same declarative scalar and tensor
  implementations. Batched construction through any wrapper automatically
  selects the tensor implementation.
- Unitree G1 retains its specialized single-environment and cloned tensor
  adapters for floating-base, learned-policy, and many-environment execution.

Scalar and tensor are execution contracts; a robot workflow must explicitly
define whether they share one physics realization. Unitree WBT does: both
one-environment teleoperation and parallel benchmarks use the specialized
IsaacLab tensor articulation and the content-addressed USD imported from
SPARK's portable local URDF. SPARK does not require a remote vendor USD. The
actuation and physics adaptation follows the
`OpenWBT Isaac controller <https://github.com/GalaxyGeneralRobotics/OpenWBT/blob/main/deploy/controllers/controller_isaacsim.py>`_:
named WBT gains through native position drives at every physics substep,
lower-body/waist and arm armatures of 0.01 and 0.001, half-mass wrist links,
four position solver iterations, zero velocity solver iterations, a 5 ms
physics step, and four-step policy decimation. SPARK additionally enables
physical articulation self-contact so hands and arms cannot penetrate the
legs or torso; this is deliberately separate from safe-controller collision
volume constraints. The interactive adapter adds only a scalar NumPy boundary
and defaults PhysX to CPU, while parallel execution retains tensors on CUDA
and lays out cloned environments.
Environment count therefore does not select another WBT asset, controller, or
contact realization. SONIC retains its own policy-specific Isaac actuation
configuration.

The generic adapter does not branch on a robot name. URDF path, ordered joint
names, fixed/floating-base selection, self-collision option, and physical drive
gains are declared by the robot configuration. A configuration may add sparse
per-joint gain overrides when one axis has materially different loading, such
as a gravity-loaded torso lift; scalar and tensor adapters consume the same
override metadata. Its simulator path converts the selected robot dynamics
command into position/velocity targets, advances PhysX at the common qualified
physics rate, and reports measured articulation state. Simulator-owned scalar
integration retains its position target but bounds its error from measured
joint state; this preserves the drive preload needed to hold gravity-loaded
links without allowing hidden contact windup. Tensor integration anchors its
batched state to measured joints on every control step and passes requested
joint velocity into the PhysX drive. Scalar articulated gripper targets are
also feedback-anchored with a bounded drive error, so a binary close command
cannot build an arbitrarily large target behind a physical contact. The model
path uses Isaac only as a viewer.

## RGB-D camera views

Every released scalar and tensor Isaac robot adapter accepts the same camera
contract as MuJoCo. `--enable-camera` creates an RTX RGB-D render product and
opens separate **SPARK RGB Camera** and **SPARK Depth Camera** windows by
default. These OpenCV windows run in a small display process so their Qt event
loop cannot interfere with Kit viewport keyboard input. Unitree G1 WBT uses a
torso-mounted head sensor;
the generic robot agents use a backend-neutral overview sensor unless a robot
configuration supplies a link-mounted camera. The latest RGB and metric-depth
arrays are included in `camera_feedback`.

Camera capture remains active without the native viewport. Combine
`--headless --enable-camera --no-camera-display` for programmatic capture.
The camera still requires RTX render updates, so it is intentionally slower
than a fully headless physics-only run. Resolution and rate use the shared
`--camera-width`, `--camera-height`, and `--camera-rate-hz` options.

The same generic contract is released for all 35 AgiBot G1, Galaxea R1 Lite,
Kinova Gen3, KUKA iiwa 14, and FANUC LR Mate 200iD physical configurations.
Planar mobile bases are generated from declarative x/y/yaw metadata instead of
robot-name conditionals. The release gate covers scalar and four-environment
tensor hold, nonzero command tracking, clone isolation, simulator-owned
dynamics, model-owned dynamics, teleoperation motion, and goal reaching.
For fixed-base URDFs, Isaac Sim 6 can replicate articulation data while
initializing every clone root at the source world pose. The tensor adapter
therefore applies `GridCloner`'s declared root positions explicitly and
restores them on reset. Physics/Fabric state and the recorded viewport then
show the same spatially separated clone grid.

## Self-collision ownership

Isaac physical link contact and SPARK self-collision avoidance are separate
mechanisms. `IsaacArticulationSpec.allow_self_collision` controls PhysX contact
generation, while the safety index controls whether collision-volume pairs are
constraints in the safe controller. Physical contact is robot-owned metadata,
not a runtime command-line choice; this keeps scalar and tensor agents on the
same qualified setting. FANUC single- and dual-arm configurations enable it by
default and provide palm/phalanx collision meshes. The shared manipulator
runners expose only `--enable-self-collision`, which enables SPARK
collision-volume self-pairs as safe-controller constraints. Those controller
constraints default off unless requested explicitly.

PhysX contact is authoritative only with `--dynamics-backend simulator`. With
`--dynamics-backend model`, Isaac visualizes state imposed by the SPARK model;
use `--enable-self-collision` with a safety algorithm such as RSSA when link
avoidance must constrain that modeled motion.

Repeated qualification across unrelated robot assets uses one Kit process per
robot configuration. This is deliberate: PhysX replication registrations are
native process state and should not be reused after a cloned source prim has
been deleted. Normal applications that create one robot world do not need an
extra process boundary.

## Device and viewport performance

Use CPU PhysX for one interactive environment and CUDA for cloned batches.
With the common four-by-5-ms schedule, synchronized headless qualification on
the release workstation measured 73.65 control steps/s for one CPU AgiBot
environment and 87.81 steps/s for one CPU R1 Lite environment. Sixteen CUDA
environments measured 29.42 steps/s for AgiBot (470.8 aggregate environment
steps/s), 35.87 steps/s for R1 Lite (573.9 aggregate), and 68.18 steps/s for
KUKA (1090.9 aggregate). The 20 ms simulated control clock is identical in all
cases, but a cloned workload whose measured loop exceeds 20 ms is not a
wall-clock-real-time workload. Controller and feedback work remained small;
the limiting stage was four synchronized PhysX submissions. These measurements
used `--profile-frequency`, which deliberately adds CUDA synchronization.

The viewport adds RTX presentation cost independently of physics. SPARK uses
one render owner, presents the interactive viewport at 10 Hz by default, and
captures RGB-D at up to 5 Hz. Control and physics retain the robot's declared
rate between presentation frames; the absolute-deadline real-time pacer lets
lighter non-rendered cycles recover after an RTX frame. Use
`--profile-frequency` on a teleoperation runner to verify the effective rate on
the local GPU and CPU. Interactive parallel benchmarks render goals and
obstacles for every clone by default. Detailed robot collision volumes are
shown on clone zero because authoring the full AgiBot sphere model on every
clone can block Kit's UI thread; `--max-visualized-envs N` raises that limit
deliberately. Physics and control run on every declared control step. The
AgiBot and shared cross-robot runners open a viewer by default; pass
`--headless` for an unattended run.

The viewer is navigation-only while a simulation is running: camera orbit,
pan, and zoom remain available, while USD prim selection, transform
manipulation, and the editor context menu are disabled. This prevents a ground
click or drag from interleaving Kit selection commands with live debug-material
updates. Use a full Isaac editor session outside the SPARK runner when scene
editing is required.

The AgiBot whole-body benchmark uses the same tensor runtime for one and many
Isaac environments. Nominal control, articulation feedback, collision queries,
and first- or second-order safety filtering stay batched; CPU inverse
kinematics runs only for rows being reset. Each row receives independently
sampled goals and obstacles and resets without advancing the other clones.
The generic tensor agent stores obstacle markers as persistent per-clone USD
prims; an indexed reset moves the affected markers immediately from the same
resampled tensors consumed by collision safety, matching the Unitree reset
contract.
The runtime reads its physics step and control decimation from the selected
AgiBot robot configuration. AgiBot uses four 5 ms physics steps per command,
matching Unitree's 20 ms, 50 Hz simulation contract. In this runner, use
`--max-visualized-collision-envs N` to increase detailed collision overlays.

On the release workstation, the post-unification headless AgiBot v0 workload
measured 73.65 control steps/s for one CPU environment and 29.42 control
steps/s for 16 CUDA environments (470.8 aggregate environment-steps/s). The
single environment exceeds the 50 Hz real-time requirement. The 16-environment
job remains a stable high-throughput data workload but does not execute its
20 ms simulated cycles in 20 ms of wall time on that machine. These are
workload- and hardware-specific measurements; enabling safety or the viewport
adds cost.

With the viewer enabled, a matched 16-environment comparison presented one
frame every three control steps and disabled collision-volume and witness-line
overlays. AgiBot measured 35.25 control steps/s for v0 and 24.88 for v1 with
RSSA; Unitree WBT measured 22.09 and 18.36 control steps/s, respectively. The
Unitree run reported fall or workspace resets during this short measurement,
so these values compare loop frequency rather than task success. Both v1
runtimes represented independent spherical obstacles as `PointCloudBatch`
tensors and called the common
`TorchSphereCollisionBackend.query_environment_nearest` implementation with
one retained constraint per robot sphere for this comparison.
