# SPARK 2.0 release notes

SPARK 2.0 is the next major release after the repository's `v1.0` tag. It
turns the original safe-control research code into a backend-neutral toolbox
with explicit robot, task, policy, environment, and execution contracts.

## Major new capabilities

- **General robot toolbox:** dynamics, LQR/MPC/ILC, estimation, planning,
  kinematics, and safety code now consume robot-provided dimensions and model
  metadata instead of Unitree- or course-specific assumptions.
- **Expanded robot library:** AgiBot G1, Galaxea R1 Lite, Kinova Gen3, KUKA
  iiwa 14, FANUC LR Mate 200iD, and Unitree G1 include documented fixed,
  mobile, single-arm, dual-arm, or whole-body configurations where applicable.
- **Two simulation backends:** all executable physical robot configurations
  declare MuJoCo and Isaac capabilities. Isaac provides both scalar and cloned
  tensor execution, while robot-specific learned Unitree paths remain isolated
  behind the same public launchers.
- **Matched workflows:** safe keyboard teleoperation and deterministic v0/v1
  benchmarks share goal, obstacle, reset, collision-geometry, camera, and
  rendering contracts across MuJoCo and Isaac.
- **Parallel GPU evaluation:** independent per-environment goals, obstacles,
  metrics, resets, batched kinematics, distance queries, and tensor safety
  filtering support reproducible cloned Isaac benchmarks. Scalar and tensor
  physical drives share a bounded-target state-ownership contract.
- **Improved robot behavior:** simulator timing, tracking gains, actuation,
  self-contact, collision spheres, grippers, materials, cameras, and benchmark
  workspaces were qualified for the supported robot families.
- **Unitree learned control:** WBT, Sport, external SONIC, and optional native
  batched SONIC paths have explicit dependency and GPU validation gates.
- **Visualization and recording:** every simulation workflow supports RGB-D
  capture, consistent goal/obstacle/collision overlays, optional simulation
  information panels, and reproducible benchmark recordings.
- **Installation profiles:** Python 3.10 remains the core and MuJoCo default;
  the optional Isaac Sim 6 profile uses Python 3.12. ROS, Unitree SDK, PyTorch,
  learned runtimes, and development tools are opt-in, and Apple Silicon MuJoCo
  plus Intel-macOS core installations are documented.
- **Containers:** versioned core, MuJoCo, Isaac, ROS Jazzy, and WSL Compose
  profiles modernize the v1 Docker workflow without forcing CUDA into the
  default image. A compatibility launcher and VS Code dev container are
  included.
- **Release engineering:** strict CPU CI, documentation builds, package-data
  checks, optional-import boundaries, robot/workflow conformance tests,
  machine-readable support matrices, and documented GPU gates protect the
  release surface.

## Compatibility and migration

SPARK 2.0 intentionally changes public names and defaults, so it is not a
drop-in package update from 1.0.

- Every distribution and every internal SPARK dependency pin moves to
  `2.0.0`; Python 3.10 or newer is required.
- User-facing robot scripts follow `run_<robot>_teleop.py` and
  `run_<robot>_benchmark.py` naming and share `--backend`,
  `--dynamics-backend`, and `--headless`. Viewers are enabled unless
  `--headless` is supplied.
- Simulation information is disabled by default and enabled explicitly with
  `--show-simulation-info`.
- AgiBot mobility configurations name both the equation and uniform dynamics
  order: `MobileBaseUnicycleDynamic1` and `MobileBaseBicycleDynamic2`.
- Legacy course-specific analysis and unused controller shims are not part of
  the 2.0 public API. Downstream coursework remains in its own repository.
- Optional simulator, learned-policy, ROS, and hardware imports must be
  installed through their corresponding profile or installer flag.

Keep the historical `v1.0` tag intact and create a new `v2.0.0` tag on the
reviewed release commit. The detailed implementation-level list is maintained
in the repository
[CHANGELOG.md](https://github.com/intelligent-control-lab/spark/blob/master/CHANGELOG.md).

## Publisher sign-off required

The technical release gates do not establish redistribution rights. Before
publishing 2.0.0, resolve or exclude the AgiBot, Galaxea, separately bundled
Robotiq, Sport-checkpoint, and collision-monitor assets identified in
`THIRD_PARTY_NOTICES.md` and the resource-local provenance records. OpenWBT's
Apache-2.0 source, checkpoints, retained license, and SPARK modification notice
have been recorded separately. Keep every publisher decision with the release
evidence.
