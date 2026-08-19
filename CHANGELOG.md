# Changelog

All notable changes to SPARK are documented in this file. The project uses
semantic versioning across all seven Python distributions.

## Unreleased

## 2.0.0 - 2026-08-17

- Restored Python 3.10 as the default supported runtime for core, MuJoCo, CPU
  CI, and local documentation while retaining Python 3.12 for the qualified
  Isaac stack.
- Added explicit macOS installation guidance and platform guards plus a macOS
  core CI gate; native Apple Silicon supports MuJoCo while Intel Macs support
  the numerical core.
- Decoupled the core toolbox from optional simulator, learned-policy, ROS, and
  hardware dependencies through selectable installation profiles and extras.
- Restored and modernized container support with versioned core, MuJoCo,
  Isaac, ROS Jazzy, and WSL Compose profiles, a VS Code dev container, and a
  backward-compatible `run_with_docker.sh` launcher. The default image no
  longer installs CUDA or requires an NVIDIA runtime unless requested.
- Generalized linear-quadratic control, MPC, iterative learning control, and
  filtering paths around robot-provided dynamics and dimensions.
- Added lazy optional imports so the base packages load without PyTorch, ONNX,
  MuJoCo, Isaac, ROS, or Unitree SDK runtimes.
- Corrected batched SONIC height filtering and added regression coverage.
- Made namespace packages and robot resource files available in built wheels.
- Added installed-wheel coverage for robot-local collision-sphere databases;
  release checks now guard these runtime JSON assets explicitly.
- Standardized robot dynamics metadata and backend mappings across all released
  robot families; added robot-independent scalar and CUDA-cloned Isaac adapters
  for all 35 executable AgiBot, Galaxea, Kinova, KUKA, and FANUC physical
  configurations while retaining Unitree's specialized learned-policy path.
- Corrected the R1 Lite fixed-base second-order input matrix to map acceleration
  controls to DoFs by joint name instead of relying on their intentionally
  different public enum orders.
- Added a backend-neutral teleop/goal-reaching conformance runner, isolated
  full-matrix orchestration with bounded Isaac startup retries, machine-readable
  release reports, and viewport/MuJoCo GIF recording.
- Kept the conformance feedback contract backend-neutral by limiting it to the
  state and command fields consumed by the runner; MuJoCo no longer needs to
  manufacture an unused last-control field.
- Closed the release conformance controller around measured plant feedback
  instead of simulator-internal targets, so physical lag remains visible and
  scalar and CUDA-cloned articulation checks exercise the same control law.
- Added a compact 15-case cross-backend benchmark showcase with matched camera,
  grid, lighting, goals, obstacles, paths, and collision-volume overlays;
  aligned AgiBot and R1 Lite materials and industrial-arm display grippers
  between MuJoCo and Isaac.
- Switched generic Kinova, KUKA, and FANUC configurations to robot-only MuJoCo
  assets so loose example-scene objects cannot destabilize initialization.
- Restored Kinova teleoperation's explicit Cartesian home goals with canonical
  identity/gripper-down orientation, constrained its IK to declared joint
  limits, lowered its pose benchmark to the reachable tool-down workspace, and
  corrected MuJoCo position-servo velocity tracking for first- and second-order
  simulator dynamics; also damped high-frequency motion in the passive Robotiq
  linkage and made teleoperation choose a dynamics-compatible safety index.
- Bounded Kinova's affine position-servo requests below the engine force clamp,
  preserving stable tracking on both MuJoCo 3.4 and Isaac Sim's MuJoCo 3.8
  dependency after its implicit actuator-derivative correction.
- Added adjacent base/shoulder contact exclusions to the standalone Kinova
  MJCFs, removing an initialization penetration created when MuJoCo fuses a
  fixed base body into the world.
- Made simulator-owned Isaac tensor drives use the same bounded command-target
  contract as the scalar adapter: first-order commands anchor to measured
  articulation state, while second-order commands retain a finite
  gravity-supporting preload. Forwarded velocity targets to PhysX drives and
  retuned Kinova's benchmark tracking gain to remove command windup, steady
  tracking error, and goal overshoot. The nonlinear bicycle adapter converts
  its declared mass/inertia acceleration into an equivalent physical drive
  displacement instead of applying an under-scaled integration increment.
- Enabled Kinova hand control in safe teleoperation and replaced its fixed
  Isaac display gripper with a lightweight articulated two-finger adapter, so
  the shared `[`/`]` keyboard contract opens and closes the Robotiq gripper in
  both MuJoCo and Isaac without changing the arm DoF vector.
- Added a shared dual-arm goal-pair keep-out used by scalar and tensor
  benchmarks, configured Kinova targets to remain 0.25 m apart, and enabled
  physical self-contact for its dual-arm Isaac articulation.
- Made AgiBot, Galaxea, Kinova, KUKA, and FANUC safe-teleoperation entry points
  resolve MuJoCo or Isaac agents from robot capability metadata and share one
  correct simulator lifecycle.
- Made robot-local examples, simulator adapters, assets, configurations, and
  documentation independent of other robot families; robot-local benchmark
  wrappers now reject foreign configurations, and a regression gate enforces
  that boundary. Corrected single-arm benchmark completion so it evaluates
  only the available end effector.
- Qualified official/ROS descriptions and provenance for FANUC, Galaxea, and
  Kinova, including a corrected standalone Kinova mesh path; explicitly
  recorded unresolved AgiBot and Galaxea redistribution caveats.
- Corrected the Unitree dual-arm control classification and renamed the AgiBot
  analytical views to `MobileBaseUnicycleDynamic1` and
  `MobileBaseBicycleDynamic2`, making mobility equation and uniform dynamics
  order explicit while retaining the full physical mobile embodiment and its
  shared MuJoCo/Isaac agents.
- Replaced AgiBot's hand-only safety geometry with a reproducible FOAM
  whole-body sphere model, aligned gripper goal frames with the URDF, tuned
  lift/base simulator tracking, corrected its public white/charcoal palette,
  and grounded its benchmark wrapper in the shared whole-body v0/v1
  task cases with dual-arm plus base goals, runner-local X/Z arm-goal
  offsets, and ten default episode resets.
- Added a true batched AgiBot Isaac whole-body benchmark: independent goals,
  obstacles, counters, and indexed resets per clone; tensor nominal control,
  collision queries, and first-/second-order safety filtering; and timing
  inherited from the selected AgiBot robot configuration. Qualified an
  AgiBot-specific 10 ms by two-substep schedule that preserves the 20 ms
  control period and brings the 16-environment whole-body workload to parity
  with the specialized Unitree parallel pipeline. Both whole-body cases apply
  the runner-local X/Z arm-goal offset, and AgiBot and Unitree now have a
  regression gate requiring the common batched tensor sphere-distance query.
  The AgiBot viewport now also renders the batched safety query's blue trigger
  and purple residual-violation witness segments and reports initial base-goal
  diversity. Indexed resets now commit freshly sampled obstacle tensors to
  persistent per-clone markers before the next control step. Isaac simulation
  viewports disable editor selection/manipulation while retaining camera
  navigation, avoiding a Kit crash caused by ground selection interleaved with
  debug-material updates.
- Unified task and debug obstacles on one orange palette across MuJoCo and
  Isaac, and removed the conformance visualizer's redundant MuJoCo grid layer.
- Reduced Isaac collision-volume overlay opacity without changing safety
  geometry, and made generic Isaac CLIs select CPU for one environment and
  CUDA for cloned batches unless the device is explicitly overridden.
- Corrected pipeline/backend defaults, optional ROS/Pinocchio name resolution,
  and KUKA object-orientation handling found by the release correctness audit.
- Qualified second-order R1 Lite and KUKA MuJoCo execution through
  inverse-dynamics actuator-force mapping, while retaining their existing
  first-order physical servo behavior.
- Removed unconditional robot-kinematics debug output and made KUKA MeshCat
  collision visualization lazy instead of starting servers during construction.
- Removed the course-specific MRAC analysis shim; downstream experiments now
  analyze the controller's general history records in their own repository.
- Removed an unused legacy Inspire-hand controller, added a focused Ruff
  correctness gate, and normalized first-party Python formatting while keeping
  the imported Unitree WBT runtime as an explicit vendor boundary.
- Added strict CPU CI, documentation checks, package/repository release checks,
  and a downstream course-repository boundary.
- Added a robot-independent MuJoCo viewer overlay reporting the configured
  dynamics model, state propagation backend, dimensions, timing, and robot
  configuration, including runtime metadata overrides for external trajectory
  playback.
- Kept the Unitree hardware agent inspectable in simulator-only installations
  by deferring Unitree SDK imports until hardware-agent construction.
- Requalified the optional NVIDIA path end to end with CUDA PyTorch, Isaac
  PhysX articulation control, cuRobo IK, parallel WBT and Sport inference,
  tensor RSSA safety, and the external shared-session SONIC TensorRT server;
  documented repeatable GPU release gates without adding GPU dependencies to
  the core or MuJoCo profiles.
- Added an optional planner-free, GPU-native SONIC encoder/decoder adapter for
  upstream full-body references, with independent recurrent state per row,
  dynamic-batch validation, and a standalone throughput gate verified at
  4,096 rows.
- Unified Unitree WBT teleoperation and one/many-environment Isaac benchmarks
  on the same locally imported plant, named native-drive gains, armatures,
  wrist mass adaptation, self-contact model, and 5 ms by four-step timing;
  added backend-neutral deterministic scenario streams and simultaneous arm
  and base goal tracking with branch-local safety corrections.
- Added backend-neutral RGB-D camera capture to every released MuJoCo and
  Isaac teleoperation workflow, including the Unitree head camera, generic
  robot overview cameras, capture-only operation, and process-isolated display
  windows that do not interfere with Isaac keyboard input.
- Made the MuJoCo overlay and Isaac simulation-information window explicit
  `--show-simulation-info` opt-ins, disabled by default across teleoperation
  and benchmark launchers, while preserving scalar, tensor, and isolated
  renderer support.
