# FANUC LR Mate 200iD/7L assets

The FANUC arm geometry and serial-chain definition are derived from the
ROS-Industrial [`ros-industrial/fanuc`](https://github.com/ros-industrial/fanuc)
repository, `noetic-devel` commit
`d8f42bd73584b255df87098395512538882caea1` (2025-02-19).

Upstream files used:

- `fanuc_lrmate200id_support/urdf/lrmate200id7l_macro.xacro`
- `fanuc_lrmate200id_support/meshes/lrmate200id/{visual,collision}`
- `fanuc_lrmate200id_support/meshes/lrmate200id7l/{visual,collision}`

The bundled `base_link`, `link_1`, `link_3`, `link_5`, and `link_6` meshes
match the upstream LR Mate 200iD visual meshes byte-for-byte. The bundled
`link_2` and `link_4` meshes match the upstream LR Mate 200iD/7L visual
meshes byte-for-byte, which is the combination selected by the official 7L
macro.

`lrmate200id7l.urdf` is a standalone simulator form of that macro. SPARK made
the following adaptations:

- resolved xacro constants and material macros to plain URDF;
- changed ROS package mesh URLs to repository-relative paths;
- retained the official 7L joint origins, axes, frame names, and mesh choice;
- aligned joint ranges and damping with `lrmate200id_single.xml` so MuJoCo and
  Isaac expose the same SPARK configuration-space contract;
- supplied nonzero effort limits from the MuJoCo actuator limits, because the
  upstream descriptive xacro intentionally leaves all effort limits at zero;
- uses the available visual meshes for collision geometry, matching SPARK's
  MuJoCo model. The upstream repository also provides separate collision
  meshes, but those were not part of the existing SPARK asset set.

The dual-arm Isaac configuration composes two prefixed copies of
`lrmate200id7l.urdf` at runtime. Its right and left mounting transforms
(`y = -0.45 m` and `y = +0.45 m`) match `lrmate200id_dual.xml`; no additional
robot geometry or kinematic convention is introduced. The checked-in
`lrmate200id7l_dual.urdf` remains as a standalone compatibility asset.

The upstream FANUC support package is BSD-3-Clause licensed. Its full license
is retained in `LICENSE.ros_industrial_fanuc`.

The Robotiq gripper meshes in this directory are separate assets and are not
covered by the ROS-Industrial FANUC license above.

SPARK assembles those existing Robotiq 3F meshes as a nine-joint articulated
gripper in the single-arm Isaac URDF. Its frames follow the upstream articulated
Robotiq layout: finger A is the fixed opposing finger, and fingers B/C form the
pair. The open pose matches MuJoCo, while the closed pose uses the same proximal,
middle, and distal register mapping as the MuJoCo adapter. Small inertials keep
the PhysX importer valid. The bundled upstream convex-hull collision meshes cover
the palm and every phalanx so configured PhysX self-contact also prevents the
gripper from passing through non-adjacent robot links. Runtime composition gives
each dual-arm instance an independent left/right gripper. Generic configurations
load the robot-only single/dual MJCF assets; legacy task-scene objects remain
explicit example resources rather than implicit reset state.
