# Kinova Gen3 assets

The seven-DoF Gen3 arm URDF and the nine files under `meshes/gen3/` come from
Kinova Robotics' official [`ros_kortex`](https://github.com/Kinovarobotics/ros_kortex)
repository, `noetic-devel` commit
`c3280d8a6a6b5d96590d51538e3f265710a6c77d` (qualified 2026-08-14).

The bundled meshes match
`kortex_description/arms/gen3/7dof/meshes/*.STL` byte-for-byte. The bundled
The arm-chain portion of `gen3.urdf` matches the official
`GEN3-7DOF-VISION_ARM_URDF_V12.urdf` after replacing its ROS
`package://kortex_description/arms/gen3/7dof/meshes/` URLs with the local
`./meshes/gen3/` directory. Its joint transforms, inertias, limits, and visual
and collision selections are otherwise unchanged. SPARK additionally attaches
the fixed display gripper described below.

`LICENSE.kinova` retains the upstream BSD-3-Clause notice. The Robotiq gripper
meshes and MuJoCo XML files predate this provenance audit and are not claimed
as files from `ros_kortex`; their licensing should be audited separately before
redistribution if they are included in a binary asset bundle.

The Isaac URDF includes a lightweight articulated Robotiq 2F-85 assembled from
those already bundled meshes so it matches the MuJoCo embodiment. Two auxiliary
prismatic joints approximate the coupled finger travel and remain outside the
seven arm DoFs; SPARK drives them through its shared binary gripper contract.
The display gripper intentionally adds no collision geometry. Generic Kinova
configurations load the robot-only `gen3_2f85_{single,dual}.xml` assets; task
scenes with loose food/grill objects remain available as explicit example
assets and cannot destabilize a normal robot reset.
