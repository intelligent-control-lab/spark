# KUKA LBR iiwa 14 Description (MJCF)

Requires MuJoCo 2.3.3 or later.

## Overview

This package contains a simplified robot description (MJCF) of the [LBR iiwa](https://www.kuka.com/en-us/products/robotics-systems/industrial-robots/lbr-iiwa) 14kg developed
by [KUKA Robotics](https://www.kuka.com/en-us). It is derived from the [publicly available](https://github.com/RobotLocomotion/drake/blob/master/manipulation/models/iiwa_description/urdf/iiwa14_spheres_dense_collision.urdf)
URDF description created by the [Drake](https://github.com/RobotLocomotion/drake) developers.

<p float="left">
  <img src="iiwa_14.png" width="400">
</p>

## URDF → MJCF derivation steps

1. Added `<mujoco> <compiler discardvisual="false"/> </mujoco>` to the
   [URDF](https://github.com/RobotLocomotion/drake/blob/master/manipulation/models/iiwa_description/urdf/iiwa14_spheres_dense_collision.urdf)'s
   `<robot>` clause in order to preserve visual geometries.
2. Loaded the URDF into MuJoCo and saved a corresponding MJCF.
3. Created base body and added its corresponding inertial properties.
4. Added a tracking light to the base.
5. Manually edited the MJCF to extract common properties into the `<default>` section.
6.  Added `<exclude>` clauses to prevent collisions between `base` and `link1`.
7.  Added actuators for the arm.
8.  Added forcelimits to match the torque limits in the [spec sheet](https://www.reeco.co.uk/wp-content/uploads/2020/05/KUKA-LBR-iiwa-technical-data.pdf).
9.  Added `scene.xml` which includes the robot, with a textured groundplane, skybox, and haze.

SPARK's generic configurations load the robot-only 2F-85 assets rather than
the legacy manipulation scenes. The Isaac URDF carries a lightweight
articulated Robotiq 2F-85 assembled from the bundled meshes. Two auxiliary
prismatic joints approximate coupled finger travel while remaining outside
SPARK's seven arm DoFs. The dual-arm MuJoCo asset includes the
same file-backed grid floor and light sky styling used by release recordings;
file-backed textures also keep Pinocchio's MJCF parser compatible.

The SPARK MuJoCo variants replace the original 500,000-gain first-joint
servo with the same bounded 2,000-gain drive used by the remaining load-bearing
joints. This avoids visible chatter during repeated benchmark goal changes.
The released simulator contract uses 5 ms physics and four substeps per 20 ms
control update; both MuJoCo and Isaac were qualified with that 50 Hz schedule.
The 500-gain wrist drives use 90 damping, which preserves stable stopping at
the configured physics step without changing the configured wrist force limit.
The Isaac adapter treats the single-arm URDF's fixed -90-degree root rotation
as asset calibration rather than logical base motion, keeping rendered links,
Cartesian goals, and collision volumes in one frame.

## License

This model is released under a [BSD-3-Clause License](LICENSE).
