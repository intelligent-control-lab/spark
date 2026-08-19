# Third-party assets and notices

SPARK source code is distributed under the repository's MIT license. Robot
descriptions, meshes, textures, learned-model files, and imported runtimes may
be governed by their own upstream terms. The root MIT license does not replace
those terms.

The robot-resource directories retain the most precise source and adaptation
records currently available:

- `module/spark_robot/resources/fanuc_lrmate200id`: ROS-Industrial FANUC arm
  geometry under the retained BSD-3-Clause notice. The separately bundled
  Robotiq gripper meshes are not covered by that notice and still require a
  redistribution audit.
- `module/spark_robot/resources/kinova_gen3`: official Kinova Gen3 arm files
  under the retained BSD-3-Clause notice. The older bundled Robotiq gripper
  meshes and MuJoCo XML files still require a separate redistribution audit.
- `module/spark_robot/resources/kuka_iiwa14`: model distributed under the
  retained BSD-3-Clause notice.
- `module/spark_robot/resources/unitree_g1`: Unitree G1 descriptions retain
  the Unitree ROS BSD-3-Clause notice beside the URDF resources.
- `module/spark_robot/resources/galaxea_r1lite`: adapted from Galaxea's public
  R1 Lite repository, which did not contain a license when qualified. Obtain
  an explicit redistribution grant before publishing these vendor assets.
- `module/spark_robot/resources/agibot_g1`: reconciled against a
  contributor-supplied A2D/Omnipicker archive for which no public upstream
  license was identified. Obtain contributor/vendor redistribution clearance
  before publishing these assets.

Each listed directory contains a `README.md` with exact upstream revisions,
file scope, and SPARK adaptations; licensed sources also retain their license
files beside the assets. Before a public source or binary release, the
publisher must either document the missing permissions or exclude the
affected assets and any configurations that require them.

The bundled policy/runtime assets have these additional terms and provenance
records:

- `module/spark_policy/spark_policy/control/whole_body/unitree_g1/wbt/runtime`:
  adapted from GalaxyGeneralRobotics/OpenWBT commit
  `a0154ca822815c8eca26f5174afa6268f8e3d465` under Apache License 2.0. The
  retained license and modification notice are in that directory. Its
  `loco.onnx` and `squat.onnx` files are byte-identical to that revision.
- `module/spark_policy/spark_policy/control/whole_body/unitree_g1/sport/resources/motion.pt`:
  the binary entered SPARK's history without a recorded upstream source or
  separate license. Confirm that SPARK has redistribution rights or exclude
  the Sport policy checkpoint from the public release.
- `module/spark_policy/spark_policy/safety/monitoring/collision/resources/onnx_model`:
  the collision-monitor ONNX binaries are repository-native artifacts, but
  their training/export provenance is not recorded beside them. The publisher
  should archive that provenance before release.
