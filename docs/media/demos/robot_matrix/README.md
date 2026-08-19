# SPARK v2 release demonstrations

This directory contains the eight curated animations embedded in the project
README. Keeping only this representative set makes release clones and pushes
practical while preserving examples across the supported robot families and
the Unitree WBT, Sport, and SONIC workflows.

| Robot/workflow | Recording |
|---|---|
| Kinova Gen3 single-arm MuJoCo teleop | [GIF](kinova_gen3__single_arm__mujoco_teleop.gif) |
| KUKA iiwa 14 dual-arm MuJoCo teleop | [GIF](kuka_iiwa14__dual_arm__mujoco_teleop.gif) |
| FANUC LR Mate 200iD single-arm MuJoCo teleop | [GIF](fanuc_lrmate200id__single_arm__mujoco_teleop.gif) |
| AgiBot G1 mobile-base MuJoCo teleop | [GIF](agibot_g1__mobile_base__mujoco_teleop.gif) |
| Galaxea R1 Lite mobile-base MuJoCo teleop | [GIF](galaxea_r1lite__mobile_base__mujoco_teleop.gif) |
| Unitree G1 whole-body WBT MuJoCo teleop | [GIF](unitree_g1__whole_body_wbt__mujoco_teleop.gif) |
| Unitree G1 whole-body Sport MuJoCo teleop | [GIF](unitree_g1__whole_body_sport__mujoco_teleop.gif) |
| Unitree G1 whole-body SONIC Isaac parallel benchmark | [GIF](unitree_g1__whole_body_sonic__isaac_parallel.gif) |

The [manifest](manifest.json) records encoding, timing, and numerical checks
for these retained recordings. It intentionally excludes local interpreter
paths and generation commands.

## Generate the complete matrix locally

The generator still supports all 21 profiles and five workflows per profile.
Write that complete 105-recording matrix to the ignored build directory rather
than checking it into the release repository:

```bash
python tools/generate_robot_demo_gifs.py --clean \
  --output-dir build/media/robot_matrix \
  --report build/media/robot_matrix/manifest.json \
  --mujoco-python "$CONDA_PREFIX/bin/python" \
  --isaac-python /path/to/spark_isaac/bin/python
```

A complete successful local run reports `105/105 PASS`. Review the beginning,
middle, and end of every recording before selecting a replacement for the
curated README set.
