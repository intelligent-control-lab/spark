# Galaxea R1 Lite assets

The R1 Lite URDF and meshes are adapted from Galaxea Dynamics' official
[`userguide-galaxea/URDF`](https://github.com/userguide-galaxea/URDF)
repository. The current upstream reference is `galaxea/main` commit
`2e5d31e1784481a34d178006c0d0e18e0a84a82a` (2026-07-24); SPARK intentionally
retains the 2025 R1 Lite geometry rather than switching robot revisions.

Compared with the official `R1Lite/urdf_r1lite_2025.urdf`, SPARK's simulator
form resolves ROS package mesh paths to repository-relative paths, uses the
bundled textured OBJ files for visuals and dedicated STL collision meshes,
fixes wheel/steering joints for the fixed-base articulation, and preserves the
torso, arm, and gripper joint transforms used by SPARK's MuJoCo models.

Both backends use the same bundled `meshes/texture.jpeg` artwork. MuJoCo uses
the lossless `texture.png` conversion because its XML texture loader does not
accept JPEG files; no image content or UV mapping is changed.

The upstream repository does not currently publish a license file. These
assets therefore need an explicit redistribution grant from Galaxea Dynamics
before a third party can assume rights beyond the upstream project's stated
usage. SPARK records the source here rather than assigning an unsupported
license to vendor geometry.
