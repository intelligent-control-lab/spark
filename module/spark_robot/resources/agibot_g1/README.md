# AgiBot G1 assets

The MuJoCo models, `agibot_g1_mobile_base.urdf`, and meshes entered SPARK in
commit `3a520d881f31f11753adc71274194490da1b1628` (2026-05-11). The URDF's
internal robot name is `A2D`; SPARK's public configuration family is named
`AgiBotG1` for compatibility with the existing API.

The bundle was reconciled in August 2026 against the contributor-supplied
`A2D_Omnipicker/A2D.urdf`: its URDF and 31 STL meshes match this geometry set.
SPARK adds only a tiny positive inertia to the source's massless `link_arm` so
PhysX can import it. No public upstream URL or redistribution license for that
contributor-supplied archive was identified, so SPARK does not claim that it
is an official or ROS-Industrial release. Redistribution status must still be
resolved with the original contributor/vendor before an asset release.

The Isaac adapter uses this same URDF rather than creating robot-specific
simulation code. Right-arm, dual-arm, and fixed-base configurations select
their controlled joints by name; other joints remain part of the common
articulation.

The source URDF assigns one gray color to every part. Simulator adapters apply
a shared white, silver, and charcoal palette based on [AgiBot's published G1
imagery](https://www.agibot.com/products/G1). The palette changes visual
material only, not geometry, collision, or dynamics.

`agibot_g1_collision_spheres.json` is the reproducible whole-body sphere
database generated with CoMMALab FOAM from the bundled URDF:

```bash
python /path/to/foam/scripts/generate_sphere_urdf.py \
  agibot_g1_mobile_base.urdf \
  --output /tmp/agibot_g1_spherized.urdf \
  --database agibot_g1_collision_spheres.json \
  --depth 1 --branch 8 --threads 16 \
  --manifold_leaves 500 --simplification_ratio 0.2
```

Runtime keeps this sparse 118-sphere topology and applies documented
per-region radius calibration in `collision_geometry.py`. It does not use a
denser FOAM level, so the number of environment collision checks is unchanged.

SPARK loads the depth-one spheres for the chassis, torso, head, arms, and
gripper bases. The database also retains the finger results for provenance;
runtime end-effector spheres conservatively cover the grasping regions.
