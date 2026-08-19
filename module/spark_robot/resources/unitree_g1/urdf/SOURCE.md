# Unitree G1 model sources

- Repository: https://github.com/unitreerobotics/unitree_ros
- Commit: `aa0f5c68b5aba347bad409e71b6430407da758d7`
- Source directory: `robots/g1_description`
- Reference model: `g1_29dof_rev_1_0.urdf`
- License: BSD 3-Clause; see `LICENSE.unitree_ros`

SPARK keeps mechanism-specific MJCF sources in `../mjcf`. Every MJCF has a
URDF counterpart in this directory with the same basename. Run
`tools/generate_unitree_g1_urdf_variants.py` after changing an MJCF model to
regenerate the paired URDF files.

Both formats reference the single canonical mesh set in `../meshes`. The
generated hand variants include the articulated hand bodies already present
in their corresponding SPARK MJCF models.

Generated USD files are not checked into the repository.
`UnitreeG1IsaacAgent` selects the URDF matching the robot configuration's
`mujoco_model_path` and imports it into a content-addressed user cache whose
key covers the URDF, all referenced meshes, and the importer settings.
