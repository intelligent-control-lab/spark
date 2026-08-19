"""Numeric-only robot model loading compatible with Pinocchio 3 and 4."""

from __future__ import annotations

import os
from pathlib import Path
import tempfile
import xml.etree.ElementTree as ET

import pinocchio as pin


def build_robot_from_mjcf(filename, root_joint=None):
    """Build a RobotWrapper without parsing collision or visual geometry.

    Pinocchio 4 validates MJCF materials while constructing geometry models,
    which can reject otherwise valid legacy SPARK scene files. Kinematics only
    needs the numeric model, so geometry is intentionally loaded elsewhere.
    """
    major_version = int(pin.__version__.split(".", maxsplit=1)[0])
    if major_version < 4:
        if root_joint is None:
            return pin.RobotWrapper.BuildFromMJCF(filename)
        return pin.RobotWrapper.BuildFromMJCF(filename, root_joint)
    if root_joint is None:
        model = pin.buildModelFromMJCF(filename)
    else:
        model = pin.buildModelFromMJCF(filename, root_joint, "root_joint")
    if isinstance(model, tuple):
        model = model[0]
    return pin.RobotWrapper(model)


def build_reduced_robot(robot, list_of_joints_to_lock, reference_configuration):
    """Reduce a model using joint IDs on both Pinocchio 3 and 4."""
    joint_ids = []
    for joint in list_of_joints_to_lock:
        if isinstance(joint, str):
            if not robot.model.existJointName(joint):
                continue
            joint_ids.append(int(robot.model.getJointId(joint)))
        else:
            joint_id = int(joint)
            if 0 < joint_id < robot.model.njoints:
                joint_ids.append(joint_id)
    reduced_model = pin.buildReducedModel(
        robot.model,
        joint_ids,
        reference_configuration,
    )
    return pin.RobotWrapper(reduced_model)


def build_geometry_from_mjcf(model, filename, geometry_type):
    """Load MJCF geometry across Pinocchio 3 and 4 scene-parser behavior.

    Some legacy SPARK MJCF files keep renderer-only textures and materials in
    a second ``asset`` block. MuJoCo and Pinocchio 3 accept this, whereas
    Pinocchio 4's geometry parser may either miss the material declaration or
    reject procedural textures. Retry only those known parser failures with a
    temporary copy that removes renderer-only declarations. Mesh paths are
    made absolute, and the repository source file is never changed.
    """
    filename = os.fspath(filename)
    try:
        return pin.buildGeomFromMJCF(model, filename, geometry_type)
    except ValueError as exc:
        message = str(exc)
        known_scene_asset_error = (
            "Cannot find material" in message or "Only textures with files are supported" in message
        )
        if not known_scene_asset_error:
            raise

    source_path = Path(filename).resolve()
    tree = ET.parse(source_path)
    root = tree.getroot()
    compiler = root.find("compiler")
    if compiler is not None:
        for path_attribute in ("meshdir", "texturedir"):
            relative_path = compiler.get(path_attribute)
            if relative_path and not os.path.isabs(relative_path):
                compiler.set(
                    path_attribute,
                    str((source_path.parent / relative_path).resolve()),
                )

    for asset in root.findall("asset"):
        for element in list(asset):
            if element.tag in {"texture", "material"}:
                asset.remove(element)
    for geometry in root.iter("geom"):
        geometry.attrib.pop("material", None)
    worldbody = root.find("worldbody")
    if worldbody is not None:
        # Direct world geoms describe the simulator environment (for example,
        # the floor), not the robot's self-collision/visual model.
        for element in list(worldbody):
            if element.tag == "geom":
                worldbody.remove(element)

    temporary_path = None
    try:
        with tempfile.NamedTemporaryFile(
            mode="wb", suffix=".xml", prefix="spark_pinocchio_", delete=False
        ) as temporary_file:
            temporary_path = temporary_file.name
            tree.write(temporary_file, encoding="utf-8", xml_declaration=True)
        return pin.buildGeomFromMJCF(model, temporary_path, geometry_type)
    finally:
        if temporary_path is not None:
            try:
                os.unlink(temporary_path)
            except FileNotFoundError:
                pass
