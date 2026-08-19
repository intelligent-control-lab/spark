#!/usr/bin/env python3
"""Generate one URDF counterpart for every Unitree G1 MJCF model.

The MJCF files remain the source of truth for each reduced/full mechanism.
Generated URDF files preserve the MJCF body, joint, inertial, visual, and
collision structure while referencing the same ``resources/unitree_g1/meshes`` files.
"""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET


def _numbers(value: str | None, default: tuple[float, ...]) -> tuple[float, ...]:
    if value is None:
        return default
    return tuple(float(part) for part in value.split())


def _format(values: tuple[float, ...] | list[float]) -> str:
    return " ".join(f"{value:.12g}" for value in values)


def _quat_to_rpy(value: str | None) -> tuple[float, float, float]:
    w, x, y, z = _numbers(value, (1.0, 0.0, 0.0, 0.0))
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    if norm == 0.0:
        return 0.0, 0.0, 0.0
    w, x, y, z = (component / norm for component in (w, x, y, z))
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_term = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(pitch_term)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def _origin(parent: ET.Element, *, xyz: str | None = None, quat: str | None = None) -> None:
    ET.SubElement(
        parent,
        "origin",
        xyz=xyz or "0 0 0",
        rpy=_format(_quat_to_rpy(quat)),
    )


def _joint_velocity(name: str) -> float:
    if name.startswith("pelvis_"):
        return 10.0
    if "ankle" in name:
        return 30.0
    if "knee" in name or "hip_roll" in name:
        return 20.0
    if "hip" in name or "waist" in name:
        return 32.0
    if "wrist_pitch" in name or "wrist_yaw" in name:
        return 22.0
    if "hand_" in name:
        return 20.0
    return 37.0


def _add_inertial(link: ET.Element, source: ET.Element | None) -> None:
    if source is None:
        return
    inertial = ET.SubElement(link, "inertial")
    _origin(inertial, xyz=source.get("pos"), quat=source.get("quat"))
    ET.SubElement(inertial, "mass", value=source.get("mass", "0.001"))
    diagonal = _numbers(source.get("diaginertia"), (1e-7, 1e-7, 1e-7))
    ET.SubElement(
        inertial,
        "inertia",
        ixx=f"{diagonal[0]:.12g}",
        ixy="0",
        ixz="0",
        iyy=f"{diagonal[1]:.12g}",
        iyz="0",
        izz=f"{diagonal[2]:.12g}",
    )


def _add_tiny_inertial(link: ET.Element) -> None:
    inertial = ET.SubElement(link, "inertial")
    _origin(inertial)
    ET.SubElement(inertial, "mass", value="0.001")
    ET.SubElement(
        inertial,
        "inertia",
        ixx="1e-7",
        ixy="0",
        ixz="0",
        iyy="1e-7",
        iyz="0",
        izz="1e-7",
    )


def _add_geometry(parent: ET.Element, geom: ET.Element, mesh_files: dict[str, str]) -> bool:
    geometry = ET.SubElement(parent, "geometry")
    geom_type = geom.get("type", "mesh" if geom.get("mesh") else "sphere")
    size = _numbers(geom.get("size"), ())
    if geom_type == "mesh":
        mesh_name = geom.get("mesh")
        if mesh_name not in mesh_files:
            return False
        ET.SubElement(geometry, "mesh", filename=f"../meshes/{mesh_files[mesh_name]}")
    elif geom_type == "sphere" and size:
        ET.SubElement(geometry, "sphere", radius=f"{size[0]:.12g}")
    elif geom_type == "box" and len(size) >= 3:
        ET.SubElement(geometry, "box", size=_format([2.0 * value for value in size[:3]]))
    elif geom_type == "cylinder" and len(size) >= 2:
        ET.SubElement(
            geometry,
            "cylinder",
            radius=f"{size[0]:.12g}",
            length=f"{2.0 * size[1]:.12g}",
        )
    else:
        parent.remove(geometry)
        return False
    return True


def _add_body_geometries(link: ET.Element, body: ET.Element, mesh_files: dict[str, str]) -> None:
    for index, geom in enumerate(body.findall("./geom")):
        visual_only = geom.get("contype") == "0" and geom.get("conaffinity") == "0"
        is_visual = visual_only or geom.get("group") == "1" or geom.get("contype") is None
        is_collision = not visual_only and geom.get("group") != "1"
        if is_visual:
            visual = ET.SubElement(link, "visual", name=f"{link.get('name')}_visual_{index}")
            _origin(visual, xyz=geom.get("pos"), quat=geom.get("quat"))
            if not _add_geometry(visual, geom, mesh_files):
                link.remove(visual)
            elif geom.get("rgba"):
                material = ET.SubElement(
                    visual, "material", name=f"{link.get('name')}_material_{index}"
                )
                ET.SubElement(material, "color", rgba=geom.get("rgba"))
        if is_collision:
            collision = ET.SubElement(
                link, "collision", name=f"{link.get('name')}_collision_{index}"
            )
            _origin(collision, xyz=geom.get("pos"), quat=geom.get("quat"))
            if not _add_geometry(collision, geom, mesh_files):
                link.remove(collision)


def _joint_type(source: ET.Element) -> str:
    mjcf_type = source.get("type", "hinge")
    if mjcf_type == "slide":
        return "prismatic"
    if mjcf_type == "hinge" and source.get("range") is None:
        return "continuous"
    if mjcf_type == "hinge":
        return "revolute"
    raise ValueError(f"Unsupported MJCF joint type: {mjcf_type}")


def _add_joint(
    robot: ET.Element,
    source: ET.Element,
    *,
    parent_link: str,
    child_link: str,
    origin_xyz: str | None,
    origin_quat: str | None,
) -> None:
    name = source.get("name")
    joint_type = _joint_type(source)
    joint = ET.SubElement(robot, "joint", name=name, type=joint_type)
    _origin(joint, xyz=origin_xyz, quat=origin_quat)
    ET.SubElement(joint, "parent", link=parent_link)
    ET.SubElement(joint, "child", link=child_link)
    ET.SubElement(joint, "axis", xyz=source.get("axis", "0 0 1"))
    if joint_type != "continuous":
        limits = _numbers(source.get("range"), (-1000.0, 1000.0))
        effort_range = _numbers(source.get("actuatorfrcrange"), (-1000.0, 1000.0))
        ET.SubElement(
            joint,
            "limit",
            lower=f"{limits[0]:.12g}",
            upper=f"{limits[1]:.12g}",
            effort=f"{max(abs(value) for value in effort_range):.12g}",
            velocity=f"{_joint_velocity(name):.12g}",
        )
    else:
        effort_range = _numbers(source.get("actuatorfrcrange"), (-1000.0, 1000.0))
        ET.SubElement(
            joint,
            "limit",
            effort=f"{max(abs(value) for value in effort_range):.12g}",
            velocity=f"{_joint_velocity(name):.12g}",
        )
    ET.SubElement(joint, "dynamics", damping="0.001", friction="0.1")


def _convert_body(
    robot: ET.Element,
    source: ET.Element,
    mesh_files: dict[str, str],
    *,
    parent_link: str | None,
) -> str:
    link_name = source.get("name") or "unnamed_body"
    is_root = parent_link is None
    joints = [joint for joint in source.findall("./joint") if joint.get("type") != "free"]
    free_joint = source.find("./freejoint") is not None or any(
        joint.get("type") == "free" for joint in source.findall("./joint")
    )

    if parent_link is None and len(joints) > 1:
        anchor_name = f"{link_name}_anchor"
        anchor = ET.SubElement(robot, "link", name=anchor_name)
        _add_tiny_inertial(anchor)
        parent_link = anchor_name

    if parent_link is not None and len(joints) > 1:
        current_parent = parent_link
        for index, joint_source in enumerate(joints):
            child_name = (
                link_name
                if index == len(joints) - 1
                else f"{link_name}_{joint_source.get('name')}_link"
            )
            if child_name != link_name:
                intermediate = ET.SubElement(robot, "link", name=child_name)
                _add_tiny_inertial(intermediate)
            _add_joint(
                robot,
                joint_source,
                parent_link=current_parent,
                child_link=child_name,
                origin_xyz=(source.get("pos") if index == 0 and not is_root else None),
                origin_quat=(source.get("quat") if index == 0 and not is_root else None),
            )
            current_parent = child_name
    else:
        link = ET.SubElement(robot, "link", name=link_name)
        _add_inertial(link, source.find("./inertial"))
        _add_body_geometries(link, source, mesh_files)
        if parent_link is not None:
            if joints:
                _add_joint(
                    robot,
                    joints[0],
                    parent_link=parent_link,
                    child_link=link_name,
                    origin_xyz=source.get("pos"),
                    origin_quat=source.get("quat"),
                )
            elif not free_joint:
                fixed = ET.SubElement(
                    robot,
                    "joint",
                    name=f"{parent_link}_to_{link_name}_fixed_joint",
                    type="fixed",
                )
                _origin(fixed, xyz=source.get("pos"), quat=source.get("quat"))
                ET.SubElement(fixed, "parent", link=parent_link)
                ET.SubElement(fixed, "child", link=link_name)

    if len(joints) > 1:
        link = ET.SubElement(robot, "link", name=link_name)
        _add_inertial(link, source.find("./inertial"))
        _add_body_geometries(link, source, mesh_files)

    for child in source.findall("./body"):
        _convert_body(robot, child, mesh_files, parent_link=link_name)
    return link_name


def convert_file(source_path: Path, output_path: Path) -> None:
    mjcf = ET.parse(source_path).getroot()
    mesh_files = {
        mesh.get("name"): mesh.get("file")
        for mesh in mjcf.findall("./asset/mesh")
        if mesh.get("name") and mesh.get("file")
    }
    root_bodies = mjcf.findall("./worldbody/body")
    if len(root_bodies) != 1:
        raise ValueError(f"Expected one root body in {source_path}, found {len(root_bodies)}")

    robot = ET.Element("robot", name=source_path.stem)
    robot.append(ET.Comment(f"Generated from ../mjcf/{source_path.name}; do not edit by hand."))
    _convert_body(robot, root_bodies[0], mesh_files, parent_link=None)
    ET.indent(robot, space="  ")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(robot).write(output_path, encoding="utf-8", xml_declaration=True)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--resource-dir",
        type=Path,
        default=Path(__file__).resolve().parents[1]
        / "module"
        / "spark_robot"
        / "resources"
        / "unitree_g1",
    )
    args = parser.parse_args()
    mjcf_dir = args.resource_dir / "mjcf"
    urdf_dir = args.resource_dir / "urdf"
    for source_path in sorted(mjcf_dir.glob("*.xml")):
        output_path = urdf_dir / f"{source_path.stem}.urdf"
        convert_file(source_path, output_path)
        print(f"generated {output_path.relative_to(args.resource_dir)}")


if __name__ == "__main__":
    main()
