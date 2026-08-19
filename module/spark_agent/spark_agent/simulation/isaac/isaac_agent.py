"""Single-environment Isaac articulation agent and URDF asset support."""

from __future__ import annotations

import copy
from fnmatch import fnmatchcase
import hashlib
import json
import math
import os
from pathlib import Path
import time
from typing import Any

import numpy as np
from spark_robot import SPARK_ROBOT_RESOURCE_DIR
from spark_utils import Geometry, VizColor

from spark_agent.dynamics import DynamicsExecutor
from spark_agent.simulation.camera_display import (
    close_camera_display,
    submit_camera_display,
)
from spark_agent.simulation.isaac.viewer_info import (
    ISAAC_SIMULATION_INFO_WINDOW_TITLE,
    IsaacViewerInfoMixin,
)
from spark_agent.simulation.simulation_agent import SimulationAgent
from spark_agent.simulation.viewer_config import (
    camera_eye_target,
    normalize_sensor_camera_config,
    normalize_viewer_config,
    sensor_camera_eye_target,
)


ISAAC_COLLISION_VOLUME_OPACITY_SCALE = 1.0
ISAAC_COLLISION_VOLUME_OPACITY_FLOOR = 0.05
ISAAC_GOAL_OPACITY = 0.65


def _disable_debug_prim_shadows(prim) -> None:
    """Prevent translucent diagnostic geometry from shading robot links."""

    from pxr import Sdf

    prim.CreateAttribute("primvars:doNotCastShadows", Sdf.ValueTypeNames.Bool).Set(True)


def _look_at_quaternion_wxyz(eye, target):
    """Return a USD-camera orientation whose local -Z axis sees ``target``."""

    from scipy.spatial.transform import Rotation

    eye = np.asarray(eye, dtype=float).reshape(3)
    target = np.asarray(target, dtype=float).reshape(3)
    forward = target - eye
    forward /= max(float(np.linalg.norm(forward)), 1.0e-12)
    up = np.array([0.0, 0.0, 1.0], dtype=float)
    if abs(float(np.dot(forward, up))) > 0.999:
        up = np.array([0.0, 1.0, 0.0], dtype=float)
    x_axis = np.cross(forward, up)
    x_axis /= max(float(np.linalg.norm(x_axis)), 1.0e-12)
    z_axis = -forward
    y_axis = np.cross(z_axis, x_axis)
    rotation = np.column_stack((x_axis, y_axis, z_axis))
    quaternion_xyzw = Rotation.from_matrix(rotation).as_quat()
    return np.roll(quaternion_xyzw, 1)


def _isaac_sphere_rgba(color):
    """Keep Isaac goal spheres legible without changing shared UI colors."""

    rgba = np.asarray(color, dtype=float).reshape(-1)
    if rgba.size < 4:
        rgba = np.pad(rgba, (0, 4 - rgba.size), constant_values=1.0)
    rgba = np.clip(rgba[:4], 0.0, 1.0)
    goal = np.asarray(VizColor.goal, dtype=float)
    if rgba[3] > 0.0 and np.allclose(rgba[:3], goal[:3], rtol=0.0, atol=1.0e-7):
        rgba[3] = max(float(rgba[3]), ISAAC_GOAL_OPACITY)
    return rgba[:3], float(rgba[3])


def _quantize_debug_rgba(color):
    """Return a compact RGB palette key without crushing faint alpha values."""
    rgba = np.asarray(color, dtype=float).reshape(-1)
    if rgba.size < 4:
        rgba = np.pad(rgba, (0, 4 - rgba.size), constant_values=1.0)
    rgba = np.clip(rgba[:4], 0.0, 1.0)
    rgb_levels = 63
    alpha_levels = 255
    key = (
        *(int(round(float(channel) * rgb_levels)) for channel in rgba[:3]),
        int(round(float(rgba[3]) * alpha_levels)),
    )
    quantized = np.array(
        [
            *(channel / rgb_levels for channel in key[:3]),
            key[3] / alpha_levels,
        ],
        dtype=float,
    )
    return key, quantized


def _bind_debug_rgba_material(
    stage,
    prim,
    color,
    material_cache: dict,
    *,
    previous_key=None,
    material_root="/World/SparkDebugMaterials",
):
    """Bind an RTX-compatible, quantized RGBA material to a debug solid.

    ``displayOpacity`` is not consistently honored for analytic USD solids by
    every Isaac RTX render mode.  A small shared material palette preserves
    transparency without creating one shader per sphere in parallel scenes.
    """
    import carb
    from pxr import Gf, Sdf, UsdGeom, UsdShade

    # RaytracedLighting (legacy real-time) treats every non-zero cutout value
    # as a fully present surface unless fractional cutout opacity is enabled.
    # RT2 and path-traced modes use the pathtracing setting. Enable both so a
    # fractional debug alpha has the same meaning across Isaac render modes.
    settings = carb.settings.get_settings()
    settings.set("/rtx/raytracing/fractionalCutoutOpacity", True)
    settings.set("/rtx/pathtracing/fractionalCutoutOpacity", True)

    key, quantized = _quantize_debug_rgba(color)
    if key == previous_key:
        return key

    material = material_cache.get(key)
    if material is None:
        UsdGeom.Scope.Define(stage, material_root)
        material_path = f"{material_root}/RGBA_{key[0]}_{key[1]}_{key[2]}_{key[3]}"
        # Author the same network as CreateMdlMaterialPrim directly in USD.
        # The Kit command participates in editor command/selection state,
        # which can mutate viewport callbacks during a live keyboard update.
        material = UsdShade.Material.Define(stage, material_path)
        shader = UsdShade.Shader.Define(stage, f"{material_path}/Shader")
        shader_output = shader.CreateOutput("out", Sdf.ValueTypeNames.Token)
        shader_output.SetRenderType("material")
        material.CreateSurfaceOutput("mdl").ConnectToSource(shader_output)
        material.CreateVolumeOutput("mdl").ConnectToSource(shader_output)
        material.CreateDisplacementOutput("mdl").ConnectToSource(shader_output)
        shader.GetImplementationSourceAttr().Set(UsdShade.Tokens.sourceAsset)
        shader.SetSourceAsset(Sdf.AssetPath("OmniPBR.mdl"), "mdl")
        shader.SetSourceAssetSubIdentifier("OmniPBR", "mdl")
        if not shader.GetPrim().IsValid():
            raise RuntimeError(f"Unable to create OmniPBR debug material at {material_path}")
        material.GetPrim().CreateAttribute(
            "omni:rtx:enableCutoutOpacity",
            Sdf.ValueTypeNames.Bool,
        ).Set(True)
        shader.CreateInput("diffuse_color_constant", Sdf.ValueTypeNames.Float3).Set(
            Gf.Vec3f(*quantized[:3])
        )
        shader.CreateInput("enable_opacity", Sdf.ValueTypeNames.Bool).Set(True)
        shader.CreateInput("enable_opacity_texture", Sdf.ValueTypeNames.Bool).Set(False)
        shader.CreateInput("opacity_constant", Sdf.ValueTypeNames.Float).Set(float(quantized[3]))
        shader.CreateInput("opacity_threshold", Sdf.ValueTypeNames.Float).Set(0.0)
        shader.CreateInput("reflection_roughness_constant", Sdf.ValueTypeNames.Float).Set(1.0)
        shader.CreateInput("metallic_constant", Sdf.ValueTypeNames.Float).Set(0.0)
        material_cache[key] = material
    UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)
    return key


def fit_viewer_config_to_prim(prim_path: str, viewer_config=None) -> dict:
    """Fit the default camera to one imported robot or cloned environment root."""

    import omni.usd
    from pxr import Usd, UsdGeom

    config = normalize_viewer_config(viewer_config)
    stage = omni.usd.get_context().get_stage()
    prim = stage.GetPrimAtPath(prim_path)
    if not prim.IsValid():
        return config
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
        useExtentsHint=True,
    )
    aligned = cache.ComputeWorldBound(prim).ComputeAlignedBox()
    minimum = np.asarray(aligned.GetMin(), dtype=float)
    maximum = np.asarray(aligned.GetMax(), dtype=float)
    if not np.isfinite(minimum).all() or not np.isfinite(maximum).all():
        return config
    extent = maximum - minimum
    if np.max(extent) <= 1.0e-6:
        return config
    center = 0.5 * (minimum + maximum)
    vertical_fov = np.deg2rad(config["camera_vertical_fov"])
    distance = float(np.max(extent)) / np.tan(0.5 * vertical_fov)
    config.update(
        camera_lookat=tuple(center),
        camera_distance=max(0.75, distance),
        camera_azimuth=135.0,
        grid_extent=max(config["grid_extent"], 0.75 * float(np.max(extent[:2]))),
    )
    return config


def _resolve_urdf_mesh_paths(source: Path, urdf_tree) -> dict[str, Path]:
    """Resolve mesh declarations using ROS package and SPARK bundle conventions."""
    mesh_paths: dict[str, Path] = {}
    for mesh in sorted({element.attrib["filename"] for element in urdf_tree.findall(".//mesh")}):
        if mesh.startswith("package://"):
            package, relative = mesh.removeprefix("package://").split("/", 1)
            package_root = next(
                (parent for parent in source.parents if parent.name == package), None
            )
            mesh_path = (
                (package_root / relative).resolve()
                if package_root
                else (source.parent / package / relative).resolve()
            )
        else:
            declared_path = (source.parent / mesh).resolve()
            mesh_name = Path(mesh)
            bundled_root = source.parent / "meshes" / source.stem
            bundled_paths = (
                (bundled_root / mesh_name.name).resolve(),
                (bundled_root / f"{mesh_name.stem}_grey{mesh_name.suffix}").resolve(),
                (bundled_root / f"{mesh_name.stem}_orange{mesh_name.suffix}").resolve(),
            )
            mesh_path = next(
                (candidate for candidate in (declared_path, *bundled_paths) if candidate.is_file()),
                declared_path,
            )
        if not mesh_path.is_file():
            raise FileNotFoundError(f"URDF mesh not found: {mesh_path}")
        mesh_paths[mesh] = mesh_path
    return mesh_paths


def apply_urdf_visual_palette(
    urdf_path: str | Path,
    palette,
    *,
    cache_dir: str | Path | None = None,
) -> str:
    """Create a content-addressed URDF with inline per-link visual colors."""

    import xml.etree.ElementTree as ET

    source = Path(urdf_path).expanduser().resolve()
    records = tuple(
        (str(pattern).lower(), tuple(float(channel) for channel in rgba))
        for pattern, rgba in palette
    )
    if not records:
        return str(source)
    for pattern, rgba in records:
        if len(rgba) != 4 or not all(0.0 <= channel <= 1.0 for channel in rgba):
            raise ValueError(f"Invalid RGBA visual palette entry {pattern!r}: {rgba}")

    tree = ET.parse(source)
    mesh_paths = _resolve_urdf_mesh_paths(source, tree)
    digest = hashlib.sha256(source.read_bytes())
    digest.update(json.dumps(records, sort_keys=True).encode("utf-8"))
    cache_root = Path(
        cache_dir or os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")
    ).expanduser()
    output_dir = cache_root / "spark" / "isaac" / "palette_urdf" / digest.hexdigest()[:16]
    output_path = output_dir / f"{source.stem}_palette.urdf"
    if output_path.is_file():
        return str(output_path)

    for mesh in tree.findall(".//mesh"):
        mesh.set("filename", str(mesh_paths[mesh.attrib["filename"]]))
    for link in tree.findall("link"):
        link_name = link.attrib.get("name", "")
        color = next(
            (rgba for pattern, rgba in records if fnmatchcase(link_name.lower(), pattern)),
            None,
        )
        if color is None:
            continue
        rgba_text = " ".join(f"{channel:.6g}" for channel in color)
        for visual_index, visual in enumerate(link.findall("visual")):
            material = visual.find("material")
            if material is None:
                material = ET.SubElement(visual, "material")
            material.set("name", f"spark_{link_name}_{visual_index}")
            color_element = material.find("color")
            if color_element is None:
                color_element = ET.SubElement(material, "color")
            color_element.set("rgba", rgba_text)

    output_dir.mkdir(parents=True, exist_ok=True)
    ET.indent(tree, space="  ")
    tree.write(output_path, encoding="utf-8", xml_declaration=True)
    return str(output_path)


def compose_urdf_instances(
    urdf_path: str | Path,
    instances,
    *,
    cache_dir: str | Path | None = None,
) -> str:
    """Compose prefixed copies of one serial URDF under a fixed shared root."""
    import xml.etree.ElementTree as ET

    source = Path(urdf_path).expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(f"Isaac URDF not found: {source}")
    source_tree = ET.parse(source)
    source_root = source_tree.getroot()
    links = {link.attrib["name"] for link in source_root.findall("link")}
    child_links = {
        child.attrib["link"]
        for child in source_root.findall("joint/child")
        if "link" in child.attrib
    }
    root_links = sorted(links - child_links)
    if len(root_links) != 1:
        raise ValueError(
            f"Composite URDF source must have one root link; {source} has {root_links}"
        )

    mesh_paths = _resolve_urdf_mesh_paths(source, source_tree)
    digest = hashlib.sha256(source.read_bytes())
    for declared, mesh_path in sorted(mesh_paths.items()):
        digest.update(declared.encode("utf-8"))
        digest.update(mesh_path.read_bytes())
    instance_records = [
        {
            "prefix": instance.prefix,
            "translation": tuple(instance.translation),
            "rpy": tuple(instance.rpy),
        }
        for instance in instances
    ]
    digest.update(json.dumps(instance_records, sort_keys=True).encode("utf-8"))
    cache_root = Path(
        cache_dir or os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")
    ).expanduser()
    output_dir = cache_root / "spark" / "isaac" / "composed_urdf" / digest.hexdigest()[:16]
    output_path = output_dir / f"{source.stem}_composite.urdf"
    if output_path.is_file():
        return str(output_path)

    robot = ET.Element(
        "robot", {"name": f"{source_root.attrib.get('name', source.stem)}_composite"}
    )
    ET.SubElement(robot, "link", {"name": "world"})
    source_children = [child for child in source_root if child.tag in {"material", "link", "joint"}]
    for instance in instances:
        prefix = f"{instance.prefix}_"
        for child in source_children:
            clone = copy.deepcopy(child)
            for element in clone.iter():
                if element.tag in {"link", "joint"} and "name" in element.attrib:
                    element.set("name", prefix + element.attrib["name"])
                elif element.tag == "material" and element.attrib.get("name"):
                    element.set("name", prefix + element.attrib["name"])
                if element.tag in {"parent", "child"} and "link" in element.attrib:
                    element.set("link", prefix + element.attrib["link"])
                if element.tag == "mimic" and "joint" in element.attrib:
                    element.set("joint", prefix + element.attrib["joint"])
                if element.tag == "mesh" and "filename" in element.attrib:
                    element.set("filename", str(mesh_paths[element.attrib["filename"]]))
            robot.append(clone)

        fixed_joint = ET.SubElement(
            robot,
            "joint",
            {"name": f"world_to_{instance.prefix}_{root_links[0]}", "type": "fixed"},
        )
        ET.SubElement(
            fixed_joint,
            "origin",
            {
                "xyz": " ".join(str(value) for value in instance.translation),
                "rpy": " ".join(str(value) for value in instance.rpy),
            },
        )
        ET.SubElement(fixed_joint, "parent", {"link": "world"})
        ET.SubElement(fixed_joint, "child", {"link": prefix + root_links[0]})

    output_dir.mkdir(parents=True, exist_ok=True)
    ET.indent(robot, space="  ")
    ET.ElementTree(robot).write(output_path, encoding="utf-8", xml_declaration=True)
    return str(output_path)


def augment_urdf_planar_base(
    urdf_path: str | Path,
    planar_base,
    *,
    cache_dir: str | Path | None = None,
) -> str:
    """Add explicit global x/y/yaw joints below a URDF's root link.

    URDF has no portable three-coordinate planar joint representation across
    MuJoCo and Isaac.  The generated chain uses two prismatic joints followed
    by one yaw joint, preserving SPARK's x/y/yaw DoF contract while keeping
    the vendor/adapted source asset unchanged.
    """
    import xml.etree.ElementTree as ET

    source = Path(urdf_path).expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(f"Isaac URDF not found: {source}")
    source_tree = ET.parse(source)
    source_root = source_tree.getroot()
    links = {link.attrib["name"] for link in source_root.findall("link")}
    child_links = {
        child.attrib["link"]
        for child in source_root.findall("joint/child")
        if "link" in child.attrib
    }
    root_links = sorted(links - child_links)
    if len(root_links) != 1:
        raise ValueError(f"Planar URDF source must have one root link; {source} has {root_links}")

    generated_joint_names = tuple(planar_base.joint_names)
    existing_joint_names = {joint.attrib["name"] for joint in source_root.findall("joint")}
    conflicts = sorted(existing_joint_names.intersection(generated_joint_names))
    if conflicts:
        raise ValueError(f"Planar joint names already exist in {source}: {conflicts}")

    mesh_paths = _resolve_urdf_mesh_paths(source, source_tree)
    digest = hashlib.sha256(source.read_bytes())
    for declared, mesh_path in sorted(mesh_paths.items()):
        digest.update(declared.encode("utf-8"))
        digest.update(mesh_path.read_bytes())
    settings = {
        "version": 2,
        "joint_names": generated_joint_names,
        "translation_limit": planar_base.translation_limit,
        "effort_limit": planar_base.effort_limit,
        "linear_velocity_limit": planar_base.linear_velocity_limit,
        "angular_velocity_limit": planar_base.angular_velocity_limit,
        "damping": planar_base.damping,
    }
    digest.update(json.dumps(settings, sort_keys=True).encode("utf-8"))
    cache_root = Path(
        cache_dir or os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")
    ).expanduser()
    output_dir = cache_root / "spark" / "isaac" / "planar_urdf" / digest.hexdigest()[:16]
    output_path = output_dir / f"{source.stem}_planar.urdf"
    if output_path.is_file():
        return str(output_path)

    robot = ET.Element("robot", {"name": f"{source_root.attrib.get('name', source.stem)}_planar"})
    generated_links = (
        "spark_planar_world",
        "spark_planar_x_link",
        "spark_planar_y_link",
        "spark_planar_yaw_link",
    )
    for link_name in generated_links:
        link = ET.SubElement(robot, "link", {"name": link_name})
        inertial = ET.SubElement(link, "inertial")
        ET.SubElement(inertial, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        ET.SubElement(inertial, "mass", {"value": "0.001"})
        ET.SubElement(
            inertial,
            "inertia",
            {
                "ixx": "0.000001",
                "ixy": "0",
                "ixz": "0",
                "iyy": "0.000001",
                "iyz": "0",
                "izz": "0.000001",
            },
        )

    intermediate_links = generated_links[1:]
    parent_links = (generated_links[0], *intermediate_links[:2])
    child_planar_links = intermediate_links
    axes = ("1 0 0", "0 1 0", "0 0 1")
    joint_types = ("prismatic", "prismatic", "revolute")
    for index, (joint_name, parent, child, axis, joint_type) in enumerate(
        zip(
            generated_joint_names,
            parent_links,
            child_planar_links,
            axes,
            joint_types,
        )
    ):
        joint = ET.SubElement(robot, "joint", {"name": joint_name, "type": joint_type})
        ET.SubElement(joint, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
        ET.SubElement(joint, "parent", {"link": parent})
        ET.SubElement(joint, "child", {"link": child})
        ET.SubElement(joint, "axis", {"xyz": axis})
        limit = planar_base.translation_limit if index < 2 else math.pi * 1000.0
        velocity = (
            planar_base.linear_velocity_limit if index < 2 else planar_base.angular_velocity_limit
        )
        ET.SubElement(
            joint,
            "limit",
            {
                "lower": str(-limit),
                "upper": str(limit),
                "effort": str(planar_base.effort_limit),
                "velocity": str(velocity),
            },
        )
        ET.SubElement(joint, "dynamics", {"damping": str(planar_base.damping)})

    mount = ET.SubElement(
        robot,
        "joint",
        {"name": "spark_planar_root_mount", "type": "fixed"},
    )
    ET.SubElement(mount, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
    ET.SubElement(mount, "parent", {"link": intermediate_links[-1]})
    ET.SubElement(mount, "child", {"link": root_links[0]})

    for child in source_root:
        clone = copy.deepcopy(child)
        for element in clone.iter("mesh"):
            element.set("filename", str(mesh_paths[element.attrib["filename"]]))
        robot.append(clone)

    output_dir.mkdir(parents=True, exist_ok=True)
    ET.indent(robot, space="  ")
    ET.ElementTree(robot).write(output_path, encoding="utf-8", xml_declaration=True)
    return str(output_path)


def _resolve_usd_path(path: str) -> str:
    """Resolve SPARK's bundled Isaac smoke-test asset alias."""
    if path != "isaacsim://local/cartpole":
        return path
    import isaacsim

    return str(
        Path(isaacsim.__file__).resolve().parent
        / "exts"
        / "isaacsim.physics.newton.tensors"
        / "data"
        / "usd"
        / "CartPole.usda"
    )


def import_urdf_to_usd(
    urdf_path: str | Path,
    *,
    fix_base: bool,
    merge_fixed_joints: bool = False,
    allow_self_collision: bool = False,
    cache_dir: str | Path | None = None,
) -> str:
    """Import a URDF into a content-addressed Isaac USD cache.

    ``SimulationApp`` must already be running. The cache key includes the URDF,
    every relative mesh referenced by it, and the import settings so stale USD
    files cannot silently survive a source-asset update.
    """
    import xml.etree.ElementTree as ET

    source = Path(urdf_path).expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(f"Isaac URDF not found: {source}")

    digest = hashlib.sha256()
    digest.update(source.read_bytes())
    urdf_tree = ET.parse(source)
    mesh_paths = _resolve_urdf_mesh_paths(source, urdf_tree)
    for mesh, mesh_path in sorted(mesh_paths.items()):
        digest.update(mesh.encode("utf-8"))
        digest.update(mesh_path.read_bytes())
    settings = {
        "fix_base": bool(fix_base),
        "allow_self_collision": bool(allow_self_collision),
        "merge_fixed_joints": bool(merge_fixed_joints),
        "joint_drive_type": "force",
        "joint_target_type": "none",
    }
    digest.update(json.dumps(settings, sort_keys=True).encode("utf-8"))
    cache_root = Path(
        cache_dir or os.environ.get("XDG_CACHE_HOME", Path.home() / ".cache")
    ).expanduser()
    output_dir = cache_root / "spark" / "isaac" / source.stem / digest.hexdigest()[:16]
    manifest = output_dir / "manifest.json"
    if manifest.is_file():
        metadata = json.loads(manifest.read_text(encoding="utf-8"))
        output_path = Path(metadata["usd_path"])
        if output_path.is_file():
            return str(output_path)

    try:
        from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    except ImportError:
        # Isaac Lab's lean headless experience does not preload asset
        # importers. Enable the extension only on a cache miss.
        try:
            import omni.kit.app

            manager = omni.kit.app.get_app().get_extension_manager()
            manager.set_extension_enabled_immediate("isaacsim.asset.importer.urdf", True)
            from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
        except (ImportError, RuntimeError) as exc:
            raise RuntimeError(
                "URDF import requires the isaacsim.asset.importer.urdf extension"
            ) from exc

    output_dir.mkdir(parents=True, exist_ok=True)
    # The standalone Isaac importer has no ROS package index, and several
    # upstream robot assets use a source-layout mesh alias. Materialize a
    # cached equivalent URDF with resolved absolute paths in both cases.
    for element in urdf_tree.findall(".//mesh"):
        element.set("filename", str(mesh_paths[element.attrib["filename"]]))
    import_source = output_dir / source.name
    urdf_tree.write(import_source, encoding="utf-8", xml_declaration=True)
    config = URDFImporterConfig(
        urdf_path=str(import_source),
        usd_path=str(output_dir),
        merge_fixed_joints=merge_fixed_joints,
        allow_self_collision=allow_self_collision,
        fix_base=fix_base,
        joint_drive_type="force",
        joint_target_type="none",
        override_joint_stiffness=0.0,
        override_joint_damping=0.0,
    )
    output_path = Path(URDFImporter(config).import_urdf()).resolve()
    if not output_path.is_file():
        raise RuntimeError(f"Isaac URDF importer did not create its reported USD: {output_path}")
    manifest.write_text(
        json.dumps(
            {"source": str(source), "usd_path": str(output_path), "settings": settings},
            indent=2,
        )
        + "\n",
        encoding="utf-8",
    )
    return str(output_path)


def _quaternion_wxyz_to_matrix(quaternion) -> np.ndarray:
    w, x, y, z = np.asarray(quaternion, dtype=float)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def _rpy_to_quaternion_wxyz(rpy) -> np.ndarray:
    """Convert intrinsic roll/pitch/yaw to an Isaac-order quaternion."""

    roll, pitch, yaw = np.asarray(rpy, dtype=float)
    cr, sr = np.cos(0.5 * roll), np.sin(0.5 * roll)
    cp, sp = np.cos(0.5 * pitch), np.sin(0.5 * pitch)
    cy, sy = np.cos(0.5 * yaw), np.sin(0.5 * yaw)
    return np.array(
        [
            cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
        ],
        dtype=float,
    )


def _multiply_quaternion_wxyz(left, right) -> np.ndarray:
    """Compose two wxyz quaternions."""

    def host_array(value):
        # Isaac returns articulation poses as tensors on the simulation
        # device.  NumPy cannot consume CUDA tensors directly, so make this
        # small presentation-time conversion explicit at the API boundary.
        if hasattr(value, "detach"):
            value = value.detach()
        if hasattr(value, "cpu"):
            value = value.cpu()
        return np.asarray(value, dtype=float)

    lw, lx, ly, lz = host_array(left)
    rw, rx, ry, rz = host_array(right)
    return np.array(
        [
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        ],
        dtype=float,
    )


def compose_planar_robot_base_frame(root_frame, dof_pos, robot_cfg):
    """Resolve an imported x/y/yaw chain to the physical robot base frame."""
    frame = np.asarray(root_frame, dtype=float).reshape(4, 4).copy()
    planar_spec = getattr(getattr(robot_cfg, "isaac_articulation", None), "planar_base", None)
    members = getattr(robot_cfg.DoFs, "__members__", {})
    if planar_spec is None or not all(name in members for name in ("LinearX", "LinearY", "RotYaw")):
        return frame

    position = np.asarray(dof_pos, dtype=float).reshape(-1)
    x = float(position[int(members["LinearX"])])
    y = float(position[int(members["LinearY"])])
    yaw = float(position[int(members["RotYaw"])])
    cosine, sine = math.cos(yaw), math.sin(yaw)
    planar = np.eye(4, dtype=float)
    planar[:3, :3] = np.array(
        [[cosine, -sine, 0.0], [sine, cosine, 0.0], [0.0, 0.0, 1.0]],
        dtype=float,
    )
    planar[:2, 3] = (x, y)
    return frame @ planar


def _show_viewport_only():
    """Hide editor chrome and overlays while preserving the viewport compositor."""
    import omni.ui

    viewport = None
    layout_changed = False
    for window in omni.ui.Workspace.get_windows():
        title = str(getattr(window, "title", ""))
        is_sensor_output = "Sensors Output" in title
        if title.startswith("Viewport") and not is_sensor_output:
            viewport = window
            if not window.visible:
                window.visible = True
                layout_changed = True
            # These controls are children/layers of the viewport and are not
            # returned as independent Workspace windows.
            if getattr(window, "dock_tab_bar_enabled", False):
                window.dock_tab_bar_enabled = False
                layout_changed = True
            if getattr(window, "dock_tab_bar_visible", False):
                window.dock_tab_bar_visible = False
                layout_changed = True
            find_layer = getattr(window, "_find_viewport_layer", None)
            if find_layer is not None:
                menubar = find_layer("Menubar", "menubar")
                if menubar is not None and getattr(menubar, "visible", False):
                    menubar.visible = False
                    layout_changed = True
        elif title == ISAAC_SIMULATION_INFO_WINDOW_TITLE:
            if not window.visible:
                window.visible = True
                layout_changed = True
        elif title != "DockSpace" and window.visible:
            window.visible = False
            layout_changed = True
    # Do not force focus while Kit is dispatching mouse events. Re-focusing a
    # docked viewport from inside the UI update callback can invalidate the
    # active mouse target when the user clicks during startup. The native
    # viewport receives focus naturally on the first click.


class IsaacAgent(IsaacViewerInfoMixin, SimulationAgent):
    """Expose one Isaac Sim articulation through SPARK's agent contract.

    A ``SimulationApp`` must be created before constructing this agent. The
    agent owns its ``SimulationContext`` and articulation, while the outer
    ``SparkEnvironment`` continues to own reset/step/task orchestration exactly
    as it does for MuJoCo agents.
    """

    def __init__(
        self,
        robot_cfg,
        *,
        usd_path: str,
        prim_path: str = "/World/Robot",
        joint_names: list[str] | tuple[str, ...] | None = None,
        control_mode: str = "effort",
        dt: float = 0.01,
        device: str = "cuda:0",
        add_ground_plane: bool = True,
        ground_static_friction: float = 0.5,
        ground_dynamic_friction: float = 0.5,
        ground_restitution: float = 0.8,
        solver_position_iteration_count: int | None = None,
        solver_velocity_iteration_count: int | None = None,
        enabled_self_collisions: bool | None = None,
        render: bool = False,
        enable_viewer: bool | None = None,
        obstacle_debug: dict | None = None,
        enable_keyboard_control: bool = False,
        render_on_step: bool = True,
        viewer_hz: float = 0.0,
        scene_style: str = "mujoco",
        match_mujoco_robot_materials: bool = True,
        viewer_config: dict | None = None,
        camera_eye=None,
        camera_target=None,
        camera_vertical_fov=None,
        enable_contact_visualization: bool = False,
        contact_force_scale: float = 5.0e-4,
        contact_torque_scale: float = 5.0e-3,
        contact_force_threshold: float = 1.0,
        show_contact_forces: bool = False,
        show_contact_wrench: bool = False,
        enable_camera: bool = False,
        camera_width: int = 1280,
        camera_height: int = 720,
        camera_rate_hz: float = 30.0,
        camera_display: bool = False,
        camera_config=None,
        **kwargs,
    ) -> None:
        super().__init__(robot_cfg)
        if control_mode not in {"effort", "position", "velocity"}:
            raise ValueError("control_mode must be effort, position, or velocity")

        try:
            import torch
            import omni.kit.app
            from isaacsim.core.api import World
            from isaacsim.core.prims import SingleArticulation
            from isaacsim.core.utils.stage import add_reference_to_stage
        except ImportError as exc:
            raise RuntimeError("IsaacAgent requires the SPARK Isaac installation profile") from exc

        self._torch = torch
        self.dt = float(dt)
        self.device = str(device)
        self.control_mode = control_mode
        self.render_enabled = bool(render if enable_viewer is None else enable_viewer)
        self._initialize_viewer_simulation_info(**kwargs)
        self.render_on_step = bool(render_on_step)
        self.viewer_hz = float(viewer_hz)
        self._render_period = 0.0 if self.viewer_hz <= 0.0 else 1.0 / self.viewer_hz
        self._next_render_time = 0.0
        # Kit starts in its editor layout.  Re-apply the presentation layout
        # for the first few lazy UI updates so Stage/Property sidebars cannot
        # reclaim space after the viewport is initialized.
        self._viewport_layout_updates_remaining = int(
            kwargs.get("viewport_layout_updates", 6 if self.render_enabled else 0)
        )
        self.enable_keyboard_control = bool(enable_keyboard_control)
        self.enable_contact_visualization = bool(enable_contact_visualization)
        self.show_contact_forces = bool(show_contact_forces)
        self.show_contact_wrench = bool(show_contact_wrench)
        self.render_robot_collision_volumes = bool(
            kwargs.get("render_robot_collision_volumes", True)
        )
        self.collision_volume_opacity_scale = float(
            kwargs.get(
                "collision_volume_opacity_scale",
                ISAAC_COLLISION_VOLUME_OPACITY_SCALE,
            )
        )
        if not 0.0 <= self.collision_volume_opacity_scale <= 1.0:
            raise ValueError("collision_volume_opacity_scale must be between 0 and 1")
        self.collision_volume_opacity_floor = float(
            kwargs.get(
                "collision_volume_opacity_floor",
                ISAAC_COLLISION_VOLUME_OPACITY_FLOOR,
            )
        )
        if not 0.0 <= self.collision_volume_opacity_floor <= 1.0:
            raise ValueError("collision_volume_opacity_floor must be between 0 and 1")
        self.contact_force_scale = float(contact_force_scale)
        self.contact_torque_scale = float(contact_torque_scale)
        self.contact_force_threshold = float(contact_force_threshold)
        self.viewer_config = normalize_viewer_config(viewer_config)
        self.enable_camera = bool(enable_camera)
        self.camera_width = int(camera_width)
        self.camera_height = int(camera_height)
        self.camera_rate_hz = float(camera_rate_hz)
        self.camera_display = bool(camera_display)
        self.camera_config = normalize_sensor_camera_config(camera_config, self.viewer_config)
        self.camera_feedback = {}
        self._sensor_cameras = {}
        self._camera_display_process = None
        self._camera_ready_reported = False
        self._last_camera_capture_time = -np.inf
        self._contact_view = None
        self._contact_body_names: list[str] = []
        self._contact_warning_emitted = False
        configured_eye, configured_target = camera_eye_target(self.viewer_config)
        camera_eye = configured_eye if camera_eye is None else np.asarray(camera_eye, dtype=float)
        camera_target = (
            configured_target if camera_target is None else np.asarray(camera_target, dtype=float)
        )
        camera_vertical_fov = (
            self.viewer_config["camera_vertical_fov"]
            if camera_vertical_fov is None
            else float(camera_vertical_fov)
        )
        self.last_control = np.zeros(self.num_control, dtype=float)
        self.step_index = 0
        self.time = 0.0
        self._simulation_app = None
        self._closed = False
        self._viewport_interaction_guards = []

        # Direct tensor writes do not author every link transform back into
        # USD. Fabric is the bridge used by the viewport for those live PhysX
        # transforms, so it must be active before World creates its physics
        # and rendering contexts. Without it, SPARK's state and collision
        # volumes move while the imported visual mesh remains at its USD pose.
        extension_manager = omni.kit.app.get_app().get_extension_manager()
        if not extension_manager.is_extension_enabled("omni.physx.fabric"):
            extension_manager.set_extension_enabled_immediate("omni.physx.fabric", True)

        self.sim = World(
            physics_dt=self.dt,
            rendering_dt=self.dt,
            backend="torch",
            device=self.device,
        )
        if add_ground_plane:
            self.sim.scene.add_default_ground_plane(
                static_friction=float(ground_static_friction),
                dynamic_friction=float(ground_dynamic_friction),
                restitution=float(ground_restitution),
            )
        resolved_usd_path = _resolve_usd_path(usd_path)
        if not resolved_usd_path.startswith(("omniverse://", "http://", "https://")):
            if not Path(resolved_usd_path).is_file():
                raise FileNotFoundError(f"Isaac articulation USD not found: {resolved_usd_path}")
        add_reference_to_stage(usd_path=resolved_usd_path, prim_path=prim_path)
        if scene_style == "mujoco" and match_mujoco_robot_materials:
            self._apply_mujoco_robot_material_style(prim_path)
        if (
            solver_position_iteration_count is not None
            or solver_velocity_iteration_count is not None
            or enabled_self_collisions is not None
        ):
            import omni.usd
            from pxr import PhysxSchema, UsdPhysics

            stage = omni.usd.get_context().get_stage()
            articulation_root = next(
                (
                    prim
                    for prim in stage.Traverse()
                    if str(prim.GetPath()).startswith(f"{prim_path}/")
                    and prim.HasAPI(UsdPhysics.ArticulationRootAPI)
                ),
                None,
            )
            if articulation_root is None:
                raise RuntimeError(f"No articulation root found below imported asset {prim_path}")
            physx_articulation = PhysxSchema.PhysxArticulationAPI.Apply(articulation_root)
            if solver_position_iteration_count is not None:
                physx_articulation.CreateSolverPositionIterationCountAttr().Set(
                    int(solver_position_iteration_count)
                )
            if solver_velocity_iteration_count is not None:
                physx_articulation.CreateSolverVelocityIterationCountAttr().Set(
                    int(solver_velocity_iteration_count)
                )
            if enabled_self_collisions is not None:
                physx_articulation.CreateEnabledSelfCollisionsAttr().Set(
                    bool(enabled_self_collisions)
                )
        self.articulation = self.sim.scene.add(
            SingleArticulation(prim_path=prim_path, name="spark_robot")
        )
        if self.enable_contact_visualization:
            # Contact-report APIs must be authored before reset; creating the
            # sensor after PhysX initialization leaves its native handle
            # invalid. The view is intentionally unfiltered so F/C report all
            # external contacts, not only contacts with the ground plane.
            import omni.usd
            from isaacsim.core.api.sensors import RigidContactView
            from pxr import UsdPhysics

            stage = omni.usd.get_context().get_stage()
            robot_prefix = f"{prim_path}/"
            contact_prim_paths = [
                str(prim.GetPath())
                for prim in stage.Traverse()
                if (
                    str(prim.GetPath()) == prim_path or str(prim.GetPath()).startswith(robot_prefix)
                )
                and prim.HasAPI(UsdPhysics.RigidBodyAPI)
            ]
            if contact_prim_paths:
                self._contact_body_names = [path.rsplit("/", 1)[-1] for path in contact_prim_paths]
                contact_view = RigidContactView(
                    prim_paths_expr=contact_prim_paths,
                    filter_paths_expr=[[] for _ in contact_prim_paths],
                    name="spark_robot_contacts",
                    prepare_contact_sensors=True,
                    max_contact_count=512,
                )
                # Isaac Sim 6's compatibility wrapper lacks the ``name``,
                # ``is_valid``, and ``post_reset`` methods expected by
                # SceneRegistry. Keep it outside the registry and initialize
                # it against World's authoritative tensor view after reset.
                self._contact_view = contact_view
            else:
                print(
                    f"[contact] no rigid bodies found below {prim_path}; "
                    "F/C visualization is unavailable"
                )
        self.sim.reset()
        if self._contact_view is not None:
            self._contact_view.initialize(self.sim.physics_sim_view)
        self._hide_imported_guide_geometry(prim_path)
        if viewer_config is None:
            self.viewer_config = fit_viewer_config_to_prim(prim_path, self.viewer_config)
            camera_eye, camera_target = camera_eye_target(self.viewer_config)
            camera_vertical_fov = self.viewer_config["camera_vertical_fov"]
        if self.enable_camera:
            self._initialize_sensor_cameras(prim_path)

        # Pipeline-compatible visualization state. Rendering requests are
        # collected during pipeline post-processing and flushed together.
        self._debug_primitives: list[dict[str, Any]] = []
        self._renderer_process = None
        self.renderer_process_status = None
        self._debug_prim_count = 0
        self._debug_solid_cache = {"sphere": [], "box": []}
        self._debug_root = f"{prim_path.rsplit('/', 1)[0]}/SparkDebug"
        debug_cfg = dict(obstacle_debug or {})
        self.num_obstacle_debug = int(debug_cfg.get("num_obstacle", 0))
        if self.num_obstacle_debug:
            self.obstacle_debug_frame = np.repeat(
                np.eye(4, dtype=float)[None, :, :], self.num_obstacle_debug, axis=0
            )
            for frame in self.obstacle_debug_frame:
                frame[:3, 3] = np.array([0.6, 0.0, 0.793]) + np.random.uniform(-0.2, 0.2, 3)
        else:
            self.obstacle_debug_frame = np.empty((0, 4, 4), dtype=float)
        self.obstacle_debug_geom = [
            Geometry(type="sphere", radius=0.05, color=VizColor.obstacle_debug)
            for _ in range(self.num_obstacle_debug)
        ]
        self.obstacle_debug_velocity = np.zeros((self.num_obstacle_debug, 6), dtype=float)
        self.obstacle_debug_selected = 0 if self.num_obstacle_debug else None
        self.left_goal_debug_frame = np.eye(4, dtype=float)
        self.right_goal_debug_frame = np.eye(4, dtype=float)
        self.base_goal_debug_frame = np.eye(4, dtype=float)
        self.debug_object = self.base_goal_debug_frame
        self.translate_in_local_frame = False
        self.manual_step_size = float(debug_cfg.get("manual_movement_step_size", 0.02))
        self.manual_step_size_min = float(debug_cfg.get("manual_movement_step_size_min", 0.001))
        self.manual_step_size_max = float(debug_cfg.get("manual_movement_step_size_max", 0.5))
        self.manual_step_size_delta = float(debug_cfg.get("manual_movement_step_size_delta", 0.01))
        self.left_gripper_goal = False
        self.right_gripper_goal = False
        self.left_gripper_debug_state = False
        self.right_gripper_debug_state = False
        self._keyboard_gripper_goal_override = {}
        self._keyboard_input = None
        self._keyboard = None
        self._keyboard_subscription = None
        if scene_style == "mujoco":
            self._apply_mujoco_scene_style(
                camera_eye,
                camera_target,
                vertical_fov=camera_vertical_fov,
                viewer_config=self.viewer_config,
            )
        self._disable_viewport_editing()
        if self.enable_keyboard_control:
            self._subscribe_keyboard()

        requested_names = list(joint_names or [dof.name for dof in self.robot_cfg.DoFs])
        available_names = list(self.articulation.dof_names)
        missing_names = [name for name in requested_names if name not in available_names]
        if missing_names:
            raise ValueError(
                f"Isaac articulation is missing joints {missing_names}; available joints: {available_names}"
            )
        # ``get_dof_index`` on imported Isaac 6 articulations can expose the
        # USD/PhysX link offset (returning 1..N for an N-DoF robot). Tensor
        # joint APIs are always zero-based in ``dof_names`` order.
        dof_index_by_name = {name: index for index, name in enumerate(self.articulation.dof_names)}
        joint_indices = [dof_index_by_name[name] for name in requested_names]
        self.joint_names = tuple(requested_names)
        self.joint_indices = torch.as_tensor(joint_indices, device=self.device, dtype=torch.long)
        self._refresh_feedback()

    def reset(self, agent_reset_info=None, **kwargs) -> None:
        super().reset()
        reset_info = dict(agent_reset_info or {})
        joint_pos = np.array(
            [self.robot_cfg.DefaultDoFVal[dof] for dof in self.robot_cfg.DoFs], dtype=float
        )
        joint_vel = np.zeros(self.num_dof, dtype=float)
        requested_state = reset_info.get("state", reset_info.get("reset_state"))
        if requested_state is not None:
            selected = np.asarray(requested_state, dtype=float).reshape(self.num_dof)
            joint_pos = selected
        joint_pos_tensor = self._torch.as_tensor(
            joint_pos, device=self.device, dtype=self._torch.float32
        )
        joint_vel_tensor = self._torch.as_tensor(
            joint_vel, device=self.device, dtype=self._torch.float32
        )
        self.articulation.set_joint_positions(joint_pos_tensor, joint_indices=self.joint_indices)
        self.articulation.set_joint_velocities(joint_vel_tensor, joint_indices=self.joint_indices)
        self.last_control = np.zeros(self.num_control, dtype=float)
        self.step_index = 0
        self.time = float(reset_info.get("time", 0.0))
        self.sim.step(render=False)
        # The commit step initializes PhysX/Fabric but also applies one gravity
        # step.  Reassert the requested pose and zero velocity so every episode
        # begins from the declared reset state rather than a backend-dependent
        # settling impulse.
        self.articulation.set_joint_positions(joint_pos_tensor, joint_indices=self.joint_indices)
        self.articulation.set_joint_velocities(joint_vel_tensor, joint_indices=self.joint_indices)
        self._refresh_feedback()

    def _send_control_sim_dynamics(self, command: np.ndarray, **kwargs) -> None:
        command = np.asarray(command, dtype=float).reshape(self.num_control)
        if command.size != self.joint_indices.numel():
            raise ValueError(
                "The initial Isaac backend requires one control value per selected articulation joint"
            )
        if self.control_mode == "effort":
            target = self._torch.as_tensor(command, device=self.device, dtype=self._torch.float32)
            self.articulation.set_joint_efforts(target, joint_indices=self.joint_indices)
        elif self.control_mode == "position":
            from isaacsim.core.utils.types import ArticulationAction

            self.articulation.apply_action(
                ArticulationAction(
                    joint_positions=self._torch.as_tensor(
                        command, device=self.device, dtype=self._torch.float32
                    ),
                    joint_indices=self.joint_indices,
                )
            )
        else:
            from isaacsim.core.utils.types import ArticulationAction

            self.articulation.apply_action(
                ArticulationAction(
                    joint_velocities=self._torch.as_tensor(
                        command, device=self.device, dtype=self._torch.float32
                    ),
                    joint_indices=self.joint_indices,
                )
            )
        self.last_control = command.copy()

    def _send_control_modeled_dynamics(self, command: np.ndarray, **kwargs) -> None:
        raise NotImplementedError("IsaacAgent currently uses Isaac simulation dynamics")

    def _post_control_processing(self, **kwargs) -> None:
        render_frame = self.render_enabled and self.render_on_step
        if render_frame:
            self._update_simulation_info_overlay()
        self.sim.step(render=render_frame)
        self.step_index += 1
        self.time += self.dt
        self._refresh_feedback()
        if self.enable_camera and self.render_enabled and self.render_on_step:
            self._capture_camera_feedback()

    def _refresh_feedback(self) -> None:
        joint_pos = self.articulation.get_joint_positions(joint_indices=self.joint_indices)
        joint_vel = self.articulation.get_joint_velocities(joint_indices=self.joint_indices)
        self.dof_pos_fbk = joint_pos.detach().cpu().numpy().astype(float, copy=True)
        self.dof_vel_fbk = joint_vel.detach().cpu().numpy().astype(float, copy=True)
        self.dof_pos_cmd = self.dof_pos_fbk.copy() if self.dof_pos_cmd is None else self.dof_pos_cmd
        self.dof_vel_cmd = self.dof_vel_fbk.copy() if self.dof_vel_cmd is None else self.dof_vel_cmd
        self.dof_acc_cmd = np.zeros(self.num_dof, dtype=float)

    def get_feedback(self):
        self._poll_renderer_input()
        self._refresh_feedback()
        # Query the live articulation tensor rather than the authored wrapper
        # Xform.  For floating bases the wrapper can remain at the spawn pose
        # while PhysX has already advanced the root.
        root_position, root_orientation = self.articulation._articulation_view.get_world_poses()
        root_position = root_position[0]
        root_orientation = root_orientation[0]
        robot_base_frame = np.eye(4, dtype=float)
        if hasattr(root_position, "detach"):
            root_position = root_position.detach().cpu().numpy()
        if hasattr(root_orientation, "detach"):
            root_orientation = root_orientation.detach().cpu().numpy()
        robot_base_frame[:3, :3] = _quaternion_wxyz_to_matrix(root_orientation)
        robot_base_frame[:3, 3] = np.asarray(root_position, dtype=float)
        robot_base_frame = compose_planar_robot_base_frame(
            robot_base_frame, self.dof_pos_fbk, self.robot_cfg
        )
        return {
            "state": self.robot_cfg.compose_state_from_dof(self.dof_pos_fbk, self.dof_vel_fbk),
            "time": self.time,
            "step_index": self.step_index,
            "robot_base_frame": robot_base_frame,
            "dof_pos_fbk": self.dof_pos_fbk.copy(),
            "dof_vel_fbk": self.dof_vel_fbk.copy(),
            "dof_pos_cmd": self.dof_pos_cmd.copy(),
            "dof_vel_cmd": self.dof_vel_cmd.copy(),
            "dof_acc_cmd": self.dof_acc_cmd.copy(),
            "control": self.last_control.copy(),
            "obstacle_debug_frame": self.obstacle_debug_frame.copy(),
            "obstacle_debug_geom": self.obstacle_debug_geom,
            "obstacle_debug_velocity": np.asarray(self.obstacle_debug_velocity, dtype=float),
            "robot_goal_left_offset": self.left_goal_debug_frame.copy(),
            "robot_goal_right_offset": self.right_goal_debug_frame.copy(),
            "robot_goal_base_offset": self.base_goal_debug_frame.copy(),
            "left_gripper_goal": self.left_gripper_goal,
            "right_gripper_goal": self.right_gripper_goal,
            "left_gripper_debug_state": self.left_gripper_debug_state,
            "right_gripper_debug_state": self.right_gripper_debug_state,
            "camera_feedback": self.get_camera_feedback(),
        }

    def _initialize_sensor_cameras(self, robot_prim_path: str) -> None:
        """Create link-mounted or world overview RGB-D cameras."""
        import omni.usd
        from isaacsim.sensors.camera import Camera
        from scipy.spatial.transform import Rotation

        stage = omni.usd.get_context().get_stage()
        robot_prefix = f"{robot_prim_path}/"
        body_paths = {
            prim.GetName(): str(prim.GetPath())
            for prim in stage.Traverse()
            if str(prim.GetPath()).startswith(robot_prefix)
        }
        for name, raw_config in self.camera_config.items():
            config = dict(raw_config)
            camera_type = str(config.get("type", "free")).lower()
            body_name = config.get("body_name")
            body_path = body_paths.get(str(body_name)) if body_name else None
            if camera_type == "fixed" and body_path is None:
                print(f"[camera] Isaac body {body_name!r} not found; skipping {name!r}")
                continue
            safe_name = "".join(character if character.isalnum() else "_" for character in name)
            camera_path = (
                f"{body_path}/SparkCamera_{safe_name}"
                if body_path is not None
                else f"/World/SparkCamera_{safe_name}"
            )
            camera = Camera(
                prim_path=camera_path,
                # The isolated WBT renderer presents directly-written robot
                # states without advancing simulation time. Acquire each Kit
                # frame and enforce camera_rate_hz with a wall-clock throttle
                # below so the same sensor works in both execution modes.
                frequency=-1,
                resolution=(self.camera_width, self.camera_height),
            )
            camera.initialize()
            if body_path is None:
                eye, target = sensor_camera_eye_target(config)
                camera.set_world_pose(
                    position=eye,
                    orientation=_look_at_quaternion_wxyz(eye, target),
                    camera_axes="usd",
                )
            else:
                position = np.asarray(config.get("pos", [0.0, 0.0, 0.0]), dtype=float)
                xyaxes = np.asarray(config.get("xyaxes", [1, 0, 0, 0, 1, 0]), dtype=float)
                x_axis, y_axis = xyaxes[:3], xyaxes[3:6]
                x_axis /= max(np.linalg.norm(x_axis), 1.0e-12)
                y_axis /= max(np.linalg.norm(y_axis), 1.0e-12)
                z_axis = np.cross(x_axis, y_axis)
                rotation = np.column_stack((x_axis, y_axis, z_axis))
                quat_xyzw = Rotation.from_matrix(rotation).as_quat()
                quat_wxyz = np.roll(quat_xyzw, 1)
                # MuJoCo's xyaxes specifies local camera +X/+Y, with viewing
                # along -Z. That is exactly USD's camera convention.
                camera.set_local_pose(
                    translation=position,
                    orientation=quat_wxyz,
                    camera_axes="usd",
                )
            fovy = float(config.get("fovy", 58.0))
            vertical_aperture = float(camera.get_vertical_aperture())
            camera.set_focal_length(vertical_aperture / (2.0 * np.tan(0.5 * np.deg2rad(fovy))))
            clipping_range = config.get("clipping_range", (0.03, 10.0))
            camera.set_clipping_range(float(clipping_range[0]), float(clipping_range[1]))
            if config.get("publish_depth", True):
                camera.add_distance_to_image_plane_to_frame()
            self._sensor_cameras[name] = (camera, config)

    def _capture_camera_feedback(self) -> None:
        if not self._sensor_cameras:
            return
        now = time.perf_counter()
        period = 0.0 if self.camera_rate_hz <= 0 else 1.0 / self.camera_rate_hz
        if now - self._last_camera_capture_time < period:
            return
        feedback = {}
        for name, (camera, config) in self._sensor_cameras.items():
            rgba = camera.get_rgba()
            if rgba is None:
                continue
            rgb = np.asarray(rgba)[..., :3].copy()
            frame = camera.get_current_frame()
            depth = frame.get("distance_to_image_plane")
            feedback[name] = {
                "rgb": rgb,
                "depth": None if depth is None else np.asarray(depth).copy(),
                "frame_id": config.get("frame_id", name),
            }
        self.camera_feedback = feedback
        if self.camera_display and feedback:
            if not submit_camera_display(self, feedback):
                self.camera_display = False
                print("[camera] OpenCV display process unavailable; disabling camera display")
        if feedback and not self._camera_ready_reported:
            shapes = ", ".join(
                f"{name}: rgb={data['rgb'].shape}, "
                f"depth={None if data['depth'] is None else data['depth'].shape}"
                for name, data in feedback.items()
            )
            print(f"[camera] Isaac RGB-D stream ready ({shapes})")
            self._camera_ready_reported = True
        self._last_camera_capture_time = now

    def get_camera_feedback(self):
        return {
            name: {
                key: value.copy() if isinstance(value, np.ndarray) else value
                for key, value in data.items()
            }
            for name, data in self.camera_feedback.items()
        }

    def _compose_cmd_state(self):
        return self.robot_cfg.compose_state_from_dof(self.dof_pos_cmd, self.dof_vel_cmd)

    def close(self) -> None:
        """Release references owned by this agent; the caller owns SimulationApp."""
        if self._closed:
            return
        self._closed = True
        self._close_simulation_info_overlay()
        close_camera_display(self)
        self._close_renderer_process()
        if self._keyboard_subscription is not None:
            try:
                self._keyboard_input.unsubscribe_to_keyboard_events(
                    self._keyboard, self._keyboard_subscription
                )
            except (AttributeError, RuntimeError):
                # The native window can destroy its input interface before
                # Python receives the final application-running update.
                pass
            self._keyboard_subscription = None
        simulation_app = self._simulation_app
        if self.sim is not None and (simulation_app is None or simulation_app.is_running()):
            try:
                self.sim.stop()
            except (AttributeError, RuntimeError):
                pass
            try:
                # ``World.clear_instance`` releases callbacks and singleton
                # state, but intentionally leaves authored stage prims in
                # place. Clear the owned scene first so another articulation
                # can be constructed in the same Kit process without stale
                # /World/Robot prims or duplicate scene-object names.
                self.sim.clear()
            except (AttributeError, RuntimeError):
                pass
        # World is a process-wide singleton. Explicitly clear its callbacks
        # and scene references before SimulationApp tears down Kit; otherwise
        # repeated launches can retain native subscriptions until process exit.
        try:
            from isaacsim.core.api import World

            if World.instance() is self.sim:
                World.clear_instance()
        except (ImportError, AttributeError, RuntimeError):
            pass
        self._contact_view = None
        self.articulation = None
        self.sim = None
        self._simulation_app = None

    def close_viewer(self) -> None:
        """Match the MuJoCo pipeline lifecycle; SimulationApp owns the window."""
        self.render_enabled = False
        self._close_simulation_info_overlay()
        close_camera_display(self)
        self._close_renderer_process()

    def attach_simulation_app(self, simulation_app) -> None:
        """Attach the Kit lifetime owner used by same-process Isaac dynamics."""
        self._simulation_app = simulation_app

    def is_running(self) -> bool:
        """Report native viewer/application closure to the pipeline loop."""
        if self._renderer_process is not None:
            return bool(self._renderer_process.is_alive)
        if self._simulation_app is not None:
            return bool(self._simulation_app.is_running())
        return True

    def attach_renderer_process(self, renderer_process) -> None:
        """Route visualization and keyboard teleoperation through a renderer child."""
        self._renderer_process = renderer_process
        self._poll_renderer_input()

    def _close_renderer_process(self):
        if self._renderer_process is None:
            return None
        renderer = self._renderer_process
        self._renderer_process = None
        self.renderer_process_status = renderer.close(force=True)
        return self.renderer_process_status

    def _poll_renderer_input(self):
        if self._renderer_process is None:
            return
        state = self._renderer_process.poll_input()
        if state is None:
            return
        self.obstacle_debug_frame = np.asarray(state["obstacle_debug_frame"], dtype=float).copy()
        self.obstacle_debug_geom = list(state["obstacle_debug_geom"])
        self.num_obstacle_debug = len(self.obstacle_debug_geom)
        self.obstacle_debug_velocity = np.zeros((self.num_obstacle_debug, 6), dtype=float)
        self.left_goal_debug_frame = np.asarray(state["robot_goal_left_offset"], dtype=float).copy()
        self.right_goal_debug_frame = np.asarray(
            state["robot_goal_right_offset"], dtype=float
        ).copy()
        self.base_goal_debug_frame = np.asarray(state["robot_goal_base_offset"], dtype=float).copy()
        previous_gripper_goals = {
            side: bool(getattr(self, f"{side}_gripper_goal")) for side in ("left", "right")
        }
        for name in (
            "left_gripper_goal",
            "right_gripper_goal",
            "left_gripper_debug_state",
            "right_gripper_debug_state",
            "show_contact_forces",
            "show_contact_wrench",
            "render_robot_collision_volumes",
        ):
            setattr(self, name, bool(state[name]))
        for side in ("left", "right"):
            new_goal = bool(getattr(self, f"{side}_gripper_goal"))
            if new_goal != previous_gripper_goals[side]:
                # The task/action packet trails keyboard feedback by one
                # control cycle. Retain the viewer command until that packet
                # acknowledges the same value, matching MuJoCo's semantics.
                self._keyboard_gripper_goal_override[side] = new_goal

    # ----------------------------- Scene and keyboard ----------------------------

    def _disable_viewport_editing(self) -> None:
        """Keep camera navigation while preventing editor selection/manipulation.

        SPARK viewers are simulation displays, not USD editors. In Isaac Sim,
        selecting the ground while debug materials are updated can interleave a
        ``SelectPrimsCommand`` with stage edits and crash Kit. NVIDIA's viewport
        utility disables picking/context menus without disabling camera orbit,
        pan, or zoom.
        """

        if not self.render_enabled or self._viewport_interaction_guards:
            return
        try:
            import omni.usd
            from omni.kit.viewport.utility import (
                disable_context_menu,
                disable_selection,
                get_active_viewport_window,
            )

            viewport_window = get_active_viewport_window()
            if viewport_window is None:
                return
            omni.usd.get_context().get_selection().set_selected_prim_paths([], False)
            self._viewport_interaction_guards = [
                disable_selection(viewport_window),
                disable_context_menu(viewport_window),
            ]
        except (AttributeError, ImportError, RuntimeError):
            # Headless and minimal Kit experiences may not provide a viewport
            # selection layer. Rendering remains valid in those configurations.
            self._viewport_interaction_guards = []

    @staticmethod
    def _hide_imported_guide_geometry(prim_path: str) -> None:
        """Hide URDF collision guides without disabling their physics APIs.

        Isaac's URDF converter authors primitive collision shapes (notably the
        four foot spheres) with USD purpose ``guide``. Some Kit viewport
        presets render guides, which made the Isaac robot appear to have extra
        SPARK collision volumes even though both pipelines supplied the same
        ``RobotConfig.CollisionVol`` set.
        """
        import omni.usd
        from pxr import UsdGeom

        stage = omni.usd.get_context().get_stage()
        prefix = f"{prim_path}/"
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            if path != prim_path and not path.startswith(prefix):
                continue
            imageable = UsdGeom.Imageable(prim)
            if not imageable:
                continue
            if imageable.ComputePurpose() == UsdGeom.Tokens.guide:
                imageable.MakeInvisible()

    @staticmethod
    def _apply_mujoco_robot_material_style(prim_path: str, *, roughness: float = 0.78) -> None:
        """Remove metallic PBR response without changing a robot's link colors.

        Unitree's qualified Isaac USD carries the correct articulation and
        contact realization for WBT, but its metallic shader makes the same
        gray meshes look unlike SPARK's MuJoCo asset.  Authoring local shader
        overrides preserves the qualified plant while matching MuJoCo's
        diffuse, nonmetallic presentation.
        """
        import omni.usd
        from pxr import UsdShade

        stage = omni.usd.get_context().get_stage()
        prefix = f"{prim_path}/"
        metallic_names = {"metallic", "metallic_constant", "metalness"}
        roughness_names = {"roughness", "reflection_roughness_constant"}
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            if path != prim_path and not path.startswith(prefix):
                continue
            if not prim.IsA(UsdShade.Shader):
                continue
            shader = UsdShade.Shader(prim)
            for shader_input in shader.GetInputs():
                name = shader_input.GetBaseName()
                if name in metallic_names:
                    shader_input.Set(0.0)
                elif name in roughness_names:
                    shader_input.Set(float(roughness))

    def _apply_mujoco_scene_style(
        self, camera_eye, camera_target, *, vertical_fov=45.0, viewer_config=None
    ):
        """Match SPARK's MuJoCo floor, lighting, and free-camera pose."""
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdShade
        import carb
        import omni.kit.commands
        import omni.usd

        viewer_config = normalize_viewer_config(viewer_config)
        stage = omni.usd.get_context().get_stage()
        ground_color = Gf.Vec3f(*viewer_config["ground_color"])
        for prim in stage.Traverse():
            if str(prim.GetPath()).startswith("/World/defaultGroundPlane") and prim.IsA(
                UsdGeom.Gprim
            ):
                UsdGeom.Gprim(prim).CreateDisplayColorAttr([ground_color])
                material = UsdShade.Material.Define(stage, "/World/SparkEnvironment/GroundMaterial")
                shader = UsdShade.Shader.Define(
                    stage, "/World/SparkEnvironment/GroundMaterial/PreviewSurface"
                )
                shader.CreateIdAttr("UsdPreviewSurface")
                shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(ground_color)
                shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.78)
                shader.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
                material.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
                UsdShade.MaterialBindingAPI.Apply(prim).Bind(material)
        # Do not depend on Isaac's editor grid: it can be toggled by viewport
        # hotkeys and is not part of the rendered world. A persistent USD grid
        # gives MuJoCo-like ground appearance in every frame.
        carb.settings.get_settings().set("/app/viewport/grid/enabled", False)
        if viewer_config["ground_style"] == "grid":
            grid_material = UsdShade.Material.Define(stage, "/World/SparkEnvironment/GridMaterial")
            grid_shader = UsdShade.Shader.Define(
                stage, "/World/SparkEnvironment/GridMaterial/PreviewSurface"
            )
            grid_shader.CreateIdAttr("UsdPreviewSurface")
            grid_shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(
                Gf.Vec3f(*viewer_config["grid_color"])
            )
            grid_shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.9)
            grid_material.CreateSurfaceOutput().ConnectToSource(
                grid_shader.ConnectableAPI(), "surface"
            )
            grid_root = UsdGeom.Xform.Define(stage, "/World/SparkEnvironment/GroundGrid")
            extent = viewer_config["grid_extent"]
            spacing = viewer_config["grid_spacing"]
            line_width = viewer_config["grid_line_width"]
            coordinates = np.arange(-extent, extent + 0.5 * spacing, spacing)
            for index, coordinate in enumerate(coordinates):
                for axis in ("X", "Y"):
                    line = UsdGeom.Cube.Define(
                        stage,
                        f"{grid_root.GetPath()}/{axis}_{index}",
                    )
                    line.CreateSizeAttr(1.0)
                    xform = UsdGeom.Xformable(line)
                    if axis == "X":
                        position = Gf.Vec3d(0.0, float(coordinate), 0.0015)
                        scale = Gf.Vec3d(2.0 * extent, line_width, 0.002)
                    else:
                        position = Gf.Vec3d(float(coordinate), 0.0, 0.0015)
                        scale = Gf.Vec3d(line_width, 2.0 * extent, 0.002)
                    xform.AddTranslateOp().Set(position)
                    xform.AddScaleOp().Set(scale)
                    UsdShade.MaterialBindingAPI.Apply(line.GetPrim()).Bind(grid_material)
        dome = UsdLux.DomeLight.Define(stage, "/World/SparkEnvironment/DomeLight")
        dome.CreateIntensityAttr(450.0)
        dome.CreateColorAttr(Gf.Vec3f(1.0, 1.0, 1.0))
        key = UsdLux.DistantLight.Define(stage, "/World/SparkEnvironment/KeyLight")
        key.CreateIntensityAttr(900.0)
        key.CreateAngleAttr(1.0)
        key_xform = UsdGeom.Xformable(key)
        key_xform.AddRotateXYZOp().Set(Gf.Vec3f(-35.0, 25.0, -25.0))
        if self.render_enabled:
            try:
                from isaacsim.core.utils.viewports import set_camera_view
                from omni.kit.viewport.utility import get_active_viewport

                set_camera_view(eye=list(camera_eye), target=list(camera_target))
                viewport = get_active_viewport()
                camera_path = str(viewport.camera_path)
                camera = UsdGeom.Camera(stage.GetPrimAtPath(camera_path))
                vertical_aperture = camera.GetVerticalApertureAttr().Get()
                if vertical_aperture and vertical_fov:
                    focal_length = float(vertical_aperture) / (
                        2.0 * np.tan(np.deg2rad(float(vertical_fov)) / 2.0)
                    )
                    focal_attr = camera.GetFocalLengthAttr()
                    # The Kit perspective camera is authored in a stronger
                    # session layer, so a plain USD Attr.Set on the root layer
                    # is ignored. Use Kit's property command, as its own camera
                    # utility does for pose edits.
                    omni.kit.commands.create(
                        "ChangePropertyCommand",
                        prop_path=focal_attr.GetPath(),
                        value=focal_length,
                        prev=focal_attr.Get(),
                        timecode=Usd.TimeCode.Default(),
                        type_to_create_if_not_exist=focal_attr.GetTypeName(),
                    ).do()
                camera_matrix = UsdGeom.XformCache().GetLocalToWorldTransform(camera.GetPrim())
                camera_forward = np.asarray(
                    camera_matrix.TransformDir(Gf.Vec3d(0.0, 0.0, -1.0)), dtype=float
                )
                camera_forward /= np.linalg.norm(camera_forward)
                actual_focal_length = float(camera.GetFocalLengthAttr().Get())
                self.camera_diagnostics = {
                    "eye": np.asarray(camera_matrix.ExtractTranslation(), dtype=float),
                    "forward": camera_forward,
                    "vertical_aperture": float(vertical_aperture),
                    "focal_length": actual_focal_length,
                    "vertical_fov": float(
                        np.rad2deg(
                            2.0 * np.arctan(float(vertical_aperture) / (2.0 * actual_focal_length))
                        )
                    ),
                }
            except ImportError:
                pass

    def _subscribe_keyboard(self):
        import carb
        import omni.appwindow

        app_window = omni.appwindow.get_default_app_window()
        if app_window is None:
            return
        self._keyboard_input = carb.input.acquire_input_interface()
        self._keyboard = app_window.get_keyboard()
        self._keyboard_subscription = self._keyboard_input.subscribe_to_keyboard_events(
            self._keyboard, self._on_keyboard_event
        )
        print(
            "Isaac keyboard teleop: O/P/B select right/left/base goal; Space selects obstacle; "
            "arrows+Q/E translate; 2-7 rotate; PageUp/PageDown add/remove obstacle; "
            "N toggles local/world; [ opens and ] closes the selected gripper; "
            "V toggles robot collision volumes; F toggles contact forces; "
            "C toggles contact wrench."
        )

    def _on_keyboard_event(self, event, *args):
        import carb

        key = event.input
        keyboard = carb.input.KeyboardInput
        # Consume every Space event. Kit normally binds Space to timeline
        # play/pause, while SPARK uses it exclusively to select an obstacle.
        # Cycle only on the initial press so key repeat cannot skip obstacles.
        if key == keyboard.SPACE:
            if event.type == carb.input.KeyboardEventType.KEY_PRESS:
                self._cycle_obstacle_debug_object()
            return False
        if key == keyboard.F:
            if event.type == carb.input.KeyboardEventType.KEY_PRESS:
                self.show_contact_forces = not self.show_contact_forces
                print(
                    f"[contact] link force vectors = {'ON' if self.show_contact_forces else 'OFF'}"
                )
            return False
        if key == keyboard.C:
            if event.type == carb.input.KeyboardEventType.KEY_PRESS:
                self.show_contact_wrench = not self.show_contact_wrench
                print(
                    f"[contact] resultant root wrench = "
                    f"{'ON' if self.show_contact_wrench else 'OFF'}"
                )
            return False
        if key == keyboard.V:
            if event.type == carb.input.KeyboardEventType.KEY_PRESS:
                self.render_robot_collision_volumes = not self.render_robot_collision_volumes
                print(
                    "[visualization] robot collision volumes = "
                    f"{'ON' if self.render_robot_collision_volumes else 'OFF'}"
                )
            return False
        if event.type not in (
            carb.input.KeyboardEventType.KEY_PRESS,
            carb.input.KeyboardEventType.KEY_REPEAT,
        ):
            return True
        s = self.manual_step_size
        translations = {
            keyboard.RIGHT: (0.0, s, 0.0),
            keyboard.LEFT: (0.0, -s, 0.0),
            keyboard.UP: (-s, 0.0, 0.0),
            keyboard.DOWN: (s, 0.0, 0.0),
            keyboard.E: (0.0, 0.0, s),
            keyboard.Q: (0.0, 0.0, -s),
        }
        rotations = {
            keyboard.KEY_2: (0.0, 0.0, 0.1),
            keyboard.KEY_3: (0.0, 0.0, -0.1),
            keyboard.KEY_4: (0.0, 0.1, 0.0),
            keyboard.KEY_5: (0.0, -0.1, 0.0),
            keyboard.KEY_6: (0.1, 0.0, 0.0),
            keyboard.KEY_7: (-0.1, 0.0, 0.0),
        }
        if key in translations:
            self._translate_debug_object(translations[key])
        elif key in rotations:
            from scipy.spatial.transform import Rotation

            self.debug_object[:3, :3] @= Rotation.from_euler("xyz", rotations[key]).as_matrix()
        elif key == keyboard.O:
            self.debug_object = self.right_goal_debug_frame
        elif key == keyboard.P:
            self.debug_object = self.left_goal_debug_frame
        elif key == keyboard.B:
            self.debug_object = self.base_goal_debug_frame
        elif key == keyboard.PAGE_UP:
            self._add_obstacle()
        elif key == keyboard.PAGE_DOWN:
            self._remove_obstacle()
        elif key == keyboard.N:
            self.translate_in_local_frame = not self.translate_in_local_frame
            print(
                f"[debug] translate frame = {'LOCAL' if self.translate_in_local_frame else 'WORLD'}"
            )
        elif key in tuple(
            value
            for value in (keyboard.KEY_8, getattr(keyboard, "RIGHT_BRACKET", None))
            if value is not None
        ):
            self._set_selected_gripper_state(True)
        elif key in tuple(
            value
            for value in (keyboard.KEY_9, getattr(keyboard, "LEFT_BRACKET", None))
            if value is not None
        ):
            self._set_selected_gripper_state(False)
        elif key in tuple(
            value
            for value in (
                getattr(keyboard, "MINUS", None),
                getattr(keyboard, "NUMPAD_SUBTRACT", None),
            )
            if value is not None
        ):
            self._adjust_manual_step_size(-self.manual_step_size_delta)
        elif key in tuple(
            value
            for value in (getattr(keyboard, "EQUAL", None), getattr(keyboard, "NUMPAD_ADD", None))
            if value is not None
        ):
            self._adjust_manual_step_size(self.manual_step_size_delta)
        return True

    def _translate_debug_object(self, delta):
        delta = np.asarray(delta, dtype=float)
        if self.translate_in_local_frame:
            delta = self.debug_object[:3, :3] @ delta
        self.debug_object[:3, 3] += delta

    def _adjust_manual_step_size(self, delta):
        self.manual_step_size = float(
            np.clip(
                self.manual_step_size + float(delta),
                self.manual_step_size_min,
                self.manual_step_size_max,
            )
        )
        print(f"[debug] manual movement step size = {self.manual_step_size:.4f} m")

    def _add_obstacle(self):
        frame = np.eye(4, dtype=float)
        frame[:3, 3] = np.array([0.6, 0.0, 0.793]) + np.random.uniform(-0.2, 0.2, 3)
        self.obstacle_debug_frame = np.concatenate((self.obstacle_debug_frame, frame[None]), axis=0)
        self.obstacle_debug_geom.append(
            Geometry(type="sphere", radius=0.05, color=VizColor.obstacle_debug)
        )
        self.num_obstacle_debug += 1
        self.obstacle_debug_velocity = np.zeros((self.num_obstacle_debug, 6), dtype=float)
        self.obstacle_debug_selected = self.num_obstacle_debug - 1
        self.debug_object = self.obstacle_debug_frame[self.obstacle_debug_selected]

    def _remove_obstacle(self):
        if not self.num_obstacle_debug:
            return
        index = self.obstacle_debug_selected or 0
        self.obstacle_debug_frame = np.delete(self.obstacle_debug_frame, index, axis=0)
        del self.obstacle_debug_geom[index]
        self.num_obstacle_debug -= 1
        self.obstacle_debug_velocity = np.zeros((self.num_obstacle_debug, 6), dtype=float)
        self.obstacle_debug_selected = 0 if self.num_obstacle_debug else None
        self.debug_object = (
            self.obstacle_debug_frame[self.obstacle_debug_selected]
            if self.num_obstacle_debug
            else self.base_goal_debug_frame
        )

    def _cycle_obstacle_debug_object(self):
        if not self.num_obstacle_debug:
            return
        current = -1 if self.obstacle_debug_selected is None else self.obstacle_debug_selected
        self.obstacle_debug_selected = (current + 1) % self.num_obstacle_debug
        self.debug_object = self.obstacle_debug_frame[self.obstacle_debug_selected]

    def _set_selected_gripper_state(self, closed):
        # The renderer receives an authoritative state packet every control
        # cycle.  Retain a local keyboard command until the dynamics process
        # echoes it back; otherwise the next (older) packet overwrites the key
        # press before the 20 Hz input channel can publish it.
        available_sides = self._available_keyboard_gripper_sides()
        selected_sides = []
        if self.debug_object is self.right_goal_debug_frame:
            selected_sides = ["right"]
        elif self.debug_object is self.left_goal_debug_frame:
            selected_sides = ["left"]
        elif len(available_sides) == 1:
            selected_sides = list(available_sides)
        else:
            print(
                "[gripper] Select the right hand with O or the left hand with P first",
                flush=True,
            )
            return
        for side in selected_sides:
            if available_sides and side not in available_sides:
                print(f"[gripper] The selected robot has no {side} gripper", flush=True)
                continue
            setattr(self, f"{side}_gripper_goal", bool(closed))
            setattr(self, f"{side}_gripper_debug_state", bool(closed))
            self._keyboard_gripper_goal_override[side] = bool(closed)

    def _selected_goal_debug_side(self):
        """Return the arm goal currently selected by keyboard teleoperation."""

        if self.debug_object is self.right_goal_debug_frame:
            return "right"
        if self.debug_object is self.left_goal_debug_frame:
            return "left"
        return None

    def _available_keyboard_gripper_sides(self):
        """Return configured gripper sides without importing robot-specific types."""

        for attribute in ("_configured_grippers", "_gripper_specs", "gripper_joint_ids"):
            configured = getattr(self, attribute, None)
            if configured:
                return tuple(side for side in ("left", "right") if side in configured)
        config_name = type(self.robot_cfg).__name__
        if "LeftArm" in config_name and "DualArm" not in config_name:
            return ("left",)
        if any(name in config_name for name in ("RightArm", "SingleArm")):
            return ("right",)
        return ()

    # ---------------------------- Pipeline rendering ----------------------------

    @staticmethod
    def _tensor_numpy(value):
        if hasattr(value, "detach"):
            value = value.detach()
        if hasattr(value, "cpu"):
            value = value.cpu()
        if hasattr(value, "numpy"):
            value = value.numpy()
        return np.asarray(value)

    def _contact_force_samples(self):
        """Return ``(link_position, world_force)`` samples for active contacts."""
        if self._contact_view is None or not self._contact_view.is_physics_handle_valid():
            return []
        try:
            force_data = self._contact_view.get_net_contact_forces(dt=self.dt)
            if force_data is None:
                return []
            forces = self._tensor_numpy(force_data).reshape(-1, 3)

            articulation_view = self.articulation._articulation_view
            link_data = articulation_view._physics_view.get_link_transforms()
            link_transforms = self._tensor_numpy(link_data)
            if link_transforms.ndim == 3:
                link_transforms = link_transforms[0]
            body_names = list(articulation_view.body_names or [])
            positions = {
                name: link_transforms[index, :3]
                for index, name in enumerate(body_names)
                if index < len(link_transforms)
            }
            root_position, _ = self.articulation.get_world_pose()
            root_position = self._tensor_numpy(root_position).reshape(3)
            return [
                (
                    np.asarray(positions.get(name, root_position), dtype=float),
                    np.asarray(force, dtype=float),
                )
                for name, force in zip(self._contact_body_names, forces)
                if np.linalg.norm(force) >= self.contact_force_threshold
            ]
        except (AttributeError, RuntimeError, ValueError) as exc:
            if not self._contact_warning_emitted:
                print(f"[contact] unable to read Isaac contact tensors: {exc}")
                self._contact_warning_emitted = True
            return []

    def _render_vector_arrow(self, origin, vector, *, scale, color, radius=0.003):
        origin = np.asarray(origin, dtype=float)
        endpoint = origin + float(scale) * np.asarray(vector, dtype=float)
        shaft = endpoint - origin
        length = float(np.linalg.norm(shaft))
        if length <= 1.0e-8:
            return
        direction = shaft / length
        reference = np.array([0.0, 0.0, 1.0])
        if abs(float(direction @ reference)) > 0.9:
            reference = np.array([0.0, 1.0, 0.0])
        side = np.cross(direction, reference)
        side /= np.linalg.norm(side)
        head_length = min(0.04, 0.25 * length)
        head_width = 0.45 * head_length
        self.render_line_segment(origin, endpoint, radius=radius, color=color)
        self.render_line_segment(
            endpoint,
            endpoint - head_length * direction + head_width * side,
            radius=radius,
            color=color,
        )
        self.render_line_segment(
            endpoint,
            endpoint - head_length * direction - head_width * side,
            radius=radius,
            color=color,
        )

    def _render_contact_debug(self):
        if not (self.show_contact_forces or self.show_contact_wrench):
            return
        samples = self._contact_force_samples()
        if not samples:
            return
        if self.show_contact_forces:
            for position, force in samples:
                self._render_vector_arrow(
                    position,
                    force,
                    scale=self.contact_force_scale,
                    color=(1.0, 0.75, 0.0, 1.0),
                )
        if self.show_contact_wrench:
            root_position, _ = self.articulation.get_world_pose()
            root_position = self._tensor_numpy(root_position).reshape(3)
            net_force = np.sum([force for _, force in samples], axis=0)
            # RigidContactView exposes net force per body without a contact
            # point. Applying each force at its link origin gives a useful,
            # deterministic resultant moment about the robot root.
            net_torque = np.sum(
                [np.cross(position - root_position, force) for position, force in samples],
                axis=0,
            )
            self._render_vector_arrow(
                root_position,
                net_force,
                scale=self.contact_force_scale,
                color=(0.0, 0.9, 1.0, 1.0),
                radius=0.004,
            )
            self._render_vector_arrow(
                root_position + np.array([0.0, 0.0, 0.12]),
                net_torque,
                scale=self.contact_torque_scale,
                color=(1.0, 0.1, 0.9, 1.0),
                radius=0.004,
            )

    @staticmethod
    def _rgba(color):
        value = np.asarray(color, dtype=float).reshape(-1)
        return value[:3], float(value[3] if value.size > 3 else 1.0)

    def render_sphere(self, pos, mat, size, color):
        # MuJoCo treats alpha-zero debug geometry as fully absent.  Do the
        # same here instead of authoring a transparent USD solid: RTX can
        # still show a faint silhouette for zero-opacity primvars, which made
        # ignored pelvis/foot collision volumes appear Isaac-specific.
        if self._rgba(color)[1] <= 0.0:
            return
        self._debug_primitives.append(
            {
                "type": "sphere",
                "pos": np.asarray(pos, float),
                "size": np.asarray(size, float),
                "color": color,
            }
        )

    def render_box(self, pos, mat, size, color):
        if self._rgba(color)[1] <= 0.0:
            return
        self._debug_primitives.append(
            {
                "type": "box",
                "pos": np.asarray(pos, float),
                "mat": np.asarray(mat, float).reshape(3, 3),
                "size": np.asarray(size, float),
                "color": color,
            }
        )

    def render_line_segment(self, pos1, pos2, radius=0.002, color=VizColor.goal):
        if self._rgba(color)[1] <= 0.0:
            return
        self._debug_primitives.append(
            {
                "type": "line",
                "start": np.asarray(pos1, float),
                "end": np.asarray(pos2, float),
                "radius": float(radius),
                "color": color,
            }
        )

    def render_pixel_line_segment(self, pos1, pos2, width=2.0, color=VizColor.goal, **kwargs):
        self.render_line_segment(pos1, pos2, radius=max(float(width), 1.0) * 0.001, color=color)

    def render_coordinate_frame(self, frame, size=0.1):
        frame = np.asarray(frame, dtype=float).reshape(4, 4)
        origin = frame[:3, 3]
        self.render_line_segment(origin, origin + size * frame[:3, 0], color=VizColor.x_axis)
        self.render_line_segment(origin, origin + size * frame[:3, 1], color=VizColor.y_axis)
        self.render_line_segment(origin, origin + size * frame[:3, 2], color=VizColor.z_axis)

    def render_surface(self, triangles, color):
        for triangle in np.asarray(triangles, dtype=float).reshape(-1, 3, 3):
            self.render_line_segment(triangle[0], triangle[1], color=color)
            self.render_line_segment(triangle[1], triangle[2], color=color)
            self.render_line_segment(triangle[2], triangle[0], color=color)

    def _flush_debug_primitives(self):
        from pxr import Gf, UsdGeom
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        UsdGeom.Xform.Define(stage, self._debug_root)
        material_cache = getattr(self, "_debug_rgba_material_cache", None)
        if material_cache is None:
            material_cache = {}
            self._debug_rgba_material_cache = material_cache
        spheres, boxes = [], []
        line_starts, line_ends, line_colors, line_widths = [], [], [], []
        for item in self._debug_primitives:
            if item["type"] == "sphere":
                rgb, alpha = _isaac_sphere_rgba(item["color"])
            else:
                rgb, alpha = self._rgba(item["color"])
            if item["type"] == "line":
                line_starts.append(tuple(float(value) for value in item["start"]))
                line_ends.append(tuple(float(value) for value in item["end"]))
                line_colors.append((*tuple(float(value) for value in rgb), alpha))
                line_widths.append(max(1.0, item["radius"] * 1000.0))
            elif item["type"] == "sphere":
                spheres.append((item, rgb, alpha))
            else:
                boxes.append((item, rgb, alpha))

        def update_solids(records, solid_type):
            cache = self._debug_solid_cache[solid_type]
            while len(cache) < len(records):
                index = len(cache)
                path = f"{self._debug_root}/{solid_type.title()}_{index}"
                if solid_type == "sphere":
                    prim = UsdGeom.Sphere.Define(stage, path)
                    prim.CreateRadiusAttr(1.0)
                else:
                    prim = UsdGeom.Cube.Define(stage, path)
                    prim.CreateSizeAttr(1.0)
                _disable_debug_prim_shadows(prim.GetPrim())
                xform = UsdGeom.Xformable(prim)
                color_primvar = prim.CreateDisplayColorPrimvar(UsdGeom.Tokens.constant)
                color_primvar.SetIndices([0])
                opacity_primvar = prim.CreateDisplayOpacityPrimvar(UsdGeom.Tokens.constant)
                opacity_primvar.SetIndices([0])
                entry = {
                    "prim": prim,
                    "translate": xform.AddTranslateOp(),
                    "orient": xform.AddOrientOp(),
                    "scale": xform.AddScaleOp(),
                    "color": color_primvar.GetAttr(),
                    "opacity": opacity_primvar.GetAttr(),
                    "visibility": UsdGeom.Imageable(prim).CreateVisibilityAttr(),
                    "material_key": None,
                }
                cache.append(entry)
            for index, (item, rgb, alpha) in enumerate(records):
                entry = cache[index]
                entry["translate"].Set(Gf.Vec3d(*np.asarray(item["pos"], dtype=float)))
                if solid_type == "box":
                    from scipy.spatial.transform import Rotation

                    q_xyzw = Rotation.from_matrix(item["mat"]).as_quat()
                    entry["orient"].Set(Gf.Quatf(q_xyzw[3], *q_xyzw[:3]))
                else:
                    entry["orient"].Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
                entry["scale"].Set(Gf.Vec3d(*np.asarray(item["size"], dtype=float)))
                entry["color"].Set([Gf.Vec3f(*rgb)])
                entry["opacity"].Set([float(alpha)])
                entry["material_key"] = _bind_debug_rgba_material(
                    stage,
                    entry["prim"].GetPrim(),
                    (*rgb, alpha),
                    material_cache,
                    previous_key=entry["material_key"],
                )
                entry["visibility"].Set(UsdGeom.Tokens.inherited)
            for entry in cache[len(records) :]:
                entry["visibility"].Set(UsdGeom.Tokens.invisible)

        update_solids(spheres, "sphere")
        update_solids(boxes, "box")
        self._debug_prim_count = len(spheres) + len(boxes)
        try:
            from isaacsim.util.debug_draw import _debug_draw
            import carb

            draw = _debug_draw.acquire_debug_draw_interface()
            draw.clear_lines()
            if line_starts:
                draw.draw_lines(
                    [carb.Float3(*point) for point in line_starts],
                    [carb.Float3(*point) for point in line_ends],
                    [carb.ColorRgba(*color) for color in line_colors],
                    [float(width) for width in line_widths],
                )
        except (ImportError, AttributeError):
            pass
        self._debug_primitives.clear()

    def _render_obstacle_debug(self):
        for frame, geom in zip(self.obstacle_debug_frame, self.obstacle_debug_geom):
            if geom.type == "sphere":
                self.render_sphere(
                    frame[:3, 3], frame[:3, :3], geom.attributes["radius"] * np.ones(3), geom.color
                )
            elif geom.type == "box":
                self.render_box(
                    frame[:3, 3],
                    frame[:3, :3],
                    np.array(
                        [
                            geom.attributes["length"],
                            geom.attributes["width"],
                            geom.attributes["height"],
                        ]
                    ),
                    geom.color,
                )

    def render(self):
        """Flush visualization requested through the backend-neutral pipeline API."""
        if self._renderer_process is not None:
            # Contact data belongs to the authoritative simulation process,
            # while its draw primitives are presented by the renderer child.
            self._render_contact_debug()
            primitives = self._debug_primitives
            self._debug_primitives = []
            root_reader = getattr(self, "_root_pose", None)
            if callable(root_reader):
                root_position, root_orientation = root_reader()
            else:
                root_position, root_orientation = self.articulation.get_world_pose()
            if hasattr(root_position, "detach"):
                root_position = root_position.detach().cpu().numpy()
            if hasattr(root_orientation, "detach"):
                root_orientation = root_orientation.detach().cpu().numpy()
            robot_base_frame = np.eye(4, dtype=float)
            robot_base_frame[:3, :3] = _quaternion_wxyz_to_matrix(root_orientation)
            robot_base_frame[:3, 3] = np.asarray(root_position, dtype=float)
            robot_base_frame = compose_planar_robot_base_frame(
                robot_base_frame, self.dof_pos_fbk, self.robot_cfg
            )
            self._renderer_process.submit(
                {
                    "sequence_id": self.step_index,
                    "dof_pos": np.asarray(self.dof_pos_fbk, dtype=float).copy(),
                    "robot_base_frame": robot_base_frame,
                    "debug_primitives": primitives,
                    "obstacle_debug_colors": [geom.color for geom in self.obstacle_debug_geom],
                    "left_gripper_goal": bool(self.left_gripper_goal),
                    "right_gripper_goal": bool(self.right_gripper_goal),
                    "simulation_info": self.get_simulation_info(),
                }
            )
            return
        if not self.render_enabled:
            self._debug_primitives.clear()
            return
        now = time.perf_counter()
        if self._render_period > 0.0 and now < self._next_render_time:
            # Pipeline visualization is reconstructed every control cycle, so
            # skipped frames should be dropped rather than accumulated.
            self._debug_primitives.clear()
            return
        self._next_render_time = now + self._render_period
        self._render_obstacle_debug()
        self._render_contact_debug()
        self._flush_debug_primitives()
        self._update_simulation_info_overlay()
        # SimulationContext.render() already calls
        # physics_sim_view.update_articulations_kinematic() for CUDA-backed
        # worlds before updating Fabric and Kit. Calling it here as well
        # duplicates the full articulation propagation every rendered frame.
        self.sim.render()
        if self.enable_camera:
            self._capture_camera_feedback()
        if self._viewport_layout_updates_remaining > 0:
            _show_viewport_only()
            self._viewport_layout_updates_remaining -= 1

    def _synchronize_kinematic_articulation(self) -> None:
        """Propagate teleported joint tensors to rendered link transforms.

        ``set_joint_positions`` changes PhysX's articulation state immediately,
        but viewer-only/model-dynamics execution does not take a physics step.
        Explicit kinematic propagation keeps the USD/Fabric link transforms in
        lockstep with the state used by SPARK's kinematics and safety rendering.
        """
        physics_view = getattr(self.sim, "physics_sim_view", None)
        if physics_view is not None:
            physics_view.update_articulations_kinematic()


class ConfiguredIsaacAgent(IsaacAgent):
    """Generic single-environment Isaac adapter driven by robot metadata.

    Robot-specific knowledge stays in ``RobotConfig.isaac_articulation``. The
    adapter imports that URDF, resolves joints by name, and uses the selected
    robot dynamics model to turn SPARK controls into physical drive targets.
    """

    def __new__(cls, robot_cfg, *args, **kwargs):
        num_envs = int(kwargs.get("num_envs", 1))
        tensor_api = bool(kwargs.pop("tensor_api", False))
        # Robot-family subclasses are intentional public extension points.
        # Preserve automatic tensor selection for those thin adapters instead
        # of limiting batched execution to the generic class itself.
        if num_envs > 1 or tensor_api:
            from .configured_tensor_agent import ConfiguredIsaacTensorAgent

            return ConfiguredIsaacTensorAgent(robot_cfg, *args, **kwargs)
        return super().__new__(cls)

    def __init__(
        self,
        robot_cfg,
        *,
        urdf_path: str | None = None,
        usd_path: str | None = None,
        joint_names: list[str] | tuple[str, ...] | None = None,
        fixed_base: bool | None = None,
        merge_fixed_joints: bool | None = None,
        allow_self_collision: bool | None = None,
        stiffness: float | None = None,
        damping: float | None = None,
        control_decimation: int | None = None,
        dynamics_backend: str = "simulator",
        use_sim_dynamics: bool | None = None,
        model_integrator: str | None = None,
        model_substeps: int = 1,
        enable_hand_control: bool = False,
        sim_position_error_limit: float | None = 0.05,
        gripper_position_error_limit: float | None = 0.025,
        asset_cache_dir: str | None = None,
        dt: float | None = None,
        **kwargs,
    ) -> None:
        spec = getattr(robot_cfg, "isaac_articulation", None)
        if spec is None:
            raise ValueError(
                f"{type(robot_cfg).__name__} does not declare isaac_articulation metadata"
            )
        selected_joint_names = tuple(joint_names or spec.joint_names)
        if len(selected_joint_names) != len(robot_cfg.DoFs):
            raise ValueError(
                f"Isaac joint metadata for {type(robot_cfg).__name__} has "
                f"{len(selected_joint_names)} joints, but the config has {len(robot_cfg.DoFs)} DoFs"
            )

        source = Path(urdf_path or spec.urdf_path).expanduser()
        if not source.is_absolute():
            source = Path(SPARK_ROBOT_RESOURCE_DIR) / source
        source = Path(
            apply_urdf_visual_palette(
                source,
                getattr(robot_cfg, "visual_link_colors", ()),
                cache_dir=asset_cache_dir,
            )
        )
        if spec.instances:
            source = Path(compose_urdf_instances(source, spec.instances, cache_dir=asset_cache_dir))
        if spec.planar_base is not None:
            source = Path(
                augment_urdf_planar_base(
                    source,
                    spec.planar_base,
                    cache_dir=asset_cache_dir,
                )
            )
        resolved_fixed_base = spec.fixed_base if fixed_base is None else bool(fixed_base)
        resolved_merge_fixed_joints = (
            spec.merge_fixed_joints if merge_fixed_joints is None else bool(merge_fixed_joints)
        )
        resolved_self_collision = (
            spec.allow_self_collision
            if allow_self_collision is None
            else bool(allow_self_collision)
        )
        resolved_usd = usd_path or import_urdf_to_usd(
            source,
            fix_base=resolved_fixed_base,
            merge_fixed_joints=resolved_merge_fixed_joints,
            allow_self_collision=resolved_self_collision,
            cache_dir=asset_cache_dir,
        )
        simulator_spec = robot_cfg.simulator_dynamics
        self.control_decimation = int(
            simulator_spec.control_decimation if control_decimation is None else control_decimation
        )
        if self.control_decimation < 1:
            raise ValueError("control_decimation must be positive")
        physics_dt = simulator_spec.physics_dt if dt is None else float(dt)
        if use_sim_dynamics is not None:
            dynamics_backend = "simulator" if use_sim_dynamics else "model"
        dynamics_backend = str(dynamics_backend).lower()
        if dynamics_backend not in {"model", "simulator"}:
            raise ValueError("dynamics_backend must be 'model' or 'simulator'")
        self.dynamics_backend = dynamics_backend
        self.fixed_base = resolved_fixed_base
        self.allow_self_collision = resolved_self_collision
        self.drive_stiffness = float(spec.stiffness if stiffness is None else stiffness)
        self.drive_damping = float(spec.damping if damping is None else damping)
        self.joint_gain_overrides = dict(
            (name, (float(joint_stiffness), float(joint_damping)))
            for name, joint_stiffness, joint_damping in spec.joint_gain_overrides
        )
        if self.drive_stiffness < 0.0 or self.drive_damping < 0.0:
            raise ValueError("Isaac drive gains must be non-negative")
        self.enable_hand_control = bool(enable_hand_control)
        self.sim_position_error_limit = None
        if sim_position_error_limit is not None:
            self.sim_position_error_limit = float(sim_position_error_limit)
            if self.sim_position_error_limit <= 0.0:
                raise ValueError("sim_position_error_limit must be positive or None")
        self.gripper_position_error_limit = None
        if gripper_position_error_limit is not None:
            self.gripper_position_error_limit = float(gripper_position_error_limit)
            if self.gripper_position_error_limit <= 0.0:
                raise ValueError("gripper_position_error_limit must be positive or None")

        super().__init__(
            robot_cfg,
            usd_path=resolved_usd,
            joint_names=selected_joint_names,
            control_mode=spec.control_mode,
            dt=physics_dt,
            enabled_self_collisions=resolved_self_collision,
            **kwargs,
        )
        if any(abs(value) > 1.0e-12 for value in spec.base_rpy):
            root_position, root_orientation = self.articulation.get_world_pose()
            self.articulation.set_world_pose(
                position=root_position,
                orientation=_multiply_quaternion_wxyz(
                    root_orientation, _rpy_to_quaternion_wxyz(spec.base_rpy)
                ),
            )
        # A RobotConfig may intentionally expose only a controllable subset of
        # a larger articulation (for example one arm of a dual-arm robot).
        # Keep the remaining joints at their imported rest pose instead of
        # leaving them as unactuated mechanisms that can fall under gravity
        # and disturb the selected chain.
        self.all_joint_indices = self._torch.arange(
            len(self.articulation.dof_names), device=self.device, dtype=self._torch.long
        )
        self._configured_grippers = {}
        articulation_joint_names = tuple(self.articulation.dof_names)
        for gripper in spec.grippers:
            if not all(name in articulation_joint_names for name in gripper.joint_names):
                continue
            indices = self._torch.as_tensor(
                [articulation_joint_names.index(name) for name in gripper.joint_names],
                device=self.device,
                dtype=self._torch.long,
            )
            self._configured_grippers[gripper.side] = (gripper, indices)
        selected_indices = set(self.joint_indices.detach().cpu().tolist())
        if self.enable_hand_control:
            selected_indices.update(
                index
                for _gripper, indices in self._configured_grippers.values()
                for index in indices.detach().cpu().tolist()
            )
        self.unselected_joint_indices = self._torch.as_tensor(
            [
                index
                for index in range(len(self.articulation.dof_names))
                if index not in selected_indices
            ],
            device=self.device,
            dtype=self._torch.long,
        )
        self._default_all_joint_positions = self.articulation.get_joint_positions().detach().clone()
        missing_defaults = [
            name
            for name, _position in spec.joint_position_defaults
            if name not in articulation_joint_names
        ]
        if missing_defaults:
            raise ValueError(
                f"Isaac articulation is missing joints with declared defaults: {missing_defaults}"
            )
        # Scalar and tensor Isaac agents must start the complete imported
        # articulation from the same declared pose.  This matters when a
        # RobotConfig controls only one arm: the remaining torso/arm joints
        # still participate in rendering and physical contacts, and leaving
        # them at the URDF import pose misaligns them with Pinocchio collision
        # frames computed from the declared locked-joint defaults.
        for name, position in spec.joint_position_defaults:
            self._default_all_joint_positions[articulation_joint_names.index(name)] = float(
                position
            )
        for gripper, indices in self._configured_grippers.values():
            self._default_all_joint_positions[indices] = self._torch.as_tensor(
                gripper.open_positions, device=self.device, dtype=self._torch.float32
            )
        self.use_sim_dynamics = self.dynamics_backend == "simulator"
        self.dynamics_model = robot_cfg.create_dynamics_model()
        self.dynamics_executor = DynamicsExecutor(
            self.dynamics_model,
            dt=self.dt * self.control_decimation,
            integrator=model_integrator,
            substeps=model_substeps,
        )
        self._configure_position_drives()
        self.reset()

    def _configure_position_drives(self) -> None:
        if self.control_mode != "position":
            return
        kps = self._torch.full(
            (len(self.articulation.dof_names),),
            self.drive_stiffness,
            device=self.device,
            dtype=self._torch.float32,
        )
        kds = self._torch.full(
            (len(self.articulation.dof_names),),
            self.drive_damping,
            device=self.device,
            dtype=self._torch.float32,
        )
        dof_index_by_name = {name: index for index, name in enumerate(self.articulation.dof_names)}
        for name, (stiffness, damping) in self.joint_gain_overrides.items():
            index = dof_index_by_name.get(name)
            if index is not None:
                kps[index] = stiffness
                kds[index] = damping
        self.articulation._articulation_view.set_gains(
            kps=kps,
            kds=kds,
            joint_indices=self.all_joint_indices,
        )

    def reset(self, agent_reset_info=None, **kwargs) -> None:
        super().reset(agent_reset_info=agent_reset_info, **kwargs)
        if self.unselected_joint_indices.numel():
            self.articulation.set_joint_positions(
                self._default_all_joint_positions[self.unselected_joint_indices],
                joint_indices=self.unselected_joint_indices,
            )
            self.articulation.set_joint_velocities(
                self._torch.zeros(
                    self.unselected_joint_indices.numel(),
                    device=self.device,
                    dtype=self._torch.float32,
                ),
                joint_indices=self.unselected_joint_indices,
            )
        reset_info = dict(agent_reset_info or {})
        initial_model_state = reset_info.get("dynamics_state")
        if initial_model_state is None:
            # The World reset requires one settling step.  Gravity can create
            # a small measured velocity during that step; using it to seed a
            # double-integrator command model makes a zero-acceleration hold
            # drift indefinitely.  SPARK resets define zero DoF velocity
            # unless an explicit dynamics_state is supplied.
            initial_model_state = self.dynamics_model.extract_state(
                self.dof_pos_fbk,
                np.zeros_like(self.dof_vel_fbk),
            )
        self.dynamics_executor.reset(
            initial_model_state,
            time=float(reset_info.get("time", 0.0)),
            seed=reset_info.get("seed"),
        )
        self._update_command_from_model_state(self.dynamics_executor.state)
        self._apply_position_targets()

    def _update_command_from_model_state(self, model_state) -> None:
        full_state = self.dynamics_model.expand_state(model_state)
        self.dof_pos_cmd = np.asarray(
            self.robot_cfg.decompose_state_to_dof_pos(full_state), dtype=float
        )
        self.dof_vel_cmd = np.asarray(
            self.robot_cfg.decompose_state_to_dof_vel(full_state), dtype=float
        )
        self.dof_acc_cmd = np.zeros(self.num_dof, dtype=float)

    def _advance_command_model(self, command, action_info) -> None:
        control = self.dynamics_model.project_control(command)
        next_state = self.dynamics_executor.step(
            control,
            parameters=action_info.get("dynamics_parameters"),
            exogenous=action_info.get("dynamics_exogenous"),
        )
        self._update_command_from_model_state(next_state)
        self.last_control = np.asarray(command, dtype=float).reshape(-1).copy()

    def _bound_simulator_command_state(self) -> None:
        """Bound drive error without discarding gravity-supporting preload.

        Resetting the command target to measured position every cycle prevents
        windup, but it also ratchets a gravity-loaded articulation downward.
        Retain the integrated command while keeping it within a finite error
        of PhysX feedback, matching the qualified Unitree target contract.
        """

        if self.sim_position_error_limit is None:
            return
        measured = np.asarray(self.dof_pos_fbk, dtype=float)
        command = np.asarray(self.dof_pos_cmd, dtype=float)
        self.dof_pos_cmd = measured + np.clip(
            command - measured,
            -self.sim_position_error_limit,
            self.sim_position_error_limit,
        )
        # Keep the executor's internal state synchronized with the bounded
        # target so a blocked drive cannot accumulate hidden command windup.
        self.dynamics_executor.state = self.dynamics_model.extract_state(
            self.dof_pos_cmd,
            self.dof_vel_cmd,
        )

    def _update_configured_gripper_targets(self, joint_positions, action_info) -> None:
        if not self.enable_hand_control:
            return
        action_info = dict(action_info or {})
        for side, (gripper, indices) in self._configured_grippers.items():
            control_key = f"{side}_gripper_control"
            goal_key = f"{side}_gripper_goal"
            if control_key in action_info:
                target = np.asarray(action_info[control_key], dtype=float).reshape(-1)
                if target.size != len(gripper.joint_names):
                    raise ValueError(f"{control_key} must have {len(gripper.joint_names)} entries")
                closed = bool(
                    np.linalg.norm(target - np.asarray(gripper.open_positions, dtype=float)) > 1e-6
                )
                self._keyboard_gripper_goal_override.pop(side, None)
            else:
                action_goal = action_info.get(
                    goal_key,
                    action_info.get("gripper_goal") if side == "right" else None,
                )
                if side in self._keyboard_gripper_goal_override:
                    keyboard_goal = bool(self._keyboard_gripper_goal_override[side])
                    if action_goal is not None and bool(action_goal) == keyboard_goal:
                        self._keyboard_gripper_goal_override.pop(side, None)
                    else:
                        action_goal = keyboard_goal
                closed = bool(getattr(self, goal_key) if action_goal is None else action_goal)
                target = gripper.closed_positions if closed else gripper.open_positions
            setattr(self, goal_key, closed)
            setattr(self, f"{side}_gripper_debug_state", closed)
            target = self._torch.as_tensor(target, device=self.device, dtype=self._torch.float32)
            if self.use_sim_dynamics and self.gripper_position_error_limit is not None:
                # Binary open/close commands otherwise submit the full target
                # in one frame.  If a finger is blocked by another robot link,
                # that large persistent drive error can overpower PhysX
                # contact.  Anchor the auxiliary joints to measured state just
                # like the main simulator-owned command model.
                measured = self.articulation.get_joint_positions(joint_indices=indices)
                error = self._torch.clamp(
                    target - measured,
                    -self.gripper_position_error_limit,
                    self.gripper_position_error_limit,
                )
                target = measured + error
            joint_positions[indices] = target

    def _position_targets(self, action_info=None):
        joint_positions = self._default_all_joint_positions.clone()
        joint_velocities = self._torch.zeros_like(joint_positions)
        joint_positions[self.joint_indices] = self._torch.as_tensor(
            self.dof_pos_cmd, device=self.device, dtype=self._torch.float32
        )
        joint_velocities[self.joint_indices] = self._torch.as_tensor(
            self.dof_vel_cmd, device=self.device, dtype=self._torch.float32
        )
        self._update_configured_gripper_targets(joint_positions, action_info)
        return joint_positions, joint_velocities

    def _apply_position_targets(self, action_info=None) -> None:
        from isaacsim.core.utils.types import ArticulationAction

        joint_positions, joint_velocities = self._position_targets(action_info)
        self.articulation.apply_action(
            ArticulationAction(
                joint_positions=joint_positions,
                joint_velocities=joint_velocities,
                joint_indices=self.all_joint_indices,
            )
        )

    def _send_control_sim_dynamics(self, command: np.ndarray, **kwargs) -> None:
        action_info = dict(kwargs.get("action_info") or {})
        self._advance_command_model(command, action_info)
        self._bound_simulator_command_state()
        self._apply_position_targets(action_info)

    def _send_control_modeled_dynamics(self, command: np.ndarray, **kwargs) -> None:
        action_info = dict(kwargs.get("action_info") or {})
        self._advance_command_model(command, action_info)
        joint_positions, joint_velocities = self._position_targets(action_info)
        self.articulation.set_joint_positions(
            joint_positions,
            joint_indices=self.all_joint_indices,
        )
        self.articulation.set_joint_velocities(
            joint_velocities,
            joint_indices=self.all_joint_indices,
        )

    def _post_control_processing(self, **kwargs) -> None:
        render_frame = self.render_enabled and self.render_on_step
        if render_frame:
            self._update_simulation_info_overlay()
        if self.use_sim_dynamics:
            for substep in range(self.control_decimation):
                self.sim.step(render=(render_frame and substep == self.control_decimation - 1))
            self.time += self.dt * self.control_decimation
        else:
            if render_frame:
                self._synchronize_kinematic_articulation()
                self.sim.render()
            self.time = self.dynamics_executor.time
        self.step_index += 1
        self._refresh_feedback()
        if self.enable_camera and self.render_enabled and self.render_on_step:
            self._capture_camera_feedback()

    def _compose_cmd_state(self) -> np.ndarray:
        return np.asarray(
            self.robot_cfg.compose_state_from_dof(self.dof_pos_cmd, self.dof_vel_cmd),
            dtype=float,
        )
