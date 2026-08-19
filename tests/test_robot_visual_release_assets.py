"""Release contracts for cross-backend robot visuals and representative GIFs."""

from __future__ import annotations

import importlib.util
from pathlib import Path
import sys
import xml.etree.ElementTree as ET

import spark_robot
from spark_agent.simulation.isaac.isaac_agent import apply_urdf_visual_palette
from spark_robot import SPARK_ROBOT_RESOURCE_DIR


ROOT = Path(__file__).resolve().parents[1]
RESOURCE_ROOT = Path(SPARK_ROBOT_RESOURCE_DIR)


def _load_generator():
    path = ROOT / "tools" / "generate_robot_demo_gifs.py"
    spec = importlib.util.spec_from_file_location("spark_demo_gif_generator_test", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_demo_gif_manifest_has_twenty_one_profiles_and_five_modes():
    generator = _load_generator()
    assert len(generator.PROFILES) == 21
    assert generator.MODES == {
        "mujoco_teleop": ("mujoco", "teleop", 1),
        "isaac_teleop": ("isaac", "teleop", 1),
        "mujoco_benchmark": ("mujoco", "benchmark", 1),
        "isaac_benchmark": ("isaac", "benchmark", 1),
        "isaac_parallel": ("isaac", "benchmark", 4),
    }
    assert len(generator.PROFILES) * len(generator.MODES) == 105
    assert {case.family for case in generator.PROFILES.values()} == {
        "agibot_g1",
        "fanuc_lrmate200id",
        "galaxea_r1lite",
        "kinova_gen3",
        "kuka_iiwa14",
        "unitree_g1",
    }


def test_demo_gifs_use_real_time_release_encoding_and_v1_tasks():
    generator = _load_generator()
    args = generator._parser().parse_args([])
    assert args.duration == 16.0
    assert args.fps == 10.0
    assert (args.width, args.height) == (960, 540)
    assert all(case.test_case.endswith("_v1") for case in generator.PROFILES.values())


def test_generic_industrial_configs_use_robot_only_mujoco_assets():
    expected = {
        "KinovaGen3": ("kinova_gen3/gen3_2f85_single.xml", "kinova_gen3/gen3_2f85_dual.xml"),
        "KukaIIWA14": (
            "kuka_iiwa14/iiwa14_2f85_single.xml",
            "kuka_iiwa14/iiwa14_2f85_dual.xml",
        ),
        "FanucLRMate200iD": (
            "fanuc_lrmate200id/lrmate200id_single.xml",
            "fanuc_lrmate200id/lrmate200id_dual.xml",
        ),
    }
    for prefix, (single_path, dual_path) in expected.items():
        configs = [
            value()
            for name, value in vars(spark_robot).items()
            if name.startswith(prefix) and name.endswith("Config") and isinstance(value, type)
        ]
        for config in configs:
            if "DualArm" in type(config).__name__:
                assert config.mujoco_model_path == dual_path
            elif config.mujoco_model_path is not None:
                assert config.mujoco_model_path == single_path


def test_isaac_arm_urdfs_include_declared_visual_grippers():
    expectations = {
        "kinova_gen3/gen3.urdf": ("robotiq_2f85", 3, {"fixed": 1, "prismatic": 2}),
        "kuka_iiwa14/iiwa14.urdf": (
            "robotiq_2f85",
            3,
            {"fixed": 1, "prismatic": 2},
        ),
        "fanuc_lrmate200id/lrmate200id7l.urdf": (
            "robotiq_3f",
            13,
            {"fixed": 4, "revolute": 9},
        ),
        "fanuc_lrmate200id/lrmate200id7l_dual.urdf": (
            "robotiq_3f",
            2,
            {"fixed": 2},
        ),
    }
    for relative_path, (mesh_fragment, link_count, joint_type_counts) in expectations.items():
        root = ET.parse(RESOURCE_ROOT / relative_path).getroot()
        links = [link for link in root.findall("link") if "spark_robotiq" in link.get("name", "")]
        assert len(links) == link_count
        assert all(
            mesh_fragment in mesh.get("filename", "")
            for link in links
            for mesh in link.findall(".//mesh")
        )
        spark_joints = [
            joint for joint in root.findall("joint") if "spark_robotiq" in joint.get("name", "")
        ]
        assert len(spark_joints) == link_count
        assert {
            joint_type: sum(joint.get("type") == joint_type for joint in spark_joints)
            for joint_type in joint_type_counts
        } == joint_type_counts


def test_kuka_gripper_base_visual_has_an_unambiguous_isaac_prim_name():
    root = ET.parse(RESOURCE_ROOT / "kuka_iiwa14" / "iiwa14.urdf").getroot()
    visual = next(
        visual
        for visual in root.findall(".//visual")
        if visual.find("geometry/mesh").get("filename", "").endswith("/base.stl")
    )

    assert visual.get("name") == "spark_gripper_base"


def test_gripper_keyboard_models_do_not_define_conflicting_named_cameras():
    model_paths = {
        spark_robot.AgiBotG1RightArmDynamic1Config().mujoco_model_path,
        spark_robot.GalaxeaR1LiteRightArmDynamic1CollisionConfig().mujoco_model_path,
        spark_robot.UnitreeG1RightArmWithHandDynamic1Config().mujoco_model_path,
        spark_robot.FanucLRMate200iDSingleArmDynamic1CollisionConfig().mujoco_model_path,
        spark_robot.KinovaGen3SingleArmDynamic1CollisionConfig().mujoco_model_path,
        spark_robot.KukaIIWA14SingleArmDynamic1CollisionConfig().mujoco_model_path,
    }
    for model_path in model_paths:
        assert ET.parse(RESOURCE_ROOT / model_path).findall(".//camera") == []


def test_galaxea_mujoco_uses_the_bundled_visual_texture():
    texture = RESOURCE_ROOT / "galaxea_r1lite" / "meshes" / "texture.png"
    assert texture.is_file() and texture.stat().st_size > 0
    for name in (
        "r1lite_fixed_base.xml",
        "r1lite_dual_arm.xml",
        "r1lite_mobile_base.xml",
        "robot_mjcf.xml",
    ):
        root = ET.parse(RESOURCE_ROOT / "galaxea_r1lite" / name).getroot()
        texture_element = root.find(".//texture[@name='r1lite_texture']")
        material = root.find(".//material[@name='default_material']")
        assert texture_element is not None
        assert texture_element.get("file") == "meshes/texture.png"
        assert material is not None and material.get("texture") == "r1lite_texture"


def test_galaxea_mujoco_restores_original_white_scene_and_black_grid():
    for name in (
        "r1lite_dual_arm.xml",
        "r1lite_fixed_base.xml",
        "r1lite_mobile_base.xml",
    ):
        root = ET.parse(RESOURCE_ROOT / "galaxea_r1lite" / name).getroot()
        skybox = root.find(".//texture[@type='skybox']")
        ground = root.find(".//texture[@name='groundplane']")
        material = root.find(".//material[@name='groundplane']")
        assert skybox is not None and skybox.get("rgb1") == skybox.get("rgb2") == "1 1 1"
        assert ground is not None
        assert ground.get("rgb1") == ground.get("rgb2") == "1 1 1"
        assert ground.get("markrgb") == "0 0 0"
        assert material is not None and material.get("reflectance") == "0.2"


def test_agibot_palette_is_applied_to_a_cached_urdf(tmp_path):
    config = spark_robot.AgiBotG1DualArmDynamic1Config()
    source = RESOURCE_ROOT / config.isaac_articulation.urdf_path
    output = Path(
        apply_urdf_visual_palette(
            source,
            config.visual_link_colors,
            cache_dir=tmp_path,
        )
    )
    assert output.is_file()
    root = ET.parse(output).getroot()
    colors = {color.get("rgba") for color in root.findall(".//visual/material/color")}
    assert len(colors) >= 6
    assert all(Path(mesh.get("filename", "")).is_absolute() for mesh in root.findall(".//mesh"))


def test_kuka_single_and_dual_mujoco_share_the_white_scene_and_black_grid():
    for name in ("iiwa14_2f85_single.xml", "iiwa14_2f85_dual.xml"):
        root = ET.parse(RESOURCE_ROOT / "kuka_iiwa14" / name).getroot()
        skybox = root.find(".//texture[@type='skybox']")
        ground = root.find(".//texture[@name='groundplane']")
        material = root.find(".//material[@name='groundplane']")
        light = root.find(".//worldbody/light")
        floor = root.find(".//geom[@name='floor']")

        assert skybox is not None and skybox.get("rgb1") == skybox.get("rgb2") == "1 1 1"
        assert ground is not None and ground.get("builtin") == "checker"
        assert ground.get("rgb1") == ground.get("rgb2") == "1 1 1"
        assert ground.get("markrgb") == "0 0 0"
        assert material is not None and material.get("reflectance") == "0.2"
        assert light is not None and light.get("pos") == "0 0 1.5"
        assert floor is not None and floor.get("material") == "groundplane"
