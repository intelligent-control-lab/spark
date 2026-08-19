from pathlib import Path
import xml.etree.ElementTree as ET

from spark_agent.simulation.isaac.unitree_g1 import UnitreeG1IsaacAgent
from spark_agent.simulation.isaac.unitree_g1.unitree_g1_isaac_agent import G1_29DOF_JOINT_NAMES
from spark_robot import SPARK_ROBOT_RESOURCE_DIR, UnitreeG1WholeBodyDynamic1Config
from spark_robot.unitree_g1.actuation import UNITREE_G1_GRIPPER_SPECS


def test_g1_urdf_assets_and_joint_contract():
    resource_dir = Path(SPARK_ROBOT_RESOURCE_DIR) / "unitree_g1"
    mjcf_files = {path.stem: path for path in (resource_dir / "mjcf").glob("*.xml")}
    urdf_files = {path.stem: path for path in (resource_dir / "urdf").glob("*.urdf")}
    assert mjcf_files.keys() == urdf_files.keys()

    for stem, urdf in urdf_files.items():
        urdf_root = ET.parse(urdf).getroot()
        urdf_joints = {
            joint.attrib["name"]
            for joint in urdf_root.findall("joint")
            if joint.attrib.get("type") != "fixed"
        }
        mjcf_joints = {
            joint.attrib["name"]
            for joint in ET.parse(mjcf_files[stem]).findall(".//worldbody//joint")
            if "name" in joint.attrib
            and joint.attrib.get("type") != "free"
            and joint.attrib["name"] != "floating_base_joint"
        }
        assert urdf_joints == mjcf_joints
        for mesh in urdf_root.findall(".//mesh"):
            assert (urdf.parent / mesh.attrib["filename"]).is_file()

    whole_body_urdf = ET.parse(urdf_files["g1_29dof_whole_body"]).getroot()
    whole_body_joints = tuple(
        joint.attrib["name"]
        for joint in whole_body_urdf.findall("joint")
        if joint.attrib.get("type") != "fixed"
    )
    assert whole_body_joints == G1_29DOF_JOINT_NAMES

    cfg = UnitreeG1WholeBodyDynamic1Config()
    assert len(cfg.DoFs) == 36
    assert len(cfg.MujocoMotors) == len(G1_29DOF_JOINT_NAMES) == 29
    assert UnitreeG1IsaacAgent.__name__ == "UnitreeG1IsaacAgent"


def test_g1_hand_spec_matches_every_with_hand_urdf():
    resource_dir = Path(SPARK_ROBOT_RESOURCE_DIR) / "unitree_g1" / "urdf"
    expected_hand_joints = {
        name for spec in UNITREE_G1_GRIPPER_SPECS.values() for name in spec.joint_names
    }
    assert len(expected_hand_joints) == 14

    saw_articulated_hand = False
    for urdf in resource_dir.glob("*_with_hand.urdf"):
        root = ET.parse(urdf).getroot()
        joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
        available_specs = [
            spec
            for spec in UNITREE_G1_GRIPPER_SPECS.values()
            if all(name in joints for name in spec.joint_names)
        ]
        present_hand_joints = expected_hand_joints & joints.keys()
        if not present_hand_joints:
            # The reduced dual-arm visualization intentionally fixes its hand
            # links and therefore exposes no finger articulation.
            continue
        saw_articulated_hand = True
        assert present_hand_joints == {
            name for spec in available_specs for name in spec.joint_names
        }, urdf.name
        for spec in available_specs:
            target_sets = (spec.open_position, spec.closed_position)
            for joint_name, *targets in zip(spec.joint_names, *target_sets):
                limit = joints[joint_name].find("limit")
                assert limit is not None
                lower = float(limit.attrib["lower"])
                upper = float(limit.attrib["upper"])
                assert all(lower <= target <= upper for target in targets)
                assert float(limit.attrib["effort"]) > 0.0
    assert saw_articulated_hand


def test_whole_body_hand_materials_match_between_mujoco_and_isaac_urdf():
    resource_dir = Path(SPARK_ROBOT_RESOURCE_DIR) / "unitree_g1"
    urdf_root = ET.parse(resource_dir / "urdf" / "g1_29dof_whole_body_with_hand.urdf").getroot()
    urdf_colors = {
        tuple(float(value) for value in color.attrib["rgba"].split())
        for link in urdf_root.findall("link")
        if "hand" in link.attrib["name"]
        for color in link.findall("./visual/material/color")
    }

    mjcf_root = ET.parse(resource_dir / "mjcf" / "g1_29dof_whole_body_with_hand.xml").getroot()
    mjcf_colors = {
        tuple(float(value) for value in geom.attrib["rgba"].split())
        for geom in mjcf_root.findall(".//geom")
        if "hand" in geom.attrib.get("mesh", "") and "rgba" in geom.attrib
    }

    assert urdf_colors == mjcf_colors == {(0.7, 0.7, 0.7, 1.0)}
