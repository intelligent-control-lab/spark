"""Provenance and path contracts for the official Kinova Gen3 arm asset."""

from __future__ import annotations

import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

from spark_robot import SPARK_ROBOT_RESOURCE_DIR


RESOURCE_ROOT = Path(SPARK_ROBOT_RESOURCE_DIR) / "kinova_gen3"

OFFICIAL_MESH_SHA256 = {
    "base_link.STL": "3e38517e653d58f4160cfdcee34c9dea983d8b3ecb8f2c147444f58235335260",
    "bracelet_no_vision_link.STL": "5b2ec7dafbf6a3c9a2cac717d18132a8a1efd9fcb8f2c40d9c12c8ece7323ec8",
    "bracelet_with_vision_link.STL": "8bb05e420a88532b4eee3948a0047fb8e7f341c3e5cc971d54e97e2aaffcf878",
    "forearm_link.STL": "a00d96d2c7ec37e9fe4c5fb6d3fa2b4386dfd48fea5ac25632f4cb83b3505105",
    "half_arm_1_link.STL": "8df461a71780d8e2d09104a507584424279d85173adf14867e359f7e58b3bba7",
    "half_arm_2_link.STL": "8a9f38de21648804b3ab7076ec73726031f2ade7c95eb2541d070553e025407b",
    "shoulder_link.STL": "a98e232e5197f09e9cd520e7a450c21bcec74bc49e2a19ce7cf86c5dc25f2e18",
    "spherical_wrist_1_link.STL": "0b97a5e3b2de535344f8e8d4176d77f0cdb3455b235f35430fa0413208856e66",
    "spherical_wrist_2_link.STL": "12271fb7d0ee78fb5642424cf5078be20f0476061e74203db964b674031ce879",
}


def test_kinova_urdf_mesh_references_resolve_locally():
    urdf = RESOURCE_ROOT / "gen3.urdf"
    mesh_nodes = ET.parse(urdf).findall(".//mesh")
    arm_mesh_nodes = [
        node for node in mesh_nodes if node.attrib["filename"].startswith("./meshes/gen3/")
    ]
    display_gripper_nodes = [
        node for node in mesh_nodes if node.attrib["filename"].startswith("./meshes/robotiq_2f85/")
    ]
    assert len(arm_mesh_nodes) == 16
    assert len(display_gripper_nodes) == 14
    for node in mesh_nodes:
        reference = node.attrib["filename"]
        assert (RESOURCE_ROOT / reference).is_file(), reference


def test_kinova_arm_meshes_match_recorded_official_revision():
    mesh_root = RESOURCE_ROOT / "meshes/gen3"
    assert {path.name for path in mesh_root.glob("*.STL")} == set(OFFICIAL_MESH_SHA256)
    for name, expected in OFFICIAL_MESH_SHA256.items():
        digest = hashlib.sha256((mesh_root / name).read_bytes()).hexdigest()
        assert digest == expected, name


def test_kinova_provenance_retains_upstream_license():
    readme = (RESOURCE_ROOT / "README.md").read_text()
    license_text = (RESOURCE_ROOT / "LICENSE.kinova").read_text()
    assert "c3280d8a6a6b5d96590d51538e3f265710a6c77d" in readme
    assert "Kinova inc." in license_text
    assert "Redistribution and use" in license_text
