from __future__ import annotations

import xml.etree.ElementTree as ET

import numpy as np

from spark_agent.simulation.isaac.isaac_agent import (
    augment_urdf_planar_base,
    compose_planar_robot_base_frame,
    compose_urdf_instances,
)
from spark_agent.simulation.isaac.kuka_iiwa14.kuka_iiwa14_isaac_agent import (
    remove_asset_alignment_from_base_frame,
)
from spark_robot import (
    AgiBotG1MobileBaseDynamic1Config,
    IsaacArticulationInstanceSpec,
    IsaacPlanarBaseSpec,
)


def test_compose_urdf_instances_prefixes_chains_and_mounts(tmp_path):
    source = tmp_path / "arm.urdf"
    source.write_text(
        """<?xml version="1.0"?>
<robot name="arm">
  <link name="base"/>
  <link name="tip"/>
  <joint name="axis" type="revolute">
    <parent link="base"/>
    <child link="tip"/>
    <axis xyz="0 0 1"/>
    <limit effort="1" lower="-1" upper="1" velocity="1"/>
  </joint>
</robot>
""",
        encoding="utf-8",
    )
    instances = (
        IsaacArticulationInstanceSpec("right", translation=(0.0, -0.3, 0.0)),
        IsaacArticulationInstanceSpec("left", translation=(0.0, 0.3, 0.0)),
    )

    composed = compose_urdf_instances(source, instances, cache_dir=tmp_path / "cache")
    root = ET.parse(composed).getroot()
    links = {link.attrib["name"] for link in root.findall("link")}
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}

    assert links == {"world", "right_base", "right_tip", "left_base", "left_tip"}
    assert set(joints) == {
        "right_axis",
        "left_axis",
        "world_to_right_base",
        "world_to_left_base",
    }
    assert joints["right_axis"].find("parent").attrib["link"] == "right_base"
    assert joints["left_axis"].find("child").attrib["link"] == "left_tip"
    assert joints["world_to_right_base"].find("origin").attrib["xyz"] == "0.0 -0.3 0.0"
    assert joints["world_to_left_base"].find("origin").attrib["xyz"] == "0.0 0.3 0.0"


def test_augment_urdf_planar_base_adds_global_xy_yaw_chain(tmp_path):
    source = tmp_path / "mobile.urdf"
    source.write_text(
        """<?xml version="1.0"?>
<robot name="mobile">
  <link name="base"/>
  <link name="tip"/>
  <joint name="arm" type="revolute">
    <parent link="base"/>
    <child link="tip"/>
    <axis xyz="0 0 1"/>
    <limit effort="1" lower="-1" upper="1" velocity="1"/>
  </joint>
</robot>
""",
        encoding="utf-8",
    )
    spec = IsaacPlanarBaseSpec(joint_names=("base_x", "base_y", "base_yaw"))

    augmented = augment_urdf_planar_base(source, spec, cache_dir=tmp_path / "cache")
    root = ET.parse(augmented).getroot()
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}

    assert joints["base_x"].attrib["type"] == "prismatic"
    assert joints["base_x"].find("axis").attrib["xyz"] == "1 0 0"
    assert joints["base_y"].find("axis").attrib["xyz"] == "0 1 0"
    assert joints["base_yaw"].attrib["type"] == "revolute"
    assert joints["base_yaw"].find("axis").attrib["xyz"] == "0 0 1"
    assert joints["spark_planar_root_mount"].find("child").attrib["link"] == "base"
    assert joints["arm"].find("parent").attrib["link"] == "base"
    for name in ("spark_planar_x_link", "spark_planar_y_link", "spark_planar_yaw_link"):
        link = root.find(f"link[@name='{name}']")
        assert float(link.find("inertial/mass").attrib["value"]) > 0.0


def test_planar_isaac_feedback_reports_the_moving_robot_base_frame():
    config = AgiBotG1MobileBaseDynamic1Config()
    dof_pos = np.zeros(len(config.DoFs))
    dof_pos[config.DoFs.LinearX] = 0.4
    dof_pos[config.DoFs.LinearY] = -0.2
    dof_pos[config.DoFs.RotYaw] = np.pi / 2.0

    frame = compose_planar_robot_base_frame(np.eye(4), dof_pos, config)

    np.testing.assert_allclose(frame[:2, 3], [0.4, -0.2], atol=1e-12)
    np.testing.assert_allclose(frame[:2, :2], [[0.0, -1.0], [1.0, 0.0]], atol=1e-12)


def test_kuka_fixed_asset_alignment_is_not_reported_as_logical_base_motion():
    calibration = np.eye(4)
    calibration[:3, :3] = np.array(
        [
            [0.0, 1.0, 0.0],
            [-1.0, 0.0, 0.0],
            [0.0, 0.0, 1.0],
        ]
    )

    logical = remove_asset_alignment_from_base_frame(
        calibration,
        (0.0, 0.0, -np.pi / 2.0),
    )

    np.testing.assert_allclose(logical, np.eye(4), atol=1.0e-12)
