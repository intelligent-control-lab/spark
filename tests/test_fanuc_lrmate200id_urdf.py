from __future__ import annotations

import hashlib
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

import spark_robot


RESOURCE_DIR = Path(spark_robot.SPARK_ROBOT_RESOURCE_DIR) / "fanuc_lrmate200id"
URDF_PATH = RESOURCE_DIR / "lrmate200id7l.urdf"
DUAL_URDF_PATH = RESOURCE_DIR / "lrmate200id7l_dual.urdf"

EXPECTED_JOINTS = {
    "joint_1": ((0.0, 0.0, 0.330), (0.0, 0.0, 1.0), (-3.141592654, 3.141592654)),
    "joint_2": ((0.050, 0.0, 0.0), (0.0, 1.0, 0.0), (-2.138028334, 2.138028334)),
    "joint_3": ((0.0, 0.0, 0.440), (0.0, -1.0, 0.0), (-3.752457892, 3.752457892)),
    "joint_4": ((0.0, 0.0, 0.035), (-1.0, 0.0, 0.0), (-3.316125579, 3.316125579)),
    "joint_5": ((0.420, 0.0, 0.0), (0.0, -1.0, 0.0), (-2.181661565, 2.181661565)),
    "joint_6": ((0.080, 0.0, 0.0), (-1.0, 0.0, 0.0), (-6.283185307, 6.283185307)),
}
EXPECTED_GRIPPER_JOINTS = {
    f"spark_robotiq_{finger}_joint_{index}" for finger in ("A", "B", "C") for index in (1, 2, 3)
}

# These hashes identify the ROS-Industrial noetic-devel assets documented in
# the resource README. Link 2 and link 4 are the 7L-specific meshes.
EXPECTED_MESH_HASHES = {
    "base_link.stl": "e7d6f17f3012445ab8bacd18885db9437646b131253b92ac7f880dcc03267642",
    "link_1.stl": "353992336fd3daaca21d8adc28bcc5b4e64052df262f8e40738c1b147bc44db3",
    "link_2.stl": "89a761f3d0c7a724ceca75390a6ec75aa9103659220e200fc4357cbd737a9be6",
    "link_3.stl": "dc1a5d033ea34ca5fdc1b1305eec3993fa63afb715e69e9ee934315de2fa07ed",
    "link_4.stl": "26f7f2b13f3f86f09888203c10a51d5f3f7fb2f22b710575d1dd2d6769814e82",
    "link_5.stl": "4ece3a0851d58a1bbf50a917b80659ad6a3b90083f7def9db6fd230514b0dc5f",
    "link_6.stl": "77b9b50f1c2edfd5384aa88b2371807640e638e5d2b597b5c7ecffe9a897a720",
}


def _numbers(value: str) -> tuple[float, ...]:
    return tuple(float(number) for number in value.split())


def test_fanuc_urdf_matches_mujoco_serial_chain_contract():
    root = ET.parse(URDF_PATH).getroot()
    movable = {
        joint.attrib["name"]: joint
        for joint in root.findall("joint")
        if joint.attrib.get("type") != "fixed"
    }
    assert set(EXPECTED_JOINTS) <= set(movable)
    assert set(movable) - set(EXPECTED_JOINTS) == EXPECTED_GRIPPER_JOINTS

    for name, (origin, axis, limits) in EXPECTED_JOINTS.items():
        joint = movable[name]
        np.testing.assert_allclose(_numbers(joint.find("origin").attrib["xyz"]), origin)
        np.testing.assert_allclose(_numbers(joint.find("axis").attrib["xyz"]), axis)
        limit = joint.find("limit")
        np.testing.assert_allclose(
            (float(limit.attrib["lower"]), float(limit.attrib["upper"])), limits
        )
        assert float(limit.attrib["effort"]) > 0.0
        assert float(limit.attrib["velocity"]) > 0.0


def test_fanuc_urdf_meshes_are_resolvable_ros_industrial_assets():
    root = ET.parse(URDF_PATH).getroot()
    referenced = {mesh.attrib["filename"] for mesh in root.findall(".//mesh")}
    arm_meshes = {f"./meshes/lrmate200id/{name}" for name in EXPECTED_MESH_HASHES}
    assert {reference for reference in referenced if "lrmate200id" in reference} == arm_meshes
    gripper_visual_meshes = {f"./meshes/robotiq_3f/link_{index}_vis.stl" for index in range(4)} | {
        "./meshes/robotiq_3f/palm_vis.stl"
    }
    gripper_collision_meshes = {f"./meshes/robotiq_3f/link_{index}.STL" for index in range(4)} | {
        "./meshes/robotiq_3f/palm.STL"
    }
    assert referenced - arm_meshes == gripper_visual_meshes | gripper_collision_meshes
    assert all((RESOURCE_DIR / reference).is_file() for reference in referenced)

    mesh_dir = RESOURCE_DIR / "meshes" / "lrmate200id"
    for name, expected_hash in EXPECTED_MESH_HASHES.items():
        mesh_path = mesh_dir / name
        assert mesh_path.is_file()
        assert hashlib.sha256(mesh_path.read_bytes()).hexdigest() == expected_hash

    assert (RESOURCE_DIR / "README.md").is_file()
    assert (RESOURCE_DIR / "LICENSE.ros_industrial_fanuc").is_file()


def test_all_fanuc_single_arm_configs_use_the_shared_isaac_asset():
    config_types = (
        spark_robot.FanucLRMate200iDSingleArmDynamic1Config,
        spark_robot.FanucLRMate200iDSingleArmDynamic1CollisionConfig,
        spark_robot.FanucLRMate200iDSingleArmDynamic2Config,
        spark_robot.FanucLRMate200iDSingleArmDynamic2CollisionConfig,
    )
    for config_type in config_types:
        config = config_type()
        assert config.agent_class_names["isaac"] == "FanucLRMate200iDSingleArmIsaacAgent"
        assert config.isaac_articulation.urdf_path == "fanuc_lrmate200id/lrmate200id7l.urdf"
        assert config.isaac_articulation.joint_names == tuple(EXPECTED_JOINTS)
        assert config.isaac_articulation.allow_self_collision is True
        assert config.isaac_articulation.stiffness == 4000.0
        assert config.isaac_articulation.damping == 250.0


def test_fanuc_gripper_uses_official_opposing_finger_layout_and_physical_shapes():
    root = ET.parse(URDF_PATH).getroot()
    links = {link.attrib["name"]: link for link in root.findall("link")}
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}

    # Finger A is the fixed opposing finger in the Robotiq 3F layout. Fingers
    # B/C are the paired fingers on the other side of the palm.
    np.testing.assert_allclose(
        _numbers(joints["spark_robotiq_A_mount"].find("origin").attrib["xyz"]),
        (0.0455, 0.0214, 0.0),
    )
    np.testing.assert_allclose(
        _numbers(joints["spark_robotiq_A_mount"].find("origin").attrib["rpy"]),
        (0.0, 0.0, np.pi / 2),
    )
    for finger, z in (("B", -0.036), ("C", 0.036)):
        origin = joints[f"spark_robotiq_{finger}_mount"].find("origin")
        np.testing.assert_allclose(_numbers(origin.attrib["xyz"]), (-0.0455, 0.0214, z))
        np.testing.assert_allclose(_numbers(origin.attrib["rpy"]), (0.0, np.pi, -np.pi / 2))

    expected_collision_mesh = {"spark_robotiq_palm": "palm.STL"}
    expected_collision_mesh.update(
        {
            f"spark_robotiq_{finger}_link_{index}": f"link_{index}.STL"
            for finger in ("A", "B", "C")
            for index in range(4)
        }
    )
    for link_name, mesh_name in expected_collision_mesh.items():
        collision_mesh = links[link_name].find("collision/geometry/mesh")
        assert collision_mesh is not None, link_name
        assert collision_mesh.attrib["filename"].endswith(f"/robotiq_3f/{mesh_name}")


def test_fanuc_self_collision_masks_accept_the_qualified_home_poses():
    cases = (
        (
            spark_robot.FanucLRMate200iDSingleArmDynamic1CollisionConfig,
            spark_robot.FanucLRMate200iDSingleArmKinematics,
        ),
        (
            spark_robot.FanucLRMate200iDSingleArmDynamic2CollisionConfig,
            spark_robot.FanucLRMate200iDSingleArmKinematics,
        ),
        (
            spark_robot.FanucLRMate200iDDualArmDynamic1CollisionConfig,
            spark_robot.FanucLRMate200iDDualArmKinematics,
        ),
    )
    for config_type, kinematics_type in cases:
        config = config_type()
        kinematics = kinematics_type(config)
        dof_position = np.asarray([config.DefaultDoFVal[dof] for dof in config.DoFs], dtype=float)
        frames = kinematics.forward_kinematics(dof_position)
        ignored = {frozenset(pair) for pair in config.AdjacentCollisionVolPairs}
        active_clearances = []
        collision_frames = list(config.CollisionVol)
        for index, frame_a in enumerate(collision_frames):
            for frame_b in collision_frames[index + 1 :]:
                if frozenset((frame_a, frame_b)) in ignored:
                    continue
                center_distance = np.linalg.norm(frames[frame_a][:3, 3] - frames[frame_b][:3, 3])
                active_clearances.append(
                    center_distance
                    - config.CollisionVol[frame_a].size[0]
                    - config.CollisionVol[frame_b].size[0]
                )

        assert active_clearances
        assert min(active_clearances) >= -1.0e-9


def test_fanuc_dual_urdf_reuses_both_chains_at_mujoco_mounts():
    root = ET.parse(DUAL_URDF_PATH).getroot()
    joints = {joint.attrib["name"]: joint for joint in root.findall("joint")}
    assert _numbers(joints["world_to_right_base_link"].find("origin").attrib["xyz"]) == (
        0.0,
        -0.45,
        0.0,
    )
    assert _numbers(joints["world_to_left_base_link"].find("origin").attrib["xyz"]) == (
        0.0,
        0.45,
        0.0,
    )

    for side in ("right", "left"):
        for index, (origin, axis, limits) in enumerate(EXPECTED_JOINTS.values(), start=1):
            joint = joints[f"{side}_joint_{index}"]
            np.testing.assert_allclose(_numbers(joint.find("origin").attrib["xyz"]), origin)
            np.testing.assert_allclose(_numbers(joint.find("axis").attrib["xyz"]), axis)
            limit = joint.find("limit")
            np.testing.assert_allclose(
                (float(limit.attrib["lower"]), float(limit.attrib["upper"])), limits
            )

    config = spark_robot.FanucLRMate200iDDualArmDynamic1Config()
    assert config.agent_class_names["isaac"] == "FanucLRMate200iDDualArmIsaacAgent"
    assert config.isaac_articulation.urdf_path.endswith("lrmate200id7l.urdf")
    assert config.isaac_articulation.joint_names == tuple(
        f"{side}_joint_{index}" for side in ("right", "left") for index in range(1, 7)
    )
    assert tuple(instance.prefix for instance in config.isaac_articulation.instances) == (
        "right",
        "left",
    )
    assert tuple(instance.translation for instance in config.isaac_articulation.instances) == (
        (0.0, -0.45, 0.0),
        (0.0, 0.45, 0.0),
    )
    assert config.isaac_articulation.allow_self_collision is True
    assert [gripper.side for gripper in config.isaac_articulation.grippers] == [
        "left",
        "right",
    ]
