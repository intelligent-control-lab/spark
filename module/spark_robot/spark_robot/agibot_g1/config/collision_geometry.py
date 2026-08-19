"""FOAM-generated whole-body collision geometry for the AgiBot G1.

The source database is generated from the bundled AgiBot URDF.  Runtime
collision geometry uses the depth-one FOAM approximation for the chassis,
torso, head, arms, and gripper bases.  Individual finger meshes are omitted
because the end-effector spheres conservatively cover the grasping region.
"""

from enum import IntEnum
import json
from pathlib import Path

import numpy as np
import pinocchio as pin

from spark_robot import SPARK_ROBOT_RESOURCE_DIR
from spark_utils import Geometry, VizColor


_DATABASE_PATH = Path(SPARK_ROBOT_RESOURCE_DIR) / "agibot_g1" / "agibot_g1_collision_spheres.json"
_BODY_LINKS = (
    "base_link",
    "link_up_down_body",
    "link_pitch_body",
    "link_yaw_head",
    "link_pitch_head",
    "base_link_l",
    *(f"Link{index}_l" for index in range(1, 8)),
    "left_base_link",
    "base_link_r",
    *(f"Link{index}_r" for index in range(1, 8)),
    "right_base_link",
)

# Keep FOAM's sparse 118-sphere topology while removing the visibly excessive
# conservatism of its depth-one enclosing radii.  These factors calibrate the
# runtime envelope by region without increasing collision-check complexity.
_RADIUS_SCALE_BY_LINK = {
    "base_link": 0.82,
    "link_up_down_body": 0.88,
    "link_pitch_body": 0.85,
    "link_yaw_head": 0.88,
    "link_pitch_head": 0.85,
}
_DEFAULT_BODY_RADIUS_SCALE = 0.90


def _load_foam_spheres():
    with _DATABASE_PATH.open(encoding="utf-8") as database_file:
        database = json.load(database_file)

    by_link = {
        mesh_key.split("::", maxsplit=1)[0]: branch_data
        for mesh_key, branch_data in database.items()
    }
    spheres = []
    for link_name in _BODY_LINKS:
        try:
            level = by_link[link_name]["8"]["1"]
        except KeyError as exc:
            raise ValueError(
                f"FOAM collision database is missing depth-one data for {link_name!r}"
            ) from exc
        for sphere_index, sphere in enumerate(level["spheres"]):
            radius_scale = _RADIUS_SCALE_BY_LINK.get(link_name, _DEFAULT_BODY_RADIUS_SCALE)
            spheres.append(
                (
                    f"collision_{link_name.lower()}_{sphere_index:02d}",
                    link_name,
                    np.asarray(sphere["origin"], dtype=float),
                    radius_scale * float(sphere["radius"]),
                )
            )
    return tuple(spheres)


AGIBOT_G1_FOAM_SPHERES = _load_foam_spheres()

# Keep the end effectors first: task and benchmark code intentionally discovers
# these frames by their stable names and expects right/left ordering.
AgiBotG1Frames = IntEnum(
    "AgiBotG1Frames",
    {
        "R_ee": 0,
        "L_ee": 1,
        **{
            frame_name: frame_index
            for frame_index, (frame_name, _, _, _) in enumerate(AGIBOT_G1_FOAM_SPHERES, start=2)
        },
    },
)

AGIBOT_G1_COLLISION_VOLUMES = {
    AgiBotG1Frames.R_ee: Geometry(type="sphere", radius=0.05, color=VizColor.collision_volume),
    AgiBotG1Frames.L_ee: Geometry(type="sphere", radius=0.05, color=VizColor.collision_volume),
    **{
        AgiBotG1Frames[frame_name]: Geometry(
            type="sphere", radius=radius, color=VizColor.collision_volume
        )
        for frame_name, _, _, radius in AGIBOT_G1_FOAM_SPHERES
    },
}

# Simulator tensor backends need the rigid body and body-local offset behind
# every configured collision sphere.  Keep that mapping next to the FOAM
# source records so Isaac, MuJoCo-side diagnostics, and the Pinocchio model all
# consume the same sparse geometry instead of reconstructing robot-specific
# names in a backend.
AGIBOT_G1_COLLISION_BODY_NAMES = {
    AgiBotG1Frames.R_ee: "right_base_link",
    AgiBotG1Frames.L_ee: "left_base_link",
    **{
        AgiBotG1Frames[frame_name]: link_name
        for frame_name, link_name, _, _ in AGIBOT_G1_FOAM_SPHERES
    },
}
AGIBOT_G1_COLLISION_LOCAL_OFFSETS = {
    # The shared Pinocchio/MuJoCo R_ee and L_ee frames are 0.10 m along
    # Link7's local Z axis.  The gripper-base links are fixed to Link7 with an
    # identity transform, so Isaac must use the same local axis.
    AgiBotG1Frames.R_ee: np.array([0.0, 0.0, 0.10], dtype=float),
    AgiBotG1Frames.L_ee: np.array([0.0, 0.0, 0.10], dtype=float),
    **{
        AgiBotG1Frames[frame_name]: origin.copy()
        for frame_name, _, origin, _ in AGIBOT_G1_FOAM_SPHERES
    },
}


def add_agibot_g1_collision_frames(model):
    """Attach FOAM sphere centers to their corresponding Pinocchio link frame."""

    for frame_name, link_name, origin, _ in AGIBOT_G1_FOAM_SPHERES:
        if model.existFrame(frame_name):
            continue
        source_name = "robot" if link_name == "base_link" else link_name
        if not model.existFrame(source_name):
            raise ValueError(
                f"AgiBot kinematics model has no frame {source_name!r} "
                f"required by collision sphere {frame_name!r}"
            )
        source_frame = model.frames[model.getFrameId(source_name)]
        model.addFrame(
            pin.Frame(
                frame_name,
                source_frame.parentJoint,
                source_frame.placement * pin.SE3(np.eye(3), origin),
                pin.FrameType.OP_FRAME,
            )
        )
