"""Sparse R1 Lite arm collision geometry shared by scalar and tensor backends."""

from __future__ import annotations

from enum import IntEnum
import json
from pathlib import Path

from spark_utils import Geometry


_COMMON_LINKS = (
    "torso_link1",
    "torso_link2",
    "torso_link3",
    "camera_head_left_link",
)
_ARM_LINK_SUFFIXES = (
    "arm_base_link",
    "arm_link1",
    "arm_link2",
    "arm_link3",
    "arm_link4",
    "arm_link5",
    "arm_link6",
    "gripper_link",
    "D405_link",
)


def sparse_arm_collision_definition(*, dual_arm: bool):
    """Build the moderate-density sphere set used by arm-only workflows."""

    json_path = Path(__file__).with_name("galaxea_r1lite_collision_spheres.json")
    records = json.loads(json_path.read_text(encoding="utf-8"))
    selected_links = list(_COMMON_LINKS)
    sides = ("left", "right") if dual_arm else ("right",)
    for side in sides:
        selected_links.extend(f"{side}_{suffix}" for suffix in _ARM_LINK_SUFFIXES)

    member_names = ["R_ee"]
    if dual_arm:
        member_names.append("L_ee")
    spheres = []
    for key, configurations in records.items():
        link_name = key.split("::", 1)[0]
        if link_name not in selected_links:
            continue
        # Level 1 uses several small spheres on long links while keeping the
        # arm-only model compact (39 volumes for one arm, 62 for two arms).
        for index, sphere in enumerate(configurations["20"]["1"]["spheres"]):
            frame_name = f"{link_name}_sphere_{index}"
            member_names.append(frame_name)
            spheres.append((frame_name, link_name, sphere))

    frames = IntEnum("Frames", {name: index for index, name in enumerate(member_names)})
    volumes = {frames.R_ee: Geometry(type="sphere", radius=0.05)}
    body_names = {frames.R_ee: "right_gripper_link"}
    local_offsets = {frames.R_ee: (0.06835, 0.0, 0.0)}
    if dual_arm:
        volumes[frames.L_ee] = Geometry(type="sphere", radius=0.05)
        body_names[frames.L_ee] = "left_gripper_link"
        local_offsets[frames.L_ee] = (0.06835, 0.0, 0.0)
    for frame_name, link_name, sphere in spheres:
        frame = frames[frame_name]
        volumes[frame] = Geometry(type="sphere", radius=float(sphere["radius"]))
        body_names[frame] = link_name
        local_offsets[frame] = tuple(float(value) for value in sphere["origin"])
    return frames, volumes, body_names, local_offsets


def sparse_whole_body_collision_definition():
    """Build the moderate-density whole-body set used by mobile workflows."""

    json_path = Path(__file__).with_name("galaxea_r1lite_collision_spheres.json")
    records = json.loads(json_path.read_text(encoding="utf-8"))
    member_names = ["R_ee", "L_ee"]
    spheres = []
    for key, configurations in records.items():
        link_name = key.split("::", 1)[0]
        for index, sphere in enumerate(configurations["20"]["1"]["spheres"]):
            frame_name = f"{link_name}_sphere_{index}"
            member_names.append(frame_name)
            spheres.append((frame_name, link_name, sphere))

    frames = IntEnum("Frames", {name: index for index, name in enumerate(member_names)})
    volumes = {
        frames.R_ee: Geometry(type="sphere", radius=0.05),
        frames.L_ee: Geometry(type="sphere", radius=0.05),
    }
    body_names = {
        frames.R_ee: "right_gripper_link",
        frames.L_ee: "left_gripper_link",
    }
    local_offsets = {
        frames.R_ee: (0.06835, 0.0, 0.0),
        frames.L_ee: (0.06835, 0.0, 0.0),
    }
    for frame_name, link_name, sphere in spheres:
        frame = frames[frame_name]
        volumes[frame] = Geometry(type="sphere", radius=float(sphere["radius"]))
        body_names[frame] = link_name
        local_offsets[frame] = tuple(float(value) for value in sphere["origin"])
    return frames, volumes, body_names, local_offsets


def whole_body_collision_mappings(frames):
    """Map an existing whole-body frame enum to its link-local sphere data."""

    json_path = Path(__file__).with_name("galaxea_r1lite_collision_spheres.json")
    records = json.loads(json_path.read_text(encoding="utf-8"))
    body_names = {
        frames.R_ee: "right_gripper_link",
        frames.L_ee: "left_gripper_link",
    }
    local_offsets = {
        frames.R_ee: (0.06835, 0.0, 0.0),
        frames.L_ee: (0.06835, 0.0, 0.0),
    }
    for key, configurations in records.items():
        link_name = key.split("::", 1)[0]
        for index, sphere in enumerate(configurations["20"]["1"]["spheres"]):
            frame = frames[f"{link_name}_sphere_{index}"]
            body_names[frame] = link_name
            local_offsets[frame] = tuple(float(value) for value in sphere["origin"])
    return body_names, local_offsets
