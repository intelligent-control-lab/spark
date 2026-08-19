"""Sparse iiwa 14 collision geometry shared by MuJoCo and Isaac."""

from __future__ import annotations

from enum import IntEnum
from itertools import combinations
import json
from pathlib import Path

from spark_utils import Geometry, VizColor


_END_EFFECTOR_OFFSET = (0.0, 0.0, 0.18)
_HOME_FALSE_POSITIVE_PAIRS = {
    frozenset(("iiwa_link_5_sphere_0", "iiwa_link_7_sphere_0")),
}


def _sphere_records():
    records = json.loads(
        Path(__file__)
        .with_name("kuka_iiwa14_single_arm_collision_spheres.json")
        .read_text(encoding="utf-8")
    )
    for key, configurations in records.items():
        link_name = key.split("::", 1)[0]
        for index, sphere in enumerate(configurations["20"]["1"]["spheres"]):
            yield link_name, index, sphere


def arm_collision_definition(*, dual_arm: bool):
    """Build the moderate-density collision model for one or two arms."""

    sides = ("right", "left") if dual_arm else (None,)
    member_names = ["R_ee", *(("L_ee",) if dual_arm else ())]
    spheres = []
    for side in sides:
        for link_name, index, sphere in _sphere_records():
            prefix = f"{side}_" if side else ""
            frame_name = f"{prefix}{link_name}_sphere_{index}"
            member_names.append(frame_name)
            spheres.append((frame_name, side, link_name, sphere))

    frames = IntEnum("Frames", {name: index for index, name in enumerate(member_names)})
    volumes = {frames.R_ee: Geometry(type="sphere", radius=0.05, color=VizColor.collision_volume)}
    body_names = {frames.R_ee: "right_iiwa_link_7" if dual_arm else "iiwa_link_7"}
    local_offsets = {frames.R_ee: _END_EFFECTOR_OFFSET}
    if dual_arm:
        volumes[frames.L_ee] = Geometry(type="sphere", radius=0.05, color=VizColor.collision_volume)
        body_names[frames.L_ee] = "left_iiwa_link_7"
        local_offsets[frames.L_ee] = _END_EFFECTOR_OFFSET

    for frame_name, side, link_name, sphere in spheres:
        frame = frames[frame_name]
        volumes[frame] = Geometry(type="sphere", radius=float(sphere["radius"]))
        body_names[frame] = f"{side}_{link_name}" if side else link_name
        local_offsets[frame] = tuple(float(value) for value in sphere["origin"])
    return frames, volumes, body_names, local_offsets


def single_arm_collision_mappings(frames):
    """Map the generated single-arm enum to Isaac link-local sphere data."""

    body_names = {frames.R_ee: "iiwa_link_7"}
    local_offsets = {frames.R_ee: _END_EFFECTOR_OFFSET}
    for link_name, index, sphere in _sphere_records():
        frame = frames[f"{link_name}_sphere_{index}"]
        body_names[frame] = link_name
        local_offsets[frame] = tuple(float(value) for value in sphere["origin"])
    return body_names, local_offsets


def self_collision_ignored_pairs(frames, body_names):
    """Ignore only same/neighbor links and one qualified-home FOAM overlap."""

    ignored = []
    for frame_a, frame_b in combinations(body_names, 2):
        body_a = body_names[frame_a]
        body_b = body_names[frame_b]
        side_a = next((side for side in ("right", "left") if body_a.startswith(f"{side}_")), None)
        side_b = next((side for side in ("right", "left") if body_b.startswith(f"{side}_")), None)
        link_a = body_a.removeprefix(f"{side_a}_") if side_a else body_a
        link_b = body_b.removeprefix(f"{side_b}_") if side_b else body_b
        index_a = int(link_a.rsplit("_", 1)[1])
        index_b = int(link_b.rsplit("_", 1)[1])
        same_arm = side_a == side_b
        topology_neighbor = same_arm and abs(index_a - index_b) <= 1

        name_a = frame_a.name.removeprefix(f"{side_a}_") if side_a else frame_a.name
        name_b = frame_b.name.removeprefix(f"{side_b}_") if side_b else frame_b.name
        qualified_home_overlap = same_arm and frozenset((name_a, name_b)) in (
            _HOME_FALSE_POSITIVE_PAIRS
        )
        if topology_neighbor or qualified_home_overlap:
            ignored.append([frame_a, frame_b])
    return ignored
