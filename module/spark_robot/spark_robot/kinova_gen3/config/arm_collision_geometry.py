"""Sparse Gen3 arm collision geometry shared by MuJoCo and Isaac."""

from __future__ import annotations

from enum import IntEnum
import json
from pathlib import Path

from spark_utils import Geometry, VizColor


_END_EFFECTOR_OFFSET = (0.0, 0.0, -0.2)


def _sphere_records():
    records = json.loads(
        Path(__file__)
        .with_name("kinova_gen3_single_arm_collision_spheres.json")
        .read_text(encoding="utf-8")
    )
    for key, configurations in records.items():
        link_name = key.split("::", 1)[0]
        # Level 1 is the existing moderate-density representation: it follows
        # long links without making batched distance checks unnecessarily dense.
        for index, sphere in enumerate(configurations["8"]["1"]["spheres"]):
            yield link_name, index, sphere


def arm_collision_definition(*, dual_arm: bool):
    """Build the moderate-density sphere set for one or two Gen3 arms."""

    sides = ("right", "left") if dual_arm else (None,)
    member_names = ["R_ee", *(("L_ee",) if dual_arm else ())]
    spheres = []
    for side in sides:
        for link_name, index, sphere in _sphere_records():
            if dual_arm and link_name == "base_link":
                # Isaac merges each fixed base into its composed articulation
                # root. The bases are immobile and below the benchmark
                # workspace, so model the moving arm links consistently.
                continue
            prefix = f"{side}_" if side else ""
            frame_name = f"{prefix}{link_name}_sphere_{index}"
            member_names.append(frame_name)
            spheres.append((frame_name, side, link_name, sphere))

    frames = IntEnum("Frames", {name: index for index, name in enumerate(member_names)})
    volumes = {frames.R_ee: Geometry(type="sphere", radius=0.05, color=VizColor.collision_volume)}
    body_names = {frames.R_ee: "right_bracelet_link" if dual_arm else "bracelet_link"}
    local_offsets = {frames.R_ee: _END_EFFECTOR_OFFSET}
    if dual_arm:
        volumes[frames.L_ee] = Geometry(type="sphere", radius=0.05, color=VizColor.collision_volume)
        body_names[frames.L_ee] = "left_bracelet_link"
        local_offsets[frames.L_ee] = _END_EFFECTOR_OFFSET

    for frame_name, side, link_name, sphere in spheres:
        frame = frames[frame_name]
        volumes[frame] = Geometry(type="sphere", radius=float(sphere["radius"]))
        body_names[frame] = f"{side}_{link_name}" if side else link_name
        local_offsets[frame] = tuple(float(value) for value in sphere["origin"])
    return frames, volumes, body_names, local_offsets


def single_arm_collision_mappings(frames):
    """Map the generated single-arm frame enum to Isaac link-local data."""

    body_names = {frames.R_ee: "bracelet_link"}
    local_offsets = {frames.R_ee: _END_EFFECTOR_OFFSET}
    for link_name, index, sphere in _sphere_records():
        frame = frames[f"{link_name}_sphere_{index}"]
        body_names[frame] = link_name
        local_offsets[frame] = tuple(float(value) for value in sphere["origin"])
    return body_names, local_offsets
