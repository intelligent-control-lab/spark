"""Sparse LR Mate collision geometry shared by MuJoCo and Isaac."""

from __future__ import annotations

from itertools import combinations
from enum import IntEnum
import json
from pathlib import Path

from spark_utils import Geometry, VizColor


_END_EFFECTOR_OFFSET = (0.2, 0.0, 0.0)
_LINK_ORDER = {
    name: index
    for index, name in enumerate(
        ("base_link", "link_1", "link_2", "link_3", "link_4", "link_5", "link_6")
    )
}
# The moderate FOAM cover slightly overlaps in the qualified folded home pose
# even though the corresponding LR Mate meshes do not. Ignore only these two
# sphere pairs instead of disabling all link-2/link-4 checks.
_HOME_FALSE_POSITIVE_PAIRS = {
    frozenset(("link_2_sphere_0", "link_4_sphere_0")),
    frozenset(("link_2_sphere_1", "link_4_sphere_0")),
    frozenset(("link_2_sphere_1", "link_4_sphere_1")),
    frozenset(("link_2_sphere_2", "link_4_sphere_1")),
    frozenset(("link_4_sphere_3", "link_6_sphere_0")),
    frozenset(("link_4_sphere_3", "link_6_sphere_1")),
    frozenset(("link_4_sphere_3", "link_6_sphere_2")),
    frozenset(("link_4_sphere_4", "link_6_sphere_0")),
    frozenset(("link_4_sphere_4", "link_6_sphere_1")),
    frozenset(("link_4_sphere_4", "link_6_sphere_2")),
}


def _sphere_records():
    records = json.loads(
        Path(__file__)
        .with_name("fanuc_lrmate200id_single_arm_collision_spheres.json")
        .read_text(encoding="utf-8")
    )
    for key, configurations in records.items():
        link_name = key.split("::", 1)[0]
        # Level 1 follows the arm links without making tensor distance checks dense.
        for index, sphere in enumerate(configurations["20"]["1"]["spheres"]):
            yield link_name, index, sphere


def arm_collision_definition(*, dual_arm: bool):
    """Build the moderate-density collision model for one or two arms."""

    sides = ("right", "left") if dual_arm else (None,)
    member_names = ["R_ee", *(("L_ee",) if dual_arm else ())]
    spheres = []
    for side in sides:
        for link_name, index, sphere in _sphere_records():
            if dual_arm and link_name == "base_link":
                # The fixed dual bases merge into the shared Isaac root and remain
                # below the benchmark workspace. Cover every moving link instead.
                continue
            prefix = f"{side}_" if side else ""
            frame_name = f"{prefix}{link_name}_sphere_{index}"
            member_names.append(frame_name)
            spheres.append((frame_name, side, link_name, sphere))

    frames = IntEnum("Frames", {name: index for index, name in enumerate(member_names)})
    volumes = {frames.R_ee: Geometry(type="sphere", radius=0.05, color=VizColor.collision_volume)}
    body_names = {frames.R_ee: "right_link_6" if dual_arm else "link_6"}
    local_offsets = {frames.R_ee: _END_EFFECTOR_OFFSET}
    if dual_arm:
        volumes[frames.L_ee] = Geometry(type="sphere", radius=0.05, color=VizColor.collision_volume)
        body_names[frames.L_ee] = "left_link_6"
        local_offsets[frames.L_ee] = _END_EFFECTOR_OFFSET

    for frame_name, side, link_name, sphere in spheres:
        frame = frames[frame_name]
        volumes[frame] = Geometry(type="sphere", radius=float(sphere["radius"]))
        body_names[frame] = f"{side}_{link_name}" if side else link_name
        local_offsets[frame] = tuple(float(value) for value in sphere["origin"])
    return frames, volumes, body_names, local_offsets


def single_arm_collision_mappings(frames):
    """Map the generated single-arm enum to Isaac link-local sphere data."""

    body_names = {frames.R_ee: "link_6"}
    local_offsets = {frames.R_ee: _END_EFFECTOR_OFFSET}
    for link_name, index, sphere in _sphere_records():
        frame = frames[f"{link_name}_sphere_{index}"]
        body_names[frame] = link_name
        local_offsets[frame] = tuple(float(value) for value in sphere["origin"])
    return body_names, local_offsets


def self_collision_ignored_pairs(frames, body_names):
    """Return topology and qualified-home exclusions for FANUC sphere checks."""

    ignored = []
    for frame_a, frame_b in combinations(body_names, 2):
        body_a = body_names[frame_a]
        body_b = body_names[frame_b]
        side_a = next((side for side in ("right", "left") if body_a.startswith(f"{side}_")), None)
        side_b = next((side for side in ("right", "left") if body_b.startswith(f"{side}_")), None)
        link_a = body_a.removeprefix(f"{side_a}_") if side_a else body_a
        link_b = body_b.removeprefix(f"{side_b}_") if side_b else body_b

        # Do not suppress any cross-arm pair. Within an arm, same-link and
        # directly connected-link contacts are structural rather than hazards.
        same_arm = side_a == side_b
        topology_neighbor = same_arm and abs(_LINK_ORDER[link_a] - _LINK_ORDER[link_b]) <= 1

        name_a = frame_a.name.removeprefix(f"{side_a}_") if side_a else frame_a.name
        name_b = frame_b.name.removeprefix(f"{side_b}_") if side_b else frame_b.name
        qualified_home_overlap = same_arm and frozenset((name_a, name_b)) in (
            _HOME_FALSE_POSITIVE_PAIRS
        )
        if topology_neighbor or qualified_home_overlap:
            ignored.append([frame_a, frame_b])
    return ignored
