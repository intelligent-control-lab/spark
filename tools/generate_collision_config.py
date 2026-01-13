#!/usr/bin/env python3
import argparse
import json
import re
import hashlib
from pathlib import Path
from types import SimpleNamespace
from collections import defaultdict
from itertools import combinations

import numpy as np
from spark_utils import initialize_class


# =============================================================================
# Constants
# =============================================================================

REQUIRED_IMPORTS = """\
from enum import IntEnum
import numpy as np
from spark_utils import Geometry
from spark_robot.base.base_robot_config import RobotConfig
"""


# =============================================================================
# Utilities
# =============================================================================

def normalize(name: str) -> str:
    return name.replace("::", "_").replace(".", "_").replace("/", "_")


def load_collision_json(path: str, depth: int = 1) -> dict:
    with open(path, "r") as f:
        data = json.load(f)

    out = {}
    for key, levels in data.items():
        link = normalize(key.split("::")[0])
        finest = max(levels.keys(), key=lambda x: int(x))
        cluster = levels[finest][str(depth)]
        out[link] = cluster.get("spheres", [])
    return out


# =============================================================================
# Regex helpers
# =============================================================================

FRAMES_RE = re.compile(
    r"(?ms)^([ \t]*)class\s+Frames\s*\(\s*IntEnum\s*\)\s*:\s*\n"
    r"(?P<body>.*?)(?=^[ \t]*(?:CollisionVol\s*=|Visualize|class\s+\w+)|\Z)"
)

FRAME_LINE_RE = re.compile(r"^\s*(\w+)\s*=\s*(\d+)", re.M)

ASSIGN_RE_TEMPLATE = r"(?ms)^[ \t]*{name}\s*=\s*(\{{.*?\}}|\[.*?\])"


def replace_or_insert_assignment(text, name, new_block, anchor="CollisionVol"):
    assign_re = re.compile(ASSIGN_RE_TEMPLATE.format(name=name))

    if assign_re.search(text):
        return assign_re.sub(new_block, text, count=1)

    anchor_re = re.compile(
        rf"(?ms)^[ \t]*{anchor}\s*=\s*\{{.*?\}}\s*"
    )
    m = anchor_re.search(text)
    if not m:
        raise RuntimeError(f"Cannot find anchor '{anchor}' to insert '{name}'")

    return text[:m.end()] + "\n\n" + new_block + text[m.end():]


# =============================================================================
# Frames parsing / generation
# =============================================================================

def parse_existing_frames(text):
    m = FRAMES_RE.search(text)
    if not m:
        raise RuntimeError("Frames enum not found")
    body = m.group("body")
    return [(n, int(i)) for n, i in FRAME_LINE_RE.findall(body)]


def gen_frames_block(existing, generated):
    lines = ["    class Frames(IntEnum):"]
    for name, idx in existing:
        lines.append(f"        {name} = {idx}")
    if generated:
        lines.append("")
        for name, idx in generated:
            lines.append(f"        {name} = {idx}")
    lines.append("")
    return "\n".join(lines)


# =============================================================================
# Collision blocks
# =============================================================================

def emit_collision_vol(existing_lines, generated_lines):
    lines = ["    CollisionVol = {"]
    for l in existing_lines:
        lines.append(f"        {l},")
    for l in generated_lines:
        lines.append(f"        {l},")
    lines.append("    }")
    return "\n".join(lines)


def emit_list_block(name, entries):
    if not entries:
        body = "[]"
    else:
        body = "[\n" + "\n".join(f"        {e}," for e in entries) + "\n    ]"
    return f"    {name} = {body}"


def parse_existing_collision_vol(text):
    m = re.search(r"(?ms)CollisionVol\s*=\s*\{(?P<body>.*?)\n\s*\}", text)
    if not m:
        return []
    return [
        line.strip().rstrip(",")
        for line in m.group("body").splitlines()
        if line.strip()
    ]


# =============================================================================
# Collision detection
# =============================================================================

def extract_centers(frames):
    return frames[:, :3, 3]


def check_sphere_collision(p1, r1, p2, r2):
    return np.linalg.norm(p1 - p2) <= (r1 + r2)


def auto_detect_ignored_collision_pairs(
    robot_cfg,
    robot_kinematics,
    num_samples=10000,
    collision_ratio_thresh=0.9,
    seed=0,
):
    rng = np.random.default_rng(seed)

    frames_enum = list(robot_cfg.Frames)
    volumes = list(robot_cfg.CollisionVol.values())
    radii = np.array([v.size[0] for v in volumes])
    names = [f.name for f in frames_enum]

    def link_id(name):
        return name.split("_sphere")[0]

    pair_hits = defaultdict(int)
    lower_limits = np.clip(
        robot_kinematics.reduced_fixed_base_model.lowerPositionLimit,
        -np.pi, np.pi,
    )
    upper_limits = np.clip(
        robot_kinematics.reduced_fixed_base_model.upperPositionLimit,
        -np.pi, np.pi,
    )
    for _ in range(num_samples):
        q = rng.uniform(
            lower_limits,
            upper_limits,
        )
        frames = robot_kinematics.forward_kinematics(q)
        centers = extract_centers(frames)

        for i, j in combinations(range(len(frames_enum)), 2):
            if link_id(names[i]) == link_id(names[j]):
                pair_hits[(i, j)] += 1
            if check_sphere_collision(
                centers[i], radii[i],
                centers[j], radii[j]
            ):
                pair_hits[(i, j)] += 1

    ignored = []
    for (i, j), c in pair_hits.items():
        if c / num_samples >= collision_ratio_thresh:
            ignored.append((frames_enum[i], frames_enum[j]))
    return ignored


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-config", default="module/spark_robot/spark_robot/r1lite/config/r1lite_upper_dynamic_1_config.py")
    parser.add_argument("--collision-json", default="module/spark_robot/spark_robot/r1lite/config/r1lite_collision_spheres.json")
    parser.add_argument("--output", default="module/spark_robot/spark_robot/r1lite/config/r1lite_upper_dynamic_1_collision_config.py")
    parser.add_argument("--class-name", default="R1LiteUpperDynamic1CollisionConfig")

    # parser.add_argument("--base-config", default="module/spark_robot/spark_robot/gen3/config/gen3_single_dynamic_1_config.py")
    # parser.add_argument("--collision-json", default="module/spark_robot/spark_robot/gen3/config/gen3_single_collision_spheres.json")
    # parser.add_argument("--output", default="module/spark_robot/spark_robot/gen3/config/gen3_single_dynamic_1_collision_config.py")
    # parser.add_argument("--class-name", default="Gen3SingleDynamic1CollisionConfig")
    args = parser.parse_args()

    base_text = Path(args.base_config).read_text()

    # Imports
    if "from spark_utils import Geometry" not in base_text:
        base_text = REQUIRED_IMPORTS + "\n\n" + base_text

    # Rename class
    cls_match = re.search(r"class\s+(\w+)\s*\(RobotConfig\):", base_text)
    base_class = cls_match.group(1)
    base_text = re.sub(
        rf"class\s+{base_class}\s*\(RobotConfig\):",
        f"class {args.class_name}(RobotConfig):",
        base_text,
        count=1,
    )

    # Frames
    existing_frames = parse_existing_frames(base_text)
    max_idx = max(i for _, i in existing_frames)

    collision = load_collision_json(args.collision_json, depth=1)
    new_frames = []
    new_collision_lines = []

    idx = max_idx + 1
    for link, spheres in collision.items():
        for i, s in enumerate(spheres):
            fname = f"{link}_sphere_{i}"
            new_frames.append((fname, idx))
            idx += 1
            new_collision_lines.append(
                f"Frames.{fname}: Geometry(type='sphere', radius={s['radius']})"
            )

    base_text = FRAMES_RE.sub(
        gen_frames_block(existing_frames, new_frames),
        base_text,
        count=1,
    )

    # First pass: CollisionVol only
    existing_collision = parse_existing_collision_vol(base_text)
    base_text = replace_or_insert_assignment(
        base_text,
        "CollisionVol",
        emit_collision_vol(existing_collision, new_collision_lines),
    )

    # Write once to allow import
    Path(args.output).write_text(base_text)

    # Second pass: adjacency
    robot_cfg = initialize_class(SimpleNamespace(class_name=args.class_name))
    robot_kin = initialize_class(
        SimpleNamespace(class_name=robot_cfg.kinematics_class_name),
        robot_cfg=robot_cfg,
    )

    ignored = auto_detect_ignored_collision_pairs(robot_cfg, robot_kin)
    adjacent = [
        f"[Frames.{a.name}, Frames.{b.name}]"
        for a, b in ignored
    ]

    base_text = replace_or_insert_assignment(
        base_text,
        "AdjacentCollisionVolPairs",
        emit_list_block("AdjacentCollisionVolPairs", adjacent),
    )
    base_text = replace_or_insert_assignment(
        base_text,
        "SelfCollisionVolIgnored",
        emit_list_block("SelfCollisionVolIgnored", []),
    )
    base_text = replace_or_insert_assignment(
        base_text,
        "EnvCollisionVolIgnored",
        emit_list_block("EnvCollisionVolIgnored", []),
    )

    header = (
        "# AUTO-GENERATED FILE\n"
        f"# Base config: {base_class}\n"
        f"# Collision JSON: {args.collision_json}\n"
        f"# JSON hash: {hashlib.md5(Path(args.collision_json).read_bytes()).hexdigest()[:16]}\n"
        "# DO NOT EDIT MANUALLY\n\n"
    )

    Path(args.output).write_text(header + base_text)
    print(f"[OK] Generated {args.output}")


if __name__ == "__main__":
    main()
