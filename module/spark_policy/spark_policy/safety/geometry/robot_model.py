"""Build tensor collision models from existing SPARK robot configuration."""

from __future__ import annotations

import numpy as np


def _frame_name(frame) -> str:
    return str(getattr(frame, "name", frame))


def _candidate_body_names(frame_name: str) -> tuple[str, ...]:
    candidates = [frame_name]
    if frame_name.endswith("_joint"):
        candidates.insert(0, frame_name.removesuffix("_joint") + "_link")
    if frame_name == "L_ee":
        candidates.extend(("left_wrist_yaw_link", "left_hand_palm_link"))
    elif frame_name == "R_ee":
        candidates.extend(("right_wrist_yaw_link", "right_hand_palm_link"))
    elif frame_name == "waist_pitch_joint" or frame_name.startswith("torso_link_"):
        candidates.insert(0, "torso_link")
    elif frame_name.startswith("pelvis_link_"):
        candidates.insert(0, "waist_yaw_link")
    return tuple(candidates)


def _local_offset(frame_name: str):
    if frame_name in ("L_ee", "R_ee"):
        return (0.1, 0.0, 0.0)
    if frame_name.startswith("torso_link_"):
        index = int(frame_name.rsplit("_", 1)[1])
        return (0.0, 0.0, (0.1, 0.2, 0.4)[index - 1])
    if frame_name == "pelvis_link_2":
        return (0.0, 0.15, 0.0)
    if frame_name == "pelvis_link_3":
        return (0.0, -0.15, 0.0)
    return (0.0, 0.0, 0.0)


def build_link_sphere_model(robot_cfg, body_names, *, device="cuda:0"):
    """Map configured collision frames onto Isaac/canonical rigid bodies.

    Synthetic visualization-only frames without a simulator body are skipped.
    Their omission is reported by ``unmapped_frames`` on the returned model.
    """
    import torch

    from .types import LinkSphereModel

    name_to_id = {name: index for index, name in enumerate(body_names)}
    explicit_body_names = getattr(robot_cfg, "CollisionVolBodyNames", {})
    explicit_local_offsets = getattr(robot_cfg, "CollisionVolLocalOffsets", {})
    records = []
    unmapped = []
    for frame, geometry in robot_cfg.CollisionVol.items():
        frame_name = _frame_name(frame)
        declared_body_name = explicit_body_names.get(frame)
        body_name = (
            declared_body_name
            if declared_body_name in name_to_id
            else next(
                (name for name in _candidate_body_names(frame_name) if name in name_to_id),
                None,
            )
        )
        if body_name is None:
            unmapped.append(frame_name)
            continue
        local_offset = explicit_local_offsets.get(frame, _local_offset(frame_name))
        records.append((frame, frame_name, body_name, float(geometry.size[0]), local_offset))
    if not records:
        raise ValueError("No configured robot collision frames map to simulator bodies")

    frame_to_model = {frame: index for index, (frame, *_rest) in enumerate(records)}
    frame_name_to_model = {
        frame_name: index for index, (_frame, frame_name, *_rest) in enumerate(records)
    }
    ignored_env = {_frame_name(frame) for frame in getattr(robot_cfg, "EnvCollisionVolIgnored", [])}
    ignored_self = {
        _frame_name(frame) for frame in getattr(robot_cfg, "SelfCollisionVolIgnored", [])
    }
    adjacent = {
        frozenset((_frame_name(first), _frame_name(second)))
        for first, second in getattr(robot_cfg, "AdjacentCollisionVolPairs", [])
    }

    pair_i = []
    pair_j = []
    for i, (_fi, name_i, _bi, _ri, _oi) in enumerate(records):
        if name_i in ignored_self:
            continue
        for j in range(i + 1, len(records)):
            name_j = records[j][1]
            if name_j in ignored_self or frozenset((name_i, name_j)) in adjacent:
                continue
            pair_i.append(i)
            pair_j.append(j)

    model = LinkSphereModel(
        body_names=tuple(record[2] for record in records),
        body_ids=torch.tensor(
            [name_to_id[record[2]] for record in records],
            device=device,
            dtype=torch.long,
        ),
        local_offsets=torch.as_tensor(
            np.asarray([record[4] for record in records], dtype=float),
            device=device,
            dtype=torch.float32,
        ),
        radii=torch.tensor([record[3] for record in records], device=device, dtype=torch.float32),
        environment_mask=torch.tensor(
            [record[1] not in ignored_env for record in records],
            device=device,
            dtype=torch.bool,
        ),
        self_pair_i=torch.tensor(pair_i, device=device, dtype=torch.long),
        self_pair_j=torch.tensor(pair_j, device=device, dtype=torch.long),
    )
    # Frozen dataclass fields contain tensors, while these diagnostics are
    # intentionally metadata rather than part of its numerical contract.
    object.__setattr__(model, "unmapped_frames", tuple(unmapped))
    object.__setattr__(model, "frame_name_to_model", frame_name_to_model)
    object.__setattr__(model, "frame_to_model", frame_to_model)
    return model
