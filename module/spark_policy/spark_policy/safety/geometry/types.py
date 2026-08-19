"""Backend-neutral tensor containers for collision queries.

The first backend uses PyTorch spheres.  The result contract deliberately
contains witness points and normals so mesh, point-cloud, and ESDF backends
can be added without changing safety-index code.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass
class PointCloudBatch:
    """Fixed-capacity, dynamically masked point-sphere environments."""

    positions: Any
    radii: Any
    valid_mask: Any
    velocities: Any | None = None
    object_ids: Any | None = None

    def validate(self) -> None:
        if self.positions.ndim != 3 or self.positions.shape[-1] != 3:
            raise ValueError("positions must have shape [batch, capacity, 3]")
        if self.radii.shape != self.positions.shape[:2]:
            raise ValueError("radii must have shape [batch, capacity]")
        if self.valid_mask.shape != self.positions.shape[:2]:
            raise ValueError("valid_mask must have shape [batch, capacity]")
        if self.velocities is not None and self.velocities.shape != self.positions.shape:
            raise ValueError("velocities must match positions")


class DynamicPointCloudBuffer:
    """Allocation-free point-sphere storage with per-environment counts.

    Perception can change valid counts every update while collision queries
    retain a fixed tensor shape suitable for CUDA graphs and fixed-size safety
    constraints. Capacity overflow is explicit and never silently truncated.
    """

    def __init__(self, batch_size: int, capacity: int, *, device="cuda:0", dtype=None):
        import torch

        dtype = torch.float32 if dtype is None else dtype
        self.batch_size = int(batch_size)
        self.capacity = int(capacity)
        self.positions = torch.zeros(batch_size, capacity, 3, device=device, dtype=dtype)
        self.radii = torch.zeros(batch_size, capacity, device=device, dtype=dtype)
        self.velocities = torch.zeros_like(self.positions)
        self.valid_mask = torch.zeros(batch_size, capacity, device=device, dtype=torch.bool)
        self.object_ids = torch.full((batch_size, capacity), -1, device=device, dtype=torch.long)
        self.overflow = torch.zeros(batch_size, device=device, dtype=torch.bool)

    def clear(self, env_ids=None) -> None:
        import torch

        if env_ids is None:
            env_ids = torch.arange(self.batch_size, device=self.positions.device, dtype=torch.long)
        else:
            env_ids = torch.as_tensor(env_ids, device=self.positions.device, dtype=torch.long)
        self.valid_mask[env_ids] = False
        self.object_ids[env_ids] = -1
        self.overflow[env_ids] = False

    def update_dense(
        self,
        positions,
        *,
        counts=None,
        radii=0.0,
        velocities=None,
        object_ids=None,
        env_ids=None,
    ) -> None:
        import torch

        positions = torch.as_tensor(
            positions, device=self.positions.device, dtype=self.positions.dtype
        )
        if positions.ndim != 3 or positions.shape[-1] != 3:
            raise ValueError("positions must have shape [selected_envs, points, 3]")
        selected = positions.shape[0]
        if env_ids is None:
            if selected != self.batch_size:
                raise ValueError("positions batch must equal buffer batch when env_ids is omitted")
            env_ids = torch.arange(selected, device=self.positions.device)
        else:
            env_ids = torch.as_tensor(env_ids, device=self.positions.device, dtype=torch.long)
            if env_ids.numel() != selected:
                raise ValueError("env_ids length must match positions batch")
        if counts is None:
            counts = torch.full(
                (selected,), positions.shape[1], device=self.positions.device, dtype=torch.long
            )
        else:
            counts = torch.as_tensor(counts, device=self.positions.device, dtype=torch.long)
        overflow = counts > self.capacity
        self.overflow[env_ids] = overflow
        if bool(torch.any(overflow).item()):
            bad = env_ids[overflow].detach().cpu().tolist()
            raise OverflowError(
                f"Point-cloud capacity {self.capacity} exceeded in environments {bad}"
            )
        width = min(positions.shape[1], self.capacity)
        self.clear(env_ids)
        self.positions[env_ids, :width] = positions[:, :width]
        index = torch.arange(self.capacity, device=self.positions.device)[None]
        self.valid_mask[env_ids] = index < counts[:, None]

        radius = torch.as_tensor(radii, device=self.positions.device, dtype=self.radii.dtype)
        if radius.ndim == 0:
            self.radii[env_ids] = radius
        else:
            self.radii[env_ids, :width] = radius[:, :width]
        if velocities is not None:
            velocity = torch.as_tensor(
                velocities, device=self.positions.device, dtype=self.positions.dtype
            )
            self.velocities[env_ids, :width] = velocity[:, :width]
        else:
            self.velocities[env_ids] = 0.0
        if object_ids is not None:
            ids = torch.as_tensor(object_ids, device=self.positions.device, dtype=torch.long)
            self.object_ids[env_ids, :width] = ids[:, :width]
        else:
            sequential = torch.arange(width, device=self.positions.device)
            self.object_ids[env_ids, :width] = sequential[None]

    def batch(self) -> PointCloudBatch:
        return PointCloudBatch(
            positions=self.positions,
            radii=self.radii,
            valid_mask=self.valid_mask,
            velocities=self.velocities,
            object_ids=self.object_ids,
        )


@dataclass(frozen=True)
class LinkSphereModel:
    """Collision spheres rigidly attached to simulator link origins."""

    body_names: tuple[str, ...]
    body_ids: Any
    local_offsets: Any
    radii: Any
    environment_mask: Any
    self_pair_i: Any
    self_pair_j: Any


@dataclass
class TriangleMeshBatch:
    """Static or batched oriented triangle meshes."""

    vertices: Any
    faces: Any
    valid_face_mask: Any | None = None


@dataclass
class DenseESDFGrid:
    """Dense per-environment ESDF with positive distance in free space."""

    values: Any
    origin: Any
    voxel_size: float


@dataclass
class CollisionQueryResult:
    """Fixed-shape signed-distance results for one collision source."""

    distance: Any
    normal: Any
    witness_robot: Any
    witness_environment: Any
    valid_mask: Any
    robot_geometry_id: Any
    environment_geometry_id: Any
    relative_velocity: Any | None = None
    source: str = "unknown"
    overflow: Any | None = None
