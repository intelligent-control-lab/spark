"""Backend-neutral depth-image conversion for dynamic collision point clouds."""

from __future__ import annotations


def depth_to_world_points(
    depth,
    intrinsics,
    camera_position,
    camera_quaternion_xyzw,
    *,
    min_depth: float = 0.1,
    max_depth: float = 4.0,
    stride: int = 4,
    capacity: int | None = None,
):
    """Unproject batched optical-frame depth images into world coordinates.

    All inputs may be CPU or CUDA tensors. Invalid depth pixels are removed and
    the result is returned as a fixed-width dense tensor plus per-environment
    counts, ready for :class:`DynamicPointCloudBuffer`. The camera quaternion
    uses ROS/Isaac's XYZW convention and the optical frame is +Z forward, +X
    right, +Y down.
    """
    import torch

    depth = torch.as_tensor(depth)
    if depth.ndim == 2:
        depth = depth.unsqueeze(0)
    if depth.ndim != 3:
        raise ValueError("depth must have shape [batch, height, width]")
    if stride < 1:
        raise ValueError("stride must be positive")
    batch, height, width = depth.shape
    device, dtype = depth.device, depth.dtype
    intrinsics = torch.as_tensor(intrinsics, device=device, dtype=dtype)
    if intrinsics.ndim == 2:
        intrinsics = intrinsics.unsqueeze(0).expand(batch, -1, -1)
    position = torch.as_tensor(camera_position, device=device, dtype=dtype)
    quaternion = torch.as_tensor(camera_quaternion_xyzw, device=device, dtype=dtype)
    if position.ndim == 1:
        position = position.unsqueeze(0).expand(batch, -1)
    if quaternion.ndim == 1:
        quaternion = quaternion.unsqueeze(0).expand(batch, -1)

    rows = torch.arange(0, height, stride, device=device, dtype=dtype)
    columns = torch.arange(0, width, stride, device=device, dtype=dtype)
    v, u = torch.meshgrid(rows, columns, indexing="ij")
    z = depth[:, ::stride, ::stride]
    fx = intrinsics[:, 0, 0][:, None, None]
    fy = intrinsics[:, 1, 1][:, None, None]
    cx = intrinsics[:, 0, 2][:, None, None]
    cy = intrinsics[:, 1, 2][:, None, None]
    x = (u[None] - cx) * z / fx
    y = (v[None] - cy) * z / fy
    optical = torch.stack((x, y, z), dim=-1).reshape(batch, -1, 3)
    valid = torch.isfinite(z).reshape(batch, -1)
    valid &= (optical[..., 2] >= min_depth) & (optical[..., 2] <= max_depth)

    # Quaternion rotation without constructing a batch of matrices.
    q_xyz = quaternion[:, None, :3]
    q_w = quaternion[:, None, 3:4]
    rotated = optical + 2.0 * torch.cross(
        q_xyz, torch.cross(q_xyz, optical, dim=-1) + q_w * optical, dim=-1
    )
    world = rotated + position[:, None]

    counts = valid.sum(dim=1)
    output_width = int(valid.shape[1] if capacity is None else capacity)
    if output_width < 1:
        raise ValueError("capacity must be positive")
    dense = torch.zeros(batch, output_width, 3, device=device, dtype=dtype)
    stored_counts = torch.minimum(
        counts, torch.as_tensor(output_width, device=device, dtype=counts.dtype)
    )
    # Each camera can have a different valid count. This small batch loop only
    # compacts pixels; all expensive unprojection/rotation remains vectorized.
    for env_index in range(batch):
        count = int(stored_counts[env_index].item())
        if count:
            dense[env_index, :count] = world[env_index, valid[env_index]][:count]
    return dense, stored_counts, counts > output_width
