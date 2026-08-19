"""Mesh and dense-ESDF signed-distance query backends."""

from __future__ import annotations


def _closest_point_on_segments(point, first, second, epsilon):
    direction = second - first
    parameter = ((point - first) * direction).sum(dim=-1) / direction.square().sum(
        dim=-1
    ).clamp_min(epsilon)
    return first + parameter.clamp(0.0, 1.0).unsqueeze(-1) * direction


class TorchMeshCollisionBackend:
    """Batched sphere-to-oriented-triangle-mesh closest-point queries.

    It is a correctness backend for moderate meshes. Large static meshes can
    later replace its exhaustive triangle reduction with Warp BVH traversal
    without changing the returned query type.
    """

    def __init__(self, *, epsilon: float = 1.0e-8):
        self.epsilon = float(epsilon)

    def query(self, centers, radii, mesh):
        import torch

        from .types import CollisionQueryResult

        vertices = mesh.vertices
        faces = mesh.faces
        if vertices.ndim == 2:
            vertices = vertices[None].expand(centers.shape[0], -1, -1)
        if faces.ndim == 2:
            faces = faces[None].expand(centers.shape[0], -1, -1)
        batch_ids = torch.arange(centers.shape[0], device=centers.device)[:, None, None]
        triangles = vertices[batch_ids, faces]
        a, b, c = triangles.unbind(dim=2)
        point = centers[:, :, None, :]
        a = a[:, None]
        b = b[:, None]
        c = c[:, None]
        ab_point = _closest_point_on_segments(point, a, b, self.epsilon)
        bc_point = _closest_point_on_segments(point, b, c, self.epsilon)
        ca_point = _closest_point_on_segments(point, c, a, self.epsilon)

        normal_face = torch.cross(b - a, c - a, dim=-1)
        normal_face = normal_face / torch.linalg.vector_norm(
            normal_face, dim=-1, keepdim=True
        ).clamp_min(self.epsilon)
        plane_point = point - ((point - a) * normal_face).sum(dim=-1, keepdim=True) * normal_face
        v0, v1, v2 = b - a, c - a, plane_point - a
        d00 = (v0 * v0).sum(dim=-1)
        d01 = (v0 * v1).sum(dim=-1)
        d11 = (v1 * v1).sum(dim=-1)
        d20 = (v2 * v0).sum(dim=-1)
        d21 = (v2 * v1).sum(dim=-1)
        denominator = (d00 * d11 - d01.square()).clamp_min(self.epsilon)
        bary_v = (d11 * d20 - d01 * d21) / denominator
        bary_w = (d00 * d21 - d01 * d20) / denominator
        inside = (bary_v >= 0.0) & (bary_w >= 0.0) & (bary_v + bary_w <= 1.0)

        candidates = torch.stack((ab_point, bc_point, ca_point, plane_point), dim=-2)
        candidate_distance = torch.linalg.vector_norm(point.unsqueeze(-2) - candidates, dim=-1)
        candidate_distance[..., 3] = torch.where(
            inside,
            candidate_distance[..., 3],
            torch.full_like(candidate_distance[..., 3], torch.inf),
        )
        closest_candidate = candidate_distance.argmin(dim=-1)
        closest = torch.gather(
            candidates,
            -2,
            closest_candidate[..., None, None].expand(*closest_candidate.shape, 1, 3),
        ).squeeze(-2)
        delta = point - closest
        triangle_distance = torch.linalg.vector_norm(delta, dim=-1)
        if mesh.valid_face_mask is not None:
            valid_face = mesh.valid_face_mask
            if valid_face.ndim == 1:
                valid_face = valid_face[None].expand(centers.shape[0], -1)
            triangle_distance = torch.where(
                valid_face[:, None],
                triangle_distance,
                torch.full_like(triangle_distance, torch.inf),
            )
        nearest_distance, triangle_id = triangle_distance.min(dim=-1)
        gather_id = triangle_id[..., None, None].expand(-1, -1, 1, 3)
        nearest_point = torch.gather(closest, 2, gather_id).squeeze(2)
        nearest_face_normal = torch.gather(
            normal_face.expand(-1, centers.shape[1], -1, -1),
            2,
            gather_id,
        ).squeeze(2)
        delta = centers - nearest_point
        unsigned_normal = delta / torch.linalg.vector_norm(delta, dim=-1, keepdim=True).clamp_min(
            self.epsilon
        )
        # For closed, consistently outward-oriented meshes, the closest face
        # normal identifies whether a query lies outside or inside. This gives
        # the safety layer a signed distance and, importantly, an outward
        # escape gradient for penetrations instead of pushing farther inward.
        sign = torch.where(
            (delta * nearest_face_normal).sum(dim=-1) >= 0.0,
            torch.ones_like(nearest_distance),
            -torch.ones_like(nearest_distance),
        )
        normal = unsigned_normal * sign[..., None]
        distance = sign * nearest_distance - radii[None]
        # ``unsigned_normal`` always points from the closest mesh surface to
        # the sphere center, so this expression selects the robot surface
        # facing that witness for both exterior and interior queries.
        witness_robot = centers - unsigned_normal * radii[None, :, None]
        robot_ids = torch.arange(centers.shape[1], device=centers.device)[None].expand(
            centers.shape[0], -1
        )
        return CollisionQueryResult(
            distance=distance,
            normal=normal,
            witness_robot=witness_robot,
            witness_environment=nearest_point,
            valid_mask=torch.isfinite(distance),
            robot_geometry_id=robot_ids,
            environment_geometry_id=triangle_id,
            source="environment_triangle_mesh",
        )


class TorchDenseESDFBackend:
    """Trilinear distance and finite-grid-gradient queries."""

    def _sample(self, positions, grid):
        import torch.nn.functional as functional

        values = grid.values
        if values.ndim == 3:
            values = values[None]
        if values.shape[0] == 1 and positions.shape[0] > 1:
            values = values.expand(positions.shape[0], -1, -1, -1)
        origin = grid.origin
        if origin.ndim == 1:
            origin = origin[None].expand(positions.shape[0], -1)
        depth, height, width = values.shape[-3:]
        index = (positions - origin[:, None]) / float(grid.voxel_size)
        normalized = index.new_empty(index.shape)
        normalized[..., 0] = 2.0 * index[..., 0] / max(width - 1, 1) - 1.0
        normalized[..., 1] = 2.0 * index[..., 1] / max(height - 1, 1) - 1.0
        normalized[..., 2] = 2.0 * index[..., 2] / max(depth - 1, 1) - 1.0
        sampled = functional.grid_sample(
            values[:, None],
            normalized[:, :, None, None],
            mode="bilinear",
            padding_mode="border",
            align_corners=True,
        )
        valid = ((normalized >= -1.0) & (normalized <= 1.0)).all(dim=-1)
        return sampled[:, 0, :, 0, 0], valid

    def query(self, centers, radii, grid):
        import torch

        from .types import CollisionQueryResult

        field_distance, valid = self._sample(centers, grid)
        step = float(grid.voxel_size)
        gradient_parts = []
        for axis in range(3):
            offset = torch.zeros_like(centers)
            offset[..., axis] = 0.5 * step
            forward, _ = self._sample(centers + offset, grid)
            backward, _ = self._sample(centers - offset, grid)
            gradient_parts.append((forward - backward) / step)
        gradient = torch.stack(gradient_parts, dim=-1)
        normal = gradient / torch.linalg.vector_norm(gradient, dim=-1, keepdim=True).clamp_min(
            1.0e-8
        )
        distance = field_distance - radii[None]
        witness_environment = centers - normal * field_distance[..., None]
        robot_ids = torch.arange(centers.shape[1], device=centers.device)[None].expand(
            centers.shape[0], -1
        )
        return CollisionQueryResult(
            distance=distance,
            normal=normal,
            witness_robot=centers - normal * radii[None, :, None],
            witness_environment=witness_environment,
            valid_mask=valid,
            robot_geometry_id=robot_ids,
            environment_geometry_id=torch.full_like(robot_ids, -1),
            source="environment_dense_esdf",
        )
