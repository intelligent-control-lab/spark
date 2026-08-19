"""GPU-resident primitive collision queries implemented with PyTorch."""

from __future__ import annotations


class TorchSphereCollisionBackend:
    """Signed-distance queries for link spheres and dynamic point spheres."""

    def __init__(self, *, epsilon: float = 1.0e-8):
        self.epsilon = float(epsilon)

    def query_environment(self, centers, robot_radii, environment, robot_mask=None):
        import torch

        from .types import CollisionQueryResult

        environment.validate()
        if centers.ndim != 3 or centers.shape[-1] != 3:
            raise ValueError("centers must have shape [batch, robot_geometry, 3]")
        if centers.shape[0] != environment.positions.shape[0]:
            raise ValueError("robot and environment batch sizes must match")
        delta = centers[:, :, None, :] - environment.positions[:, None, :, :]
        center_distance = torch.linalg.vector_norm(delta, dim=-1)
        normal = delta / center_distance.clamp_min(self.epsilon).unsqueeze(-1)
        distance = center_distance - robot_radii[None, :, None] - environment.radii[:, None, :]
        valid = environment.valid_mask[:, None, :].expand_as(distance)
        if robot_mask is not None:
            valid = valid & robot_mask[None, :, None]
        witness_robot = centers[:, :, None, :] - normal * robot_radii[None, :, None, None]
        witness_environment = (
            environment.positions[:, None, :, :] + normal * environment.radii[:, None, :, None]
        )
        relative_velocity = None
        if environment.velocities is not None:
            relative_velocity = -environment.velocities[:, None, :, :].expand_as(delta)
        robot_id = torch.arange(centers.shape[1], device=centers.device).view(1, -1, 1)
        robot_id = robot_id.expand(centers.shape[0], -1, environment.positions.shape[1])
        env_id = torch.arange(environment.positions.shape[1], device=centers.device).view(1, 1, -1)
        env_id = env_id.expand(centers.shape[0], centers.shape[1], -1)
        shape = distance.shape
        return CollisionQueryResult(
            distance=distance.reshape(shape[0], -1),
            normal=normal.reshape(shape[0], -1, 3),
            witness_robot=witness_robot.expand(-1, -1, shape[2], -1).reshape(shape[0], -1, 3),
            witness_environment=witness_environment.expand(-1, shape[1], -1, -1).reshape(
                shape[0], -1, 3
            ),
            valid_mask=valid.reshape(shape[0], -1),
            robot_geometry_id=robot_id.reshape(shape[0], -1),
            environment_geometry_id=env_id.reshape(shape[0], -1),
            relative_velocity=(
                None if relative_velocity is None else relative_velocity.reshape(shape[0], -1, 3)
            ),
            source="environment_point_spheres",
        )

    def query_environment_nearest(
        self,
        centers,
        robot_radii,
        environment,
        robot_mask=None,
        *,
        nearest_k: int = 1,
        chunk_size: int = 2048,
    ):
        """Return only the nearest point-sphere constraints per robot sphere.

        The environment capacity may be much larger than ``chunk_size``.  The
        reduction therefore bounds temporary GPU memory at O(B*R*chunk_size)
        while preserving the exact nearest-k result over all valid points.
        """
        import torch

        from .types import CollisionQueryResult, PointCloudBatch

        environment.validate()
        capacity = environment.positions.shape[1]
        if nearest_k < 1 or chunk_size < 1:
            raise ValueError("nearest_k and chunk_size must be positive")
        nearest_k = min(int(nearest_k), capacity)
        candidates = []
        for start in range(0, capacity, int(chunk_size)):
            stop = min(start + int(chunk_size), capacity)
            chunk = PointCloudBatch(
                positions=environment.positions[:, start:stop],
                radii=environment.radii[:, start:stop],
                valid_mask=environment.valid_mask[:, start:stop],
                velocities=(
                    None
                    if environment.velocities is None
                    else environment.velocities[:, start:stop]
                ),
                object_ids=(
                    None
                    if environment.object_ids is None
                    else environment.object_ids[:, start:stop]
                ),
            )
            result = self.query_environment(centers, robot_radii, chunk, robot_mask)
            batch, robot_count = centers.shape[:2]
            point_count = stop - start
            distance = result.distance.reshape(batch, robot_count, point_count)
            valid = result.valid_mask.reshape(batch, robot_count, point_count)
            distance = torch.where(valid, distance, torch.full_like(distance, torch.inf))
            local_k = min(nearest_k, point_count)
            values, indices = torch.topk(distance, local_k, dim=-1, largest=False)
            base = torch.arange(robot_count, device=centers.device)[None, :, None] * point_count
            flat_indices = (base + indices).reshape(batch, -1)

            def gather_scalar(value):
                return torch.gather(value, 1, flat_indices).reshape(batch, robot_count, local_k)

            def gather_vector(value):
                return torch.gather(value, 1, flat_indices[..., None].expand(-1, -1, 3)).reshape(
                    batch, robot_count, local_k, 3
                )

            candidates.append(
                (
                    values,
                    gather_vector(result.normal),
                    gather_vector(result.witness_robot),
                    gather_vector(result.witness_environment),
                    gather_scalar(result.valid_mask),
                    gather_scalar(result.robot_geometry_id),
                    gather_scalar(result.environment_geometry_id) + start,
                    None
                    if result.relative_velocity is None
                    else gather_vector(result.relative_velocity),
                )
            )

        fields = [torch.cat([entry[index] for entry in candidates], dim=2) for index in range(7)]
        distance = fields[0]
        selected_distance, selected = torch.topk(distance, nearest_k, dim=-1, largest=False)

        def reduce_scalar(value):
            return torch.gather(value, 2, selected)

        def reduce_vector(value):
            return torch.gather(value, 2, selected[..., None].expand(-1, -1, -1, 3))

        relative_velocity = None
        if environment.velocities is not None:
            relative_candidates = torch.cat([entry[7] for entry in candidates], dim=2)
            relative_velocity = reduce_vector(relative_candidates)
        batch = centers.shape[0]
        return CollisionQueryResult(
            distance=selected_distance.reshape(batch, -1),
            normal=reduce_vector(fields[1]).reshape(batch, -1, 3),
            witness_robot=reduce_vector(fields[2]).reshape(batch, -1, 3),
            witness_environment=reduce_vector(fields[3]).reshape(batch, -1, 3),
            valid_mask=reduce_scalar(fields[4]).reshape(batch, -1),
            robot_geometry_id=reduce_scalar(fields[5]).reshape(batch, -1),
            environment_geometry_id=reduce_scalar(fields[6]).reshape(batch, -1),
            relative_velocity=(
                None if relative_velocity is None else relative_velocity.reshape(batch, -1, 3)
            ),
            source="environment_point_spheres_nearest",
        )

    def query_self(self, centers, radii, pair_i, pair_j):
        import torch

        from .types import CollisionQueryResult

        batch = centers.shape[0]
        if pair_i.numel() == 0:
            empty_scalar = centers.new_empty((batch, 0))
            empty_vector = centers.new_empty((batch, 0, 3))
            empty_ids = torch.empty((batch, 0), device=centers.device, dtype=torch.long)
            return CollisionQueryResult(
                distance=empty_scalar,
                normal=empty_vector,
                witness_robot=empty_vector,
                witness_environment=empty_vector,
                valid_mask=torch.empty((batch, 0), device=centers.device, dtype=torch.bool),
                robot_geometry_id=empty_ids,
                environment_geometry_id=empty_ids,
                source="self_spheres",
            )
        first = centers[:, pair_i]
        second = centers[:, pair_j]
        delta = first - second
        center_distance = torch.linalg.vector_norm(delta, dim=-1)
        normal = delta / center_distance.clamp_min(self.epsilon).unsqueeze(-1)
        distance = center_distance - radii[pair_i] - radii[pair_j]
        return CollisionQueryResult(
            distance=distance,
            normal=normal,
            witness_robot=first - normal * radii[pair_i][None, :, None],
            witness_environment=second + normal * radii[pair_j][None, :, None],
            valid_mask=torch.ones_like(distance, dtype=torch.bool),
            robot_geometry_id=pair_i[None].expand(batch, -1),
            environment_geometry_id=pair_j[None].expand(batch, -1),
            source="self_spheres",
        )
