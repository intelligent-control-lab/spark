import torch

from spark_policy.safety.geometry import (
    DenseESDFGrid,
    DynamicPointCloudBuffer,
    PointCloudBatch,
    TorchDenseESDFBackend,
    TorchMeshCollisionBackend,
    TorchSphereCollisionBackend,
    TriangleMeshBatch,
    fibonacci_sphere,
    uv_sphere_mesh,
)


def test_depth_to_world_points_supports_dynamic_counts():
    from spark_policy.safety.geometry import depth_to_world_points

    depth = torch.tensor([[[1.0, float("nan")], [2.0, 5.0]], [[1.0, 1.0], [1.0, 1.0]]])
    intrinsics = torch.eye(3).repeat(2, 1, 1)
    points, counts, overflow = depth_to_world_points(
        depth,
        intrinsics,
        torch.zeros(2, 3),
        torch.tensor([[0.0, 0.0, 0.0, 1.0]]).repeat(2, 1),
        max_depth=3.0,
        stride=1,
        capacity=3,
    )
    assert tuple(points.shape) == (2, 3, 3)
    assert counts.tolist() == [2, 3]
    assert overflow.tolist() == [False, True]


from spark_policy.safety.tensor import (
    BatchedProjectionSafetyFilter,
    BatchedQPSafetyFilter,
    BatchedReactiveSafetyFilter,
    BatchedRelaxedQPSafetyFilter,
    FirstOrderTensorSafetyIndex,
    TensorSafetyConstraints,
)


def test_dynamic_point_mask_and_sphere_gradient():
    backend = TorchSphereCollisionBackend()
    centers = torch.tensor([[[0.0, 0.0, 0.0]], [[0.0, 0.0, 0.0]]])
    cloud = PointCloudBatch(
        positions=torch.tensor(
            [
                [[0.30, 0.0, 0.0], [9.0, 0.0, 0.0]],
                [[0.40, 0.0, 0.0], [0.25, 0.0, 0.0]],
            ]
        ),
        radii=torch.full((2, 2), 0.05),
        valid_mask=torch.tensor([[True, False], [True, True]]),
    )
    result = backend.query_environment(centers, torch.tensor([0.10]), cloud)
    assert result.valid_mask.tolist() == [[True, False], [True, True]]
    assert torch.allclose(result.distance[0, 0], torch.tensor(0.15))
    assert torch.allclose(result.distance[1], torch.tensor([0.25, 0.10]))
    assert torch.allclose(result.normal[:, 0], torch.tensor([[-1.0, 0.0, 0.0]] * 2))


def test_chunked_nearest_point_query_matches_dense_reduction():
    backend = TorchSphereCollisionBackend()
    centers = torch.tensor([[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]])
    cloud = PointCloudBatch(
        positions=torch.tensor([[[0.4, 0.0, 0.0], [0.1, 0.0, 0.0], [1.2, 0.0, 0.0]]]),
        radii=torch.zeros(1, 3),
        valid_mask=torch.ones(1, 3, dtype=torch.bool),
    )
    dense = backend.query_environment(centers, torch.zeros(2), cloud)
    nearest = backend.query_environment_nearest(
        centers, torch.zeros(2), cloud, nearest_k=2, chunk_size=1
    )
    expected = dense.distance.reshape(1, 2, 3).topk(2, dim=-1, largest=False).values
    assert torch.allclose(nearest.distance.reshape(1, 2, 2), expected)


def test_nearest_query_does_not_let_masked_robot_spheres_consume_top_k():
    backend = TorchSphereCollisionBackend()
    cloud = PointCloudBatch(
        positions=torch.tensor([[[0.1, 0.0, 0.0], [0.2, 0.0, 0.0]]]),
        radii=torch.zeros(1, 2),
        valid_mask=torch.ones(1, 2, dtype=torch.bool),
    )
    result = backend.query_environment_nearest(
        torch.tensor([[[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]]),
        torch.zeros(2),
        cloud,
        torch.tensor([False, True]),
        nearest_k=1,
    )
    assert result.robot_geometry_id.tolist() == [[0, 1]]
    assert result.valid_mask.tolist() == [[False, True]]
    assert torch.isinf(result.distance[0, 0])


def test_triangle_mesh_query_returns_surface_witness_and_gradient():
    mesh = TriangleMeshBatch(
        vertices=torch.tensor([[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]]),
        faces=torch.tensor([[0, 1, 2]]),
    )
    result = TorchMeshCollisionBackend().query(
        torch.tensor([[[0.25, 0.25, 0.30]]]), torch.tensor([0.10]), mesh
    )
    assert torch.allclose(result.distance, torch.tensor([[0.20]]), atol=1e-6)
    assert torch.allclose(result.normal, torch.tensor([[[0.0, 0.0, 1.0]]]), atol=1e-6)
    assert torch.allclose(
        result.witness_environment, torch.tensor([[[0.25, 0.25, 0.0]]]), atol=1e-6
    )


def test_closed_mesh_query_is_signed_and_points_out_of_penetration():
    vertices, faces = uv_sphere_mesh(6, 8)
    mesh = TriangleMeshBatch(
        vertices=torch.as_tensor(vertices),
        faces=torch.as_tensor(faces),
    )
    result = TorchMeshCollisionBackend().query(
        torch.tensor([[[0.2, 0.0, 0.0]]]), torch.tensor([0.05]), mesh
    )
    assert result.distance.item() < 0.0
    assert result.normal[0, 0, 0].item() > 0.8


def test_surface_sampling_contracts_are_finite_and_unit_length():
    points = torch.as_tensor(fibonacci_sphere(64))
    vertices, faces = uv_sphere_mesh(8, 12)
    assert points.shape == (64, 3)
    assert torch.allclose(
        torch.linalg.vector_norm(points, dim=1), torch.ones(64, dtype=points.dtype)
    )
    assert vertices.shape == (86, 3)
    assert faces.shape == (168, 3)


def test_dense_esdf_query_interpolates_distance_and_gradient():
    values = torch.arange(4, dtype=torch.float32).view(1, 1, 4).expand(4, 4, 4)
    grid = DenseESDFGrid(
        values=values,
        origin=torch.zeros(3),
        voxel_size=1.0,
    )
    result = TorchDenseESDFBackend().query(
        torch.tensor([[[1.5, 1.5, 1.5]]]), torch.tensor([0.25]), grid
    )
    assert torch.allclose(result.distance, torch.tensor([[1.25]]), atol=1e-6)
    assert torch.allclose(result.normal, torch.tensor([[[1.0, 0.0, 0.0]]]), atol=1e-6)


def test_self_collision_pairs_have_opposite_relative_jacobians():
    backend = TorchSphereCollisionBackend()
    centers = torch.tensor([[[0.0, 0.0, 0.0], [0.3, 0.0, 0.0]]])
    query = backend.query_self(
        centers,
        torch.tensor([0.05, 0.05]),
        torch.tensor([0]),
        torch.tensor([1]),
    )
    jacobian = torch.zeros(1, 2, 3, 1)
    jacobian[0, 0, 0, 0] = 1.0
    jacobian[0, 1, 0, 0] = -1.0
    index = FirstOrderTensorSafetyIndex(minimum_distance=0.05, activation_distance=0.25, alpha=2.0)
    constraints = index.build(query, jacobian[:, :1], other_point_jacobian=jacobian[:, 1:])
    # The normal points from sphere 1 (index 1) toward sphere 0.
    assert torch.allclose(constraints.A, torch.tensor([[[-2.0]]]))
    assert constraints.active_mask.item()


def test_projection_filter_handles_independent_environment_constraints():
    constraints = TensorSafetyConstraints(
        A=torch.tensor([[[1.0, 0.0]], [[0.0, 1.0]]]),
        lower=torch.tensor([[0.5], [0.25]]),
        distance=torch.zeros(2, 1),
        active_mask=torch.ones(2, 1, dtype=torch.bool),
    )
    reference = torch.zeros(2, 2)
    safe, info = BatchedProjectionSafetyFilter(iterations=4).filter(reference, constraints)
    assert torch.allclose(safe, torch.tensor([[0.5, 0.0], [0.0, 0.25]]))
    assert info["triggered"].tolist() == [True, True]
    assert info["converged"].tolist() == [True, True]


def test_projection_ignores_invalid_dynamic_capacity():
    constraints = TensorSafetyConstraints(
        A=torch.tensor([[[1.0], [-1.0]]]),
        lower=torch.tensor([[0.2, 100.0]]),
        distance=torch.zeros(1, 2),
        active_mask=torch.tensor([[True, False]]),
    )
    safe, info = BatchedProjectionSafetyFilter(iterations=2).filter(torch.zeros(1, 1), constraints)
    assert torch.allclose(safe, torch.tensor([[0.2]]))
    assert info["converged"].item()


def test_dynamic_point_cloud_updates_selected_environments_and_reports_overflow():
    buffer = DynamicPointCloudBuffer(3, 4, device="cpu")
    buffer.update_dense(
        torch.randn(2, 3, 3),
        counts=torch.tensor([1, 3]),
        radii=0.02,
        env_ids=torch.tensor([0, 2]),
    )
    assert buffer.valid_mask.tolist() == [
        [True, False, False, False],
        [False, False, False, False],
        [True, True, True, False],
    ]
    try:
        buffer.update_dense(torch.randn(1, 5, 3), counts=[5], env_ids=[1])
    except OverflowError:
        pass
    else:
        raise AssertionError("capacity overflow must not be silently truncated")


def test_batched_hard_qp_projects_independent_rows():
    constraints = TensorSafetyConstraints(
        A=torch.tensor([[[1.0, 0.0]], [[0.0, 1.0]]]),
        lower=torch.tensor([[0.5], [0.25]]),
        distance=torch.zeros(2, 1),
        active_mask=torch.ones(2, 1, dtype=torch.bool),
    )
    safe, info = BatchedQPSafetyFilter(iterations=60).filter(torch.zeros(2, 2), constraints)
    assert torch.allclose(safe, torch.tensor([[0.5, 0.0], [0.0, 0.25]]), atol=2e-3)
    assert info["converged"].all()


def test_relaxed_qp_uses_slack_for_infeasible_box_constraint():
    constraints = TensorSafetyConstraints(
        A=torch.ones(1, 1, 1),
        lower=torch.tensor([[2.0]]),
        distance=torch.zeros(1, 1),
        active_mask=torch.ones(1, 1, dtype=torch.bool),
    )
    safe, info = BatchedRelaxedQPSafetyFilter(iterations=80, slack_weight=100.0).filter(
        torch.zeros(1, 1),
        constraints,
        lower_limit=torch.tensor([[-1.0]]),
        upper_limit=torch.tensor([[1.0]]),
    )
    assert safe.item() <= 1.0 + 1e-6
    assert info["slack"].item() > 0.0
    assert info["max_violation"].item() < 0.05


def test_batched_qp_warm_start_and_selected_reset_preserve_batch_contract():
    constraints = TensorSafetyConstraints(
        A=torch.tensor([[[1.0]], [[1.0]]]),
        lower=torch.tensor([[0.3], [0.6]]),
        distance=torch.zeros(2, 1),
        active_mask=torch.ones(2, 1, dtype=torch.bool),
    )
    solver = BatchedRelaxedQPSafetyFilter(iterations=20, slack_weight=1000.0, warm_start=True)
    first, first_info = solver.filter(torch.zeros(2, 1), constraints)
    second, second_info = solver.filter(torch.zeros(2, 1), constraints)
    assert torch.allclose(second, first, atol=2e-3)
    assert torch.all(second_info["max_violation"] <= first_info["max_violation"] + 1e-5)
    solver.reset(torch.tensor([1]))
    assert torch.all(solver._warm_state["dual"][1] == 0.0)
    assert torch.any(solver._warm_state["dual"][0] != 0.0)


def test_reactive_filter_changes_only_triggered_batch_rows():
    constraints = TensorSafetyConstraints(
        A=torch.tensor([[[1.0, 0.0]], [[0.0, 1.0]]]),
        lower=torch.tensor([[0.2], [-0.2]]),
        distance=torch.zeros(2, 1),
        active_mask=torch.tensor([[True], [False]]),
    )
    safe, info = BatchedReactiveSafetyFilter(gain=0.5).filter(torch.zeros(2, 2), constraints)
    assert torch.allclose(safe, torch.tensor([[0.5, 0.0], [0.0, 0.0]]))
    assert info["triggered"].tolist() == [True, False]
