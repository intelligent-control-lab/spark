#!/usr/bin/env python3
"""Toy old/new comparison for SPARK geometry, safety QP, and dual-arm IK.

The legacy cases deliberately reproduce the former scalar CPU execution model:
NumPy pairwise distances, one freshly configured OSQP problem per robot, and
sequential Pinocchio/SciPy IK.  The current cases use the GPU tensor distance,
batched relaxed QP, and cuRobo IK implementations.  This is an architecture
microbenchmark, not a claim that the solvers have identical numerical details.
"""

from __future__ import annotations

import argparse
import csv
import json
import statistics
import time
from pathlib import Path

import numpy as np


def measure(fn, warmup, samples, sync=None):
    for _ in range(warmup):
        fn()
    values = []
    for _ in range(samples):
        if sync:
            sync()
        start = time.perf_counter_ns()
        fn()
        if sync:
            sync()
        values.append((time.perf_counter_ns() - start) / 1e6)
    return values


def record(component, implementation, batch, workload, values, **extra):
    median = statistics.median(values)
    return {
        "component": component,
        "implementation": implementation,
        "batch": batch,
        "workload": workload,
        "samples": len(values),
        "median_ms": median,
        "mean_ms": statistics.fmean(values),
        "p95_ms": sorted(values)[round(0.95 * (len(values) - 1))],
        "robots_per_s": batch * 1000.0 / median,
        **extra,
    }


def distance_comparison(torch, args):
    from spark_policy.safety.geometry import PointCloudBatch, TorchSphereCollisionBackend

    rng = np.random.default_rng(args.seed)
    records = []
    for batch in args.batches:
        centers_np = rng.standard_normal((batch, args.robot_spheres, 3)).astype(np.float32)
        points_np = rng.standard_normal((batch, args.points, 3)).astype(np.float32)
        old_out = [None]

        def old():
            # Former CPU-style all-pairs materialization followed by reduction.
            distances = (
                np.linalg.norm(centers_np[:, :, None] - points_np[:, None], axis=-1)
                - args.robot_radius
                - args.point_radius
            )
            old_out[0] = np.min(distances, axis=-1)

        old_values = measure(old, args.warmup, args.samples)

        centers = torch.as_tensor(centers_np, device=args.device)
        points = torch.as_tensor(points_np, device=args.device)
        radii = torch.full((args.robot_spheres,), args.robot_radius, device=args.device)
        cloud = PointCloudBatch(
            positions=points,
            radii=torch.full((batch, args.points), args.point_radius, device=args.device),
            valid_mask=torch.ones((batch, args.points), dtype=torch.bool, device=args.device),
        )
        backend = TorchSphereCollisionBackend()
        new_out = [None]

        def new():
            new_out[0] = backend.query_environment_nearest(
                centers, radii, cloud, nearest_k=1, chunk_size=512
            )

        new_values = measure(
            new, args.warmup, args.samples, lambda: torch.cuda.synchronize(args.device)
        )
        new_distance = new_out[0].distance.reshape(batch, args.robot_spheres).detach().cpu().numpy()
        error = float(np.max(np.abs(old_out[0] - new_distance)))
        workload = f"{args.robot_spheres} spheres x {args.points} points"
        records += [
            record(
                "distance",
                "legacy_numpy_cpu",
                batch,
                workload,
                old_values,
                max_abs_difference=error,
            ),
            record(
                "distance",
                "current_tensor_gpu",
                batch,
                workload,
                new_values,
                max_abs_difference=error,
            ),
        ]
    return records


def qp_comparison(torch, args):
    import osqp
    from scipy import sparse
    from spark_policy.safety.tensor import BatchedRelaxedQPSafetyFilter, TensorSafetyConstraints

    rng = np.random.default_rng(args.seed + 1)
    records = []
    for batch in args.batches:
        u = 0.2 * rng.standard_normal((batch, args.controls))
        A = rng.standard_normal((batch, args.constraints, args.controls))
        A /= np.maximum(np.linalg.norm(A, axis=-1, keepdims=True), 1e-6)
        lower = -0.05 - np.abs(rng.standard_normal((batch, args.constraints))) * 0.02
        old_out = [None]

        def old():
            solved = []
            for env in range(batch):
                # Legacy behavior: construct and factor a separate OSQP each call.
                problem = osqp.OSQP()
                problem.setup(
                    P=sparse.eye(args.controls, format="csc"),
                    q=-u[env],
                    A=sparse.vstack(
                        (sparse.csc_matrix(A[env]), sparse.eye(args.controls)), format="csc"
                    ),
                    l=np.r_[lower[env], -np.ones(args.controls)],
                    u=np.r_[np.full(args.constraints, np.inf), np.ones(args.controls)],
                    verbose=False,
                    eps_abs=1e-2,
                    eps_rel=1e-2,
                )
                solved.append(problem.solve().x)
            old_out[0] = np.asarray(solved)

        old_values = measure(old, args.warmup, args.samples)

        u_t = torch.as_tensor(u, dtype=torch.float32, device=args.device)
        A_t = torch.as_tensor(A, dtype=torch.float32, device=args.device)
        lower_t = torch.as_tensor(lower, dtype=torch.float32, device=args.device)
        constraints = TensorSafetyConstraints(
            A=A_t,
            lower=lower_t,
            distance=torch.zeros_like(lower_t),
            active_mask=torch.ones_like(lower_t, dtype=torch.bool),
        )
        solver = BatchedRelaxedQPSafetyFilter(iterations=20, slack_weight=1000.0)
        new_out = [None]

        def new():
            new_out[0] = solver.filter(
                u_t,
                constraints,
                lower_limit=-torch.ones_like(u_t),
                upper_limit=torch.ones_like(u_t),
            )

        new_values = measure(
            new, args.warmup, args.samples, lambda: torch.cuda.synchronize(args.device)
        )
        workload = f"{args.controls} controls, {args.constraints} constraints"
        records += [
            record("qp", "legacy_scalar_osqp_cpu", batch, workload, old_values),
            record("qp", "current_batched_gpu", batch, workload, new_values),
        ]
    return records


def ik_comparison(torch, args):
    from spark_robot import UnitreeG1DualArmDynamic1Config
    from spark_robot.kinematics import UnitreeG1CuroboDualArmIK
    from spark_robot.unitree_g1.kinematics.unitree_g1_dual_arm_kinematics import (
        UnitreeG1DualArmKinematics,
    )

    records = []
    cpu_solver = UnitreeG1DualArmKinematics(
        UnitreeG1DualArmDynamic1Config(), load_collision_geometry=False
    )
    q0 = np.zeros(14)
    cpu_solver.pre_computation(q0)
    right = cpu_solver.data.oMf[cpu_solver.R_hand_id].homogeneous.copy()
    left = cpu_solver.data.oMf[cpu_solver.L_hand_id].homogeneous.copy()
    right[0, 3] += 0.01
    left[0, 3] -= 0.01
    for batch in args.ik_batches:
        old_out = [None]

        def old():
            results = []
            for _ in range(batch):
                cpu_solver.init_data = q0.copy()
                results.append(cpu_solver.inverse_kinematics([right, left], q0)[1]["ik_result"])
            old_out[0] = results

        old_values = measure(old, args.ik_warmup, args.ik_samples)

        gpu_solver = UnitreeG1CuroboDualArmIK(
            max_batch_size=batch,
            device=args.device,
            num_seeds=args.ik_seeds,
            self_collision_check=False,
        )
        current = torch.zeros(batch, 14, device=args.device)
        left_goal, right_goal = gpu_solver.forward(current)
        left_goal = left_goal.clone()
        right_goal = right_goal.clone()
        left_goal[:, 0, 3] -= 0.01
        right_goal[:, 0, 3] += 0.01
        new_out = [None]

        def new():
            new_out[0] = gpu_solver.solve(left_goal, right_goal, current)

        new_values = measure(
            new, args.ik_warmup, args.ik_samples, lambda: torch.cuda.synchronize(args.device)
        )
        records += [
            record(
                "ik",
                "legacy_sequential_cpu",
                batch,
                "dual-arm 1 cm pose update",
                old_values,
                success_rate=float(np.mean([x.success for x in old_out[0]])),
            ),
            record(
                "ik",
                "current_curobo_gpu",
                batch,
                "dual-arm 1 cm pose update",
                new_values,
                success_rate=float(new_out[0]["success"].float().mean().item()),
            ),
        ]
    return records


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--output", type=Path, default=Path("benchmark_results/tensor_computation"))
    parser.add_argument(
        "--batches", type=lambda x: [int(v) for v in x.split(",")], default=[1, 4, 16, 64]
    )
    parser.add_argument(
        "--ik-batches", type=lambda x: [int(v) for v in x.split(",")], default=[1, 4, 16]
    )
    parser.add_argument("--samples", type=int, default=20)
    parser.add_argument("--warmup", type=int, default=5)
    parser.add_argument("--ik-samples", type=int, default=5)
    parser.add_argument("--ik-warmup", type=int, default=1)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--robot-spheres", type=int, default=37)
    parser.add_argument("--points", type=int, default=1024)
    parser.add_argument("--robot-radius", type=float, default=0.08)
    parser.add_argument("--point-radius", type=float, default=0.01)
    parser.add_argument("--controls", type=int, default=20)
    parser.add_argument("--constraints", type=int, default=64)
    parser.add_argument("--ik-seeds", type=int, default=8)
    args = parser.parse_args()
    import torch

    if not torch.cuda.is_available():
        raise RuntimeError("The current comparison requires CUDA for the new implementations")
    records = (
        distance_comparison(torch, args) + qp_comparison(torch, args) + ik_comparison(torch, args)
    )
    args.output.mkdir(parents=True, exist_ok=True)
    (args.output / "legacy_comparison.json").write_text(json.dumps(records, indent=2))
    keys = sorted({key for row in records for key in row})
    with (args.output / "legacy_comparison.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        writer.writerows(records)
    print(f"Wrote {len(records)} old/new comparison settings to {args.output}")


if __name__ == "__main__":
    main()
