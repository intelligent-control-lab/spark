#!/usr/bin/env python3
"""Reproducible microbenchmarks for SPARK tensor computation.

The benchmark intentionally excludes simulation and rendering.  It measures
the three new GPU-resident building blocks independently: point-cloud distance
queries, batched safety QPs, and cuRobo dual-arm FK/IK.
"""

from __future__ import annotations

import argparse
import csv
import gc
import json
import os
import platform
import statistics
import subprocess
import time
from datetime import datetime, timezone
from pathlib import Path


def _ints(value: str) -> list[int]:
    return [int(item) for item in value.split(",") if item]


def _command_output(command: list[str]) -> str:
    try:
        return subprocess.run(
            command, check=False, capture_output=True, text=True, timeout=5
        ).stdout.strip()
    except (OSError, subprocess.SubprocessError):
        return "unavailable"


def _metadata(torch, args) -> dict:
    governor_paths = sorted(Path("/sys/devices/system/cpu").glob("cpu*/cpufreq/scaling_governor"))
    governors = sorted({path.read_text().strip() for path in governor_paths if path.exists()})
    return {
        "timestamp_utc": datetime.now(timezone.utc).isoformat(),
        "command": " ".join(os.sys.argv),
        "hostname": platform.node(),
        "platform": platform.platform(),
        "python": platform.python_version(),
        "torch": torch.__version__,
        "cuda_runtime": torch.version.cuda,
        "device": args.device,
        "gpu": torch.cuda.get_device_name(args.device) if args.device.startswith("cuda") else None,
        "cpu_governors": governors,
        "git_commit": _command_output(["git", "rev-parse", "HEAD"]),
        "git_dirty": bool(_command_output(["git", "status", "--porcelain"])),
        "nvidia_smi": _command_output(
            [
                "nvidia-smi",
                "--query-gpu=name,driver_version,memory.total,pstate,clocks.sm,temperature.gpu",
                "--format=csv,noheader",
            ]
        ),
        "warmup": args.warmup,
        "samples": args.samples,
        "seed": args.seed,
    }


def _measure(torch, fn, *, warmup: int, samples: int, device: str) -> list[float]:
    for _ in range(warmup):
        fn()
    if device.startswith("cuda"):
        torch.cuda.synchronize(device)
        start = torch.cuda.Event(enable_timing=True)
        end = torch.cuda.Event(enable_timing=True)
        values = []
        for _ in range(samples):
            start.record()
            fn()
            end.record()
            end.synchronize()
            values.append(float(start.elapsed_time(end)))
        return values
    values = []
    for _ in range(samples):
        started = time.perf_counter_ns()
        fn()
        values.append((time.perf_counter_ns() - started) / 1.0e6)
    return values


def _percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, round(fraction * (len(ordered) - 1))))
    return ordered[index]


def _record(suite: str, operation: str, parameters: dict, values: list[float], **extra) -> dict:
    median = statistics.median(values)
    record = {
        "suite": suite,
        "operation": operation,
        **parameters,
        "samples": len(values),
        "mean_ms": statistics.fmean(values),
        "median_ms": median,
        "stdev_ms": statistics.stdev(values) if len(values) > 1 else 0.0,
        "p05_ms": _percentile(values, 0.05),
        "p95_ms": _percentile(values, 0.95),
        "p99_ms": _percentile(values, 0.99),
        "throughput_per_s": 1000.0 / median if median > 0 else float("inf"),
        "raw_ms": values,
        **extra,
    }
    return record


def benchmark_distance(torch, args) -> list[dict]:
    from spark_policy.safety.geometry import PointCloudBatch, TorchSphereCollisionBackend

    backend = TorchSphereCollisionBackend()
    records = []
    generator = torch.Generator(device=args.device).manual_seed(args.seed)
    for envs in args.env_counts:
        for points in args.point_counts:
            centers = torch.randn(
                envs,
                args.robot_spheres,
                3,
                generator=generator,
                device=args.device,
                dtype=torch.float32,
            )
            positions = torch.randn(
                envs,
                points,
                3,
                generator=generator,
                device=args.device,
                dtype=torch.float32,
            )
            cloud = PointCloudBatch(
                positions=positions,
                radii=torch.full((envs, points), args.point_radius, device=args.device),
                valid_mask=torch.ones(envs, points, dtype=torch.bool, device=args.device),
            )
            radii = torch.full((args.robot_spheres,), args.robot_radius, device=args.device)
            if args.device.startswith("cuda"):
                torch.cuda.reset_peak_memory_stats(args.device)
            holder = [None]

            def query(centers=centers, cloud=cloud, holder=holder):
                holder[0] = backend.query_environment_nearest(
                    centers,
                    radii,
                    cloud,
                    nearest_k=args.nearest_k,
                    chunk_size=args.chunk_size,
                )

            values = _measure(
                torch, query, warmup=args.warmup, samples=args.samples, device=args.device
            )
            peak = (
                torch.cuda.max_memory_allocated(args.device) / 2**20
                if args.device.startswith("cuda")
                else 0.0
            )
            records.append(
                _record(
                    "distance",
                    "nearest_point_query",
                    {
                        "envs": envs,
                        "points_per_env": points,
                        "robot_spheres": args.robot_spheres,
                        "nearest_k": args.nearest_k,
                        "chunk_size": args.chunk_size,
                    },
                    values,
                    aggregate_envs_per_s=envs * 1000.0 / statistics.median(values),
                    point_pairs_per_s=(
                        envs * args.robot_spheres * points * 1000.0 / statistics.median(values)
                    ),
                    peak_memory_mib=peak,
                )
            )
            del centers, positions, cloud, holder
    return records


def benchmark_qp(torch, args) -> list[dict]:
    from spark_policy.safety.tensor import BatchedRelaxedQPSafetyFilter, TensorSafetyConstraints

    records = []
    generator = torch.Generator(device=args.device).manual_seed(args.seed + 1)
    for envs in args.env_counts:
        for constraints_count in args.constraint_counts:
            reference = 0.2 * torch.randn(
                envs, args.controls, generator=generator, device=args.device
            )
            A = torch.randn(
                envs,
                constraints_count,
                args.controls,
                generator=generator,
                device=args.device,
            )
            A = A / torch.linalg.vector_norm(A, dim=-1, keepdim=True).clamp_min(1e-6)
            lower = 0.05 * torch.randn(
                envs, constraints_count, generator=generator, device=args.device
            )
            constraints = TensorSafetyConstraints(
                A=A,
                lower=lower,
                distance=torch.zeros_like(lower),
                active_mask=torch.ones_like(lower, dtype=torch.bool),
            )
            solver = BatchedRelaxedQPSafetyFilter(
                iterations=args.qp_iterations,
                slack_weight=args.slack_weight,
            )
            limits = torch.full_like(reference, 1.0)
            holder = [None]
            if args.device.startswith("cuda"):
                torch.cuda.reset_peak_memory_stats(args.device)

            def solve(
                constraints=constraints,
                holder=holder,
                reference=reference,
                solver=solver,
            ):
                holder[0] = solver.filter(
                    reference,
                    constraints,
                    lower_limit=-limits,
                    upper_limit=limits,
                )

            values = _measure(
                torch, solve, warmup=args.warmup, samples=args.samples, device=args.device
            )
            safe, info = holder[0]
            peak = (
                torch.cuda.max_memory_allocated(args.device) / 2**20
                if args.device.startswith("cuda")
                else 0.0
            )
            records.append(
                _record(
                    "qp",
                    "relaxed_admm",
                    {
                        "envs": envs,
                        "constraints": constraints_count,
                        "controls": args.controls,
                        "iterations": args.qp_iterations,
                    },
                    values,
                    aggregate_envs_per_s=envs * 1000.0 / statistics.median(values),
                    max_residual=float(info["max_violation"].max().item()),
                    convergence_rate=float(info["converged"].float().mean().item()),
                    mean_control_delta=float(info["control_delta_norm"].mean().item()),
                    peak_memory_mib=peak,
                )
            )
            del reference, A, lower, constraints, solver, holder, safe
    return records


def benchmark_curobo(torch, args) -> list[dict]:
    from spark_robot.kinematics import UnitreeG1CuroboDualArmIK

    if not args.device.startswith("cuda"):
        raise ValueError("cuRobo benchmark requires a CUDA device")
    records = []
    for batch in args.ik_batches:
        torch.cuda.empty_cache()
        gc.collect()
        torch.cuda.reset_peak_memory_stats(args.device)
        solver = UnitreeG1CuroboDualArmIK(
            max_batch_size=batch,
            device=args.device,
            num_seeds=args.ik_seeds,
            self_collision_check=args.ik_self_collision,
        )
        current = torch.zeros(batch, 14, device=args.device)
        left, right = solver.forward(current)
        # Small reachable perturbations avoid conflating throughput with goal
        # rejection while still exercising the optimization kernels.
        left_goal = left.clone()
        right_goal = right.clone()
        offsets = torch.linspace(-0.02, 0.02, batch, device=args.device)
        left_goal[:, 0, 3] += offsets
        right_goal[:, 0, 3] -= offsets
        holder = [None]

        def fk(current=current, holder=holder, solver=solver):
            holder[0] = solver.forward(current)

        fk_values = _measure(
            torch, fk, warmup=args.ik_warmup, samples=args.ik_samples, device=args.device
        )

        def ik(
            current=current,
            holder=holder,
            left_goal=left_goal,
            right_goal=right_goal,
            solver=solver,
        ):
            holder[0] = solver.solve(left_goal, right_goal, current)

        ik_values = _measure(
            torch, ik, warmup=args.ik_warmup, samples=args.ik_samples, device=args.device
        )
        result = holder[0]
        peak = torch.cuda.max_memory_allocated(args.device) / 2**20
        common = {"envs": batch, "ik_seeds": args.ik_seeds}
        records.append(
            _record(
                "curobo",
                "dual_arm_fk",
                common,
                fk_values,
                aggregate_envs_per_s=batch * 1000.0 / statistics.median(fk_values),
                peak_memory_mib=peak,
            )
        )
        records.append(
            _record(
                "curobo",
                "dual_arm_ik",
                common,
                ik_values,
                aggregate_envs_per_s=batch * 1000.0 / statistics.median(ik_values),
                success_rate=float(result["success"].float().mean().item()),
                max_position_error=float(result["position_error"].max().item()),
                max_rotation_error=float(result["rotation_error"].max().item()),
                peak_memory_mib=peak,
            )
        )
        del solver, current, left, right, left_goal, right_goal, holder, result
        torch.cuda.empty_cache()
    return records


def _write_outputs(output: Path, metadata: dict, records: list[dict]) -> None:
    output.mkdir(parents=True, exist_ok=True)
    payload = {"metadata": metadata, "records": records}
    (output / "tensor_computation.json").write_text(json.dumps(payload, indent=2))
    summary_keys = sorted({key for row in records for key in row if key != "raw_ms"})
    with (output / "tensor_computation_summary.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=summary_keys)
        writer.writeheader()
        writer.writerows({key: row.get(key) for key in summary_keys} for row in records)
    with (output / "tensor_computation_raw.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=["suite", "operation", "setting", "sample", "latency_ms"]
        )
        writer.writeheader()
        for setting, row in enumerate(records):
            for sample, latency in enumerate(row["raw_ms"]):
                writer.writerow(
                    {
                        "suite": row["suite"],
                        "operation": row["operation"],
                        "setting": setting,
                        "sample": sample,
                        "latency_ms": latency,
                    }
                )
    (output / "manifest.json").write_text(json.dumps(metadata, indent=2))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--suites", default="distance,qp,curobo")
    parser.add_argument("--device", default="cuda:0")
    parser.add_argument("--output", type=Path, default=Path("benchmark_results/tensor_computation"))
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--warmup", type=int, default=10)
    parser.add_argument("--samples", type=int, default=30)
    parser.add_argument("--env-counts", type=_ints, default=_ints("1,4,16,64"))
    parser.add_argument("--point-counts", type=_ints, default=_ints("64,256,1024,4096"))
    parser.add_argument("--robot-spheres", type=int, default=37)
    parser.add_argument("--nearest-k", type=int, default=2)
    parser.add_argument("--chunk-size", type=int, default=512)
    parser.add_argument("--robot-radius", type=float, default=0.08)
    parser.add_argument("--point-radius", type=float, default=0.01)
    parser.add_argument("--constraint-counts", type=_ints, default=_ints("16,64,256"))
    parser.add_argument("--controls", type=int, default=20)
    parser.add_argument("--qp-iterations", type=int, default=20)
    parser.add_argument("--slack-weight", type=float, default=1000.0)
    parser.add_argument("--ik-batches", type=_ints, default=_ints("1,4,16,64"))
    parser.add_argument("--ik-seeds", type=int, default=8)
    parser.add_argument("--ik-warmup", type=int, default=2)
    parser.add_argument("--ik-samples", type=int, default=10)
    parser.add_argument("--ik-self-collision", action=argparse.BooleanOptionalAction, default=False)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    import torch

    if args.device.startswith("cuda") and not torch.cuda.is_available():
        raise RuntimeError("CUDA benchmark requested but torch.cuda.is_available() is false")
    metadata = _metadata(torch, args)
    if metadata["cpu_governors"] and metadata["cpu_governors"] != ["performance"]:
        print(
            f"[warning] CPU governors are {metadata['cpu_governors']}, not performance", flush=True
        )
    suites = {item.strip() for item in args.suites.split(",") if item.strip()}
    unknown = suites - {"distance", "qp", "curobo"}
    if unknown:
        raise ValueError(f"Unknown suites: {sorted(unknown)}")
    records = []
    if "distance" in suites:
        records.extend(benchmark_distance(torch, args))
    if "qp" in suites:
        records.extend(benchmark_qp(torch, args))
    if "curobo" in suites:
        records.extend(benchmark_curobo(torch, args))
    _write_outputs(args.output, metadata, records)
    print(f"Wrote {len(records)} benchmark settings to {args.output}")


if __name__ == "__main__":
    main()
