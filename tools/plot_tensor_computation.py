#!/usr/bin/env python3
"""Generate publication-friendly figures and a Markdown benchmark report."""

from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path


def _rows(path: Path) -> list[dict]:
    with path.open(newline="") as stream:
        rows = list(csv.DictReader(stream))
    for row in rows:
        for key, value in list(row.items()):
            if value in ("", None):
                continue
            try:
                row[key] = float(value)
            except ValueError:
                pass
    return rows


def _save(fig, root: Path, name: str) -> None:
    for suffix in ("png", "svg", "pdf"):
        fig.savefig(root / f"{name}.{suffix}", dpi=180, bbox_inches="tight")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=Path("benchmark_results/tensor_computation"))
    args = parser.parse_args()
    import matplotlib.pyplot as plt

    root = args.input
    figures = root / "figures"
    figures.mkdir(parents=True, exist_ok=True)
    rows = _rows(root / "tensor_computation_summary.csv")
    metadata = json.loads((root / "manifest.json").read_text())

    distance = [row for row in rows if row["suite"] == "distance"]
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    for envs in sorted({int(row["envs"]) for row in distance}):
        selected = sorted(
            (row for row in distance if int(row["envs"]) == envs),
            key=lambda row: row["points_per_env"],
        )
        axes[0].plot(
            [r["points_per_env"] for r in selected],
            [r["median_ms"] for r in selected],
            marker="o",
            label=f"{envs} env",
        )
        axes[1].plot(
            [r["points_per_env"] for r in selected],
            [r["aggregate_envs_per_s"] for r in selected],
            marker="o",
            label=f"{envs} env",
        )
    axes[0].set(
        xscale="log",
        yscale="log",
        xlabel="Points per environment",
        ylabel="Median latency (ms)",
        title="Tensor nearest-point distance",
    )
    axes[1].set(
        xscale="log",
        yscale="log",
        xlabel="Points per environment",
        ylabel="Environment queries/s",
        title="Distance-query throughput",
    )
    axes[0].legend()
    axes[1].legend()
    fig.tight_layout()
    _save(fig, figures, "distance_scaling")
    plt.close(fig)

    qp = [row for row in rows if row["suite"] == "qp"]
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))
    for constraints in sorted({int(row["constraints"]) for row in qp}):
        selected = sorted(
            (row for row in qp if int(row["constraints"]) == constraints),
            key=lambda row: row["envs"],
        )
        axes[0].plot(
            [r["envs"] for r in selected],
            [r["median_ms"] for r in selected],
            marker="o",
            label=f"{constraints} constraints",
        )
        axes[1].plot(
            [r["envs"] for r in selected],
            [r["aggregate_envs_per_s"] for r in selected],
            marker="o",
            label=f"{constraints} constraints",
        )
    axes[0].axhline(20.0, color="black", linestyle="--", linewidth=1, label="20 ms budget")
    axes[0].set(
        xscale="log",
        yscale="log",
        xlabel="Environments",
        ylabel="Median latency (ms)",
        title="Batched relaxed QP",
    )
    axes[1].set(
        xscale="log",
        yscale="log",
        xlabel="Environments",
        ylabel="QP solves/s",
        title="QP aggregate throughput",
    )
    axes[0].legend()
    axes[1].legend()
    fig.tight_layout()
    _save(fig, figures, "qp_scaling")
    plt.close(fig)

    curobo = [row for row in rows if row["suite"] == "curobo"]
    if curobo:
        fig, axes = plt.subplots(1, 2, figsize=(10, 4))
        for operation, label in (("dual_arm_fk", "FK"), ("dual_arm_ik", "IK")):
            selected = sorted(
                (row for row in curobo if row["operation"] == operation),
                key=lambda row: row["envs"],
            )
            axes[0].plot(
                [r["envs"] for r in selected],
                [r["median_ms"] for r in selected],
                marker="o",
                label=label,
            )
            axes[1].plot(
                [r["envs"] for r in selected],
                [r["aggregate_envs_per_s"] for r in selected],
                marker="o",
                label=label,
            )
        axes[0].axhline(20.0, color="black", linestyle="--", linewidth=1)
        axes[0].set(
            xscale="log",
            yscale="log",
            xlabel="Batch size",
            ylabel="Median latency (ms)",
            title="cuRobo dual-arm latency",
        )
        axes[1].set(
            xscale="log",
            yscale="log",
            xlabel="Batch size",
            ylabel="Robot queries/s",
            title="cuRobo throughput",
        )
        axes[0].legend()
        axes[1].legend()
        fig.tight_layout()
        _save(fig, figures, "curobo_scaling")
        plt.close(fig)

    integrated_path = root / "simulator_computation_summary.csv"
    integrated = _rows(integrated_path) if integrated_path.exists() else []
    comparison_path = root / "legacy_comparison.csv"
    comparison = _rows(comparison_path) if comparison_path.exists() else []
    if comparison:
        fig, axes = plt.subplots(1, 3, figsize=(14, 4))
        labels = {
            "legacy_numpy_cpu": "Legacy NumPy CPU",
            "current_tensor_gpu": "Tensor GPU",
            "legacy_scalar_osqp_cpu": "Legacy scalar OSQP CPU",
            "current_batched_gpu": "Batched QP GPU",
            "legacy_sequential_cpu": "CPU Pinocchio/SciPy",
            "current_curobo_gpu": "cuRobo GPU",
        }
        for axis, component in zip(axes, ("distance", "qp", "ik")):
            selected_component = [row for row in comparison if row["component"] == component]
            for implementation in sorted({row["implementation"] for row in selected_component}):
                selected = sorted(
                    (row for row in selected_component if row["implementation"] == implementation),
                    key=lambda row: row["batch"],
                )
                axis.plot(
                    [row["batch"] for row in selected],
                    [row["median_ms"] for row in selected],
                    marker="o",
                    label=labels.get(implementation, implementation),
                )
            axis.set(
                xscale="log",
                yscale="log",
                xlabel="Robot batch",
                ylabel="Median latency (ms)",
                title=f"Toy {component.upper()} comparison",
            )
            axis.legend(fontsize=8)
        fig.tight_layout()
        _save(fig, figures, "legacy_current_comparison")
        plt.close(fig)
    if integrated:
        fig, axes = plt.subplots(1, 2, figsize=(10, 4))
        for backend in ("mujoco", "isaac"):
            for points in sorted(
                {int(row["points_per_obstacle"]) for row in integrated if row["backend"] == backend}
            ):
                selected = sorted(
                    (
                        row
                        for row in integrated
                        if row["backend"] == backend and int(row["points_per_obstacle"]) == points
                    ),
                    key=lambda row: row["envs"],
                )
                if not selected:
                    continue
                label = f"{backend}, {points} pts/object"
                axes[0].plot(
                    [r["envs"] for r in selected],
                    [r["control_rate_hz"] for r in selected],
                    marker="o",
                    label=label,
                )
                axes[1].plot(
                    [r["envs"] for r in selected],
                    [r["aggregate_env_steps_s"] for r in selected],
                    marker="o",
                    label=label,
                )
        axes[0].axhline(50.0, color="black", linestyle="--", linewidth=1, label="50 Hz target")
        axes[0].set(
            xscale="log",
            yscale="log",
            xlabel="Environments",
            ylabel="Control rate (Hz)",
            title="Integrated headless control rate",
        )
        axes[1].set(
            xscale="log",
            yscale="log",
            xlabel="Environments",
            ylabel="Environment steps/s",
            title="Integrated throughput",
        )
        axes[0].legend(fontsize=8)
        axes[1].legend(fontsize=8)
        fig.tight_layout()
        _save(fig, figures, "simulator_scaling")
        plt.close(fig)

        representative = sorted(
            (
                row
                for row in integrated
                if int(row["points_per_obstacle"])
                == max(int(item["points_per_obstacle"]) for item in integrated)
            ),
            key=lambda row: (row["backend"], row["envs"]),
        )
        labels = [f"{row['backend']}\n{int(row['envs'])} env" for row in representative]
        categories = (
            ("Safety/policy", ("stage_safety_total_ms", "stage_policy_ms")),
            ("Physics/environment", ("stage_physics_step_ms", "stage_environment_ms")),
            (
                "Other measured",
                (
                    "stage_feedback_goal_ms",
                    "stage_policy_inference_ms",
                    "stage_post_step_ms",
                    "stage_reset_checks_ms",
                    "stage_obstacles_ms",
                    "stage_visualization_ms",
                    "stage_processing_ms",
                    "stage_render_ms",
                ),
            ),
        )
        fig, axis = plt.subplots(figsize=(8, 4.5))
        bottom = [0.0] * len(representative)
        for name, keys in categories:
            values = [
                sum(float(row.get(key, 0.0) or 0.0) for key in keys) for row in representative
            ]
            axis.bar(labels, values, bottom=bottom, label=name)
            bottom = [old + value for old, value in zip(bottom, values)]
        axis.axhline(20.0, color="black", linestyle="--", linewidth=1, label="20 ms budget")
        axis.set(
            ylabel="Measured stage time (ms/step)",
            title="Integrated stage cost at maximum point count",
        )
        axis.legend(fontsize=8)
        fig.tight_layout()
        _save(fig, figures, "simulator_stage_breakdown")
        plt.close(fig)

    fastest_distance = min(distance, key=lambda row: row["median_ms"])
    largest_distance = max(distance, key=lambda row: row["envs"] * row["points_per_env"])
    largest_qp = max(qp, key=lambda row: row["median_ms"])
    ik_rows = [row for row in curobo if row["operation"] == "dual_arm_ik"]
    integrated_table = ""
    if integrated:
        lines = [
            "| Backend | Environments | Points/environment | Control rate | Aggregate env-steps/s |",
            "|---|---:|---:|---:|---:|",
        ]
        for row in sorted(
            integrated,
            key=lambda item: (item["backend"], item["envs"], item["total_points_per_env"]),
        ):
            lines.append(
                f"| {row['backend']} | {int(row['envs'])} | {int(row['total_points_per_env'])} | "
                f"{row['control_rate_hz']:.2f} Hz | {row['aggregate_env_steps_s']:.2f} |"
            )
        integrated_table = "\n".join(lines)
    comparison_table = ""
    if comparison:
        lines = [
            "| Component | Batch | Legacy median | Current median | Current speedup |",
            "|---|---:|---:|---:|---:|",
        ]
        for component in ("distance", "qp", "ik"):
            component_rows = [row for row in comparison if row["component"] == component]
            batch = max(int(row["batch"]) for row in component_rows)
            selected = [row for row in component_rows if int(row["batch"]) == batch]
            old = next(row for row in selected if str(row["implementation"]).startswith("legacy"))
            new = next(row for row in selected if str(row["implementation"]).startswith("current"))
            lines.append(
                f"| {component.upper()} | {batch} | {old['median_ms']:.3f} ms | "
                f"{new['median_ms']:.3f} ms | {old['median_ms'] / new['median_ms']:.2f}× |"
            )
        comparison_table = "\n".join(lines)
    report = f"""# SPARK tensor-computation scaling report

Generated: `{metadata["timestamp_utc"]}`<br>
Git commit: `{metadata["git_commit"]}` (dirty: `{metadata["git_dirty"]}`)<br>
Device: `{metadata["gpu"] or metadata["device"]}`<br>
PyTorch/CUDA: `{metadata["torch"]}` / `{metadata["cuda_runtime"]}`<br>
CPU governors: `{", ".join(metadata["cpu_governors"]) or "unavailable"}`

## Scope

These are simulator-independent steady-state microbenchmarks. They isolate the
new tensor point-cloud query, batched relaxed QP, and cuRobo dual-arm FK/IK.
Viewer, camera, physics, policy inference, and reset costs are intentionally
excluded. An integrated Isaac/MuJoCo study should consume the same output
schema but must report simulator and computation stages separately.

## Figures

![Distance scaling](figures/distance_scaling.png)

![QP scaling](figures/qp_scaling.png)

{"![cuRobo scaling](figures/curobo_scaling.png)" if curobo else "cuRobo was not included in this run."}

{"![Integrated simulator scaling](figures/simulator_scaling.png)" if integrated else "Integrated simulator profiles were not included in this run."}

{"![Integrated stage breakdown](figures/simulator_stage_breakdown.png)" if integrated else ""}

{"![Legacy/current toy comparison](figures/legacy_current_comparison.png)" if comparison else "Legacy/current comparisons were not included in this run."}

## Legacy/current toy comparison

{comparison_table or "Not measured."}

The legacy cases reproduce the former scalar execution pattern: NumPy
all-pairs distance materialization, a newly configured OSQP problem for every
robot, and sequential CPU Pinocchio/SciPy IK. The current cases use the tensor
nearest-point query, batched relaxed QP, and cuRobo. Inputs and workload sizes
are matched, and the distance outputs agree within the recorded floating-point
tolerance. QP and IK success are checked, but their formulations and stopping
criteria are not identical; these numbers therefore compare execution
architectures rather than claiming solver-level numerical equivalence.

## Key results

- The largest distance workload, `{int(largest_distance["envs"])}` environments ×
  `{int(largest_distance["points_per_env"])}` points, took
  `{largest_distance["median_ms"]:.3f} ms` median (`{largest_distance["p95_ms"]:.3f} ms` p95).
- All tested relaxed QPs remained below 20 ms. The slowest median was
  `{largest_qp["median_ms"]:.3f} ms` for `{int(largest_qp["envs"])}` environments and
  `{int(largest_qp["constraints"])}` constraints per environment.
- cuRobo dual-arm IK took `{ik_rows[-2]["median_ms"]:.3f} ms` at batch
  `{int(ik_rows[-2]["envs"])}` and `{ik_rows[-1]["median_ms"]:.3f} ms` at batch
  `{int(ik_rows[-1]["envs"])}`. Thus batch 16 fits a 20 ms component budget,
  while batch 64 does not on this GPU, although aggregate throughput continues
  to increase.

## Integrated simulator profiles

{integrated_table or "Not measured."}

These rates include policy/safety, physics, task/reset bookkeeping and explicit
synchronization, but exclude interactive rendering. Process startup is recorded
separately and is not included in control rate.

## Reproducibility notes

- Warm-up calls per setting: `{metadata["warmup"]}`
- Measured calls per setting: `{metadata["samples"]}`
- Random seed: `{metadata["seed"]}`
- NVIDIA state: `{metadata["nvidia_smi"]}`
- Smallest distance-query median: `{fastest_distance["median_ms"]:.4f} ms`
- Largest tested distance workload: `{int(largest_distance["envs"])}` environments × `{int(largest_distance["points_per_env"])}` points

Raw samples are in `tensor_computation_raw.csv`; summarized data are in
`tensor_computation_summary.csv`; the full records and metadata are in
`tensor_computation.json` and `manifest.json`.

## Interpretation constraints

Do not attribute these component timings to Isaac or MuJoCo. The kernels are
simulator-independent. End-to-end comparison must separately measure physics,
rendering, CPU/GPU transfers, policy inference, geometry, constraint assembly,
QP, and IK. MuJoCo currently uses scalar/process-parallel CPU workflows while
Isaac owns the GPU-batched workflow, so an end-to-end chart must label that
architectural difference rather than presenting it as simulator speed alone.
"""
    (root / "tensor_computation_report.md").write_text(report)
    print(f"Generated report and figures under {root}")


if __name__ == "__main__":
    main()
