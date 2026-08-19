#!/usr/bin/env python3
"""Collect comparable headless Isaac/MuJoCo end-to-end stage profiles."""

from __future__ import annotations

import argparse
import csv
import json
import os
import re
import subprocess
import sys
import time
from pathlib import Path


ISAAC_STAGE = re.compile(r"^\s{2}([a-z_]+)\s*:\s*([0-9.]+) ms/step$", re.MULTILINE)
MUJOCO_STAGE = re.compile(
    r"^\s{2}(policy|environment|processing|render|idle)\s*:\s*mean\s*([0-9.]+) ms", re.MULTILINE
)
ISAAC_RATE = re.compile(r"control rate:\s*([0-9.]+) Hz")
MUJOCO_RATE = re.compile(r"effective:\s*([0-9.]+) Hz mean")
SAFETY = re.compile(
    r"Safety stage profile: index mean/p95=([0-9.]+)/([0-9.]+) ms, "
    r"solve mean/p95=([0-9.]+)/([0-9.]+) ms"
)


def _ints(value: str) -> list[int]:
    return [int(item) for item in value.split(",") if item]


def _run(root: Path, *, backend: str, envs: int, points: int, steps: int) -> dict:
    command = [
        sys.executable,
        str(root / "example/unitree_g1/run_unitree_g1_benchmark.py"),
        "--robot-config",
        "UnitreeG1WholeBodyDynamic1Config",
        "--policy-config",
        "UnitreeG1WBTSafePolicy",
        "--backend",
        backend,
        "--num-envs",
        str(envs),
        "--test-case",
        "whole_goal_static_v1",
        "--environment-representation",
        "point_cloud",
        "--points-per-obstacle",
        str(points),
        "--max-num-steps",
        str(steps),
        "--headless",
        "--no-real-time",
        "--profile-frequency",
    ]
    if backend == "isaac":
        command += ["--isaac-device", "cuda:0"]
    started = time.perf_counter()
    completed = subprocess.run(
        command,
        cwd=root,
        env={**os.environ, "OMNI_KIT_ACCEPT_EULA": "YES"},
        capture_output=True,
        text=True,
    )
    elapsed = time.perf_counter() - started
    output = completed.stdout + "\n" + completed.stderr
    if completed.returncode:
        raise RuntimeError(
            f"Integrated benchmark failed ({backend}, envs={envs}, points={points})\n"
            + output[-6000:]
        )
    rate_match = (ISAAC_RATE if backend == "isaac" else MUJOCO_RATE).search(output)
    stages = {
        name: float(value)
        for name, value in (ISAAC_STAGE if backend == "isaac" else MUJOCO_STAGE).findall(output)
    }
    safety = SAFETY.search(output)
    if safety:
        stages["safety_index"] = float(safety.group(1))
        stages["safety_solve"] = float(safety.group(3))
    return {
        "backend": backend,
        "envs": envs,
        "points_per_obstacle": points,
        "obstacles": 10,
        "total_points_per_env": 10 * points,
        "steps": steps,
        "control_rate_hz": float(rate_match.group(1)) if rate_match else 0.0,
        "aggregate_env_steps_s": (envs * float(rate_match.group(1)) if rate_match else 0.0),
        "process_wall_seconds": elapsed,
        **{f"stage_{name}_ms": value for name, value in stages.items()},
        "command": " ".join(command),
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, default=Path("benchmark_results/tensor_computation"))
    parser.add_argument("--isaac-envs", type=_ints, default=_ints("1,16,64"))
    parser.add_argument("--mujoco-envs", type=_ints, default=_ints("1"))
    parser.add_argument("--points", type=_ints, default=_ints("64,1024"))
    parser.add_argument("--steps", type=int, default=100)
    args = parser.parse_args()
    root = Path(__file__).resolve().parents[1]
    records = []
    for backend, counts in (("mujoco", args.mujoco_envs), ("isaac", args.isaac_envs)):
        for envs in counts:
            for points in args.points:
                print(f"[integrated] {backend}: envs={envs}, points/obstacle={points}", flush=True)
                records.append(
                    _run(root, backend=backend, envs=envs, points=points, steps=args.steps)
                )
    args.output.mkdir(parents=True, exist_ok=True)
    keys = sorted({key for row in records for key in row})
    with (args.output / "simulator_computation_summary.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=keys)
        writer.writeheader()
        writer.writerows(records)
    (args.output / "simulator_computation.json").write_text(json.dumps(records, indent=2))
    print(f"Wrote {len(records)} integrated profiles to {args.output}")


if __name__ == "__main__":
    main()
