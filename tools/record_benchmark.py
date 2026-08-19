#!/usr/bin/env python3
"""Record deterministic Unitree G1 WBT benchmark demonstrations headlessly."""

from __future__ import annotations

import argparse
import math
from pathlib import Path
import subprocess
import sys

import cv2
import imageio.v2 as imageio
import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
BENCHMARK = REPO_ROOT / "example/unitree_g1/run_unitree_g1_benchmark.py"


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Record a real SPARK benchmark workflow without an interactive viewer."
    )
    parser.add_argument("--robot-config", default="UnitreeG1WholeBodyDynamic1Config")
    parser.add_argument("--policy-config", default="UnitreeG1WBTSafePolicy")
    parser.add_argument("--test-case", default="base_goal_static_v0")
    parser.add_argument("--backend", choices=("mujoco", "isaac"), default="mujoco")
    parser.add_argument("--num-envs", type=int, default=1)
    parser.add_argument("--safe-algo", default="bypass")
    parser.add_argument(
        "--environment-representation",
        choices=("sphere", "point_cloud", "mesh"),
        default="sphere",
    )
    parser.add_argument("--shared-obstacle-environment", action="store_true")
    parser.add_argument("--base-goal-minimum-distance", type=float, default=0.2)
    parser.add_argument("--sonic-deploy-root", default=None)
    parser.add_argument("--isaac-device", default=None)
    parser.add_argument("--object-mesh-path", default=None)
    parser.add_argument("--object-mesh-scale", type=float, default=1.0)
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--fps", type=float, default=25.0)
    parser.add_argument(
        "--resolution", type=int, nargs=2, metavar=("WIDTH", "HEIGHT"), default=(960, 540)
    )
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--gif", type=Path, default=None)
    parser.add_argument("--gif-fps", type=float, default=12.5)
    parser.add_argument("--camera-lookat", type=float, nargs=3, default=None)
    parser.add_argument("--camera-distance", type=float, default=None)
    parser.add_argument("--camera-azimuth", type=float, default=None)
    parser.add_argument("--camera-elevation", type=float, default=None)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--max-num-reset", type=int, default=-1)
    parser.add_argument(
        "--render-robot-collision-volumes", action=argparse.BooleanOptionalAction, default=True
    )
    parser.add_argument("--keep-environment-videos", action="store_true")
    return parser


def _environment_path(output: Path, env_id: int) -> Path:
    return output.with_name(f"{output.stem}_env{env_id}{output.suffix}")


def _run_benchmark(args: argparse.Namespace) -> list[str]:
    if args.num_envs < 1:
        raise SystemExit("--num-envs must be positive")
    if (
        args.backend == "mujoco"
        and args.num_envs > 1
        and args.policy_config != "UnitreeG1WBTSafePolicy"
    ):
        raise SystemExit("Parallel recording is supported only for UnitreeG1WBTSafePolicy")
    if args.output.suffix.lower() != ".mp4":
        raise SystemExit("--output must use the .mp4 extension")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    width, height = args.resolution
    # Current Unitree WBT, SONIC, and Isaac tensor benchmarks all expose a
    # 20 ms policy/control period.
    max_steps = max(20, int(math.ceil(args.duration / 0.02)))
    command = [
        sys.executable,
        str(BENCHMARK),
        "--robot-config",
        args.robot_config,
        "--policy-config",
        args.policy_config,
        "--backend",
        args.backend,
        "--num-envs",
        str(args.num_envs),
        "--test-case",
        args.test_case,
        "--safe-algo",
        args.safe_algo,
        "--environment-representation",
        args.environment_representation,
        "--base-goal-minimum-distance",
        str(args.base_goal_minimum_distance),
        "--use-sim-dynamics",
        "--headless",
        "--no-real-time",
        "--seed",
        str(args.seed),
        "--max-num-steps",
        str(max_steps),
        "--max-num-reset",
        str(args.max_num_reset),
        "--record-video-path",
        str(args.output),
        "--record-duration",
        str(args.duration),
        "--record-fps",
        str(args.fps),
        "--record-width",
        str(width),
        "--record-height",
        str(height),
    ]
    if args.backend == "isaac":
        command += ["--isaac-device", args.isaac_device or "cuda:0"]
    if args.shared_obstacle_environment:
        command.append("--shared-obstacle-environment")
    if args.object_mesh_path:
        command += [
            "--object-mesh-path",
            args.object_mesh_path,
            "--object-mesh-scale",
            str(args.object_mesh_scale),
        ]
    if "Sonic" in args.policy_config:
        if not args.sonic_deploy_root:
            raise SystemExit("SONIC recording requires --sonic-deploy-root")
        command += [
            "--auto-launch-sonic-server",
            "--sonic-deploy-root",
            args.sonic_deploy_root,
        ]
    command.append(
        "--render-robot-collision-volumes"
        if args.render_robot_collision_volumes
        else "--no-render-robot-collision-volumes"
    )
    for option, value in (
        ("--viewer-lookat", args.camera_lookat),
        ("--viewer-distance", args.camera_distance),
        ("--viewer-azimuth", args.camera_azimuth),
        ("--viewer-elevation", args.camera_elevation),
    ):
        if value is not None:
            command.append(option)
            if isinstance(value, list):
                command.extend(str(item) for item in value)
            else:
                command.append(str(value))
    print("Recording command:\n  " + " \\\n  ".join(command), flush=True)
    subprocess.run(command, cwd=REPO_ROOT, check=True)
    return command


def _tile_parallel_videos(
    output: Path, count: int, fps: float, resolution: tuple[int, int]
) -> None:
    sources = [_environment_path(output, index) for index in range(count)]
    missing = [str(path) for path in sources if not path.is_file()]
    if missing:
        raise RuntimeError(f"Missing parallel environment recordings: {missing}")
    captures = [cv2.VideoCapture(str(path)) for path in sources]
    columns = int(math.ceil(math.sqrt(count)))
    rows = int(math.ceil(count / columns))
    width, height = resolution
    cell_width, cell_height = width // columns, height // rows
    writer = cv2.VideoWriter(str(output), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError(f"Could not open tiled video output {output}")
    try:
        while True:
            frames = []
            for capture in captures:
                ok, frame = capture.read()
                if not ok:
                    frames = []
                    break
                frames.append(cv2.resize(frame, (cell_width, cell_height)))
            if not frames:
                break
            canvas = np.zeros((height, width, 3), dtype=np.uint8)
            for index, frame in enumerate(frames):
                row, column = divmod(index, columns)
                y0, x0 = row * cell_height, column * cell_width
                canvas[y0 : y0 + cell_height, x0 : x0 + cell_width] = frame
            writer.write(canvas)
    finally:
        writer.release()
        for capture in captures:
            capture.release()


def _convert_gif(video: Path, output: Path, source_fps: float, gif_fps: float) -> None:
    output.parent.mkdir(parents=True, exist_ok=True)
    capture = cv2.VideoCapture(str(video))
    stride = max(1, int(round(source_fps / gif_fps)))
    frames = []
    index = 0
    while True:
        ok, frame = capture.read()
        if not ok:
            break
        if index % stride == 0:
            frames.append(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        index += 1
    capture.release()
    if not frames:
        raise RuntimeError(f"No frames could be decoded from {video}")
    imageio.mimsave(output, frames, format="GIF", duration=1000.0 / gif_fps, loop=0)


def main() -> None:
    args = _parser().parse_args()
    command = _run_benchmark(args)
    if args.backend == "mujoco" and args.num_envs > 1:
        _tile_parallel_videos(args.output, args.num_envs, args.fps, tuple(args.resolution))
        if not args.keep_environment_videos:
            for env_id in range(args.num_envs):
                _environment_path(args.output, env_id).unlink(missing_ok=True)
    if args.gif is not None:
        _convert_gif(args.output, args.gif, args.fps, args.gif_fps)
    capture = cv2.VideoCapture(str(args.output))
    frame_count = int(capture.get(cv2.CAP_PROP_FRAME_COUNT))
    capture.release()
    if frame_count <= 0:
        raise RuntimeError(f"Recording produced no decodable frames: {args.output}")
    command_file = args.output.with_suffix(".command.txt")
    command_file.write_text(" ".join(command) + "\n", encoding="utf-8")
    print(f"Wrote {args.output}")
    if args.gif:
        print(f"Wrote {args.gif}")
    print(f"Wrote {command_file}")


if __name__ == "__main__":
    main()
