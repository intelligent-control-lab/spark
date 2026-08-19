#!/usr/bin/env python3
"""Run the reproducible cross-robot MuJoCo/Isaac support matrix."""

from __future__ import annotations

import argparse
import importlib.util
import inspect
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import time
from types import SimpleNamespace


REPO_ROOT = Path(__file__).resolve().parents[1]


def _load_case_runner():
    path = REPO_ROOT / "example/run_robot_matrix_case.py"
    spec = importlib.util.spec_from_file_location("spark_robot_matrix_case", path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Could not load matrix case runner from {path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--robot-config", nargs="*", default=None)
    parser.add_argument(
        "--backend", nargs="+", choices=("mujoco", "isaac"), default=("mujoco", "isaac")
    )
    parser.add_argument(
        "--mode", nargs="+", choices=("teleop", "benchmark"), default=("teleop", "benchmark")
    )
    parser.add_argument(
        "--test-case",
        nargs="+",
        choices=("joint_goal_reaching_v0", "joint_goal_reaching_v1"),
        default=("joint_goal_reaching_v0",),
    )
    parser.add_argument("--num-envs", nargs="+", type=int, default=(1, 4))
    parser.add_argument(
        "--dynamics-backend",
        nargs="+",
        choices=("simulator", "model"),
        default=("simulator", "model"),
    )
    parser.add_argument(
        "--device",
        default=None,
        help="Isaac device; defaults to cpu for one environment and cuda:0 for batches.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=2.0,
        help="Simulated seconds per case; 2 s covers the slowest validated physical servo.",
    )
    parser.add_argument("--amplitude", type=float, default=0.08)
    parser.add_argument(
        "--position-kp",
        type=float,
        default=None,
        help="Override the order-aware conformance gain used by each child case.",
    )
    parser.add_argument(
        "--velocity-kd",
        type=float,
        default=None,
        help="Override the order-aware conformance damping used by each child case.",
    )
    parser.add_argument("--goal-tolerance", type=float, default=0.03)
    parser.add_argument("--record-dir", type=Path, default=None)
    parser.add_argument("--record-fps", type=float, default=8.0)
    parser.add_argument("--record-width", type=int, default=480)
    parser.add_argument("--record-height", type=int, default=270)
    parser.add_argument("--report", type=Path, required=True)
    parser.add_argument("--stop-on-failure", action="store_true")
    parser.add_argument(
        "--isaac-startup-retries",
        type=int,
        default=2,
        help="Retry an Isaac configuration when Kit exits before writing its report.",
    )
    parser.add_argument(
        "--isaac-restart-backoff",
        type=float,
        default=2.0,
        help="Seconds to wait between isolated Isaac Kit processes.",
    )
    parser.add_argument("--in-process", action="store_true", help=argparse.SUPPRESS)
    return parser


def _configured_robot_names(spark_robot) -> list[str]:
    from spark_robot import RobotConfig

    names = []
    seen = set()
    for name in sorted(dir(spark_robot)):
        config_type = getattr(spark_robot, name)
        if (
            not inspect.isclass(config_type)
            or config_type is RobotConfig
            or not issubclass(config_type, RobotConfig)
            or inspect.isabstract(config_type)
            or config_type in seen
        ):
            continue
        seen.add(config_type)
        try:
            config = config_type()
        except TypeError:
            continue
        isaac = config.backend_capabilities().get("isaac")
        if config.isaac_articulation is not None and isaac is not None and isaac.batched:
            names.append(name)
    return names


def _slug(name: str) -> str:
    result = []
    for index, character in enumerate(name):
        if character.isupper() and index and result[-1] != "_":
            result.append("_")
        result.append(character.lower())
    return "".join(result).replace("_config", "")


def _case_args(matrix_args, *, name, backend, mode, test_case, num_envs, dynamics_backend):
    gif_path = None
    if matrix_args.record_dir is not None:
        gif_path = matrix_args.record_dir / (
            f"{_slug(name)}__{backend}_{num_envs}env__{dynamics_backend}__{mode}__{test_case}.gif"
        )
    return SimpleNamespace(
        robot_config=name,
        backend=backend,
        mode=mode,
        test_case=test_case,
        num_envs=num_envs,
        seed=0,
        dynamics_backend=dynamics_backend,
        use_sim_dynamics=None,
        device=matrix_args.device or ("cpu" if num_envs == 1 else "cuda:0"),
        headless=True,
        real_time=False,
        duration=matrix_args.duration,
        max_episode_steps=1000,
        amplitude=matrix_args.amplitude,
        control_name=None,
        position_kp=matrix_args.position_kp,
        velocity_kd=matrix_args.velocity_kd,
        goal_tolerance=matrix_args.goal_tolerance,
        safe_algo="control_limit",
        record_video_path=None,
        record_gif_path=None if gif_path is None else str(gif_path),
        record_fps=matrix_args.record_fps,
        record_width=matrix_args.record_width,
        record_height=matrix_args.record_height,
        render_robot_collision_volumes=True,
        camera_lookat=None,
        camera_distance=None,
        camera_azimuth=None,
        camera_elevation=None,
        camera_vertical_fov=None,
        env_spacing=2.5,
        max_visualized_envs=1,
        render_every=1,
        report=None,
    )


def _attach_recording_metadata(case, result: dict) -> None:
    """Verify a requested GIF was finalized and describe it in the report."""

    if not case.record_gif_path:
        return
    gif_path = Path(case.record_gif_path)
    result["record_gif_path"] = str(gif_path)
    if gif_path.is_file() and gif_path.stat().st_size > 0:
        result["record_gif_bytes"] = gif_path.stat().st_size
        return
    if result.get("status") == "PASS":
        result.update(
            status="ERROR",
            error_type="MissingRecordingArtifact",
            error=f"Requested GIF was not finalized: {gif_path}",
        )


def _run_backend_children(args) -> int:
    """Keep MuJoCo/OpenCV and Isaac Kit native libraries in separate processes."""

    reports = []
    return_codes = []
    for backend in args.backend:
        backend_report = args.report.with_name(
            f"{args.report.stem}.{backend}{args.report.suffix or '.json'}"
        )
        command = [
            sys.executable,
            str(Path(__file__).resolve()),
            "--backend",
            backend,
            "--mode",
            *args.mode,
            "--test-case",
            *getattr(args, "test_case", ("joint_goal_reaching_v0",)),
            "--num-envs",
            *(str(value) for value in args.num_envs),
            "--dynamics-backend",
            *args.dynamics_backend,
            "--duration",
            str(args.duration),
            "--amplitude",
            str(args.amplitude),
            "--goal-tolerance",
            str(args.goal_tolerance),
            "--record-fps",
            str(args.record_fps),
            "--record-width",
            str(args.record_width),
            "--record-height",
            str(args.record_height),
            "--report",
            str(backend_report),
            "--isaac-startup-retries",
            str(args.isaac_startup_retries),
            "--isaac-restart-backoff",
            str(args.isaac_restart_backoff),
        ]
        if args.device is not None:
            command.extend(("--device", args.device))
        if args.position_kp is not None:
            command.extend(("--position-kp", str(args.position_kp)))
        if args.velocity_kd is not None:
            command.extend(("--velocity-kd", str(args.velocity_kd)))
        if args.robot_config:
            command.extend(("--robot-config", *args.robot_config))
        if args.record_dir is not None:
            command.extend(("--record-dir", str(args.record_dir)))
        if args.stop_on_failure:
            command.append("--stop-on-failure")
        completed = subprocess.run(command, cwd=REPO_ROOT, check=False)
        return_codes.append(completed.returncode)
        if backend_report.is_file():
            reports.append(json.loads(backend_report.read_text()))
    results = [result for report in reports for result in report.get("results", [])]
    summary = {
        "schema_version": 1,
        "process_isolation": "one process per simulator backend",
        "backend_reports": [
            str(
                args.report.with_name(
                    f"{args.report.stem}.{backend}{args.report.suffix or '.json'}"
                )
            )
            for backend in args.backend
        ],
        "case_count": len(results),
        "pass_count": sum(result["status"] == "PASS" for result in results),
        "failure_count": sum(result["status"] != "PASS" for result in results),
        "results": results,
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    print(
        f"SPARK combined support matrix: {summary['pass_count']}/"
        f"{summary['case_count']} PASS; report={args.report}",
        flush=True,
    )
    return 0 if return_codes and not any(return_codes) and not summary["failure_count"] else 1


def _run_isaac_config_children(args, names: list[str]) -> int:
    """Isolate GridCloner/PhysX replication state between robot assets."""

    results = []
    return_codes = []
    with tempfile.TemporaryDirectory(prefix="spark-isaac-matrix-") as temp_dir:
        temp_root = Path(temp_dir)
        for name in names:
            config_report = temp_root / f"{_slug(name)}.json"
            command = [
                sys.executable,
                str(Path(__file__).resolve()),
                "--backend",
                "isaac",
                "--robot-config",
                name,
                "--mode",
                *args.mode,
                "--test-case",
                *getattr(args, "test_case", ("joint_goal_reaching_v0",)),
                "--num-envs",
                *(str(value) for value in args.num_envs),
                "--dynamics-backend",
                *args.dynamics_backend,
                "--duration",
                str(args.duration),
                "--amplitude",
                str(args.amplitude),
                "--goal-tolerance",
                str(args.goal_tolerance),
                "--record-fps",
                str(args.record_fps),
                "--record-width",
                str(args.record_width),
                "--record-height",
                str(args.record_height),
                "--report",
                str(config_report),
                "--in-process",
            ]
            if args.device is not None:
                command.extend(("--device", args.device))
            if args.position_kp is not None:
                command.extend(("--position-kp", str(args.position_kp)))
            if args.velocity_kd is not None:
                command.extend(("--velocity-kd", str(args.velocity_kd)))
            if args.record_dir is not None:
                command.extend(("--record-dir", str(args.record_dir)))
            if args.stop_on_failure:
                command.append("--stop-on-failure")

            completed = None
            for attempt in range(args.isaac_startup_retries + 1):
                config_report.unlink(missing_ok=True)
                completed = subprocess.run(command, cwd=REPO_ROOT, check=False)
                if config_report.is_file():
                    break
                if attempt < args.isaac_startup_retries:
                    delay = args.isaac_restart_backoff * (attempt + 1)
                    print(
                        f"[RETRY] {name} Isaac child exited with code "
                        f"{completed.returncode} before writing a report; "
                        f"retrying in {delay:.1f}s "
                        f"({attempt + 1}/{args.isaac_startup_retries})",
                        flush=True,
                    )
                    time.sleep(delay)
            assert completed is not None
            return_codes.append(completed.returncode)
            if config_report.is_file():
                results.extend(json.loads(config_report.read_text()).get("results", []))
            else:
                results.append(
                    {
                        "status": "ERROR",
                        "robot_config": name,
                        "backend": "isaac",
                        "error_type": "MissingChildReport",
                        "error": f"Isaac child exited with code {completed.returncode}",
                    }
                )
            if args.stop_on_failure and completed.returncode:
                break
            if args.isaac_restart_backoff > 0.0:
                time.sleep(args.isaac_restart_backoff)

    summary = {
        "schema_version": 1,
        "process_isolation": "one Isaac Kit process per robot configuration",
        "case_count": len(results),
        "pass_count": sum(result["status"] == "PASS" for result in results),
        "failure_count": sum(result["status"] != "PASS" for result in results),
        "results": results,
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
    print(
        f"SPARK isolated Isaac support matrix: {summary['pass_count']}/"
        f"{summary['case_count']} PASS; report={args.report}",
        flush=True,
    )
    return 0 if return_codes and not any(return_codes) and not summary["failure_count"] else 1


def main() -> int:
    args = _parser().parse_args()
    if any(count < 1 for count in args.num_envs):
        raise SystemExit("--num-envs entries must be positive")
    if args.isaac_startup_retries < 0 or args.isaac_restart_backoff < 0.0:
        raise SystemExit("Isaac retry count and restart backoff must be non-negative")
    if args.record_dir is not None:
        args.record_dir.mkdir(parents=True, exist_ok=True)
    if len(args.backend) > 1:
        return _run_backend_children(args)

    import spark_robot

    runner = _load_case_runner()

    names = args.robot_config or _configured_robot_names(spark_robot)
    unknown = [name for name in names if not hasattr(spark_robot, name)]
    if unknown:
        raise SystemExit(f"Unknown robot configurations: {unknown}")
    if (
        len(args.backend) == 1
        and args.backend[0] == "isaac"
        and len(names) > 1
        and not args.in_process
    ):
        return _run_isaac_config_children(args, names)

    simulation_app = None
    results = []
    try:
        for backend in args.backend:
            if backend == "isaac" and simulation_app is None:
                from isaacsim import SimulationApp

                simulation_app = SimulationApp({"headless": True, "multi_gpu": False})
            for name in names:
                for num_envs in args.num_envs:
                    for dynamics_backend in args.dynamics_backend:
                        for mode in args.mode:
                            test_cases = (
                                args.test_case
                                if mode == "benchmark"
                                else ("joint_goal_reaching_v0",)
                            )
                            for test_case in test_cases:
                                case = _case_args(
                                    args,
                                    name=name,
                                    backend=backend,
                                    mode=mode,
                                    test_case=test_case,
                                    num_envs=num_envs,
                                    dynamics_backend=dynamics_backend,
                                )
                                config = getattr(spark_robot, name)()
                                agents = None
                                recorder = None
                                try:
                                    agents = runner._make_agents(case, config, simulation_app)
                                    if backend == "isaac" and case.record_gif_path:
                                        from spark_agent.simulation.isaac.viewport_recorder import (
                                            IsaacViewportRecorder,
                                        )

                                        recorder = IsaacViewportRecorder(
                                            gif_path=case.record_gif_path,
                                            width=case.record_width,
                                            height=case.record_height,
                                            fps=case.record_fps,
                                        )
                                    result = runner._run_case(
                                        case, config, agents, recorder=recorder
                                    )
                                    if recorder is not None:
                                        recorder.drain(agents.sim.render)
                                        recorder.close()
                                        result["recorded_frames"] = recorder.frames
                                except Exception as exc:
                                    result = {
                                        "status": "ERROR",
                                        "robot_config": name,
                                        "backend": backend,
                                        "mode": mode,
                                        "test_case": test_case,
                                        "num_envs": num_envs,
                                        "dynamics_backend": dynamics_backend,
                                        "error_type": type(exc).__name__,
                                        "error": str(exc),
                                    }
                                finally:
                                    if recorder is not None and not recorder.closed:
                                        if recorder.pending and agents is not None:
                                            recorder.drain(agents.sim.render)
                                        recorder.close()
                                    if agents is not None:
                                        runner._close_agents(agents, backend)
                                _attach_recording_metadata(case, result)
                                results.append(result)
                                print(
                                    f"[{result['status']}] {name} {backend}/{num_envs}env/"
                                    f"{dynamics_backend}/{mode}/{test_case}",
                                    flush=True,
                                )
                                if result["status"] != "PASS" and args.stop_on_failure:
                                    raise RuntimeError(json.dumps(result, sort_keys=True))
        # Isaac's fast SimulationApp shutdown can terminate native services
        # before Python resumes after close(). Persist the child result first.
        summary = {
            "schema_version": 1,
            "case_count": len(results),
            "pass_count": sum(result["status"] == "PASS" for result in results),
            "failure_count": sum(result["status"] != "PASS" for result in results),
            "results": results,
        }
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")
        print(
            f"SPARK robot support matrix: {summary['pass_count']}/"
            f"{summary['case_count']} PASS; report={args.report}",
            flush=True,
        )
        return 0 if summary["failure_count"] == 0 and summary["case_count"] else 1
    finally:
        if simulation_app is not None:
            simulation_app.close()


if __name__ == "__main__":
    raise SystemExit(main())
