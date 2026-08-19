#!/usr/bin/env python3
"""Benchmark planner-free SONIC encoder/decoder inference on CUDA."""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
from time import perf_counter


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--sonic-root",
        type=Path,
        default=None,
        help="Path to gear_sonic_deploy (or set SPARK_SONIC_DEPLOY_ROOT).",
    )
    parser.add_argument("--encoder", type=Path, default=None)
    parser.add_argument("--decoder", type=Path, default=None)
    parser.add_argument(
        "--encoder-profile",
        choices=("release", "low_latency"),
        default="release",
    )
    parser.add_argument("--batch-size", type=int, default=1024)
    parser.add_argument("--warmup-steps", type=int, default=10)
    parser.add_argument("--steps", type=int, default=100)
    parser.add_argument("--device", default="cuda:0")
    return parser


def _resolve_root(value: Path | None) -> Path:
    raw = value or os.environ.get("SPARK_SONIC_DEPLOY_ROOT")
    if raw is None:
        raise ValueError("provide --sonic-root or set SPARK_SONIC_DEPLOY_ROOT")
    root = Path(raw).expanduser().resolve()
    if not root.is_dir():
        raise FileNotFoundError(f"SONIC deployment root does not exist: {root}")
    return root


def _resolve_model(
    explicit: Path | None,
    root: Path,
    profile: str,
    component: str,
) -> Path:
    if explicit is not None:
        path = explicit.expanduser().resolve()
    else:
        directory = root / "policy" / profile
        dynamic = directory / f"model_{component}_spark_dynamic.onnx"
        regular = directory / f"model_{component}.onnx"
        path = dynamic if dynamic.is_file() else regular
    if not path.is_file():
        raise FileNotFoundError(f"SONIC {component} model does not exist: {path}")
    return path


def _has_dynamic_batch(path: Path) -> bool:
    import onnx

    model = onnx.load(str(path), load_external_data=False)
    dimension = model.graph.input[0].type.tensor_type.shape.dim[0]
    return bool(dimension.dim_param) or dimension.dim_value == 0


def main() -> None:
    args = _parser().parse_args()
    if args.batch_size < 1:
        raise ValueError("--batch-size must be positive")
    if args.warmup_steps < 0 or args.steps < 1:
        raise ValueError("warmup steps must be nonnegative and steps must be positive")

    import torch

    from spark_policy.control.whole_body.unitree_g1 import (
        UnitreeG1NativeBatchedSonicPolicy,
    )
    from spark_policy.control.whole_body.unitree_g1.sonic.native_batched_policy import (
        SONIC_DEFAULT_JOINT_POSITION,
    )

    device = torch.device(args.device)
    if device.type != "cuda" or not torch.cuda.is_available():
        raise RuntimeError("native SONIC benchmarking requires an available CUDA device")

    root = _resolve_root(args.sonic_root)
    encoder = _resolve_model(args.encoder, root, args.encoder_profile, "encoder")
    decoder = _resolve_model(args.decoder, root, args.encoder_profile, "decoder")
    if args.batch_size > 1:
        static = [path for path in (encoder, decoder) if not _has_dynamic_batch(path)]
        if static:
            names = ", ".join(str(path) for path in static)
            raise ValueError(
                f"batched inference requires dynamic-batch SONIC exports; static models: {names}"
            )

    policy = UnitreeG1NativeBatchedSonicPolicy(
        batch_size=args.batch_size,
        device=device,
        encoder_path=encoder,
        decoder_path=decoder,
        encoder_profile=args.encoder_profile,
        hold_first_target=False,
    )
    measured = torch.tensor(
        SONIC_DEFAULT_JOINT_POSITION,
        device=device,
        dtype=torch.float32,
    ).repeat(args.batch_size, 1)
    velocity = torch.zeros_like(measured)
    root_pose = torch.zeros(args.batch_size, 7, device=device)
    root_pose[:, 2] = 0.793
    root_pose[:, 6] = 1.0
    angular_velocity = torch.zeros(args.batch_size, 3, device=device)
    reference = measured[:, None].repeat(1, policy.num_reference_frames, 1)
    reference_velocity = torch.zeros_like(reference)

    def infer():
        return policy.infer_tensor(
            body_joint_pos=measured,
            body_joint_vel=velocity,
            root_pose_w=root_pose,
            root_angular_velocity=angular_velocity,
            reference_joint_position=reference,
            reference_joint_velocity=reference_velocity,
        )

    try:
        for _ in range(args.warmup_steps):
            infer()
        torch.cuda.synchronize(device)
        policy.reset()
        torch.cuda.reset_peak_memory_stats(device)
        started = perf_counter()
        target = None
        info = None
        for _ in range(args.steps):
            target, info = infer()
        torch.cuda.synchronize(device)
        elapsed = perf_counter() - started
        assert target is not None and info is not None
        if not torch.isfinite(target).all() or not info["sonic_ok"].all():
            raise RuntimeError("SONIC produced a non-finite or invalid target")

        batch_hz = args.steps / elapsed
        result = {
            "mode": "native_sonic_policy_benchmark",
            "planner": False,
            "transport": False,
            "device": torch.cuda.get_device_name(device),
            "encoder_profile": args.encoder_profile,
            "encoder": str(encoder),
            "decoder": str(decoder),
            "batch_size": args.batch_size,
            "steps": args.steps,
            "wall_seconds": elapsed,
            "batch_hz": batch_hz,
            "policy_rows_per_second": args.batch_size * batch_hz,
            "model_load_seconds": policy.model_load_seconds,
            "peak_gpu_memory_mib": torch.cuda.max_memory_allocated(device) / (1024**2),
        }
        print(json.dumps(result, sort_keys=True))
    finally:
        policy.close()


if __name__ == "__main__":
    main()
