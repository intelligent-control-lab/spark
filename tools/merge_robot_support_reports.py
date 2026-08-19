#!/usr/bin/env python3
"""Merge and validate SPARK robot support-matrix JSON reports."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


CASE_KEY_FIELDS = (
    "robot_config",
    "backend",
    "num_envs",
    "dynamics_backend",
    "mode",
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inputs", nargs="+", type=Path)
    parser.add_argument("--report", required=True, type=Path)
    parser.add_argument("--expect-case-count", type=int, default=None)
    parser.add_argument("--require-pass", action="store_true")
    parser.add_argument(
        "--record-dir",
        type=Path,
        default=None,
        help="Require and describe one matrix-named GIF for every result.",
    )
    return parser


def _slug(name: str) -> str:
    result = []
    for index, character in enumerate(name):
        if character.isupper() and index and result[-1] != "_":
            result.append("_")
        result.append(character.lower())
    return "".join(result).replace("_config", "")


def _expand_inputs(inputs: list[Path]) -> list[Path]:
    files = []
    for path in inputs:
        if path.is_dir():
            files.extend(sorted(path.glob("*.json")))
        elif path.is_file():
            files.append(path)
        else:
            raise FileNotFoundError(path)
    if not files:
        raise ValueError("No JSON reports were found")
    return files


def _case_key(result: dict) -> tuple:
    missing = [field for field in CASE_KEY_FIELDS if field not in result]
    if missing:
        raise ValueError(f"Result is missing case-key fields {missing}: {result}")
    return tuple(result[field] for field in CASE_KEY_FIELDS)


def _gif_path(record_dir: Path, result: dict) -> Path:
    return record_dir / (
        f"{_slug(result['robot_config'])}__{result['backend']}_{result['num_envs']}env__"
        f"{result['dynamics_backend']}__{result['mode']}.gif"
    )


def merge_reports(
    inputs: list[Path],
    *,
    expected_case_count: int | None = None,
    require_pass: bool = False,
    record_dir: Path | None = None,
) -> dict:
    files = _expand_inputs(inputs)
    by_key = {}
    for path in files:
        report = json.loads(path.read_text())
        for result in report.get("results", []):
            key = _case_key(result)
            if key in by_key:
                raise ValueError(f"Duplicate matrix case {key} in {path}")
            result = dict(result)
            if record_dir is not None:
                gif_path = _gif_path(record_dir, result)
                if not gif_path.is_file() or gif_path.stat().st_size <= 0:
                    raise ValueError(f"Missing or empty GIF for {key}: {gif_path}")
                result["record_gif_path"] = str(gif_path)
                result["record_gif_bytes"] = gif_path.stat().st_size
            by_key[key] = result

    results = [by_key[key] for key in sorted(by_key)]
    pass_count = sum(result.get("status") == "PASS" for result in results)
    if expected_case_count is not None and len(results) != expected_case_count:
        raise ValueError(f"Expected {expected_case_count} cases, found {len(results)}")
    if require_pass and pass_count != len(results):
        failures = [result for result in results if result.get("status") != "PASS"]
        raise ValueError(f"Merged report contains {len(failures)} non-PASS results")
    return {
        "schema_version": 1,
        "case_count": len(results),
        "pass_count": pass_count,
        "failure_count": len(results) - pass_count,
        # Individual batch paths may be temporary process-isolation details;
        # retain an auditable count without leaking machine-local paths into a
        # committed release report.
        "source_report_count": len(files),
        "results": results,
    }


def main() -> int:
    args = _parser().parse_args()
    try:
        report = merge_reports(
            args.inputs,
            expected_case_count=args.expect_case_count,
            require_pass=args.require_pass,
            record_dir=args.record_dir,
        )
    except (FileNotFoundError, ValueError, json.JSONDecodeError) as exc:
        raise SystemExit(f"SPARK report merge failed: {exc}") from exc
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n")
    print(
        f"SPARK merged support matrix: {report['pass_count']}/"
        f"{report['case_count']} PASS; report={args.report}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
