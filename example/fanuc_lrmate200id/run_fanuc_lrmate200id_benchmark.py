#!/usr/bin/env python3
"""Run FANUC LR Mate 200iD fixed-base arm safety benchmarks."""

from __future__ import annotations

import argparse

from spark_pipeline import (
    FanucLRMate200iDSingleArmBenchmarkPipelineConfig as PipelineConfig,
    TeleopPipeline as Pipeline,
    run_simulation_pipeline,
)
from spark_pipeline.autonomy.manipulator_benchmark_launcher import (
    add_fixed_base_benchmark_arguments,
    build_fixed_base_config,
    run_tensor_benchmark,
    validate_fixed_base_benchmark_arguments,
)


ROBOT_CONFIGS = (
    "FanucLRMate200iDSingleArmDynamic1CollisionConfig",
    "FanucLRMate200iDDualArmDynamic1CollisionConfig",
)
ROBOT_GROUNDING = {
    ROBOT_CONFIGS[0]: {
        "use_dual_arm": False,
        "right_ee_body": "link_6",
        "ee_local_offset": [0.2, 0.0, 0.0],
        "position_kp": 4.0,
    },
    ROBOT_CONFIGS[1]: {
        "use_dual_arm": True,
        "right_ee_body": "right_link_6",
        "left_ee_body": "left_link_6",
        "ee_local_offset": [0.2, 0.0, 0.0],
        "arm_goal_pair_keepout": 0.25,
        "position_kp": 4.0,
    },
}
# Keep the benchmark inside the LR Mate's robust identity-orientation
# workspace. Simultaneously sampling the outer x and z limits can produce a
# mathematically feasible IK solution exactly on joint 5's limit, leaving no
# physical gravity margin for either simulator. The separated dual-arm y
# ranges remain relative to the +/-0.45 m mount layout.
BENCHMARK_SINGLE_ARM_GOAL_RANGE = ((0.25, 0.60), (-0.36, 0.36), (0.30, 0.62))
BENCHMARK_RIGHT_ARM_GOAL_RANGE = ((0.25, 0.60), (-0.62, -0.12), (0.30, 0.62))
BENCHMARK_LEFT_ARM_GOAL_RANGE = ((0.25, 0.60), (0.12, 0.62), (0.30, 0.62))
BENCHMARK_OBSTACLE_RANGE = ((0.15, 0.55), (-0.48, 0.48), (0.30, 0.62))
BENCHMARK_ROBOT_KEEPOUT = 0.17
BENCHMARK_ARM_GOAL_MINIMUM_DISTANCE = 0.25
PARALLEL_ISAAC_RENDER_EVERY = 10
BENCHMARK_GOAL_ROTATION = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)
BENCHMARK_EE_LOCAL_ROTATION = (
    (0.0, 0.0, -1.0),
    (0.0, 1.0, 0.0),
    (1.0, 0.0, 0.0),
)
BENCHMARK_ORIENTATION_TOLERANCE = 0.1


def _build_config(kwargs):
    cfg, resolved = build_fixed_base_config(
        kwargs,
        pipeline_config_type=PipelineConfig,
        robot_grounding=ROBOT_GROUNDING,
        default_robot_config=ROBOT_CONFIGS[0],
        goal_offset=(0.0, 0.0, 0.0),
        goal_rotation=BENCHMARK_GOAL_ROTATION,
        arm_goal_position_only=False,
        arm_goal_orientation_size=BENCHMARK_ORIENTATION_TOLERANCE,
        arm_goal_minimum_distance=BENCHMARK_ARM_GOAL_MINIMUM_DISTANCE,
        right_arm_goal_range=BENCHMARK_SINGLE_ARM_GOAL_RANGE,
        left_arm_goal_range=BENCHMARK_SINGLE_ARM_GOAL_RANGE,
    )
    if resolved["use_dual_arm"]:
        cfg.env.task.right_arm_goal_range = [
            list(bounds) for bounds in BENCHMARK_RIGHT_ARM_GOAL_RANGE
        ]
        cfg.env.task.left_arm_goal_range = [
            list(bounds) for bounds in BENCHMARK_LEFT_ARM_GOAL_RANGE
        ]
    resolved["ee_local_rotation"] = BENCHMARK_EE_LOCAL_ROTATION
    cfg.env.task.validate_arm_goal_reachability = True
    cfg.env.task.obstacle_range = [list(bounds) for bounds in BENCHMARK_OBSTACLE_RANGE]
    cfg.env.task.robot_keepout = max(float(cfg.env.task.robot_keepout), BENCHMARK_ROBOT_KEEPOUT)
    return cfg, resolved


def build_config(**kwargs):
    cfg, _ = _build_config(kwargs)
    return cfg


def run(**kwargs):
    cfg, grounding = _build_config(kwargs)
    backend = kwargs.get("backend", "mujoco")
    num_envs = int(kwargs.get("num_envs", 1))
    if backend == "isaac" and kwargs.get("dynamics_backend", "simulator") == "simulator":
        if num_envs > 1 and kwargs.get("render_every") is None:
            kwargs = {**kwargs, "render_every": PARALLEL_ISAAC_RENDER_EVERY}
        return run_tensor_benchmark(
            kwargs,
            configured=cfg,
            grounding=grounding,
            goal_rotation=BENCHMARK_GOAL_ROTATION,
            display_name="FANUC LR Mate 200iD",
        )
    if num_envs != 1:
        raise ValueError("Multiple environments require the Isaac simulator tensor runtime")
    return run_simulation_pipeline(
        cfg,
        backend=backend,
        pipeline_class=Pipeline,
        save_path=kwargs.get("save_path"),
    )


def _parser():
    parser = argparse.ArgumentParser(description=__doc__)
    return add_fixed_base_benchmark_arguments(parser, ROBOT_CONFIGS)


def main(argv=None):
    args = vars(_parser().parse_args(argv))
    validate_fixed_base_benchmark_arguments(args)
    run(**args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
