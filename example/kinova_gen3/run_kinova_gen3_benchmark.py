#!/usr/bin/env python3
"""Run Kinova Gen3 fixed-base arm safety benchmarks."""

from __future__ import annotations

import argparse

from spark_pipeline import (
    KinovaGen3SingleArmBenchmarkPipelineConfig as PipelineConfig,
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
    "KinovaGen3SingleArmDynamic1CollisionConfig",
    "KinovaGen3DualArmDynamic1CollisionConfig",
)
ROBOT_GROUNDING = {
    ROBOT_CONFIGS[0]: {
        "use_dual_arm": False,
        "right_ee_body": "bracelet_link",
        "ee_local_offset": [0.0, 0.0, -0.2],
        "position_kp": 4.0,
    },
    ROBOT_CONFIGS[1]: {
        "use_dual_arm": True,
        "right_ee_body": "right_bracelet_link",
        "left_ee_body": "left_bracelet_link",
        "ee_local_offset": [0.0, 0.0, -0.2],
        "arm_goal_pair_keepout": 0.25,
        "position_kp": 4.0,
    },
}
# Keep goals in the lower, forward portion of each Gen3 workspace.  Avoiding
# points directly below a shoulder also leaves enough IK margin for the
# orientation tolerance used by episode completion.
BENCHMARK_RIGHT_ARM_GOAL_RANGE = ((0.30, 0.42), (-0.20, 0.05), (0.28, 0.42))
BENCHMARK_LEFT_ARM_GOAL_RANGE = ((0.30, 0.42), (-0.05, 0.20), (0.28, 0.42))
BENCHMARK_OBSTACLE_RANGE = ((-0.3, 0.5), (-0.4, 0.4), (0.20, 0.60))
BENCHMARK_ROBOT_KEEPOUT = 0.17
# The Robotiq flange reverses its local tool axis, so identity end-effector
# orientation is the canonical gripper-down pose for Kinova tasks.
BENCHMARK_GOAL_ROTATION = (
    (1.0, 0.0, 0.0),
    (0.0, 1.0, 0.0),
    (0.0, 0.0, 1.0),
)
BENCHMARK_ORIENTATION_TOLERANCE = 0.1


def _build_config(kwargs):
    cfg, grounding = build_fixed_base_config(
        kwargs,
        pipeline_config_type=PipelineConfig,
        robot_grounding=ROBOT_GROUNDING,
        default_robot_config=ROBOT_CONFIGS[0],
        goal_offset=(0.0, 0.0, 0.0),
        goal_rotation=BENCHMARK_GOAL_ROTATION,
        arm_goal_position_only=False,
        arm_goal_orientation_size=BENCHMARK_ORIENTATION_TOLERANCE,
        right_arm_goal_range=BENCHMARK_RIGHT_ARM_GOAL_RANGE,
        left_arm_goal_range=BENCHMARK_LEFT_ARM_GOAL_RANGE,
    )
    cfg.env.task.obstacle_range = [list(bounds) for bounds in BENCHMARK_OBSTACLE_RANGE]
    cfg.env.task.robot_keepout = max(float(cfg.env.task.robot_keepout), BENCHMARK_ROBOT_KEEPOUT)
    return cfg, grounding


def build_config(**kwargs):
    cfg, _ = _build_config(kwargs)
    return cfg


def run(**kwargs):
    cfg, grounding = _build_config(kwargs)
    backend = kwargs.get("backend", "mujoco")
    num_envs = int(kwargs.get("num_envs", 1))
    if backend == "isaac" and kwargs.get("dynamics_backend", "simulator") == "simulator":
        return run_tensor_benchmark(
            kwargs,
            configured=cfg,
            grounding=grounding,
            goal_rotation=BENCHMARK_GOAL_ROTATION,
            display_name="Kinova Gen3",
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
