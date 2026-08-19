from types import SimpleNamespace

import numpy as np
import pytest

from spark_pipeline.autonomy.benchmark_test_cases import (
    apply_benchmark_test_case,
    list_benchmark_test_cases,
    resolve_benchmark_test_case,
)
from spark_robot import get_agent_class_name
from spark_task.autonomy.benchmark_goals import arm_goal_pair_is_separated
from spark_task.autonomy.benchmark_task import BenchmarkTask


_FORBIDDEN = {
    "robot_config",
    "robot_cfg",
    "agent_class",
    "agent_config",
    "backend",
    "environment_representation",
    "points_per_obstacle",
    "object_mesh_path",
}


def test_arm_goal_pair_separation_has_an_inclusive_boundary():
    assert arm_goal_pair_is_separated([0.0, 0.0, 0.0], [0.0, 0.25, 0.0], 0.25)
    assert not arm_goal_pair_is_separated([0.0, 0.0, 0.0], [0.0, 0.249, 0.0], 0.25)
    with pytest.raises(ValueError, match="cannot be negative"):
        arm_goal_pair_is_separated([0.0, 0.0, 0.0], [0.0, 1.0, 0.0], -0.1)


def test_arm_goal_minimum_distance_rejects_negative_values():
    with pytest.raises(ValueError, match="arm goal minimum distance"):
        BenchmarkTask(
            robot_cfg=None,
            robot_kinematics=None,
            agent=SimpleNamespace(num_obstacle_agent=0),
            arm_goal_minimum_distance=-0.1,
        )


def test_cli_minimum_distance_preserves_case_value_unless_overridden():
    import importlib.util
    from pathlib import Path

    path = Path(__file__).resolve().parents[1] / "example/unitree_g1/run_unitree_g1_benchmark.py"
    spec = importlib.util.spec_from_file_location("benchmark_cli_precedence", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    parser = module.build_parser()
    defaults = module._resolve_case_inputs(
        vars(parser.parse_args(["--test-case", "base_goal_static_v1"]))
    )
    overridden = module._resolve_case_inputs(
        vars(
            parser.parse_args(
                [
                    "--test-case",
                    "base_goal_static_v1",
                    "--base-goal-minimum-distance",
                    "1.5",
                ]
            )
        )
    )

    assert defaults["base_goal_minimum_distance"] == 0.5
    assert overridden["base_goal_minimum_distance"] == 1.5


def test_benchmark_cli_exposes_sonic_far_goal_gait():
    import importlib.util
    from pathlib import Path

    path = Path(__file__).resolve().parents[1] / "example/unitree_g1/run_unitree_g1_benchmark.py"
    spec = importlib.util.spec_from_file_location("benchmark_sonic_gait", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    args = module.build_parser().parse_args(["--sonic-locomotion-mode", "walk"])
    assert args.sonic_locomotion_mode == "walk"


def test_sonic_benchmark_preserves_whole_goal_case():
    import importlib.util
    from pathlib import Path

    path = Path(__file__).resolve().parents[1] / "example/unitree_g1/run_unitree_g1_benchmark.py"
    spec = importlib.util.spec_from_file_location("benchmark_sonic_whole_goal", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    parser = module.build_parser()
    default_args = vars(
        parser.parse_args(
            [
                "--policy-config",
                "UnitreeG1SonicSafePolicy",
                "--robot-config",
                "UnitreeG1WholeBodyWithHandDynamic1Config",
                "--test-case",
                "whole_goal_static_v0",
            ]
        )
    )
    resolved = module._resolve_case_inputs(default_args)

    assert resolved["arm_goal_mode"] == "random"
    assert resolved["base_goal_mode"] == "random"


def test_sonic_pid_defaults_to_walk_far_and_allows_cli_override():
    from example.unitree_g1.sonic_support import config_policy_module
    from spark_pipeline import (
        UnitreeG1SonicCascadeWholeBodyWithHandTeleopPipelineConfig,
    )

    pid_cfg = UnitreeG1SonicCascadeWholeBodyWithHandTeleopPipelineConfig()
    config_policy_module(
        pid_cfg,
        goal_tracking_type="pid",
        sonic_locomotion_mode=None,
    )
    assert pid_cfg.policy.executor.sonic_locomotion_mode == "walk"

    explicit_cfg = UnitreeG1SonicCascadeWholeBodyWithHandTeleopPipelineConfig()
    config_policy_module(
        explicit_cfg,
        goal_tracking_type="pid",
        sonic_locomotion_mode="slow",
    )
    assert explicit_cfg.policy.executor.sonic_locomotion_mode == "slow"

    legged_cfg = UnitreeG1SonicCascadeWholeBodyWithHandTeleopPipelineConfig()
    config_policy_module(
        legged_cfg,
        goal_tracking_type="legged",
        sonic_locomotion_mode=None,
    )
    assert legged_cfg.policy.executor.sonic_locomotion_mode == "slow"


@pytest.mark.parametrize(
    ("policy_config", "runtime"),
    (
        ("UnitreeG1SonicSafePolicy", "sonic"),
        ("UnitreeG1WBTSafePolicy", "wbt"),
        ("UnitreeG1SportSafePolicy", "sport"),
    ),
)
@pytest.mark.parametrize("num_envs", (1, 4))
def test_learned_isaac_benchmarks_share_one_tensor_route(
    monkeypatch, policy_config, runtime, num_envs
):
    import importlib.util
    from pathlib import Path

    path = Path(__file__).resolve().parents[1] / "example/unitree_g1/run_unitree_g1_benchmark.py"
    spec = importlib.util.spec_from_file_location(
        f"benchmark_tensor_route_{runtime}_{num_envs}", path
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    captured = {}

    def fake_tensor(kwargs, *, controller, num_envs):
        captured.update(kwargs=kwargs, controller=controller, num_envs=num_envs)
        return "tensor"

    monkeypatch.setattr(module, "_run_isaac_tensor_benchmark", fake_tensor)
    cli = [
        "--backend",
        "isaac",
        "--policy-config",
        policy_config,
        "--robot-config",
        "UnitreeG1WholeBodyWithHandDynamic1Config",
        "--test-case",
        "base_goal_static_v1",
        "--num-envs",
        str(num_envs),
        "--headless",
    ]
    if runtime == "sonic":
        cli += [
            "--no-auto-launch-sonic-server",
            "--sonic-endpoints",
            "tcp://127.0.0.1:9999",
        ]

    result = module.run(**vars(module.build_parser().parse_args(cli)))

    assert result == "tensor"
    assert captured["controller"] == runtime
    assert captured["num_envs"] == num_envs
    assert captured["kwargs"]["isaac_device"] == "cuda:0"
    assert captured["kwargs"]["goal_tracking_type"] == "pid"
    assert captured["kwargs"]["max_num_steps"] == 1000
    assert captured["kwargs"]["max_num_reset"] == 10


def test_sonic_runtime_defaults_are_policy_owned():
    from example.unitree_g1.sonic_support import _with_sonic_runtime_defaults

    defaults = _with_sonic_runtime_defaults({})

    assert defaults["goal_tracking_type"] == "pid"
    assert defaults["auto_launch_sonic_server"] is True
    assert defaults["sonic_policy_precision"] == "16"
    assert defaults["sonic_planner_precision"] == "16"
    assert defaults["sonic_server_startup_timeout"] == 600.0


def _config():
    return SimpleNamespace(
        robot=SimpleNamespace(cfg=SimpleNamespace(class_name="SentinelRobot")),
        env=SimpleNamespace(
            agent=SimpleNamespace(class_name="SentinelAgent"),
            task=SimpleNamespace(),
        ),
    )


def test_case_application_does_not_ground_robot_or_agent():
    cfg = apply_benchmark_test_case(_config(), "whole_goal_static_v0")
    assert cfg.robot.cfg.class_name == "SentinelRobot"
    assert cfg.env.agent.class_name == "SentinelAgent"


def test_versioned_levels_have_expected_obstacle_density():
    names = list_benchmark_test_cases()
    for goal_kind in ("arm_goal", "whole_goal"):
        counts = []
        for level in ("v0", "v1", "v2"):
            name = f"{goal_kind}_static_{level}"
            assert name in names
            counts.append(resolve_benchmark_test_case(name).task["num_obstacle_task"])
        assert counts[0] == 0
        assert 0 < counts[1] < counts[2]


def test_base_only_matrix_covers_goal_and_obstacle_motion_at_every_level():
    families = {
        "base_goal_static": (0.0, 0.0),
        "base_goal_dynamic_static": (0.05, 0.0),
        "base_goal_static_dynamic": (0.0, 0.05),
        "base_goal_dynamic_dynamic": (0.05, 0.05),
    }
    for family, (goal_velocity, obstacle_velocity) in families.items():
        for level, obstacle_count in (("v0", 0), ("v1", 10), ("v2", 50)):
            task = resolve_benchmark_test_case(f"{family}_{level}").task
            assert not task["arm_goal_enable"]
            assert not task["arm_goal_reach_done"]
            assert task["base_goal_enable"]
            assert task["base_goal_reach_done"]
            assert task["base_goal_velocity"] == goal_velocity
            assert task["obstacle_velocity"] == (obstacle_velocity if obstacle_count else 0.0)
            assert task["num_obstacle_task"] == obstacle_count


def test_every_case_is_a_complete_self_contained_task_dictionary():
    required = {
        "class_name",
        "max_episode_length",
        "fall_height_threshold",
        "seed",
        "reset_on_success",
        "reset_on_timeout",
        "completion_mode",
        "arm_goal_enable",
        "left_arm_goal_range",
        "right_arm_goal_range",
        "base_goal_enable",
        "base_goal_range",
        "base_goal_rot_range",
        "obstacle_mode",
        "num_obstacle_task",
        "obstacle_range",
    }
    for name in list_benchmark_test_cases():
        task = resolve_benchmark_test_case(name).task
        assert required <= set(task), name
        assert not _FORBIDDEN.intersection(task), name


def test_v0_is_obstacle_free_goal_reaching():
    task = resolve_benchmark_test_case("whole_goal_static_v0").task
    assert task["arm_goal_enable"]
    assert task["base_goal_enable"]
    assert task["num_obstacle_task"] == 0
    assert task["obstacle_keepout"] == 0.0


def test_legacy_density_names_resolve_to_versioned_levels():
    assert resolve_benchmark_test_case("whole_goal_static_sparse").name.endswith("v1")
    assert resolve_benchmark_test_case("whole_goal_static_dense").name.endswith("v2")


def test_historical_robot_prefixed_names_resolve_for_all_examples():
    names = (
        "AgiBotG1FixedBase_D1_AG_SO_v0",
        "FanucLRMate200iDSingleArm_D2_AG_SO_v0",
        "GalaxeaR1LiteFixedBase_D2_AG_SO_v0",
        "KinovaGen3SingleArm_D2_AG_SO_v0",
        "KukaIIWA14SingleArm_D2_AG_SO_v0",
        "UnitreeG1WholeBody_D1_WG_SO_v0",
    )
    for name in names:
        assert resolve_benchmark_test_case(name).name.endswith("v1")


def test_robot_config_declares_default_mujoco_agent():
    assert (
        get_agent_class_name("UnitreeG1FixedBaseDynamic1Config", "mujoco")
        == "UnitreeG1FixedBaseMujocoAgent"
    )
    assert (
        get_agent_class_name("UnitreeG1WholeBodyWithHandDynamic1Config", "mujoco")
        == "UnitreeG1WholeBodyMujocoAgent"
    )


def test_benchmark_task_terminates_a_fallen_robot():
    task = BenchmarkTask.__new__(BenchmarkTask)
    task.arm_goal_enable = False
    task.base_goal_enable = False
    task.arm_goal_reach_done = False
    task.base_goal_reach_done = False
    task.max_episode_length = 500
    task.episode_length = 10
    task.reset_on_success = True
    task.reset_on_timeout = True
    task.completion_mode = "all_enabled_goals"
    task.resample_arm_goals_on_reset = True
    task.resample_base_goal_on_reset = True
    task.resample_obstacles_on_reset = True
    task.fall_height_threshold = 0.45
    task.robot_base_frame = np.eye(4)
    task.robot_base_frame[2, 3] = 0.2
    task._update_done()
    assert task._done
    assert task._done_info["reason"] == "fallen"


def test_benchmark_task_completes_a_single_arm_goal_without_a_left_arm():
    task = BenchmarkTask.__new__(BenchmarkTask)
    task.arm_goal_enable = True
    task.base_goal_enable = False
    task.use_dual_arm = False
    task.arm_goal_reach_done = True
    task.base_goal_reach_done = False
    task.arm_goal_size = 0.05
    task.max_episode_length = 500
    task.episode_length = 10
    task.reset_on_success = True
    task.reset_on_timeout = True
    task.completion_mode = "all_enabled_goals"
    task.resample_arm_goals_on_reset = True
    task.resample_base_goal_on_reset = True
    task.resample_obstacles_on_reset = True
    task.fall_height_threshold = None
    task.robot_base_frame = np.eye(4)
    task.robot_cfg = SimpleNamespace(Frames=SimpleNamespace(R_ee=0))
    task.robot_frames_world = np.eye(4)[None]
    task.robot_goal_right = SimpleNamespace(frame=np.eye(4))

    task._update_done()

    assert task._done
    assert task._done_info["arm_goal_reached"]
    assert task._done_info["arm_goal_distance_left"] is None
    assert task._done_info["arm_goal_distance_right"] == 0.0


def test_benchmark_base_goal_requires_position_and_yaw():
    task = BenchmarkTask.__new__(BenchmarkTask)
    task.arm_goal_enable = False
    task.base_goal_enable = True
    task.arm_goal_reach_done = False
    task.base_goal_reach_done = True
    task.base_goal_size = 0.15
    task.base_goal_yaw_size = 0.10
    task.max_episode_length = 500
    task.episode_length = 10
    task.reset_on_success = True
    task.reset_on_timeout = True
    task.completion_mode = "all_enabled_goals"
    task.resample_arm_goals_on_reset = True
    task.resample_base_goal_on_reset = True
    task.resample_obstacles_on_reset = True
    task.fall_height_threshold = 0.45
    task.robot_base_frame = np.eye(4)
    task.robot_base_frame[2, 3] = 0.793
    goal = np.eye(4)
    yaw = 0.30
    goal[:2, :2] = [[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]]
    task.robot_goal_base = SimpleNamespace(frame=goal)

    task._update_done()
    assert not task._done
    assert not task._done_info["base_goal_reached"]
    assert np.isclose(task._done_info["base_goal_yaw_error"], yaw)

    task.robot_base_frame[:2, :2] = goal[:2, :2]
    task._update_done()
    assert task._done
    assert task._done_info["base_goal_reached"]
