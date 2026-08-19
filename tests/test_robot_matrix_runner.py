"""Pure-Python contracts for the cross-robot conformance runner."""

from __future__ import annotations

import ast
import importlib.util
import json
from pathlib import Path
import subprocess
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import numpy as np
import pytest

from spark_robot import (
    AgiBotG1DualArmDynamic1Config,
    AgiBotG1FixedBaseDynamic1Config,
    AgiBotG1FixedBaseKinematics,
    AgiBotG1MobileBaseDynamic1Config,
    AgiBotG1MobileBaseDynamic2Config,
    AgiBotG1MobileBaseBicycleDynamic2Config,
    AgiBotG1MobileBaseUnicycleDynamic1Config,
    AgiBotG1RightArmDynamic1Config,
    FanucLRMate200iDSingleArmDynamic2Config,
    GalaxeaR1LiteMobileBaseDynamic1Config,
    KinovaGen3SingleArmDynamic1Config,
    SPARK_ROBOT_RESOURCE_DIR,
    UnitreeG1RightArmDynamic1Config,
)


ROOT = Path(__file__).resolve().parents[1]


def _load_module(name: str, relative_path: str):
    path = ROOT / relative_path
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_tensor_benchmarks_share_batched_distance_query():
    """Keep whole-body and manipulator runtimes on the common tensor backend."""
    for relative_path in (
        "pipeline/spark_pipeline/autonomy/agibot_g1_isaac_tensor_benchmark.py",
        "pipeline/spark_pipeline/autonomy/isaac_tensor_manipulator_benchmark.py",
        "pipeline/spark_pipeline/autonomy/unitree_g1_isaac_tensor_benchmark.py",
    ):
        tree = ast.parse((ROOT / relative_path).read_text(encoding="utf-8"))
        imported_names = {
            alias.name
            for node in ast.walk(tree)
            if isinstance(node, ast.ImportFrom) and node.module == "spark_policy.safety.geometry"
            for alias in node.names
        }
        called_methods = {
            node.func.attr
            for node in ast.walk(tree)
            if isinstance(node, ast.Call) and isinstance(node.func, ast.Attribute)
        }
        assert "TorchSphereCollisionBackend" in imported_names, relative_path
        assert "PointCloudBatch" in imported_names, relative_path
        assert "query_environment_nearest" in called_methods, relative_path


def test_matrix_runner_selects_declared_dynamics_control_axes():
    runner = _load_module("spark_matrix_case_test", "example/run_robot_matrix_case.py")

    d1 = KinovaGen3SingleArmDynamic1Config()
    dynamics, mapping = runner._control_mapping(d1)
    assert dynamics.order == 1
    assert mapping[2:] == ("vShoulderPan", "ShoulderPan")

    d2 = FanucLRMate200iDSingleArmDynamic2Config()
    dynamics, mapping = runner._control_mapping(d2)
    assert dynamics.order == 2
    assert mapping[2:] == ("ajoint_1", "joint_1")

    agibot = AgiBotG1MobileBaseDynamic1Config()
    _, mapping = runner._control_mapping(agibot)
    assert mapping[2:] == ("vRightJoint1", "RightJoint1")


def test_matrix_runner_mujoco_feedback_does_not_require_unused_control_field():
    runner = _load_module("spark_matrix_feedback_test", "example/run_robot_matrix_case.py")
    record = {
        "dof_pos_fbk": np.array([0.1]),
        "dof_vel_fbk": np.array([0.2]),
        "dof_pos_cmd": np.array([0.3]),
        "dof_vel_cmd": np.array([0.4]),
    }
    agent = SimpleNamespace(get_feedback=lambda: record)
    bundle = SimpleNamespace(agents=(agent,), backend="mujoco")

    feedback = runner._feedback(bundle)

    assert set(feedback) == set(record)
    assert all(value.shape == (1, 1) for value in feedback.values())


def test_matrix_runner_prefers_unitree_case_names_and_accepts_legacy_aliases():
    runner = _load_module("spark_matrix_case_version_test", "example/run_robot_matrix_case.py")

    names = (
        "arm_goal_static_v0",
        "arm_goal_static_v1",
        "joint_goal_reaching_v0",
        "joint_goal_reaching_v1",
    )
    for name in names:
        args = runner._parser().parse_args(
            ["--robot-config", "AgiBotG1FixedBaseDynamic1Config", "--test-case", name]
        )
        assert args.test_case == name

    defaults = runner._parser().parse_args(["--robot-config", "AgiBotG1FixedBaseDynamic1Config"])
    assert defaults.test_case == "arm_goal_static_v0"


@pytest.mark.parametrize(
    "relative_path",
    (
        "example/agibot_g1/run_agibot_g1_teleop.py",
        "example/fanuc_lrmate200id/run_fanuc_lrmate200id_teleop.py",
        "example/galaxea_r1lite/run_galaxea_r1lite_teleop.py",
        "example/kinova_gen3/run_kinova_gen3_teleop.py",
        "example/kuka_iiwa14/run_kuka_iiwa14_teleop.py",
        "example/unitree_g1/run_unitree_g1_teleop.py",
    ),
)
def test_safe_teleop_scripts_have_one_display_switch_and_default_to_viewer(relative_path):
    module_name = "headless_cli_" + relative_path.replace("/", "_").replace(".", "_")
    teleop = _load_module(module_name, relative_path)

    assert teleop._parse_cli_args([])["enable_viewer"] is True
    assert teleop._parse_cli_args(["--headless"])["enable_viewer"] is False


def test_agibot_teleop_defaults_safety_index_to_the_configured_dynamics_order():
    teleop = _load_module(
        "spark_agibot_ordered_safety_index",
        "example/agibot_g1/run_agibot_g1_teleop.py",
    )

    first_order = teleop.config_safety_module(
        teleop.PipelineConfig(),
        robot_cfg="AgiBotG1MobileBaseUnicycleDynamic1Config",
        safe_algo="bypass",
        safety_index=None,
        minimum_distance=0.05,
    )
    second_order = teleop.config_safety_module(
        teleop.PipelineConfig(),
        robot_cfg="AgiBotG1MobileBaseBicycleDynamic2Config",
        safe_algo="bypass",
        safety_index=None,
        minimum_distance=0.05,
    )

    assert (
        first_order.policy.safe_controller.safety_index.class_name
        == "FirstOrderCollisionSafetyIndex"
    )
    assert (
        second_order.policy.safe_controller.safety_index.class_name
        == "SecondOrderCollisionSafetyIndex"
    )


@pytest.mark.parametrize("backend", ("mujoco", "isaac"))
def test_kinova_teleop_enables_keyboard_gripper_control(backend):
    teleop = _load_module(
        f"spark_kinova_gripper_teleop_{backend}",
        "example/kinova_gen3/run_kinova_gen3_teleop.py",
    )
    kwargs = {
        "backend": backend,
        "robot_cfg": "KinovaGen3SingleArmDynamic1CollisionConfig",
        "agent_cfg": (
            "KinovaGen3SingleArmAgent" if backend == "mujoco" else "KinovaGen3SingleArmIsaacAgent"
        ),
        "enable_viewer": True,
    }
    cfg = teleop.config_pipeline(teleop.PipelineConfig(), **kwargs)
    cfg = teleop.config_agent_module(cfg, **kwargs)

    assert cfg.env.agent.enable_keyboard_control is True
    assert cfg.env.agent.enable_hand_control is True


@pytest.mark.parametrize("backend", ("mujoco", "isaac"))
def test_fanuc_teleop_enables_keyboard_three_finger_gripper_control(backend):
    teleop = _load_module(
        f"spark_fanuc_gripper_teleop_{backend}",
        "example/fanuc_lrmate200id/run_fanuc_lrmate200id_teleop.py",
    )
    kwargs = {
        "backend": backend,
        "robot_cfg": "FanucLRMate200iDSingleArmDynamic1CollisionConfig",
        "agent_cfg": (
            "FanucLRMate200iDSingleArmAgent"
            if backend == "mujoco"
            else "FanucLRMate200iDSingleArmIsaacAgent"
        ),
        "enable_viewer": True,
    }
    cfg = teleop.config_pipeline(teleop.PipelineConfig(), **kwargs)
    cfg = teleop.config_agent_module(cfg, **kwargs)

    assert cfg.env.agent.enable_keyboard_control is True
    assert cfg.env.agent.enable_hand_control is True


def test_fanuc_teleop_separates_physx_contact_from_sphere_pair_safety():
    teleop = _load_module(
        "spark_fanuc_self_contact_cli",
        "example/fanuc_lrmate200id/run_fanuc_lrmate200id_teleop.py",
    )
    default_arguments = teleop._parse_cli_args(["--backend", "isaac"])
    default_arguments["agent_cfg"] = "FanucLRMate200iDSingleArmIsaacAgent"
    default_cfg = teleop.config_pipeline(teleop.PipelineConfig(), **default_arguments)
    default_cfg = teleop.config_agent_module(default_cfg, **default_arguments)
    default_cfg = teleop.config_safety_module(default_cfg, **default_arguments)

    arguments = teleop._parse_cli_args(["--backend", "isaac", "--enable-self-collision"])
    arguments["agent_cfg"] = "FanucLRMate200iDSingleArmIsaacAgent"
    cfg = teleop.config_pipeline(teleop.PipelineConfig(), **arguments)
    cfg = teleop.config_agent_module(cfg, **arguments)
    cfg = teleop.config_safety_module(cfg, **arguments)

    assert default_cfg.env.agent.allow_self_collision is True
    assert default_cfg.policy.safe_controller.safety_index.enable_self_collision is False
    assert cfg.env.agent.allow_self_collision is True
    assert cfg.policy.safe_controller.safety_index.enable_self_collision is True


def test_matrix_runner_defaults_to_ten_resets_and_auto_selects_isaac_device():
    runner = _load_module("spark_matrix_reset_test", "example/run_robot_matrix_case.py")

    single = runner._parser().parse_args(
        ["--robot-config", "AgiBotG1MobileBaseDynamic1Config", "--num-envs", "1"]
    )
    assert single.num_resets == 10
    assert single.seed == 0
    assert single.device is None
    assert single.render_every is None
    assert single.max_visualized_envs == 1
    assert single.headless is False
    assert single.max_episode_steps == 1000
    runner._validate_args(single)
    assert single.device == "cpu"

    batch = runner._parser().parse_args(
        ["--robot-config", "AgiBotG1MobileBaseDynamic1Config", "--num-envs", "32"]
    )
    assert batch.device is None
    runner._validate_args(batch)
    assert batch.device == "cuda:0"

    continuous = runner._parser().parse_args(
        [
            "--robot-config",
            "AgiBotG1MobileBaseDynamic1Config",
            "--num-resets",
            "-1",
        ]
    )
    runner._validate_args(continuous)
    assert continuous.num_resets == -1


def test_parallel_goal_sampling_is_row_local_and_resamples_each_reset():
    import numpy as np

    runner = _load_module("spark_matrix_goal_sampling_test", "example/run_robot_matrix_case.py")

    first = runner._sample_goal_offsets(16, 0.4, seed=7, reset_index=0)
    repeated = runner._sample_goal_offsets(16, 0.4, seed=7, reset_index=0)
    second = runner._sample_goal_offsets(16, 0.4, seed=7, reset_index=1)

    assert first == pytest.approx(repeated)
    assert len(set(first)) == 16
    assert np.all(np.abs(first) >= 0.14)
    assert first != pytest.approx(second)


def test_parallel_episode_quota_waits_for_every_row_without_freezing_fast_rows():
    from spark_pipeline.autonomy.benchmark_lifecycle import reset_quota_reached

    assert not reset_quota_reached(np.array([10, 9, 12]), 10)
    assert reset_quota_reached(np.array([10, 10, 12]), 10)
    assert not reset_quota_reached(np.array([100, 100, 100]), -1)
    with pytest.raises(ValueError, match="positive or -1"):
        reset_quota_reached(np.array([1, 1]), 0)


def test_multi_reset_report_preserves_episode_metrics_and_variance():
    runner = _load_module("spark_matrix_aggregate_test", "example/run_robot_matrix_case.py")
    reports = []
    for reset_index, goal_error in enumerate((0.01, 0.02, 0.03)):
        reports.append(
            {
                "status": "PASS",
                "finite": True,
                "num_envs": 2,
                "steps": 10,
                "simulated_seconds": 0.2,
                "wall_seconds": 0.1,
                "max_goal_error": goal_error,
                "max_abs_control": 0.2,
                "motion_span": 0.1,
                "reset_initial_position_error": 0.0,
                "reset_initial_velocity_max": 0.0,
                "stage_seconds": {"physics": 0.08, "feedback": 0.01},
                "reset_index": reset_index,
            }
        )

    aggregate = runner._aggregate_reset_reports(reports)

    assert aggregate["status"] == "PASS"
    assert aggregate["num_resets"] == 3
    assert aggregate["steps"] == 30
    assert aggregate["max_goal_error"] == pytest.approx(0.03)
    assert aggregate["goal_error_mean"] == pytest.approx(0.02)
    assert aggregate["goal_error_std"] > 0.0
    assert aggregate["aggregate_control_hz"] == pytest.approx(200.0)
    assert "reset_index" not in aggregate


def test_matrix_runner_detects_velocity_only_planar_servos():
    runner = _load_module("spark_matrix_servo_test", "example/run_robot_matrix_case.py")

    mobile = GalaxeaR1LiteMobileBaseDynamic1Config()
    _, mapping = runner._control_mapping(mobile, "vLinearX")
    assert runner._mujoco_control_stiffness(mobile, mapping[0]) == 0.0

    arm = KinovaGen3SingleArmDynamic1Config()
    _, mapping = runner._control_mapping(arm, "vShoulderPan")
    assert runner._mujoco_control_stiffness(arm, mapping[0]) > 0.0


def test_agibot_planar_visual_mesh_is_not_physics_collision_geometry():
    config = AgiBotG1MobileBaseUnicycleDynamic1Config()
    root = ET.parse(Path(SPARK_ROBOT_RESOURCE_DIR) / config.mujoco_model_path).getroot()
    visual = root.find(".//geom[@name='robot_visual']")
    assert visual is not None
    assert visual.attrib["contype"] == "0"
    assert visual.attrib["conaffinity"] == "0"
    # SPARK reserves key 2 for goal yaw. Keep the base out of MuJoCo visual
    # group 2 so the viewer's native shortcut cannot flash this mesh.
    assert visual.attrib["group"] == "1"


def test_agibot_foam_geometry_covers_the_whole_body_and_matches_gripper_centers():
    import numpy as np

    config = AgiBotG1FixedBaseDynamic1Config()
    assert len(config.Frames) == len(config.CollisionVol) == 120
    assert max(geometry.attributes["radius"] for geometry in config.CollisionVol.values()) < 0.24
    frame_names = {frame.name for frame in config.Frames}
    for marker in (
        "collision_base_link_",
        "collision_link_pitch_body_",
        "collision_link_pitch_head_",
        "collision_link4_l_",
        "collision_link4_r_",
        "collision_left_base_link_",
        "collision_right_base_link_",
    ):
        assert any(name.startswith(marker) for name in frame_names), marker

    kinematics = AgiBotG1FixedBaseKinematics(config)
    position = np.asarray([config.DefaultDoFVal[dof] for dof in config.DoFs])
    kinematics.pre_computation(position)
    model, data = kinematics.model, kinematics.data
    for ee_name, link_name in (("L_ee", "Link7_l"), ("R_ee", "Link7_r")):
        ee = data.oMf[model.getFrameId(ee_name)]
        link = data.oMf[model.getFrameId(link_name)]
        local_offset = link.rotation.T @ (ee.translation - link.translation)
        assert local_offset == pytest.approx([0.0, 0.0, 0.10])

    mobile = AgiBotG1MobileBaseDynamic1Config()
    assert mobile.CollisionVolLocalOffsets[mobile.Frames.R_ee] == pytest.approx([0.0, 0.0, 0.10])
    assert mobile.CollisionVolLocalOffsets[mobile.Frames.L_ee] == pytest.approx([0.0, 0.0, 0.10])


def test_all_agibot_embodiments_share_one_sparse_collision_model():
    configs = [
        AgiBotG1FixedBaseDynamic1Config(),
        AgiBotG1RightArmDynamic1Config(),
        AgiBotG1DualArmDynamic1Config(),
        AgiBotG1MobileBaseDynamic1Config(),
        AgiBotG1MobileBaseDynamic2Config(),
        AgiBotG1MobileBaseUnicycleDynamic1Config(),
        AgiBotG1MobileBaseBicycleDynamic2Config(),
    ]

    collision_model = configs[0].CollisionVol
    assert len(collision_model) == 120
    assert all(config.CollisionVol is collision_model for config in configs[1:])
    for config in (configs[0], configs[3], configs[4], configs[5], configs[6]):
        overrides = {
            name: (stiffness, damping)
            for name, stiffness, damping in config.isaac_articulation.joint_gain_overrides
        }
        assert overrides["joint_lift_body"] == (40000.0, 400.0)


def test_every_non_unitree_robot_routes_isaac_to_a_family_specific_agent():
    import inspect

    import spark_agent
    import spark_robot
    from spark_robot import RobotConfig

    expected_prefixes = {
        "AgiBotG1": "AgiBotG1",
        "FanucLRMate200iD": "FanucLRMate200iD",
        "GalaxeaR1Lite": "GalaxeaR1Lite",
        "KinovaGen3": "KinovaGen3",
        "KukaIIWA14": "KukaIIWA14",
    }
    checked = []
    for name in dir(spark_robot):
        expected = next(
            (prefix for family, prefix in expected_prefixes.items() if name.startswith(family)),
            None,
        )
        config_type = getattr(spark_robot, name)
        if (
            expected is None
            or not inspect.isclass(config_type)
            or not issubclass(config_type, RobotConfig)
            or inspect.isabstract(config_type)
        ):
            continue
        config = config_type()
        capability = config.backend_capabilities().get("isaac")
        if capability is None:
            continue
        assert capability.agent_class_name.startswith(expected)
        assert capability.agent_class_name.endswith("IsaacAgent")
        assert getattr(spark_agent, capability.agent_class_name).__name__ == (
            capability.agent_class_name
        )
        checked.append(name)

    assert checked


def test_benchmark_obstacles_start_outside_agibot_collision_geometry():
    import numpy as np

    runner = _load_module("spark_matrix_obstacle_test", "example/run_robot_matrix_case.py")
    config = AgiBotG1FixedBaseDynamic1Config()
    kinematics = AgiBotG1FixedBaseKinematics(config)
    position = np.asarray([config.DefaultDoFVal[dof] for dof in config.DoFs])
    frames = kinematics.forward_kinematics(position)
    collision_items = list(config.CollisionVol.items())
    obstacles = runner._deterministic_obstacle_ring(frames, collision_items, 5, 0.05)

    assert len(obstacles) == 5
    assert all(np.linalg.norm(obstacle) > 0.1 for obstacle in obstacles)
    obstacle_radius = 0.05
    closest_clearances = []
    for obstacle in obstacles:
        obstacle_clearances = []
        for frame_id, geometry in collision_items:
            center = frames[int(frame_id), :3, 3]
            minimum_distance = obstacle_radius + runner._geometry_bounding_radius(geometry)
            clearance = np.linalg.norm(obstacle - center) - minimum_distance
            obstacle_clearances.append(clearance)
            assert clearance >= 0.08 - 1.0e-9
        closest_clearances.append(min(obstacle_clearances))
    assert closest_clearances == pytest.approx([0.08] * 5)


def test_benchmark_visualizer_uses_full_robot_model_and_sphere_obstacles():
    import numpy as np
    from spark_pipeline import resolve_benchmark_test_case

    runner = _load_module("spark_matrix_visualizer_test", "example/run_robot_matrix_case.py")
    config = AgiBotG1FixedBaseDynamic1Config()
    position = np.repeat(
        np.asarray([[config.DefaultDoFVal[dof] for dof in config.DoFs]]),
        3,
        axis=0,
    )
    target = position.copy()
    test_case = resolve_benchmark_test_case("arm_goal_static_v1")
    visualizer = runner._BenchmarkVisualizer(
        config,
        SimpleNamespace(env_positions=np.zeros((3, 3))),
        "isaac",
        position,
        target,
        test_case,
    )

    assert len(visualizer.collision_items) == len(config.CollisionVol) == 120
    assert visualizer.visualized_env_ids == (0, 1, 2)
    assert visualizer.collision_volume_env_ids == (0,)
    assert visualizer.obstacle_positions.shape == (
        3,
        test_case.task["num_obstacle_task"],
        3,
    )
    assert not np.allclose(
        visualizer.obstacle_positions[0],
        visualizer.obstacle_positions[1],
    )
    first_obstacles = {tuple(np.round(row[0], decimals=6)) for row in visualizer.obstacle_positions}
    assert len(first_obstacles) == 3

    next_visualizer = runner._BenchmarkVisualizer(
        config,
        SimpleNamespace(env_positions=np.zeros((3, 3))),
        "isaac",
        position,
        target,
        test_case,
        reset_index=1,
    )
    assert not np.allclose(
        visualizer.obstacle_positions,
        next_visualizer.obstacle_positions,
    )
    assert len(visualizer.obstacle_geometries) == test_case.task["num_obstacle_task"]
    assert all(geometry.type == "sphere" for geometry in visualizer.obstacle_geometries)


@pytest.mark.parametrize("test_case", ("whole_goal_static_v0", "whole_goal_static_v1"))
def test_agibot_benchmark_uses_shared_whole_body_case_and_mobile_agent(test_case):
    runner = _load_module(
        "spark_agibot_benchmark_wrapper_test",
        "example/agibot_g1/run_agibot_g1_benchmark.py",
    )
    from spark_pipeline import resolve_benchmark_test_case

    args = runner._parser().parse_args([])
    assert args.test_case == "whole_goal_static_v0"
    assert args.num_resets == 10
    assert args.device is None
    assert args.viewer is True
    assert args.max_episode_length == 1000

    cfg = runner.build_config(**{**vars(args), "test_case": test_case})
    shared = resolve_benchmark_test_case(test_case).task
    assert cfg.env.agent.class_name == "AgiBotG1MobileBaseAgent"
    assert cfg.env.task.arm_goal_enable
    assert cfg.env.task.base_goal_enable
    for role, field_name in (
        ("left_arm_goal", "left_arm_goal_range"),
        ("right_arm_goal", "right_arm_goal_range"),
    ):
        offset = runner.BENCHMARK_GOAL_OFFSETS[role]
        expected = [
            (low + delta, high + delta) for (low, high), delta in zip(shared[field_name], offset)
        ]
        assert getattr(cfg.env.task, field_name) == pytest.approx(expected)
        # Robot grounding must not modify the shared task case.
        assert resolve_benchmark_test_case(test_case).task[field_name] == shared[field_name]
    assert cfg.env.task.base_goal_range == shared["base_goal_range"]
    assert cfg.env.agent.device == "cpu"
    assert cfg.max_num_reset == 10
    assert cfg.env.task.max_episode_length == 1000
    assert cfg.max_num_steps == 10000
    assert cfg.env.agent.enable_viewer is True
    assert cfg.env.task.fall_height_threshold is None


def test_agibot_shared_arm_case_selects_only_the_right_arm_goal():
    runner = _load_module(
        "spark_agibot_single_arm_benchmark_test",
        "example/agibot_g1/run_agibot_g1_benchmark.py",
    )

    cfg = runner.build_config(test_case="arm_goal_static_v0", backend="mujoco")
    assert cfg.robot.cfg.class_name == "AgiBotG1RightArmDynamic1Config"
    assert cfg.env.agent.class_name == "AgiBotG1RightArmAgent"
    assert cfg.env.task.use_dual_arm is False
    assert cfg.env.task.base_goal_enable is False


def test_unitree_shared_arm_case_supports_reduced_right_arm_config():
    runner = _load_module(
        "spark_unitree_single_arm_benchmark_test",
        "example/unitree_g1/run_unitree_g1_benchmark.py",
    )
    arguments = vars(
        runner.build_parser().parse_args(
            [
                "--robot-config",
                "UnitreeG1RightArmDynamic1Config",
                "--test-case",
                "arm_goal_static_v0",
                "--headless",
            ]
        )
    )

    cfg = runner.build_config(**arguments)
    assert cfg.env.task.use_dual_arm is False
    assert cfg.env.agent.class_name == "UnitreeG1RightArmMujocoAgent"
    timing = UnitreeG1RightArmDynamic1Config().simulator_dynamics
    assert cfg.env.agent.dt == pytest.approx(timing.physics_dt)
    assert cfg.env.agent.control_decimation == timing.control_decimation
    assert cfg.env.task.dt == pytest.approx(timing.control_period)


def test_agibot_benchmark_offsets_are_runner_local_and_agibot_grounded():
    runner = _load_module(
        "spark_agibot_benchmark_offset_test",
        "example/agibot_g1/run_agibot_g1_benchmark.py",
    )

    assert not hasattr(AgiBotG1MobileBaseDynamic1Config, "benchmark_goal_offsets")
    assert set(runner.BENCHMARK_GOAL_OFFSETS.values()) == {(0.25, 0.0, 0.5)}
    assert "arm_goal_z_offset" not in vars(runner._parser().parse_args([]))


def test_agibot_v1_uses_shared_obstacles_and_safety_filter():
    runner = _load_module(
        "spark_agibot_benchmark_v1_test",
        "example/agibot_g1/run_agibot_g1_benchmark.py",
    )
    from spark_pipeline import resolve_benchmark_test_case

    cfg = runner.build_config(test_case="whole_goal_static_v1", backend="mujoco")
    shared = resolve_benchmark_test_case("whole_goal_static_v1").task
    assert cfg.env.task.num_obstacle_task == shared["num_obstacle_task"] == 10
    assert cfg.env.task.obstacle_range == shared["obstacle_range"]
    assert cfg.policy.safe_controller.safe_algo.class_name == "RelaxedSafeSetAlgorithm"


@pytest.mark.parametrize("num_envs", (1, 16))
def test_agibot_isaac_benchmarks_share_tensor_whole_body_route(monkeypatch, num_envs):
    runner = _load_module(
        f"spark_agibot_tensor_route_{num_envs}",
        "example/agibot_g1/run_agibot_g1_benchmark.py",
    )
    captured = {}

    def fake_tensor(kwargs):
        captured.update(kwargs)
        return "tensor"

    monkeypatch.setattr(runner, "_run_isaac_tensor_benchmark", fake_tensor)
    result = runner.run(
        backend="isaac",
        robot_config="AgiBotG1MobileBaseDynamic1Config",
        test_case="whole_goal_static_v1",
        num_envs=num_envs,
        dynamics_backend="simulator",
    )

    assert result == "tensor"
    assert captured["num_envs"] == num_envs
    assert captured["test_case"] == "whole_goal_static_v1"


@pytest.mark.parametrize("test_case", ("whole_goal_static_v0", "whole_goal_static_v1"))
def test_agibot_tensor_command_preserves_agibot_timing(monkeypatch, test_case):
    runner = _load_module(
        "spark_agibot_tensor_command",
        "example/agibot_g1/run_agibot_g1_benchmark.py",
    )
    captured = {}

    def fake_run(command, *, check, env):
        captured.update(command=command, check=check, env=env)
        return "launched"

    monkeypatch.setattr(runner.subprocess, "run", fake_run)
    result = runner._run_isaac_tensor_benchmark(
        {
            "backend": "isaac",
            "robot_config": "AgiBotG1MobileBaseDynamic1Config",
            "test_case": test_case,
            "num_envs": 16,
            "num_resets": 10,
            "max_episode_length": 1000,
            "max_num_steps": 25,
            "viewer": False,
            "profile_frequency": True,
        }
    )

    command = captured["command"]
    dynamics = AgiBotG1MobileBaseDynamic1Config().simulator_dynamics
    assert result == "launched"
    assert captured["check"] is True
    assert command[command.index("--dt") + 1] == str(dynamics.physics_dt)
    assert command[command.index("--control-decimation") + 1] == str(dynamics.control_decimation)
    assert command[command.index("--num-envs") + 1] == "16"
    assert command[command.index("--test-case") + 1] == test_case
    assert "--headless" in command
    assert "--profile-frequency" in command
    assert "--render-safety-trigger-constraints" in command
    assert "--render-safety-violations" in command
    assert json.loads(captured["env"]["SPARK_AGIBOT_BENCHMARK_GOAL_OFFSETS"]) == {
        role: list(offset) for role, offset in runner.BENCHMARK_GOAL_OFFSETS.items()
    }


def test_agibot_tensor_viewer_uses_batched_closest_pair_segments():
    runtime_source = (
        ROOT / "pipeline/spark_pipeline/autonomy/agibot_g1_isaac_tensor_benchmark.py"
    ).read_text(encoding="utf-8")
    agent_source = (
        ROOT / "module/spark_agent/spark_agent/simulation/isaac/configured_tensor_agent.py"
    ).read_text(encoding="utf-8")

    assert "query.witness_robot" in runtime_source
    assert "query.witness_environment" in runtime_source
    assert "agent.set_visual_safety_constraints" in runtime_source
    assert "agent.set_visual_obstacles" in runtime_source
    assert "def set_visual_safety_constraints" in agent_source
    assert "def set_visual_obstacles" in agent_source


@pytest.mark.parametrize(
    ("family", "default_config", "dual_config"),
    (
        (
            "kinova_gen3",
            "KinovaGen3SingleArmDynamic1CollisionConfig",
            "KinovaGen3DualArmDynamic1CollisionConfig",
        ),
        (
            "kuka_iiwa14",
            "KukaIIWA14SingleArmDynamic1CollisionConfig",
            "KukaIIWA14DualArmDynamic1CollisionConfig",
        ),
        (
            "fanuc_lrmate200id",
            "FanucLRMate200iDSingleArmDynamic1CollisionConfig",
            "FanucLRMate200iDDualArmDynamic1CollisionConfig",
        ),
    ),
)
def test_fixed_base_family_benchmarks_bind_single_and_dual_collision_configs(
    family, default_config, dual_config
):
    launcher = _load_module(
        f"spark_{family}_benchmark_launcher_test",
        f"example/{family}/run_{family}_benchmark.py",
    )

    single = launcher.build_config(backend="mujoco", test_case="v0")
    dual = launcher.build_config(
        backend="mujoco", robot_config=dual_config, test_case="arm_goal_static_v1"
    )
    assert single.robot.cfg.class_name == default_config
    assert single.env.task.task_name == "arm_goal_static_v0"
    assert single.env.task.use_dual_arm is False
    assert dual.robot.cfg.class_name == dual_config
    assert dual.env.task.task_name == "arm_goal_static_v1"
    assert dual.env.task.use_dual_arm is True
    assert single.max_num_reset == dual.max_num_reset == 10
    assert single.env.task.max_episode_length == dual.env.task.max_episode_length == 1000
    assert single.policy.safe_controller.safety_index.enable_self_collision is False
    assert dual.policy.safe_controller.safety_index.enable_self_collision is False
    assert dual.env.agent.allow_self_collision is True

    forced_safety = launcher.build_config(
        backend="isaac",
        test_case="arm_goal_static_v0",
        enable_self_collision=True,
    )
    assert forced_safety.policy.safe_controller.safety_index.enable_self_collision is True
    import spark_robot

    expected_physical_contact = bool(
        getattr(spark_robot, default_config)().isaac_articulation.allow_self_collision
    )
    assert single.env.agent.allow_self_collision is expected_physical_contact
    assert forced_safety.env.agent.allow_self_collision is expected_physical_contact


def test_kinova_benchmark_uses_lower_identity_gripper_down_pose_targets():
    launcher = _load_module(
        "spark_kinova_pose_benchmark_launcher_test",
        "example/kinova_gen3/run_kinova_gen3_benchmark.py",
    )

    configured = launcher.build_config(backend="mujoco", test_case="arm_goal_static_v1")
    np.testing.assert_allclose(
        configured.env.task.right_arm_goal_rotation,
        np.eye(3),
    )
    assert configured.env.task.arm_goal_position_only is False
    assert configured.env.task.arm_goal_orientation_size == pytest.approx(0.1)
    assert configured.policy.nominal_controller.position_kp == pytest.approx(4.0)
    np.testing.assert_allclose(
        configured.env.task.right_arm_goal_range,
        ((0.30, 0.42), (-0.20, 0.05), (0.28, 0.42)),
    )
    assert configured.env.task.obstacle_range[2] == pytest.approx((0.20, 0.60))
    assert configured.env.task.robot_keepout == pytest.approx(0.17)

    dual = launcher.build_config(
        backend="isaac",
        robot_config="KinovaGen3DualArmDynamic1CollisionConfig",
        test_case="arm_goal_static_v1",
    )
    assert dual.env.task.arm_goal_pair_keepout == pytest.approx(0.25)


def test_kinova_enables_isaac_self_contact_only_for_the_dual_articulation():
    import spark_robot

    single = spark_robot.KinovaGen3SingleArmDynamic1CollisionConfig()
    dual = spark_robot.KinovaGen3DualArmDynamic1CollisionConfig()

    assert single.isaac_articulation.allow_self_collision is False
    assert dual.isaac_articulation.allow_self_collision is True


def test_kuka_benchmark_uses_shared_fixed_manipulator_pose_targets():
    launcher = _load_module(
        "spark_kuka_pose_benchmark_launcher_test",
        "example/kuka_iiwa14/run_kuka_iiwa14_benchmark.py",
    )

    single = launcher.build_config(backend="mujoco", test_case="arm_goal_static_v1")
    np.testing.assert_allclose(single.env.task.right_arm_goal_rotation, np.eye(3))
    assert single.env.task.arm_goal_position_only is False
    assert single.env.task.arm_goal_orientation_size == pytest.approx(0.1)
    assert single.policy.nominal_controller.position_kp == pytest.approx(4.0)
    np.testing.assert_allclose(
        single.env.task.right_arm_goal_range,
        ((0.20, 0.65), (-0.40, 0.40), (0.20, 0.70)),
    )
    np.testing.assert_allclose(
        single.env.task.obstacle_range, ((0.10, 0.60), (-0.40, 0.40), (0.20, 0.70))
    )
    assert single.env.task.robot_keepout == pytest.approx(0.17)
    assert single.env.task.arm_goal_minimum_distance == pytest.approx(0.25)
    assert single.env.task.validate_arm_goal_reachability is True
    np.testing.assert_allclose(
        launcher.BENCHMARK_EE_LOCAL_ROTATION,
        np.diag((1.0, -1.0, -1.0)),
    )

    dual = launcher.build_config(
        backend="isaac",
        robot_config="KukaIIWA14DualArmDynamic1CollisionConfig",
        test_case="arm_goal_static_v1",
    )
    assert dual.env.task.arm_goal_pair_keepout == pytest.approx(0.25)
    assert dual.env.task.right_arm_goal_range[1] == pytest.approx((-0.45, -0.05))
    assert dual.env.task.left_arm_goal_range[1] == pytest.approx((0.05, 0.45))


def test_fixed_manipulator_scalar_benchmark_enables_frequency_profile():
    launcher = _load_module(
        "spark_kuka_frequency_benchmark_launcher_test",
        "example/kuka_iiwa14/run_kuka_iiwa14_benchmark.py",
    )

    configured = launcher.build_config(
        backend="mujoco",
        test_case="arm_goal_static_v0",
        profile_frequency=True,
    )

    assert configured.profile_frequency is True


def test_kuka_parallel_isaac_benchmark_reduces_viewport_submission_rate(monkeypatch):
    launcher = _load_module(
        "spark_kuka_parallel_render_benchmark_launcher_test",
        "example/kuka_iiwa14/run_kuka_iiwa14_benchmark.py",
    )
    captured = {}

    def fake_tensor(kwargs, **_):
        captured.update(kwargs)
        return "launched"

    monkeypatch.setattr(launcher, "run_tensor_benchmark", fake_tensor)
    result = launcher.run(
        backend="isaac",
        robot_config="KukaIIWA14SingleArmDynamic1CollisionConfig",
        test_case="arm_goal_static_v1",
        num_envs=16,
    )

    assert result == "launched"
    assert captured["render_every"] == launcher.PARALLEL_ISAAC_RENDER_EVERY == 10


def test_kuka_isaac_articulations_enable_grippers_and_dual_arm_contact():
    import spark_robot

    single = spark_robot.KukaIIWA14SingleArmDynamic1CollisionConfig()
    dual = spark_robot.KukaIIWA14DualArmDynamic1CollisionConfig()

    assert [spec.side for spec in single.isaac_articulation.grippers] == ["right"]
    assert [spec.side for spec in dual.isaac_articulation.grippers] == ["left", "right"]
    assert single.isaac_articulation.allow_self_collision is False
    assert dual.isaac_articulation.allow_self_collision is True
    assert single.isaac_articulation.stiffness == pytest.approx(2000.0)
    assert single.isaac_articulation.damping == pytest.approx(160.0)
    assert single.simulator_dynamics.physics_dt == pytest.approx(0.005)
    assert single.simulator_dynamics.control_decimation == 4
    assert dual.simulator_dynamics.physics_dt == pytest.approx(0.005)
    assert dual.simulator_dynamics.control_decimation == 4


def test_r1lite_articulations_enable_physical_self_contact():
    import spark_robot

    configurations = (
        spark_robot.GalaxeaR1LiteRightArmDynamic1CollisionConfig(),
        spark_robot.GalaxeaR1LiteDualArmDynamic1CollisionConfig(),
        spark_robot.GalaxeaR1LiteFixedBaseDynamic1CollisionConfig(),
        spark_robot.GalaxeaR1LiteMobileBaseDynamic1CollisionConfig(),
    )

    assert all(config.isaac_articulation.allow_self_collision for config in configurations)


def test_agibot_articulations_enable_physical_self_contact():
    import spark_robot

    configurations = (
        spark_robot.AgiBotG1RightArmDynamic1Config(),
        spark_robot.AgiBotG1DualArmDynamic1Config(),
        spark_robot.AgiBotG1FixedBaseDynamic1Config(),
        spark_robot.AgiBotG1MobileBaseDynamic1Config(),
    )

    assert all(config.isaac_articulation.allow_self_collision for config in configurations)


def test_release_hold_gate_configs_select_declarative_isaac_subclasses():
    import spark_agent
    import spark_robot

    from spark_agent import ConfiguredIsaacAgent

    for config_name in (
        "KinovaGen3SingleArmDynamic1Config",
        "KukaIIWA14SingleArmDynamic2Config",
    ):
        config = getattr(spark_robot, config_name)()
        agent_type = getattr(spark_agent, config.agent_class_name("isaac"))
        assert issubclass(agent_type, ConfiguredIsaacAgent)


def test_fanuc_isaac_articulations_enable_three_finger_grippers_and_dual_arm_contact():
    import spark_robot

    single = spark_robot.FanucLRMate200iDSingleArmDynamic1CollisionConfig()
    dual = spark_robot.FanucLRMate200iDDualArmDynamic1CollisionConfig()

    assert [spec.side for spec in single.isaac_articulation.grippers] == ["right"]
    assert [spec.side for spec in dual.isaac_articulation.grippers] == ["left", "right"]
    assert len(single.isaac_articulation.grippers[0].joint_names) == 9
    assert single.isaac_articulation.allow_self_collision is True
    assert dual.isaac_articulation.allow_self_collision is True
    assert single.isaac_articulation.stiffness == pytest.approx(4000.0)
    assert single.isaac_articulation.damping == pytest.approx(250.0)
    assert single.simulator_dynamics.physics_dt == pytest.approx(0.005)
    assert single.simulator_dynamics.control_decimation == 4
    assert dual.simulator_dynamics.physics_dt == pytest.approx(0.005)
    assert dual.simulator_dynamics.control_decimation == 4


@pytest.mark.parametrize(
    ("family", "own_config"),
    (
        ("kinova_gen3", "KinovaGen3SingleArmDynamic1CollisionConfig"),
        ("kuka_iiwa14", "KukaIIWA14SingleArmDynamic1CollisionConfig"),
        ("fanuc_lrmate200id", "FanucLRMate200iDSingleArmDynamic1CollisionConfig"),
    ),
)
def test_fixed_base_family_benchmarks_reject_foreign_robot_configs(family, own_config):
    launcher = _load_module(
        f"spark_{family}_isolated_benchmark_launcher_test",
        f"example/{family}/run_{family}_benchmark.py",
    )

    own = launcher.build_config(backend="mujoco", robot_config=own_config)
    assert own.robot.cfg.class_name == own_config
    with pytest.raises(ValueError, match="Unsupported robot configuration"):
        launcher.build_config(backend="mujoco", robot_config="AgiBotG1FixedBaseDynamic1Config")


def test_r1lite_benchmark_cases_bind_arm_mode_to_sparse_config():
    wrapper = _load_module(
        "spark_galaxea_r1lite_cartesian_benchmark_test",
        "example/galaxea_r1lite/run_galaxea_r1lite_benchmark.py",
    )

    right_name, right_config, right = wrapper._grounding("arm_goal_static_v1", None)
    dual_name, dual_config, dual = wrapper._grounding(
        "arm_goal_static_v1", "GalaxeaR1LiteDualArmDynamic1CollisionConfig"
    )
    mobile_name, mobile_config, mobile = wrapper._grounding("whole_goal_static_v1", None)
    assert right_name == "arm_goal_static_v1"
    assert right_config == "GalaxeaR1LiteRightArmDynamic1CollisionConfig"
    assert right["use_dual_arm"] is False
    alias_name, alias_config, alias = wrapper._grounding("right_arm_goal_static_v0", None)
    assert alias_name == "arm_goal_static_v0"
    assert alias_config == "GalaxeaR1LiteRightArmDynamic1CollisionConfig"
    assert alias["use_dual_arm"] is False
    assert dual_name == "arm_goal_static_v1"
    assert dual_config == "GalaxeaR1LiteDualArmDynamic1CollisionConfig"
    assert dual["use_dual_arm"] is True
    assert mobile_name == "whole_goal_static_v1"
    assert mobile_config == "GalaxeaR1LiteMobileBaseDynamic1CollisionConfig"
    assert mobile["mobile_base"] is True
    with pytest.raises(ValueError, match="requires the mobile-base configuration"):
        wrapper._grounding("whole_goal_static_v0", dual_config)
    with pytest.raises(ValueError, match="requires a right-arm or dual-arm configuration"):
        wrapper._grounding("arm_goal_static_v0", mobile_config)


def test_r1lite_gripper_contract_matches_mujoco_model():
    import spark_robot

    resource = Path(SPARK_ROBOT_RESOURCE_DIR) / "galaxea_r1lite/r1lite_dual_arm.xml"
    root = ET.parse(resource).getroot()
    joints = {
        element.attrib["name"]: element
        for element in root.findall(".//joint")
        if "name" in element.attrib
    }
    actuators = {element.attrib["name"] for element in root.findall(".//actuator/*")}

    dual = spark_robot.GalaxeaR1LiteDualArmDynamic1CollisionConfig()
    right = spark_robot.GalaxeaR1LiteRightArmDynamic1CollisionConfig()
    mobile = spark_robot.GalaxeaR1LiteMobileBaseDynamic1CollisionConfig()
    assert [spec.side for spec in dual.isaac_articulation.grippers] == ["left", "right"]
    assert [spec.side for spec in right.isaac_articulation.grippers] == ["right"]
    assert [spec.side for spec in mobile.isaac_articulation.grippers] == ["left", "right"]
    assert not any(name.startswith("torso_") for name in right.isaac_articulation.joint_names)
    assert not any(name.startswith("torso_") for name in dual.isaac_articulation.joint_names)
    for spec in dual.isaac_articulation.grippers:
        assert set(spec.joint_names) <= joints.keys()
        assert set(spec.actuator_names) <= actuators
        for joint_name in spec.joint_names:
            assert float(joints[joint_name].attrib["damping"]) < 10.0
            assert float(joints[joint_name].attrib["armature"]) < 0.1


def test_kinova_isaac_gripper_contract_uses_movable_auxiliary_joints():
    import spark_robot

    resource = Path(SPARK_ROBOT_RESOURCE_DIR) / "kinova_gen3/gen3.urdf"
    joints = {
        element.attrib["name"]: element
        for element in ET.parse(resource).findall(".//joint")
        if "name" in element.attrib
    }
    single = spark_robot.KinovaGen3SingleArmDynamic1CollisionConfig()
    dual = spark_robot.KinovaGen3DualArmDynamic1CollisionConfig()

    assert [spec.side for spec in single.isaac_articulation.grippers] == ["right"]
    assert [spec.side for spec in dual.isaac_articulation.grippers] == ["left", "right"]
    single_gripper = single.isaac_articulation.grippers[0]
    assert single_gripper.open_positions == (0.0, 0.0)
    assert single_gripper.closed_positions == (-0.032, 0.032)
    for joint_name in single_gripper.joint_names:
        joint = joints[joint_name]
        assert joint.attrib["type"] == "prismatic"
        limit = joint.find("limit")
        assert limit is not None
        assert float(limit.attrib["lower"]) <= 0.0 <= float(limit.attrib["upper"])


def test_kuka_isaac_gripper_contract_uses_movable_auxiliary_joints():
    import spark_robot

    resource = Path(SPARK_ROBOT_RESOURCE_DIR) / "kuka_iiwa14/iiwa14.urdf"
    joints = {
        element.attrib["name"]: element
        for element in ET.parse(resource).findall(".//joint")
        if "name" in element.attrib
    }
    single = spark_robot.KukaIIWA14SingleArmDynamic1CollisionConfig()
    dual = spark_robot.KukaIIWA14DualArmDynamic1CollisionConfig()

    assert [spec.side for spec in single.isaac_articulation.grippers] == ["right"]
    assert [spec.side for spec in dual.isaac_articulation.grippers] == ["left", "right"]
    gripper = single.isaac_articulation.grippers[0]
    assert gripper.open_positions == (0.0, 0.0)
    assert gripper.closed_positions == (-0.032, 0.032)
    for joint_name in gripper.joint_names:
        joint = joints[joint_name]
        assert joint.attrib["type"] == "prismatic"
        limit = joint.find("limit")
        assert limit is not None
        assert float(limit.attrib["lower"]) <= 0.0 <= float(limit.attrib["upper"])


def test_fanuc_isaac_gripper_contract_uses_movable_auxiliary_joints():
    import spark_robot

    resource = Path(SPARK_ROBOT_RESOURCE_DIR) / "fanuc_lrmate200id/lrmate200id7l.urdf"
    joints = {
        element.attrib["name"]: element
        for element in ET.parse(resource).findall(".//joint")
        if "name" in element.attrib
    }
    single = spark_robot.FanucLRMate200iDSingleArmDynamic1CollisionConfig()
    gripper = single.isaac_articulation.grippers[0]
    single_gains = {
        name: (stiffness, damping)
        for name, stiffness, damping in single.isaac_articulation.joint_gain_overrides
    }
    dual = spark_robot.FanucLRMate200iDDualArmDynamic1CollisionConfig()

    assert gripper.open_positions == (0.0, 0.0, 0.0) * 3
    assert gripper.closed_positions == pytest.approx((1.082104, 1.570796, -0.850324) * 3)
    assert set(single_gains) == set(gripper.joint_names)
    assert set(single_gains.values()) == {(400.0, 25.0)}
    assert len(dual.isaac_articulation.joint_gain_overrides) == 18
    for joint_name in gripper.joint_names:
        joint = joints[joint_name]
        assert joint.attrib["type"] == "revolute"
        limit = joint.find("limit")
        assert limit is not None
        assert float(limit.attrib["lower"]) <= 0.0 <= float(limit.attrib["upper"])


@pytest.mark.parametrize(
    "model_name",
    ("r1lite_dual_arm.xml", "r1lite_fixed_base.xml", "r1lite_mobile_base.xml", "robot_mjcf.xml"),
)
def test_r1lite_mujoco_gripper_keys_cannot_cycle_named_cameras(model_name):
    resource = Path(SPARK_ROBOT_RESOURCE_DIR) / "galaxea_r1lite" / model_name
    assert ET.parse(resource).findall(".//camera") == []


@pytest.mark.parametrize(
    ("config_name", "expected_count", "maximum_radius"),
    (
        ("GalaxeaR1LiteFixedBaseDynamic1CollisionConfig", 80, 0.17),
        ("GalaxeaR1LiteRightArmDynamic1CollisionConfig", 39, 0.11),
        ("GalaxeaR1LiteDualArmDynamic1CollisionConfig", 62, 0.11),
        ("GalaxeaR1LiteMobileBaseDynamic1CollisionConfig", 80, 0.17),
        ("KinovaGen3SingleArmDynamic1CollisionConfig", 47, 0.09),
        ("KinovaGen3DualArmDynamic1CollisionConfig", 90, 0.09),
        ("KukaIIWA14SingleArmDynamic1CollisionConfig", 19, 0.12),
        ("KukaIIWA14DualArmDynamic1CollisionConfig", 38, 0.12),
        ("FanucLRMate200iDSingleArmDynamic1CollisionConfig", 18, 0.17),
        ("FanucLRMate200iDDualArmDynamic1CollisionConfig", 34, 0.17),
        ("UnitreeG1FixedBaseDynamic1Config", 25, 0.11),
    ),
)
def test_family_collision_models_remain_sparse_and_calibrated(
    config_name, expected_count, maximum_radius
):
    import spark_robot

    config = getattr(spark_robot, config_name)()
    assert len(config.CollisionVol) == expected_count
    assert (
        max(
            geometry.attributes["radius"]
            for geometry in config.CollisionVol.values()
            if geometry.type == "sphere"
        )
        < maximum_radius
    )
    if config_name.startswith(("Kinova", "Kuka", "Fanuc")):
        assert set(config.CollisionVolBodyNames) == set(config.CollisionVol)
        assert set(config.CollisionVolLocalOffsets) == set(config.CollisionVol)


def test_agibot_custom_dynamics_warns_when_the_simulator_owns_state():
    from spark_agent import AgiBotG1MobileBaseAgent

    config = AgiBotG1MobileBaseBicycleDynamic2Config()
    with pytest.warns(UserWarning, match="custom 'bicycle' analytical dynamics"):
        agent = AgiBotG1MobileBaseAgent(
            config,
            dynamics_backend="simulator",
            enable_viewer=False,
            real_time=False,
        )
    assert agent.dynamics_backend == "simulator"


def test_isaac_matrix_retries_only_missing_child_reports(tmp_path, monkeypatch):
    matrix = _load_module("spark_matrix_tool_retry_test", "tools/run_robot_support_matrix.py")
    calls = []
    sleeps = []

    def fake_run(command, *, cwd, check):
        calls.append(tuple(command))
        report = Path(command[command.index("--report") + 1])
        if len(calls) == 2:
            report.write_text(
                json.dumps(
                    {
                        "results": [
                            {
                                "status": "PASS",
                                "robot_config": "ExampleConfig",
                                "backend": "isaac",
                            }
                        ]
                    }
                )
            )
            return subprocess.CompletedProcess(command, 0)
        return subprocess.CompletedProcess(command, -11)

    monkeypatch.setattr(matrix.subprocess, "run", fake_run)
    monkeypatch.setattr(matrix.time, "sleep", sleeps.append)
    args = SimpleNamespace(
        mode=("teleop",),
        num_envs=(1,),
        dynamics_backend=("simulator",),
        device="cuda:0",
        duration=0.1,
        amplitude=0.01,
        goal_tolerance=0.03,
        record_fps=5.0,
        record_width=160,
        record_height=90,
        position_kp=None,
        velocity_kd=None,
        record_dir=None,
        stop_on_failure=False,
        isaac_startup_retries=2,
        isaac_restart_backoff=0.01,
        report=tmp_path / "matrix.json",
    )

    assert matrix._run_isaac_config_children(args, ["ExampleConfig"]) == 0
    summary = json.loads(args.report.read_text())
    assert len(calls) == 2
    assert sleeps == [0.01, 0.01]
    assert summary["case_count"] == summary["pass_count"] == 1
    assert summary["failure_count"] == 0


def test_matrix_runner_marks_a_missing_requested_recording_as_error(tmp_path):
    matrix = _load_module("spark_matrix_tool_recording_test", "tools/run_robot_support_matrix.py")
    case = SimpleNamespace(record_gif_path=str(tmp_path / "missing.gif"))
    result = {"status": "PASS"}

    matrix._attach_recording_metadata(case, result)

    assert result["status"] == "ERROR"
    assert result["error_type"] == "MissingRecordingArtifact"


def test_support_report_merge_rejects_duplicate_cases_and_requires_gifs(tmp_path):
    merge = _load_module("spark_matrix_merge_test", "tools/merge_robot_support_reports.py")
    result = {
        "status": "PASS",
        "robot_config": "ExampleConfig",
        "backend": "isaac",
        "num_envs": 1,
        "dynamics_backend": "simulator",
        "mode": "teleop",
    }
    source = tmp_path / "source.json"
    source.write_text(json.dumps({"results": [result]}))
    record_dir = tmp_path / "recordings"
    record_dir.mkdir()
    gif_path = merge._gif_path(record_dir, result)
    gif_path.write_bytes(b"GIF89a")

    report = merge.merge_reports(
        [source], expected_case_count=1, require_pass=True, record_dir=record_dir
    )

    assert report["case_count"] == report["pass_count"] == 1
    assert report["results"][0]["record_gif_bytes"] == 6

    import pytest

    with pytest.raises(ValueError, match="Duplicate matrix case"):
        merge.merge_reports([source, source])
