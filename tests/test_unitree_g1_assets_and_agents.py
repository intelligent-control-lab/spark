import importlib
import inspect
from pathlib import Path
import subprocess
import sys
import textwrap

import mujoco
import pytest
import spark_agent
import spark_robot
from spark_robot import SPARK_ROBOT_RESOURCE_DIR


RETAINED_FAMILIES = ("RightArm", "DualArm", "FixedBase", "MobileBase", "WholeBody")
POLICY_FAMILIES = ("SportMode", "Sonic", "WBT")


def _retained_configs():
    for name in dir(spark_robot):
        if not name.startswith("UnitreeG1") or not name.endswith("Config"):
            continue
        if any(policy in name for policy in POLICY_FAMILIES):
            continue
        if any(family in name for family in RETAINED_FAMILIES):
            yield name, getattr(spark_robot, name)


def test_retained_configs_have_matching_assets_and_agents():
    resource_root = Path(SPARK_ROBOT_RESOURCE_DIR)
    for name, config in _retained_configs():
        mjcf = resource_root / config.mujoco_model_path
        urdf = mjcf.parents[1] / "urdf" / f"{mjcf.stem}.urdf"
        assert mjcf.is_file(), (name, mjcf)
        assert urdf.is_file(), (name, urdf)
        mujoco.MjModel.from_xml_path(str(mjcf))
        for backend in ("mujoco", "isaac", "real"):
            agent_name = config.agent_class_name(backend)
            assert getattr(spark_agent, agent_name) is not None


def test_policy_named_configs_are_mobile_base_compatibility_aliases():
    assert (
        spark_robot.UnitreeG1SportModeDynamic1Config
        is spark_robot.UnitreeG1MobileBaseDynamic1Config
    )
    assert spark_robot.UnitreeG1SonicDynamic1Config is spark_robot.UnitreeG1MobileBaseDynamic1Config
    assert spark_robot.UnitreeG1WBTDynamic1Config is spark_robot.UnitreeG1MobileBaseDynamic1Config


def test_real_agent_symbol_is_inspectable_without_unitree_sdk():
    script = textwrap.dedent(
        """
        import builtins

        original_import = builtins.__import__

        def guarded_import(name, globals=None, locals=None, fromlist=(), level=0):
            if level == 0 and name.split('.', 1)[0] == 'unitree_sdk2py':
                raise ModuleNotFoundError("optional Unitree SDK blocked by test")
            return original_import(name, globals, locals, fromlist, level)

        builtins.__import__ = guarded_import
        import spark_agent
        assert spark_agent.UnitreeG1RealAgent.__name__ == 'UnitreeG1RealAgent'
        """
    )
    subprocess.run([sys.executable, "-c", script], check=True)


def test_real_agent_digital_twin_is_an_explicit_optional_cli_setting():
    from example.unitree_g1.run_unitree_g1_teleop import _parse_cli_args, build_config

    arguments = _parse_cli_args(["--use-real", "true", "--enable-digital-twin", "false"])
    configured = build_config(**arguments)

    assert arguments["backend"] == "mujoco"
    assert configured.env.agent.class_name == "UnitreeG1RealAgent"
    assert configured.env.agent.enable_digital_twin is False
    assert configured.env.agent.digital_twin_backend == "mujoco"
    assert configured.env.agent.digital_twin_sync_hz == 500.0


def test_wbt_single_environment_launcher_uses_shared_tensor_isaac_plant(monkeypatch):
    launcher = importlib.import_module("example.unitree_g1.run_unitree_g1_teleop")
    whole_body = importlib.import_module("spark_pipeline.teleop.unitree_g1_whole_body_config")
    captured = {}

    def fake_run(**kwargs):
        captured.update(kwargs)

    monkeypatch.setattr(whole_body, "run", fake_run)
    launcher.run(policy_config="UnitreeG1WBTPolicy", backend="isaac")

    assert captured["use_isaac_tensor_backend"] is True


def test_unitree_teleop_accepts_explicit_single_environment_argument():
    launcher = importlib.import_module("example.unitree_g1.run_unitree_g1_teleop")

    parsed = launcher._parse_cli_args(["--num-envs", "1", "--headless"])

    assert parsed["num_envs"] == 1
    with pytest.raises(ValueError, match="requires --num-envs 1"):
        launcher.run(num_envs=2)


def test_wbt_single_environment_uses_policy_owned_isaac_actuation_config():
    from spark_pipeline.teleop.unitree_g1_whole_body_config import build_config
    from spark_policy.control.whole_body.unitree_g1.wbt import (
        WBT_JOINT_NAMES,
        WBT_MOTOR_KDS,
        WBT_MOTOR_KPS,
        wbt_isaac_actuation_config,
    )

    common = {
        "backend": "isaac",
        "robot_cfg": "UnitreeG1MobileBaseDynamic1Config",
        "enable_viewer": False,
        "enable_keyboard_control": False,
    }
    tensor = build_config(**common, use_isaac_tensor_backend=True)
    legacy_flag_value = build_config(**common, use_isaac_tensor_backend=False)
    contract = wbt_isaac_actuation_config()

    assert tensor.env.agent.num_envs == 1
    assert tensor.env.agent.scalar_api is True
    assert getattr(tensor.env.agent, "usd_path", None) is None
    assert tensor.env.agent.native_implicit_pd is True
    assert tensor.env.agent.hybrid_implicit_upper_body is False
    assert tensor.env.agent.joint_stiffness == contract["joint_stiffness"]
    assert tensor.env.agent.joint_damping == contract["joint_damping"]
    assert tuple(tensor.env.agent.joint_stiffness) == WBT_JOINT_NAMES
    assert list(tensor.env.agent.joint_stiffness.values()) == list(WBT_MOTOR_KPS)
    assert list(tensor.env.agent.joint_damping.values()) == list(WBT_MOTOR_KDS)
    assert tensor.env.agent.joint_stiffness["waist_yaw_joint"] == WBT_MOTOR_KPS[12]
    assert tensor.env.agent.joint_damping["waist_yaw_joint"] == WBT_MOTOR_KDS[12]
    assert tensor.env.agent.joint_stiffness["left_shoulder_pitch_joint"] == WBT_MOTOR_KPS[15]
    assert tensor.env.agent.joint_damping["left_shoulder_pitch_joint"] == WBT_MOTOR_KDS[15]
    assert tensor.env.agent.solver_position_iteration_count == 4
    assert tensor.env.agent.solver_velocity_iteration_count == 0
    assert tensor.env.agent.allow_self_collision is True
    assert tensor.env.agent.link_mass_scales == {"wrist": 0.5}
    assert legacy_flag_value.env.agent.num_envs == 1
    assert legacy_flag_value.env.agent.joint_stiffness == tensor.env.agent.joint_stiffness
    assert legacy_flag_value.env.agent.native_implicit_pd is True


def test_sport_single_environment_uses_parallel_sport_plant_defaults():
    from spark_pipeline.teleop.unitree_g1_whole_body_config import build_config

    cfg = build_config(
        backend="isaac",
        robot_cfg="UnitreeG1WholeBodyWithHandDynamic1Config",
        enable_viewer=False,
        enable_keyboard_control=False,
        composed_policy_class="UnitreeG1SportSafePolicy",
        executor_policy_class="UnitreeG1SportExecutorAdapter",
        use_isaac_tensor_backend=True,
    )

    assert cfg.env.agent.num_envs == 1
    assert cfg.env.agent.scalar_api is True
    assert getattr(cfg.env.agent, "native_implicit_pd", None) is None
    assert getattr(cfg.env.agent, "hybrid_implicit_upper_body", None) is None
    assert getattr(cfg.env.agent, "joint_stiffness", None) is None
    assert getattr(cfg.env.agent, "joint_damping", None) is None


def test_wbt_isaac_contract_uses_lower_body_and_upper_body_armatures():
    from spark_policy.control.whole_body.unitree_g1.wbt import (
        wbt_isaac_actuation_config,
    )

    contract = wbt_isaac_actuation_config()
    armature = contract["joint_armature_map"]

    assert armature["left_knee_joint"] == 0.01
    assert armature["waist_yaw_joint"] == 0.01
    assert armature["left_shoulder_pitch_joint"] == 0.001
    assert armature["right_wrist_yaw_joint"] == 0.001


def test_wbt_parallel_launcher_keeps_shared_timing_and_goal_sequence(monkeypatch):
    benchmark = importlib.import_module("example.unitree_g1.run_unitree_g1_benchmark")
    captured = {}

    def fake_run(command, *, check):
        captured["command"] = command
        captured["check"] = check
        return object()

    monkeypatch.setattr(benchmark.subprocess, "run", fake_run)
    benchmark._run_isaac_tensor_benchmark(
        {
            "robot_config": "UnitreeG1MobileBaseDynamic1Config",
            "viewer": False,
        },
        controller="wbt",
        num_envs=16,
    )

    command = captured["command"]
    assert command[command.index("--dt") + 1] == "0.005"
    assert command[command.index("--control-decimation") + 1] == "4"
    assert command[command.index("--render-every") + 1] == "5"
    assert command[command.index("--wbt-base-settle-steps") + 1] == "50"
    assert "--no-hold-arm-goals-during-locomotion" in command
    assert captured["check"] is True


def test_scalar_isaac_normalizes_robot_material_without_replacing_link_colors():
    from spark_agent.simulation.isaac.isaac_agent import IsaacAgent

    source = inspect.getsource(IsaacAgent._apply_mujoco_robot_material_style)

    assert "shader_input.Set(0.0)" in source
    assert "shader_input.Set(float(roughness))" in source
    assert "diffuse_color_constant" not in source


def test_tensor_viewer_supports_shared_collision_toggle_and_distance_colors():
    from spark_agent.simulation.isaac.unitree_g1.unitree_g1_isaac_tensor_backend import (
        _UnitreeG1IsaacTensorBackend,
    )

    keyboard_source = inspect.getsource(_UnitreeG1IsaacTensorBackend._on_keyboard_event)
    visual_source = inspect.getsource(
        _UnitreeG1IsaacTensorBackend.set_visual_robot_collision_spheres
    )

    assert "keyboard.V" in keyboard_source
    assert "render_robot_collision_volumes" in keyboard_source
    assert "closest_distance" in visual_source
    assert "collision_volume_distance_color" in visual_source
    assert "_bind_debug_rgba_material" in visual_source
