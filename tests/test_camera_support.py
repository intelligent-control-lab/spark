import importlib
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace

import numpy as np
import pytest

from spark_agent.simulation.isaac.isaac_agent import IsaacAgent, _look_at_quaternion_wxyz
from spark_agent.simulation.camera_display import _uint8_depth, _uint8_rgb
from spark_agent.simulation.isaac.unitree_g1.unitree_g1_isaac_tensor_backend import (
    _UnitreeG1IsaacTensorBackend,
    _interactive_kit_visualizer_configs,
)
from spark_agent.simulation.mujoco.mujoco_agent import MujocoAgent
from spark_agent.simulation.viewer_config import (
    normalize_sensor_camera_config,
    sensor_camera_eye_target,
)
from spark_pipeline.teleop.camera import configure_teleop_camera, configure_teleop_viewer


TELEOP_MODULES = (
    "example.unitree_g1.run_unitree_g1_teleop",
    "example.agibot_g1.run_agibot_g1_teleop",
    "example.galaxea_r1lite.run_galaxea_r1lite_teleop",
    "example.kinova_gen3.run_kinova_gen3_teleop",
    "example.kuka_iiwa14.run_kuka_iiwa14_teleop",
    "example.fanuc_lrmate200id.run_fanuc_lrmate200id_teleop",
)

BENCHMARK_MODULES = (
    ("example.unitree_g1.run_unitree_g1_benchmark", "build_parser"),
    ("example.agibot_g1.run_agibot_g1_benchmark", "_parser"),
    ("example.galaxea_r1lite.run_galaxea_r1lite_benchmark", "_parser"),
    ("example.kinova_gen3.run_kinova_gen3_benchmark", "_parser"),
    ("example.kuka_iiwa14.run_kuka_iiwa14_benchmark", "_parser"),
    ("example.fanuc_lrmate200id.run_fanuc_lrmate200id_benchmark", "_parser"),
)


def test_default_sensor_camera_uses_backend_neutral_viewer_pose():
    viewer = {
        "camera_lookat": (0.2, -0.3, 0.8),
        "camera_distance": 1.7,
        "camera_azimuth": 135.0,
        "camera_elevation": -30.0,
        "camera_vertical_fov": 55.0,
    }
    cameras = normalize_sensor_camera_config(None, viewer)

    assert tuple(cameras) == ("overview",)
    assert cameras["overview"]["type"] == "free"
    assert cameras["overview"]["lookat"] == viewer["camera_lookat"]
    assert cameras["overview"]["distance"] == viewer["camera_distance"]
    assert cameras["overview"]["fovy"] == viewer["camera_vertical_fov"]


def test_link_camera_definition_is_preserved_for_both_backends():
    cameras = normalize_sensor_camera_config(
        {
            "head": {
                "type": "fixed",
                "body_name": "torso_link",
                "pos": [0.1, 0.0, 0.4],
                "xyaxes": [0.0, -1.0, 0.0, 0.5, 0.0, 0.866],
            }
        }
    )

    assert cameras["head"]["body_name"] == "torso_link"
    assert cameras["head"]["mujoco_camera_name"] == "spark_head_camera"
    assert cameras["head"]["publish_rgb"] is True
    assert cameras["head"]["publish_depth"] is True


def test_isaac_overview_orientation_looks_at_target():
    config = normalize_sensor_camera_config(None)["overview"]
    eye, target = sensor_camera_eye_target(config)
    quaternion_wxyz = _look_at_quaternion_wxyz(eye, target)

    from scipy.spatial.transform import Rotation

    rotation = Rotation.from_quat(np.roll(quaternion_wxyz, -1)).as_matrix()
    camera_forward = -rotation[:, 2]
    expected_forward = (target - eye) / np.linalg.norm(target - eye)
    np.testing.assert_allclose(camera_forward, expected_forward, atol=1.0e-12)


def test_enable_camera_displays_by_default_but_capture_only_is_supported():
    class AgentConfig:
        pass

    displayed = AgentConfig()
    configure_teleop_camera(displayed, {"enable_camera": True, "camera_display": None})
    assert displayed.enable_camera is True
    assert displayed.camera_display is True
    assert (displayed.camera_width, displayed.camera_height) == (640, 480)

    capture_only = AgentConfig()
    configure_teleop_camera(
        capture_only,
        {"enable_camera": True, "camera_display": False, "camera_rate_hz": 15.0},
    )
    assert capture_only.enable_camera is True
    assert capture_only.camera_display is False
    assert capture_only.camera_rate_hz == 15.0


def test_simulation_information_is_an_explicit_viewer_opt_in():
    class AgentConfig:
        pass

    default = AgentConfig()
    configure_teleop_viewer(default, {})
    assert default.viewer_show_simulation_info is False

    enabled = AgentConfig()
    configure_teleop_viewer(enabled, {"viewer_show_simulation_info": True})
    assert enabled.viewer_show_simulation_info is True


@pytest.mark.parametrize("module_name", TELEOP_MODULES)
def test_every_robot_teleop_launcher_exposes_common_camera_cli(module_name):
    module = importlib.import_module(module_name)
    args = module._parse_cli_args(
        [
            "--enable-camera",
            "--camera-width",
            "320",
            "--camera-height",
            "240",
            "--camera-rate-hz",
            "15",
            "--show-simulation-info",
        ]
    )

    assert args["enable_camera"] is True
    assert args["camera_display"] is None
    assert args["camera_width"] == 320
    assert args["camera_height"] == 240
    assert args["camera_rate_hz"] == 15.0
    assert args["viewer_show_simulation_info"] is True


@pytest.mark.parametrize("module_name", TELEOP_MODULES)
def test_every_robot_teleop_launcher_hides_simulation_info_by_default(module_name):
    module = importlib.import_module(module_name)
    args = module._parse_cli_args([])

    assert args["viewer_show_simulation_info"] is False


@pytest.mark.parametrize("module_name", TELEOP_MODULES)
def test_every_robot_teleop_propagates_camera_and_viewer_options(module_name):
    module = importlib.import_module(module_name)
    args = module._parse_cli_args(
        ["--backend", "isaac", "--enable-camera", "--show-simulation-info"]
    )
    if module_name.endswith("run_unitree_g1_teleop"):
        config = module.build_config(**args)
    else:
        config = module.config_agent_module(module.PipelineConfig(), **args)

    assert config.env.agent.enable_camera is True
    assert config.env.agent.viewer_show_simulation_info is True


@pytest.mark.parametrize(("module_name", "parser_name"), BENCHMARK_MODULES)
def test_every_robot_benchmark_exposes_optional_simulation_information(module_name, parser_name):
    parser = getattr(importlib.import_module(module_name), parser_name)()

    assert parser.parse_args([]).viewer_show_simulation_info is False
    assert parser.parse_args(["--show-simulation-info"]).viewer_show_simulation_info is True


def test_isaaclab_camera_output_removes_batch_dimension():
    batched_rgb = np.zeros((1, 8, 12, 3), dtype=np.uint8)
    assert _UnitreeG1IsaacTensorBackend._camera_output_numpy(batched_rgb).shape == (
        8,
        12,
        3,
    )


def test_camera_display_normalizes_rgb_and_depth_without_opencv_in_parent():
    rgb = np.full((2, 3, 3), 0.5, dtype=np.float32)
    depth = np.array([[1.0, 2.0, np.inf], [3.0, np.nan, 1.0]], dtype=np.float32)

    normalized_rgb = _uint8_rgb(rgb)
    normalized_depth = _uint8_depth(depth)

    assert normalized_rgb.dtype == np.uint8
    assert np.all(normalized_rgb == 127)
    assert normalized_depth.dtype == np.uint8
    assert normalized_depth[0, 0] == 0
    assert normalized_depth[1, 0] == 255
    assert normalized_depth[0, 2] == 0


def test_unitree_interactive_viewer_owns_kit_application_updates(monkeypatch):
    class KitVisualizerCfg:
        def __init__(self, **kwargs):
            self.options = kwargs

    package = ModuleType("isaaclab_visualizers")
    kit_module = ModuleType("isaaclab_visualizers.kit")
    kit_module.KitVisualizerCfg = KitVisualizerCfg
    package.kit = kit_module
    monkeypatch.setitem(sys.modules, "isaaclab_visualizers", package)
    monkeypatch.setitem(sys.modules, "isaaclab_visualizers.kit", kit_module)

    assert _interactive_kit_visualizer_configs(True, False) == []
    assert _interactive_kit_visualizer_configs(False, True) == []
    configs = _interactive_kit_visualizer_configs(True, True)

    assert len(configs) == 1
    assert configs[0].options == {
        "create_viewport": False,
        "headless": False,
        "enable_markers": False,
        "enable_live_plots": False,
    }


def test_mujoco_rgb_and_depth_use_the_same_scene_projection():
    class Renderer:
        def __init__(self):
            self.depth = False
            self.update_count = 0
            self.render_modes = []

        def update_scene(self, _data, *, camera):
            assert camera == "overview"
            self.update_count += 1

        def render(self):
            self.render_modes.append(self.depth)
            shape = (3, 4) if self.depth else (3, 4, 3)
            return np.zeros(shape, dtype=float)

        def enable_depth_rendering(self):
            self.depth = True

        def disable_depth_rendering(self):
            self.depth = False

    renderer = Renderer()
    agent = SimpleNamespace(
        cam_step=0,
        camera_renderer=renderer,
        data=object(),
        _render_camera_debug_overlays=lambda: None,
    )

    rgb, depth = MujocoAgent.render_camera(agent, "overview")

    assert renderer.update_count == 1
    assert renderer.render_modes == [False, True]
    assert rgb.shape[:2] == depth.shape


def test_dual_grippers_require_explicit_hand_selection():
    right = np.eye(4)
    left = np.eye(4)
    base = np.eye(4)
    mujoco_agent = SimpleNamespace(
        class_name="KukaIIWA14DualArmDynamic1CollisionConfig",
        debug_object=base,
        right_goal_debug_frame=right,
        left_goal_debug_frame=left,
        left_gripper_goal=False,
        right_gripper_goal=False,
        left_gripper_debug_state=False,
        right_gripper_debug_state=False,
        _keyboard_gripper_goal_override={},
        _selected_goal_debug_side=lambda: None,
        _default_keyboard_gripper_sides=lambda: (),
    )
    MujocoAgent._set_selected_gripper_debug_state(mujoco_agent, True)
    assert not mujoco_agent.left_gripper_goal
    assert not mujoco_agent.right_gripper_goal

    isaac_agent = SimpleNamespace(
        debug_object=base,
        right_goal_debug_frame=right,
        left_goal_debug_frame=left,
        left_gripper_goal=False,
        right_gripper_goal=False,
        left_gripper_debug_state=False,
        right_gripper_debug_state=False,
        _keyboard_gripper_goal_override={},
        _available_keyboard_gripper_sides=lambda: ("left", "right"),
    )
    IsaacAgent._set_selected_gripper_state(isaac_agent, True)
    assert not isaac_agent.left_gripper_goal
    assert not isaac_agent.right_gripper_goal


@pytest.mark.parametrize("selected_side", ("left", "right"))
def test_dual_gripper_selection_changes_only_selected_hand(selected_side):
    right = np.eye(4)
    left = np.eye(4)
    selected = right if selected_side == "right" else left
    other_side = "left" if selected_side == "right" else "right"

    mujoco_agent = SimpleNamespace(
        debug_object=selected,
        right_goal_debug_frame=right,
        left_goal_debug_frame=left,
        left_gripper_goal=False,
        right_gripper_goal=False,
        left_gripper_debug_state=False,
        right_gripper_debug_state=False,
        _keyboard_gripper_goal_override={},
        _selected_goal_debug_side=lambda: selected_side,
    )
    mujoco_agent._set_gripper_debug_state = lambda side, closed: (
        MujocoAgent._set_gripper_debug_state(mujoco_agent, side, closed)
    )
    MujocoAgent._set_selected_gripper_debug_state(mujoco_agent, True)
    assert getattr(mujoco_agent, f"{selected_side}_gripper_goal") is True
    assert getattr(mujoco_agent, f"{other_side}_gripper_goal") is False

    isaac_agent = SimpleNamespace(
        debug_object=selected,
        right_goal_debug_frame=right,
        left_goal_debug_frame=left,
        left_gripper_goal=False,
        right_gripper_goal=False,
        left_gripper_debug_state=False,
        right_gripper_debug_state=False,
        _keyboard_gripper_goal_override={},
        _available_keyboard_gripper_sides=lambda: ("left", "right"),
    )
    IsaacAgent._set_selected_gripper_state(isaac_agent, True)
    assert getattr(isaac_agent, f"{selected_side}_gripper_goal") is True
    assert getattr(isaac_agent, f"{other_side}_gripper_goal") is False


def test_mujoco_gripper_key_restores_the_free_viewer_camera():
    free_camera_state = {
        "lookat": np.array([0.2, -0.1, 0.9]),
        "distance": 2.3,
        "elevation": -25.0,
        "azimuth": 145.0,
    }
    camera = SimpleNamespace(
        type=object(),
        fixedcamid=0,
        lookat=np.zeros(3),
        distance=0.0,
        elevation=0.0,
        azimuth=0.0,
    )
    agent = SimpleNamespace(
        viewer=SimpleNamespace(cam=camera),
        viewer_config={},
        _viewer_free_camera_state=free_camera_state,
        _restore_free_camera_after_gripper_key=True,
    )

    MujocoAgent._restore_or_remember_free_viewer_camera(agent)

    assert camera.type == pytest.importorskip("mujoco").mjtCamera.mjCAMERA_FREE
    assert camera.fixedcamid == -1
    np.testing.assert_allclose(camera.lookat, free_camera_state["lookat"])
    assert camera.distance == free_camera_state["distance"]
    assert camera.elevation == free_camera_state["elevation"]
    assert camera.azimuth == free_camera_state["azimuth"]
    assert agent._restore_free_camera_after_gripper_key is False


def test_mujoco_spark_keys_restore_camera_and_visual_groups():
    camera = SimpleNamespace(
        type=object(),
        fixedcamid=4,
        lookat=np.zeros(3),
        distance=0.0,
        elevation=0.0,
        azimuth=0.0,
    )
    option = SimpleNamespace(
        geomgroup=np.zeros(6, dtype=np.uint8),
        sitegroup=np.zeros(6, dtype=np.uint8),
        frame=7,
        label=8,
    )
    expected_option = {
        "geomgroup": np.array([1, 0, 1, 1, 0, 1], dtype=np.uint8),
        "sitegroup": np.array([1, 1, 0, 0, 1, 1], dtype=np.uint8),
        "frame": np.asarray(2),
        "label": np.asarray(3),
    }
    agent = SimpleNamespace(
        viewer=SimpleNamespace(cam=camera, opt=option),
        viewer_config={},
        _viewer_free_camera_state={
            "lookat": np.array([0.2, -0.1, 0.9]),
            "distance": 2.3,
            "elevation": -25.0,
            "azimuth": 145.0,
        },
        _viewer_visual_state=expected_option,
        _restore_viewer_state_after_spark_key=True,
    )

    MujocoAgent._restore_or_remember_viewer_state(agent)

    assert camera.type == pytest.importorskip("mujoco").mjtCamera.mjCAMERA_FREE
    assert camera.fixedcamid == -1
    np.testing.assert_array_equal(option.geomgroup, expected_option["geomgroup"])
    np.testing.assert_array_equal(option.sitegroup, expected_option["sitegroup"])
    assert option.frame == 2
    assert option.label == 3
    assert agent._restore_viewer_state_after_spark_key is False


def test_mujoco_spark_key_does_not_snapshot_native_shortcut_toggle():
    remembered = {"geomgroup": np.array([1, 1, 0, 1, 1, 1], dtype=np.uint8)}
    # Model MuJoCo handling KEY_2 before invoking SPARK's Python callback.
    viewer = SimpleNamespace(
        opt=SimpleNamespace(geomgroup=np.array([1, 1, 1, 1, 1, 1], dtype=np.uint8))
    )
    agent = SimpleNamespace(
        viewer=viewer,
        _viewer_visual_state=remembered,
        _restore_viewer_state_after_spark_key=False,
    )

    MujocoAgent._request_viewer_state_restore(agent)

    assert agent._restore_viewer_state_after_spark_key is True
    np.testing.assert_array_equal(agent._viewer_visual_state["geomgroup"], remembered["geomgroup"])


def test_unitree_link_camera_is_isolated_from_interactive_mujoco_model():
    import mujoco
    from spark_agent import UnitreeG1WholeBodyMujocoAgent
    from spark_robot import SPARK_ROBOT_RESOURCE_DIR, UnitreeG1WholeBodyWithHandDynamic1Config

    config = UnitreeG1WholeBodyWithHandDynamic1Config()
    agent = UnitreeG1WholeBodyMujocoAgent.__new__(UnitreeG1WholeBodyMujocoAgent)
    agent.mujoco_xml_path = str(Path(SPARK_ROBOT_RESOURCE_DIR) / config.mujoco_model_path)
    agent.enable_camera = True
    agent.camera_config = [
        {
            "name": "head",
            "type": "fixed",
            "body_name": "torso_link",
            "mujoco_camera_name": "spark_g1_head_camera",
            "pos": [0.12, 0.01753, 0.42987],
            "xyaxes": [0.0, -1.0, 0.0, 0.529919264, 0.0, 0.848048096],
            "fovy": 90.0,
        }
    ]

    agent.model = MujocoAgent._load_mujoco_model(agent)
    agent.camera_model = MujocoAgent._load_mujoco_camera_model(agent)
    agent.data = mujoco.MjData(agent.model)
    agent.camera_data = mujoco.MjData(agent.camera_model)
    agent.data.time = 1.25
    if agent.data.qpos.size:
        agent.data.qpos[0] = 0.01
    MujocoAgent._sync_camera_model_state(agent)

    # MuJoCo's native '['/']' shortcuts have nothing to cycle in the viewer,
    # while the offscreen renderer retains the exact link-mounted camera.
    assert agent.model.ncam == 0
    assert agent.camera_model.ncam == 1
    assert agent.camera_model is not agent.model
    assert agent.camera_model.nq == agent.model.nq
    np.testing.assert_allclose(agent.camera_data.qpos, agent.data.qpos)
    assert agent.camera_data.time == agent.data.time


def test_isaac_reports_selected_arm_goal_side():
    right = np.eye(4)
    left = np.eye(4)
    agent = SimpleNamespace(
        debug_object=right,
        right_goal_debug_frame=right,
        left_goal_debug_frame=left,
    )

    assert IsaacAgent._selected_goal_debug_side(agent) == "right"
    agent.debug_object = left
    assert IsaacAgent._selected_goal_debug_side(agent) == "left"
    agent.debug_object = np.eye(4)
    assert IsaacAgent._selected_goal_debug_side(agent) is None


def test_mujoco_debug_overlay_geometry_never_casts_opaque_shadows():
    geometry = SimpleNamespace(
        transparent=0,
        category=0,
        specular=1.0,
        reflectance=1.0,
    )

    MujocoAgent._mark_visual_overlay_geom(geometry)

    assert geometry.transparent == 1
    assert geometry.category == int(pytest.importorskip("mujoco").mjtCatBit.mjCAT_DECOR)
    assert geometry.specular == 0.0
    assert geometry.reflectance == 0.0


def test_unitree_isaac_camera_render_restores_authoritative_state():
    events = []
    state = object()
    agent = SimpleNamespace(
        render_enabled=True,
        enable_camera=True,
        _viewport_layout_updates_remaining=0,
        _snapshot_camera_render_state=lambda: state,
        _restore_camera_render_state=lambda value: events.append(("restore", value)),
        _flush_debug_primitives=lambda: events.append(("flush", None)),
        _synchronize_kinematic_articulation=lambda: events.append(("sync", None)),
        _update_simulation_info_overlay=lambda: events.append(("info", None)),
        _capture_isaaclab_camera_feedback=lambda: events.append(("capture", None)),
        sim=SimpleNamespace(render=lambda: events.append(("render", None))),
    )

    _UnitreeG1IsaacTensorBackend.render_frame(agent)

    assert events == [
        ("flush", None),
        ("sync", None),
        ("info", None),
        ("render", None),
        ("capture", None),
        ("restore", state),
    ]


def test_unitree_tensor_step_does_not_render_when_pipeline_owns_frame():
    events = []
    agent = SimpleNamespace(
        control_decimation=1,
        render_enabled=True,
        render_on_step=False,
        render_decimation=1,
        step_index=0,
        time=0.0,
        dt=0.005,
        _viewport_layout_updates_remaining=0,
        _apply_efforts=lambda: events.append("effort"),
        scene=SimpleNamespace(
            write_data_to_sim=lambda: events.append("write"),
            update=lambda _dt: events.append("update"),
        ),
        sim=SimpleNamespace(
            step=lambda *, render: events.append(("step", render)),
            render=lambda: events.append("render"),
        ),
    )

    _UnitreeG1IsaacTensorBackend._post_control_processing(agent)

    assert events == ["effort", "write", ("step", False), "update"]
    assert agent.step_index == 1
