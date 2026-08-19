"""Contracts for backend-neutral family teleoperation entry points."""

from __future__ import annotations

import importlib.util
import math
from pathlib import Path
import sys
from types import ModuleType
from types import SimpleNamespace

import numpy as np
import pytest
import spark_robot

from spark_pipeline.simulation_runtime import (
    configure_simulation_presentation,
    run_simulation_pipeline,
)
from spark_agent.simulation.isaac.configured_tensor_agent import (
    ConfiguredIsaacTensorAgent,
    _fit_grid_viewer_config,
)
from spark_agent.simulation.isaac.isaac_agent import (
    ConfiguredIsaacAgent,
    IsaacAgent,
    _multiply_quaternion_wxyz,
    _rpy_to_quaternion_wxyz,
)
from spark_robot.base.backend_capability import IsaacArticulationSpec
from spark_robot import get_agent_class_name


ROOT = Path(__file__).resolve().parents[1]


def test_isaac_scalar_simulator_targets_bound_error_without_discarding_preload():
    recorded = {}
    agent = object.__new__(ConfiguredIsaacAgent)
    agent.dof_pos_fbk = np.array([0.1, 0.2])
    agent.dof_vel_fbk = np.array([0.3, 0.4])
    agent.dof_pos_cmd = np.array([0.1, 0.2])
    agent.dof_vel_cmd = np.array([0.0, 0.0])
    agent.sim_position_error_limit = 0.05
    agent.dynamics_executor = SimpleNamespace(state=None)
    agent.dynamics_model = SimpleNamespace(
        extract_state=lambda position, velocity: np.concatenate((position, velocity))
    )

    def advance(command, action_info):
        recorded.update(command=np.asarray(command), action_info=action_info)
        agent.dof_pos_cmd = np.array([0.4, 0.0])
        agent.dof_vel_cmd = np.array([0.5, -0.5])

    agent._advance_command_model = advance
    agent._apply_position_targets = lambda action_info: recorded.update(
        applied_action_info=action_info
    )

    agent._send_control_sim_dynamics(np.array([0.5, -0.5]), action_info={"gripper_goal": True})

    assert np.allclose(agent.dof_pos_cmd, [0.15, 0.15])
    assert np.allclose(agent.dynamics_executor.state, [0.15, 0.15, 0.5, -0.5])
    assert np.allclose(recorded["command"], [0.5, -0.5])
    assert recorded["action_info"] == {"gripper_goal": True}
    assert recorded["applied_action_info"] == {"gripper_goal": True}


def test_isaac_scalar_gripper_targets_limit_drive_error_from_measured_state():
    torch = pytest.importorskip("torch")

    indices = torch.tensor([1, 3])
    agent = object.__new__(ConfiguredIsaacAgent)
    agent._torch = torch
    agent.device = "cpu"
    agent.enable_hand_control = True
    agent.use_sim_dynamics = True
    agent.gripper_position_error_limit = 0.025
    agent._keyboard_gripper_goal_override = {}
    agent.right_gripper_goal = False
    agent._configured_grippers = {
        "right": (
            SimpleNamespace(
                joint_names=("finger_1", "finger_2"),
                open_positions=(0.0, 0.0),
                closed_positions=(1.0, -1.0),
            ),
            indices,
        )
    }
    agent.articulation = SimpleNamespace(
        get_joint_positions=lambda *, joint_indices: torch.tensor([0.2, -0.4])
    )
    joint_positions = torch.zeros(4)

    agent._update_configured_gripper_targets(joint_positions, {"right_gripper_goal": True})

    assert torch.allclose(joint_positions[indices], torch.tensor([0.225, -0.425]))
    assert agent.right_gripper_goal
    assert agent.right_gripper_debug_state


def test_isaac_simulator_first_order_targets_retain_bounded_preload():
    torch = pytest.importorskip("torch")

    class FakeArticulation:
        def __init__(self):
            self.position_targets = None
            self.velocity_targets = None

        def get_joint_positions(self, *, joint_indices):
            assert torch.equal(joint_indices, torch.tensor([0, 2]))
            return torch.tensor([[1.0, 2.0]])

        def set_joint_position_targets(self, targets, *, indices):
            self.position_targets = targets.clone()
            assert torch.equal(indices, torch.tensor([0]))

        def set_joint_velocity_targets(self, targets, *, indices):
            self.velocity_targets = targets.clone()
            assert torch.equal(indices, torch.tensor([0]))

    agent = object.__new__(ConfiguredIsaacTensorAgent)
    agent._torch = torch
    agent.device = "cpu"
    agent.num_envs = 1
    agent.num_control = 2
    agent.use_sim_dynamics = True
    agent.dynamics_model = SimpleNamespace(order=1)
    agent.articulation = FakeArticulation()
    agent.joint_indices = torch.tensor([0, 2])
    agent.all_joint_indices = torch.arange(3)
    agent.default_all_joint_pos = torch.tensor([[0.0, 9.0, 0.0]])
    agent.command_pos = torch.tensor([[8.0, -8.0]])
    agent.command_vel = torch.zeros((1, 2))
    agent.last_control = torch.zeros((1, 2))
    agent.control_to_dof = [(0, 0), (1, 1)]
    agent.planar_indices = None
    agent.control_period = 0.02
    agent.sim_position_error_limit = 0.05
    agent.position_lower = torch.full((2,), -10.0)
    agent.position_upper = torch.full((2,), 10.0)

    agent.send_control(torch.tensor([0.5, -0.25]))

    assert torch.allclose(agent.command_pos, torch.tensor([[1.05, 1.95]]))
    assert torch.allclose(agent.command_vel, torch.tensor([[0.5, -0.25]]))
    assert torch.allclose(
        agent.articulation.position_targets,
        torch.tensor([[1.05, 9.0, 1.95]]),
    )
    assert torch.allclose(
        agent.articulation.velocity_targets,
        torch.tensor([[0.5, 0.0, -0.25]]),
    )


def test_isaac_simulator_second_order_retains_explicit_euler_ordering():
    torch = pytest.importorskip("torch")

    agent = object.__new__(ConfiguredIsaacTensorAgent)
    agent._torch = torch
    agent.use_sim_dynamics = True
    agent.robot_cfg = SimpleNamespace(dynamics_variant="double_integrator")
    agent.command_pos = torch.zeros((1, 1))
    agent.command_vel = torch.zeros((1, 1))
    agent.control_to_dof = [(0, 0)]
    agent.planar_indices = None
    agent.control_period = 0.02

    agent._advance_second_order(torch.ones((1, 1)))

    assert torch.allclose(agent.command_vel, torch.tensor([[0.02]]))
    assert torch.allclose(agent.command_pos, torch.zeros((1, 1)))


def test_isaac_modeled_second_order_retains_explicit_euler_ordering():
    torch = pytest.importorskip("torch")

    agent = object.__new__(ConfiguredIsaacTensorAgent)
    agent._torch = torch
    agent.use_sim_dynamics = False
    agent.robot_cfg = SimpleNamespace(dynamics_variant="double_integrator")
    agent.command_pos = torch.zeros((1, 1))
    agent.command_vel = torch.zeros((1, 1))
    agent.control_to_dof = [(0, 0)]
    agent.planar_indices = None
    agent.control_period = 0.02

    agent._advance_second_order(torch.ones((1, 1)))

    assert torch.allclose(agent.command_vel, torch.tensor([[0.02]]))
    assert torch.allclose(agent.command_pos, torch.zeros((1, 1)))


def test_isaac_simulator_second_order_target_retains_bounded_preload():
    torch = pytest.importorskip("torch")

    agent = object.__new__(ConfiguredIsaacTensorAgent)
    agent._torch = torch
    agent.device = "cpu"
    agent.num_envs = 1
    agent.joint_indices = torch.tensor([0])
    agent.command_pos = torch.tensor([[0.2]])
    agent.command_vel = torch.tensor([[0.1]])
    agent.sim_position_error_limit = 0.05
    agent.articulation = SimpleNamespace(
        get_joint_positions=lambda *, joint_indices: torch.tensor([[0.1]])
    )

    agent._bound_simulator_command_state()

    assert torch.allclose(agent.command_pos, torch.tensor([[0.15]]))
    assert torch.allclose(agent.command_vel, torch.tensor([[0.1]]))


def test_isaac_simulator_bicycle_maps_acceleration_to_drive_displacement():
    torch = pytest.importorskip("torch")

    agent = object.__new__(ConfiguredIsaacTensorAgent)
    agent._torch = torch
    agent.use_sim_dynamics = True
    agent.robot_cfg = SimpleNamespace(
        dynamics_variant="bicycle",
        bicycle_params=SimpleNamespace(
            mass=150.0,
            yaw_inertia=30.0,
            front_length=0.3,
            rear_length=0.3,
            front_cornering_stiffness=4500.0,
            rear_cornering_stiffness=4500.0,
            driven_front_fraction=0.5,
            minimum_forward_speed=0.1,
            steering_limit=0.6,
        ),
    )
    agent.command_pos = torch.zeros((1, 3))
    agent.command_vel = torch.zeros((1, 3))
    agent.control_to_dof = [(0, 0), (1, 1), (2, 2)]
    agent.planar_indices = ((0, 1, 2), (0, 1, 2))
    agent.control_period = 0.02
    agent.drive_stiffness = 8000.0

    agent._advance_second_order(torch.tensor([[1.0, 0.0, 0.0]]))

    assert torch.allclose(agent.command_vel, torch.zeros((1, 3)))
    assert torch.allclose(agent.command_pos, torch.tensor([[0.01875, 0.0, 0.0]]))


def test_isaac_simulator_inactive_rows_retain_their_hold_target():
    torch = pytest.importorskip("torch")

    agent = object.__new__(ConfiguredIsaacTensorAgent)
    agent._torch = torch
    agent.device = "cpu"
    agent.num_envs = 2
    agent.joint_indices = torch.tensor([0, 1])
    agent.dynamics_model = SimpleNamespace(order=1)
    agent.command_pos = torch.tensor([[8.0, -8.0], [5.0, 6.0]])
    agent.command_vel = torch.zeros((2, 2))
    agent.articulation = SimpleNamespace(
        get_joint_positions=lambda *, joint_indices: torch.tensor([[1.0, 2.0], [3.0, 4.0]])
    )

    agent._synchronize_command_state_from_simulator(torch.tensor([True, False]))

    assert torch.allclose(agent.command_pos[0], torch.tensor([1.0, 2.0]))
    assert torch.allclose(agent.command_pos[1], torch.tensor([5.0, 6.0]))


def test_isaac_simulator_explicit_hold_dofs_do_not_follow_measured_sag():
    torch = pytest.importorskip("torch")

    agent = object.__new__(ConfiguredIsaacTensorAgent)
    agent._torch = torch
    agent.device = "cpu"
    agent.num_envs = 2
    agent.joint_indices = torch.tensor([0, 1, 2])
    agent.dynamics_model = SimpleNamespace(order=1)
    agent.command_pos = torch.tensor([[1.0, 2.0, 3.0], [4.0, 5.0, 6.0]])
    agent.articulation = SimpleNamespace(
        get_joint_positions=lambda *, joint_indices: torch.tensor(
            [[0.8, 1.5, 2.8], [3.8, 4.5, 5.8]]
        )
    )

    agent._synchronize_command_state_from_simulator(
        torch.tensor([True, False]), hold_dof_indices=torch.tensor([1])
    )

    assert torch.allclose(agent.command_pos[0], torch.tensor([0.8, 2.0, 2.8]))
    assert torch.allclose(agent.command_pos[1], torch.tensor([4.0, 5.0, 6.0]))


def test_isaac_simulator_send_control_retains_explicit_hold_target():
    torch = pytest.importorskip("torch")

    agent = object.__new__(ConfiguredIsaacTensorAgent)
    agent._torch = torch
    agent.device = "cpu"
    agent.num_envs = 1
    agent.num_control = 2
    agent.last_control = torch.zeros((1, 2))
    agent.command_pos = torch.tensor([[1.0, 2.0]])
    agent.command_vel = torch.zeros((1, 2))
    agent.use_sim_dynamics = True
    agent.robot_cfg = SimpleNamespace(dynamics_variant="")
    agent.dynamics_model = SimpleNamespace(order=1)
    agent.position_lower = torch.full((2,), -10.0)
    agent.position_upper = torch.full((2,), 10.0)
    agent._advance_first_order = lambda control: agent.command_pos.add_(control)
    agent._bound_simulator_command_state = lambda active_mask, hold_dof_indices: (
        agent.command_pos.fill_(-4.0)
    )
    agent._set_targets = lambda: None

    agent.send_control(torch.tensor([[0.5, 0.5]]), hold_dof_indices=torch.tensor([1]))

    assert torch.allclose(agent.command_pos, torch.tensor([[-4.0, 2.0]]))
    assert torch.allclose(agent.command_vel, torch.zeros((1, 2)))


def test_isaac_articulation_base_orientation_is_validated_and_composable():
    with pytest.raises(ValueError, match="base_rpy"):
        IsaacArticulationSpec(
            urdf_path="robot.urdf",
            joint_names=("joint",),
            base_rpy=(0.0, float("nan"), 0.0),
        )

    quarter_turn = _rpy_to_quaternion_wxyz((0.0, 0.0, -math.pi / 2.0))
    composed = _multiply_quaternion_wxyz((1.0, 0.0, 0.0, 0.0), quarter_turn)
    assert composed == pytest.approx((math.sqrt(0.5), 0.0, 0.0, -math.sqrt(0.5)))


def test_isaac_articulation_base_orientation_accepts_device_tensor_pose():
    class DeviceTensorPose:
        def __init__(self, values):
            self.values = np.asarray(values, dtype=float)
            self.moved_to_host = False

        def detach(self):
            return self

        def cpu(self):
            self.moved_to_host = True
            return self.values

        def __array__(self, *_args, **_kwargs):
            raise AssertionError("device tensor must be moved to the host before NumPy conversion")

    root_orientation = DeviceTensorPose((1.0, 0.0, 0.0, 0.0))
    composed = _multiply_quaternion_wxyz(
        root_orientation,
        _rpy_to_quaternion_wxyz((0.0, 0.0, -math.pi / 2.0)),
    )

    assert root_orientation.moved_to_host
    assert composed == pytest.approx((math.sqrt(0.5), 0.0, 0.0, -math.sqrt(0.5)))


def _load_example(name: str, relative_path: str):
    path = ROOT / relative_path
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize(
    ("module_name", "relative_path", "robot_config"),
    (
        (
            "fanuc_teleop_contract",
            "example/fanuc_lrmate200id/run_fanuc_lrmate200id_teleop.py",
            "FanucLRMate200iDSingleArmDynamic1CollisionConfig",
        ),
        (
            "galaxea_teleop_contract",
            "example/galaxea_r1lite/run_galaxea_r1lite_teleop.py",
            "GalaxeaR1LiteFixedBaseDynamic1CollisionConfig",
        ),
        (
            "kinova_teleop_contract",
            "example/kinova_gen3/run_kinova_gen3_teleop.py",
            "KinovaGen3DualArmDynamic1Config",
        ),
        (
            "kuka_teleop_contract",
            "example/kuka_iiwa14/run_kuka_iiwa14_teleop.py",
            "KukaIIWA14SingleArmDynamic1CollisionConfig",
        ),
        (
            "agibot_teleop_contract",
            "example/agibot_g1/run_agibot_g1_teleop.py",
            "AgiBotG1FixedBaseDynamic1Config",
        ),
    ),
)
def test_family_teleop_resolves_declared_isaac_agent(
    monkeypatch, module_name, relative_path, robot_config
):
    module = _load_example(module_name, relative_path)
    captured = {}

    def fake_run(cfg, *, backend, pipeline_class, save_path=None):
        captured.update(cfg=cfg, backend=backend, pipeline_class=pipeline_class)
        return cfg

    monkeypatch.setattr(module, "run_simulation_pipeline", fake_run)
    result = module.run(
        backend="isaac",
        robot_cfg=robot_config,
        dynamics_backend="simulator",
        safe_algo="rssa",
        safety_index="si1",
        num_envs=1,
        enable_viewer=False,
        enable_keyboard_control=False,
        real_time=False,
        max_num_steps=1,
        minimum_distance=0.05,
        render_robot_collision_volumes=False,
    )

    assert result is captured["cfg"]
    assert captured["backend"] == "isaac"
    assert result.env.agent.class_name == get_agent_class_name(robot_config, backend="isaac")
    assert result.env.agent.dynamics_backend == "simulator"
    assert result.env.agent.use_sim_dynamics is True
    simulator_dynamics = getattr(spark_robot, robot_config)().simulator_dynamics
    assert result.env.agent.dt == simulator_dynamics.physics_dt
    assert result.env.agent.control_decimation == simulator_dynamics.control_decimation
    if robot_config.startswith(("AgiBotG1", "GalaxeaR1Lite", "KukaIIWA14")):
        assert result.env.task.dt == pytest.approx(simulator_dynamics.control_period)
    assert result.policy.safe_controller.safety_index.min_distance["environment"] == 0.05
    assert result.render_robot_collision_volumes is False
    assert len(result.policy.safe_controller.safe_algo.control_weight) == len(
        getattr(spark_robot, robot_config)().Control
    )


def test_kinova_teleop_restores_explicit_identity_home_pose(monkeypatch):
    module = _load_example(
        "kinova_teleop_initial_pose_contract",
        "example/kinova_gen3/run_kinova_gen3_teleop.py",
    )
    monkeypatch.setattr(module, "run_simulation_pipeline", lambda cfg, **_kwargs: cfg)

    configured = module.run(
        backend="isaac",
        robot_cfg="KinovaGen3SingleArmDynamic1CollisionConfig",
        num_envs=1,
        enable_viewer=False,
        enable_keyboard_control=False,
        real_time=False,
        max_num_steps=1,
    )

    assert configured.env.task.arm_goal_init_from_current_ee is False
    assert configured.env.task.goal_right_init == [0.4, 0.0, 0.3]

    configured_dual = module.run(
        backend="isaac",
        robot_cfg="KinovaGen3DualArmDynamic1CollisionConfig",
        num_envs=1,
        enable_viewer=False,
        enable_keyboard_control=False,
        real_time=False,
        max_num_steps=1,
    )
    assert configured_dual.env.task.arm_goal_init_from_current_ee is False
    assert configured_dual.env.task.goal_right_init == [0.4, -0.3, 0.5]
    assert configured_dual.env.task.goal_left_init == [0.4, 0.3, 0.5]

    configured_second_order = module.run(
        backend="mujoco",
        robot_cfg="KinovaGen3SingleArmDynamic2CollisionConfig",
        num_envs=1,
        enable_viewer=False,
        enable_keyboard_control=False,
        real_time=False,
        max_num_steps=1,
    )
    assert (
        configured_second_order.policy.safe_controller.safety_index.class_name
        == "SecondOrderCollisionSafetyIndex"
    )


def test_r1lite_teleop_derives_goal_contract_from_selected_embodiment(monkeypatch):
    module = _load_example(
        "galaxea_teleop_embodiment_contract",
        "example/galaxea_r1lite/run_galaxea_r1lite_teleop.py",
    )
    monkeypatch.setattr(module, "run_simulation_pipeline", lambda cfg, **_kwargs: cfg)

    mobile = module.run(
        backend="mujoco",
        robot_cfg="GalaxeaR1LiteMobileBaseDynamic1CollisionConfig",
        num_envs=1,
        enable_viewer=False,
        enable_keyboard_control=None,
        profile_frequency=True,
    )
    assert mobile.env.task.use_dual_arm is True
    assert mobile.env.task.base_goal_enable is True
    assert mobile.env.task.base_goal_init == [0.0, 0.0, 0.15]
    assert mobile.env.task.arm_goal_init_from_current_ee is True
    assert mobile.metric_selection.dist_goal_base is True
    assert mobile.env.agent.enable_keyboard_control is False
    assert mobile.profile_frequency is True

    right = module.run(
        backend="mujoco",
        robot_cfg="GalaxeaR1LiteRightArmDynamic1CollisionConfig",
        num_envs=1,
        enable_viewer=False,
        enable_keyboard_control=None,
    )
    assert right.env.task.use_dual_arm is False
    assert right.env.task.base_goal_enable is False
    assert right.metric_selection.dist_goal_base is False

    fixed = module.run(
        backend="mujoco",
        robot_cfg="GalaxeaR1LiteFixedBaseDynamic1CollisionConfig",
        num_envs=1,
        enable_viewer=False,
        enable_keyboard_control=None,
    )
    assert fixed.env.task.use_dual_arm is True
    assert fixed.env.task.base_goal_enable is False


def test_simulation_runtime_owns_isaac_app_lifecycle(monkeypatch):
    events = []

    class FakeApp:
        def __init__(self, launch_config):
            events.append(("app_start", launch_config))

        def close(self, *, wait_for_replicator):
            events.append(("app_close", wait_for_replicator))

    monkeypatch.setitem(
        __import__("sys").modules, "isaacsim", SimpleNamespace(SimulationApp=FakeApp)
    )

    agent = SimpleNamespace(
        attach_simulation_app=lambda app: events.append(("attach", app)),
        close_viewer=lambda: events.append(("close_viewer",)),
        close=lambda: events.append(("agent_close",)),
    )

    class FakePipeline:
        def __init__(self, cfg):
            events.append(("pipeline_init", cfg))
            self.env = SimpleNamespace(agent=agent)

        def run(self, *, save_path):
            events.append(("run", save_path))

    cfg = SimpleNamespace(
        env=SimpleNamespace(agent=SimpleNamespace(enable_viewer=False, render=None))
    )
    pipeline = run_simulation_pipeline(
        cfg,
        backend="isaac",
        pipeline_class=FakePipeline,
        save_path="metrics",
    )

    assert isinstance(pipeline, FakePipeline)
    assert cfg.env.agent.render_on_step is False
    assert cfg.render_every == 50
    assert events[0][0] == "app_start"
    assert events[0][1]["multi_gpu"] is False
    assert [event[0] for event in events] == [
        "app_start",
        "pipeline_init",
        "attach",
        "run",
        "close_viewer",
        "agent_close",
        "app_close",
    ]


def test_isaac_presentation_is_decoupled_from_fifty_hz_control():
    agent = SimpleNamespace(
        dt=0.005,
        control_decimation=4,
        viewer_hz=20.0,
        enable_camera=False,
    )
    cfg = SimpleNamespace(env=SimpleNamespace(agent=agent))

    assert configure_simulation_presentation(cfg, backend="isaac") == 3

    camera_cfg = SimpleNamespace(
        env=SimpleNamespace(
            agent=SimpleNamespace(
                dt=0.005,
                control_decimation=4,
                viewer_hz=20.0,
                enable_camera=True,
            )
        )
    )
    assert configure_simulation_presentation(camera_cfg, backend="isaac") == 10

    mujoco_cfg = SimpleNamespace(env=SimpleNamespace(agent=agent))
    assert configure_simulation_presentation(mujoco_cfg, backend="mujoco") == 1


def test_tensor_grid_camera_contains_clone_layout_without_diagonal_overlap():
    config = {
        "camera_lookat": (1.25, -1.25, 0.8),
        "camera_distance": 3.0,
        "camera_azimuth": 135.0,
        "camera_elevation": -25.0,
        "camera_vertical_fov": 45.0,
        "grid_extent": 5.0,
    }
    positions = ((1.25, -1.25, 0.0), (1.25, 1.25, 0.0), (-1.25, -1.25, 0.0), (-1.25, 1.25, 0.0))

    fitted = _fit_grid_viewer_config(config, positions)

    assert fitted["camera_lookat"] == (0.0, 0.0, 0.8)
    assert fitted["camera_distance"] > config["camera_distance"]
    assert fitted["camera_azimuth"] != 135.0
    assert fitted["camera_elevation"] == -45.0


def test_tensor_grid_camera_can_preserve_backend_neutral_orientation():
    config = {
        "camera_lookat": (0.5, 0.0, 1.3),
        "camera_distance": 2.0,
        "camera_azimuth": 180.0,
        "camera_elevation": -25.0,
        "camera_vertical_fov": 45.0,
        "grid_extent": 5.0,
    }
    positions = ((-1.25, -1.25, 0.0), (-1.25, 1.25, 0.0), (1.25, -1.25, 0.0), (1.25, 1.25, 0.0))

    fitted = _fit_grid_viewer_config(config, positions, preserve_orientation=True)

    assert fitted["camera_lookat"] == (0.0, 0.0, 1.3)
    assert fitted["camera_distance"] > config["camera_distance"]
    assert fitted["camera_azimuth"] == config["camera_azimuth"]
    assert fitted["camera_elevation"] == config["camera_elevation"]


def test_isaac_simulation_viewer_disables_editor_selection_but_keeps_viewport():
    calls = []

    class Selection:
        def set_selected_prim_paths(self, paths, expand):
            calls.append(("clear", paths, expand))

    viewport = object()
    selection_guard = object()
    context_guard = object()
    omni = ModuleType("omni")
    omni.__path__ = []
    omni_usd = ModuleType("omni.usd")
    omni_usd.get_context = lambda: SimpleNamespace(get_selection=lambda: Selection())
    omni_kit = ModuleType("omni.kit")
    omni_kit.__path__ = []
    omni_viewport = ModuleType("omni.kit.viewport")
    omni_viewport.__path__ = []
    viewport_utility = ModuleType("omni.kit.viewport.utility")
    viewport_utility.get_active_viewport_window = lambda: viewport
    viewport_utility.disable_selection = lambda window: (
        calls.append(("disable_selection", window)) or selection_guard
    )
    viewport_utility.disable_context_menu = lambda window: (
        calls.append(("disable_context_menu", window)) or context_guard
    )
    omni.usd = omni_usd

    modules = {
        "omni": omni,
        "omni.usd": omni_usd,
        "omni.kit": omni_kit,
        "omni.kit.viewport": omni_viewport,
        "omni.kit.viewport.utility": viewport_utility,
    }
    previous = {name: sys.modules.get(name) for name in modules}
    try:
        sys.modules.update(modules)
        agent = SimpleNamespace(render_enabled=True, _viewport_interaction_guards=[])
        IsaacAgent._disable_viewport_editing(agent)
    finally:
        for name, module in previous.items():
            if module is None:
                sys.modules.pop(name, None)
            else:
                sys.modules[name] = module

    assert agent._viewport_interaction_guards == [selection_guard, context_guard]
    assert calls == [
        ("clear", [], False),
        ("disable_selection", viewport),
        ("disable_context_menu", viewport),
    ]
