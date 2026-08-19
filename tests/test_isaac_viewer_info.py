import inspect
from types import SimpleNamespace

from spark_agent.simulation.isaac.isaac_agent import IsaacAgent
from spark_agent.simulation.isaac.viewer_info import (
    ISAAC_SIMULATION_INFO_COLUMN_SPACING,
    ISAAC_SIMULATION_INFO_FONT_SIZE,
    ISAAC_SIMULATION_INFO_MARGIN,
    ISAAC_SIMULATION_INFO_ROW_HEIGHT,
    ISAAC_SIMULATION_INFO_ROW_SPACING,
    ISAAC_SIMULATION_INFO_WINDOW_HEIGHT,
    ISAAC_SIMULATION_INFO_WINDOW_TITLE,
    ISAAC_SIMULATION_INFO_WINDOW_WIDTH,
    IsaacSimulationInfoWindow,
    IsaacViewerInfoMixin,
    isaac_simulation_info,
)
from spark_robot import FanucLRMate200iDSingleArmDynamic1CollisionConfig


def _agent(*, backend="simulator", num_envs=1):
    robot_cfg = FanucLRMate200iDSingleArmDynamic1CollisionConfig()
    return SimpleNamespace(
        robot_cfg=robot_cfg,
        dynamics_model=robot_cfg.create_dynamics_model(),
        dynamics_backend=backend,
        dynamics_executor=SimpleNamespace(integrator="RK4"),
        dt=0.005,
        control_decimation=4,
        control_period=0.02,
        time=1.25,
        num_envs=num_envs,
        device="cuda:0" if num_envs > 1 else "cpu",
        allow_self_collision=True,
    )


def test_isaac_simulation_info_matches_mujoco_dynamics_vocabulary():
    agent = _agent()

    assert isaac_simulation_info(agent) == {
        "Robot config": "FanucLRMate200iDSingleArmDynamic1CollisionConfig",
        "Dynamics model": "single_integrator",
        "Dynamics order": "1",
        "Dimensions": "state=6, control=6",
        "Propagation": "Isaac PhysX",
        "PhysX self-contact": "enabled",
        "Environments": "1",
        "Device": "cpu",
        "Timing": "period=0.02 s, time=1.250 s",
    }

    agent.dynamics_backend = "model"
    assert isaac_simulation_info(agent)["Propagation"] == "SPARK model (RK4)"
    assert isaac_simulation_info(agent)["PhysX self-contact"] == ("inactive (model propagation)")


def test_isaac_tensor_simulation_info_reports_environment_count():
    information = isaac_simulation_info(_agent(num_envs=16))

    assert information["Environments"] == "16"
    assert information["Device"] == "cuda:0"


class _Context:
    def __enter__(self):
        return self

    def __exit__(self, *_args):
        return False


class _FakeWindow:
    def __init__(self, title, **kwargs):
        self.title = title
        self.visible = kwargs["visible"]
        self.width = kwargs["width"]
        self.height = kwargs["height"]
        self.position_x = None
        self.position_y = None
        self.frame = _Context()


class _FakeLabel:
    def __init__(self, text, **kwargs):
        self.text = text
        self.kwargs = kwargs


class _FakeUi:
    def __init__(self):
        self.windows = []
        self.vstack_kwargs = []
        self.hstack_kwargs = []

    def Window(self, title, **kwargs):
        window = _FakeWindow(title, **kwargs)
        self.windows.append(window)
        return window

    def VStack(self, **kwargs):
        self.vstack_kwargs.append(kwargs)
        return _Context()

    def HStack(self, **kwargs):
        self.hstack_kwargs.append(kwargs)
        return _Context()

    @staticmethod
    def Label(text, **kwargs):
        return _FakeLabel(text, **kwargs)


def test_isaac_information_window_uses_caption_and_refreshes_values():
    ui = _FakeUi()
    window = IsaacSimulationInfoWindow(ui_module=ui)

    assert window.update({"Robot config": "RobotA", "Timing": "time=0.000 s"})
    assert ui.windows[0].title == ISAAC_SIMULATION_INFO_WINDOW_TITLE
    assert ui.windows[0].width == ISAAC_SIMULATION_INFO_WINDOW_WIDTH
    assert ui.windows[0].height == ISAAC_SIMULATION_INFO_WINDOW_HEIGHT
    assert window._value_labels["Robot config"].text == "RobotA"
    assert window._value_labels["Robot config"].kwargs["style"]["font_size"] == (
        ISAAC_SIMULATION_INFO_FONT_SIZE
    )
    assert ISAAC_SIMULATION_INFO_FONT_SIZE == 20
    assert ui.vstack_kwargs == [
        {
            "spacing": ISAAC_SIMULATION_INFO_ROW_SPACING,
            "style": {"margin": ISAAC_SIMULATION_INFO_MARGIN},
        }
    ]
    assert ui.hstack_kwargs == [
        {
            "height": ISAAC_SIMULATION_INFO_ROW_HEIGHT,
            "spacing": ISAAC_SIMULATION_INFO_COLUMN_SPACING,
        },
        {
            "height": ISAAC_SIMULATION_INFO_ROW_HEIGHT,
            "spacing": ISAAC_SIMULATION_INFO_COLUMN_SPACING,
        },
    ]

    assert window.update({"Robot config": "RobotA", "Timing": "time=0.020 s"})
    assert len(ui.windows) == 1
    assert window._value_labels["Timing"].text == "time=0.020 s"

    rendered_window = ui.windows[0]
    window.close()
    assert rendered_window.visible is False


def test_headless_isaac_agent_does_not_construct_an_information_window():
    class Agent(IsaacViewerInfoMixin):
        pass

    agent = Agent()
    agent.render_enabled = False
    agent._initialize_viewer_simulation_info()

    assert agent.viewer_show_simulation_info is False
    assert not agent._update_simulation_info_overlay()
    assert agent._simulation_info_window is None


def test_scalar_isaac_agent_defaults_to_viewport_only_startup_updates():
    source = inspect.getsource(IsaacAgent.__init__)

    assert "6 if self.render_enabled else 0" in source
