from types import SimpleNamespace

import mujoco

from spark_agent.simulation.mujoco.mujoco_agent import MujocoAgent


class _ConcreteMujocoAgent(MujocoAgent):
    def get_feedback(self):
        return {}


class _ViewerSpy:
    def __init__(self):
        self.texts = []

    def set_texts(self, texts):
        self.texts.append(texts)


def _agent(*, backend="simulator", show_info=True, viewer=None):
    agent = object.__new__(_ConcreteMujocoAgent)
    agent.class_name = "ExampleRobotDynamic2Config"
    agent.dynamics_backend = backend
    agent.dynamics_model = SimpleNamespace(
        variant="double_integrator",
        order=2,
        state_dim=14,
        control_dim=7,
    )
    agent.dynamics_executor = SimpleNamespace(integrator="RK4")
    agent.dt = 0.002
    agent.control_decimation = 5
    agent.data = SimpleNamespace(time=1.25)
    agent.viewer_show_simulation_info = show_info
    agent._viewer_simulation_info = {}
    agent.viewer = _ViewerSpy() if viewer is None else viewer
    return agent


def test_simulation_info_describes_configured_dynamics_and_propagation():
    agent = _agent()

    assert agent.get_simulation_info() == {
        "Robot config": "ExampleRobotDynamic2Config",
        "Dynamics model": "double_integrator",
        "Dynamics order": "2",
        "Dimensions": "state=14, control=7",
        "Propagation": "MuJoCo physics",
        "Control period": "0.01 s",
        "Simulation time": "1.250 s",
    }

    agent.dynamics_backend = "model"
    assert agent.get_simulation_info()["Propagation"] == "SPARK model (RK4)"


def test_simulation_info_is_written_to_top_left_viewer_overlay():
    agent = _agent()

    assert agent._update_simulation_info_overlay()
    assert len(agent.viewer.texts) == 1
    font, position, labels, values = agent.viewer.texts[0]
    assert font == mujoco.mjtFontScale.mjFONTSCALE_100
    assert position == mujoco.mjtGridPos.mjGRID_TOPLEFT
    assert labels.startswith("SPARK simulation\nRobot config\nDynamics model")
    assert "ExampleRobotDynamic2Config" in values
    assert "double_integrator" in values


def test_external_simulation_can_override_viewer_information():
    agent = _agent()
    agent.set_viewer_simulation_info(
        {
            "Robot config": "AgiBotG1MobileBaseUnicycleDynamic1Config",
            "Dynamics model": "unicycle",
            "Dynamics order": 1,
            "Dimensions": "state=3, control=2",
            "Propagation": "trajectory replay (Euler)",
            "Simulation time": "2.400 s",
            "Experiment": "Lecture 2 unicycle",
        },
        replace=True,
    )

    information = agent.get_simulation_info()
    assert information["Robot config"] == "AgiBotG1MobileBaseUnicycleDynamic1Config"
    assert information["Dynamics model"] == "unicycle"
    assert information["Dimensions"] == "state=3, control=2"
    assert information["Propagation"] == "trajectory replay (Euler)"
    assert information["Simulation time"] == "2.400 s"
    assert information["Experiment"] == "Lecture 2 unicycle"


def test_simulation_info_can_be_disabled_or_ignored_by_older_viewer():
    agent = _agent(show_info=False)
    assert not agent._update_simulation_info_overlay()
    assert agent.viewer.texts == []

    agent.viewer_show_simulation_info = True
    agent.viewer = object()
    assert not agent._update_simulation_info_overlay()
