import unittest

import numpy as np

from spark_agent import DynamicsExecutor
from spark_env import (
    EnvironmentOutput,
    SparkEnvConfig,
    SparkEnvironment,
    SparkEnvWrapper,
)
from spark_robot import AgiBotG1MobileBaseDynamic1Config, LinearDiscreteDynamicsConfig


def _make_test_dynamics_environment(robot_cfg, initial_state, *, dt, max_steps):
    cfg = SparkEnvConfig()
    cfg.agent.class_name = "DynamicsModelAgent"
    cfg.agent.dt = dt
    cfg.task.class_name = "DynamicsTask"
    cfg.task.initial_state = initial_state
    cfg.task.max_steps = max_steps
    return SparkEnvironment(cfg, robot_cfg, robot_kinematics=None)


class EnvironmentArchitectureTests(unittest.TestCase):
    def test_legacy_name_aliases_canonical_environment(self):
        self.assertIs(SparkEnvWrapper, SparkEnvironment)

    def test_output_supports_legacy_tuple(self):
        output = EnvironmentOutput(
            agent_feedback={"state": 1},
            task_info={"done": False},
        )
        self.assertEqual(output.as_legacy_tuple(), ({"state": 1}, {"done": False}))

    def test_dynamics_executor_uses_config_model_and_tracks_time(self):
        robot_cfg = AgiBotG1MobileBaseDynamic1Config()
        model = robot_cfg.create_dynamics_model(["LinearX"], ["vLinearX"])
        executor = DynamicsExecutor(model, dt=0.1, integrator="RK4", substeps=2)
        executor.reset([0.0])
        np.testing.assert_allclose(executor.step([2.0]), [0.2])
        self.assertIs(executor.model.robot_cfg, robot_cfg)
        self.assertAlmostEqual(executor.time, 0.1)
        self.assertEqual(executor.step_index, 1)

    def test_pure_dynamics_environment_uses_native_discrete_transition(self):
        robot_cfg = LinearDiscreteDynamicsConfig([[1.0]], [[0.5]])
        env = _make_test_dynamics_environment(
            robot_cfg,
            [2.0],
            dt=0.1,
            max_steps=2,
        )
        reset = env.reset_output()
        first = env.step_output(np.array([3.0]))
        second = env.step_output(np.array([3.0]))
        np.testing.assert_allclose(reset.agent_feedback["state"], [2.0])
        np.testing.assert_allclose(first.agent_feedback["state"], [3.5])
        self.assertTrue(second.done)
        self.assertIs(env.agent.dynamics_model.robot_cfg, robot_cfg)

    def test_online_parameters_update_without_rebuilding_config(self):
        robot_cfg = LinearDiscreteDynamicsConfig([[1.0]], [[1.0]])
        env = _make_test_dynamics_environment(robot_cfg, [1.0], dt=1.0, max_steps=1)
        env.reset_output()
        output = env.step_output(
            [2.0],
            action_info={"dynamics_parameters": {"A": np.array([[0.5]])}},
        )
        np.testing.assert_allclose(output.agent_feedback["state"], [2.5])
        self.assertEqual(robot_cfg.dynamics_parameters["A"][0, 0], 1.0)


if __name__ == "__main__":
    unittest.main()
