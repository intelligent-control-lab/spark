import inspect
import unittest

import numpy as np
import spark_robot

from spark_policy import ILQRPolicy, LQRPolicy
from spark_robot import (
    AgiBotG1MobileBaseBicycleDynamic2Config,
    AgiBotG1MobileBaseDynamic1Config,
    AgiBotG1MobileBaseDynamic2Config,
    AgiBotG1MobileBaseUnicycleDynamic1Config,
    GalaxeaR1LiteFixedBaseDynamic2Config,
    RobotConfig,
    UnitreeG1WholeBodyDynamic1Config,
    UnitreeG1WholeBodyDynamic2Config,
    UnitreeG1WholeBodyKinematics,
    UnitreeG1WholeBodyWithHandDynamic1Config,
)
from spark_utils import initialize_class


class _ClassConfig:
    class_name = None


class _TrackingDynamic2Config(AgiBotG1MobileBaseDynamic2Config):
    def __init__(self):
        self.model_request = None
        super().__init__()

    def create_dynamics_model(self, state_dof_names=None, control_names=None):
        self.model_request = (tuple(state_dof_names or ()), tuple(control_names or ()))
        return super().create_dynamics_model(state_dof_names, control_names)


class DynamicsConfigTests(unittest.TestCase):
    def test_config_and_reduced_dimensions(self):
        dynamic1 = AgiBotG1MobileBaseDynamic1Config()
        dynamic2 = AgiBotG1MobileBaseDynamic2Config()
        unicycle = AgiBotG1MobileBaseUnicycleDynamic1Config()
        bicycle = AgiBotG1MobileBaseBicycleDynamic2Config()

        self.assertEqual((dynamic1.num_state, len(dynamic1.Control)), (19, 19))
        self.assertEqual((dynamic2.num_state, len(dynamic2.Control)), (38, 19))
        self.assertEqual((bicycle.num_state, len(bicycle.Control)), (38, 19))
        single_model = dynamic1.create_dynamics_model(
            ["LinearX", "LinearY"], ["vLinearX", "vLinearY"]
        )
        double_model = dynamic2.create_dynamics_model(
            ["LinearX", "LinearY"], ["aLinearX", "aLinearY"]
        )
        unicycle_model = unicycle.create_dynamics_model()
        bicycle_model = bicycle.create_dynamics_model()
        self.assertEqual((single_model.state_dim, single_model.control_dim), (2, 2))
        self.assertEqual((double_model.state_dim, double_model.control_dim), (4, 2))
        self.assertEqual((unicycle_model.state_dim, unicycle_model.control_dim), (3, 2))
        self.assertEqual((bicycle_model.state_dim, bicycle_model.control_dim), (6, 2))
        self.assertEqual(
            bicycle_model.state_names,
            ("X", "Y", "v_x", "v_y", "r", "psi"),
        )

    def test_expected_derivatives(self):
        single = AgiBotG1MobileBaseDynamic1Config().create_dynamics_model(
            ["LinearX", "LinearY"], ["vLinearX", "vLinearY"]
        )
        double = AgiBotG1MobileBaseDynamic2Config().create_dynamics_model(
            ["LinearX", "LinearY"], ["aLinearX", "aLinearY"]
        )
        unicycle = AgiBotG1MobileBaseUnicycleDynamic1Config().create_dynamics_model()
        bicycle = AgiBotG1MobileBaseBicycleDynamic2Config().create_dynamics_model()

        np.testing.assert_allclose(single.derivative([1.0, 2.0], [0.3, -0.4]), [0.3, -0.4])
        np.testing.assert_allclose(
            double.derivative([1.0, 2.0, 3.0, 4.0], [0.5, -0.25]),
            [3.0, 4.0, 0.5, -0.25],
        )
        np.testing.assert_allclose(
            unicycle.derivative([0.0, 0.0, np.pi / 2.0], [2.0, 0.5]),
            [0.0, 2.0, 0.5],
            atol=1e-12,
        )
        np.testing.assert_allclose(
            bicycle.derivative(np.zeros(6), [4.0, 0.0]),
            [0.0, 0.0, 4.0, 0.0, 0.0, 0.0],
        )

    def test_bicycle_state_round_trip_uses_body_frame_matlab_order(self):
        config = AgiBotG1MobileBaseBicycleDynamic2Config()
        model = config.create_dynamics_model()
        bicycle_state = np.array([1.0, 2.0, 3.0, 0.4, -0.2, np.pi / 3.0])

        full_state = model.expand_state(bicycle_state)

        np.testing.assert_allclose(model.project_state(full_state), bicycle_state, atol=1e-12)
        position = config.decompose_state_to_dof_pos(full_state)
        velocity = config.decompose_state_to_dof_vel(full_state)
        self.assertAlmostEqual(position[config.DoFs.RotYaw], bicycle_state[5])
        np.testing.assert_allclose(
            velocity[[config.DoFs.LinearX, config.DoFs.LinearY]],
            [
                np.cos(bicycle_state[5]) * bicycle_state[2]
                - np.sin(bicycle_state[5]) * bicycle_state[3],
                np.sin(bicycle_state[5]) * bicycle_state[2]
                + np.cos(bicycle_state[5]) * bicycle_state[3],
            ],
            atol=1e-12,
        )

    def test_bicycle_derivative_matches_course_dynamic_single_track_equation(self):
        model = AgiBotG1MobileBaseBicycleDynamic2Config().create_dynamics_model()

        derivative = model.derivative(
            [1.0, 2.0, 4.0, 0.3, 0.2, 0.4],
            [1.2, 0.1],
        )

        np.testing.assert_allclose(
            derivative,
            [3.56741847, 1.83399167, 1.22632821, -2.23222482, 3.24519673, 0.2],
            rtol=1e-8,
            atol=1e-8,
        )

    def test_double_integrator_zoh_transition(self):
        model = AgiBotG1MobileBaseDynamic2Config().create_dynamics_model(["LinearX"], ["aLinearX"])
        np.testing.assert_allclose(model.step([1.0, 2.0], [3.0], 0.5, "ZOH"), [2.375, 3.5])

    def test_r1_lite_double_integrator_maps_controls_by_joint_name(self):
        config = GalaxeaR1LiteFixedBaseDynamic2Config()
        model = config.create_dynamics_model(["RightJoint1"], ["aRightJoint1"])

        np.testing.assert_allclose(model.derivative([0.0, 0.0], [0.5]), [0.0, 0.5])

    def test_reduced_model_projects_and_merges_full_robot_state(self):
        config = AgiBotG1MobileBaseUnicycleDynamic1Config()
        model = config.create_dynamics_model()
        full_state = np.asarray([config.DefaultDoFVal[dof] for dof in config.DoFs], dtype=float)
        full_state[config.DoFs.LinearX] = 1.0
        full_state[config.DoFs.LinearY] = 2.0
        full_state[config.DoFs.RotYaw] = np.pi / 2.0
        full_control = np.zeros(len(config.Control), dtype=float)
        full_control[config.Control.vLinearX] = 0.5
        full_control[config.Control.vRotYaw] = -0.25

        reduced_state = model.project_state(full_state)
        reduced_control = model.project_control(full_control)
        next_reduced_state = model.step(reduced_state, reduced_control, 0.2)
        next_full_state = model.merge_state(next_reduced_state, into=full_state)

        np.testing.assert_allclose(reduced_state, [1.0, 2.0, np.pi / 2.0])
        np.testing.assert_allclose(reduced_control, [0.5, -0.25])
        np.testing.assert_allclose(
            next_full_state[[config.DoFs.LinearX, config.DoFs.LinearY, config.DoFs.RotYaw]],
            [1.0, 2.1, np.pi / 2.0 - 0.05],
            atol=1e-12,
        )
        arm_indices = [idx for idx in range(config.num_dof) if idx not in model.dof_indices]
        np.testing.assert_array_equal(next_full_state[arm_indices], full_state[arm_indices])

    def test_configs_are_registered(self):
        for name in (
            "AgiBotG1MobileBaseDynamic1Config",
            "AgiBotG1MobileBaseDynamic2Config",
            "AgiBotG1MobileBaseUnicycleDynamic1Config",
            "AgiBotG1MobileBaseBicycleDynamic2Config",
        ):
            cfg = _ClassConfig()
            cfg.class_name = name
            self.assertEqual(type(initialize_class(cfg)).__name__, name)

    def test_every_exported_concrete_robot_config_supports_model_step(self):
        validated = []
        for name in sorted(dir(spark_robot)):
            config_type = getattr(spark_robot, name)
            if (
                not inspect.isclass(config_type)
                or config_type is RobotConfig
                or not issubclass(config_type, RobotConfig)
            ):
                continue
            required = [
                parameter
                for parameter in inspect.signature(config_type).parameters.values()
                if parameter.default is inspect.Parameter.empty
                and parameter.kind
                in (inspect.Parameter.POSITIONAL_ONLY, inspect.Parameter.POSITIONAL_OR_KEYWORD)
            ]
            if required:
                continue
            config = config_type()
            model = config.create_dynamics_model()
            next_state = model.step(
                np.zeros(model.state_dim),
                np.zeros(model.control_dim),
                0.001,
                getattr(model, "default_integrator", "Euler"),
            )
            self.assertEqual(next_state.shape, (model.state_dim,), name)
            self.assertEqual(len(model.state_names), model.state_dim, name)
            validated.append(name)
        self.assertGreaterEqual(len(validated), 50)

    def test_floating_base_and_hand_state_layouts(self):
        floating = UnitreeG1WholeBodyDynamic2Config().create_dynamics_model()
        with_hand = UnitreeG1WholeBodyWithHandDynamic1Config().create_dynamics_model()
        self.assertEqual((floating.state_dim, floating.control_dim), (71, 29))
        self.assertEqual((with_hand.state_dim, with_hand.control_dim), (50, 29))

    def test_whole_body_dynamic2_accepts_aligned_isaac_velocity(self):
        config = UnitreeG1WholeBodyDynamic2Config()
        position = np.arange(config.num_dof, dtype=float)
        physical_velocity = np.arange(config.num_dof - 1, dtype=float)
        aligned_velocity = np.insert(physical_velocity, 6, 1234.0)

        physical_state = config.compose_state_from_dof(position, physical_velocity)
        aligned_state = config.compose_state_from_dof(position, aligned_velocity)

        self.assertEqual(aligned_state.shape, (config.num_state,))
        np.testing.assert_array_equal(aligned_state, physical_state)

    def test_hand_configuration_is_projected_to_whole_body_kinematics(self):
        robot_cfg = UnitreeG1WholeBodyWithHandDynamic1Config()
        kinematics = UnitreeG1WholeBodyKinematics(robot_cfg=robot_cfg)
        configured_position = np.arange(len(robot_cfg.DoFs), dtype=float)

        projected_position = kinematics._pin_position(configured_position)
        expected_position = np.asarray(
            [
                configured_position[int(getattr(robot_cfg.DoFs, dof.name))]
                for dof in UnitreeG1WholeBodyDynamic1Config.DoFs
            ]
        )

        self.assertEqual(projected_position.shape, (kinematics.model.nq,))
        np.testing.assert_array_equal(projected_position, expected_position)

        default_position = np.asarray(
            [robot_cfg.DefaultDoFVal[dof] for dof in robot_cfg.DoFs],
            dtype=float,
        )
        frames = kinematics.forward_kinematics(default_position)
        self.assertTrue(np.isfinite(frames).all())


class PolicyDynamicsTests(unittest.TestCase):
    def test_lqr_requests_dynamics_from_robot_config(self):
        robot_cfg = _TrackingDynamic2Config()
        policy = LQRPolicy(
            robot_cfg,
            None,
            state_dim=4,
            control_dim=2,
            position_dof_names=["LinearX", "LinearY"],
            control_names=["aLinearX", "aLinearY"],
            Q=[1.0, 1.0, 0.0, 0.0],
            R=[1.0, 1.0],
        )
        self.assertIs(policy.dynamics_model.robot_cfg, robot_cfg)
        self.assertEqual(
            robot_cfg.model_request, (("LinearX", "LinearY"), ("aLinearX", "aLinearY"))
        )
        self.assertEqual(policy.dynamics_model.variant, "double_integrator")

    def test_legacy_model_cannot_override_robot_config(self):
        with self.assertRaisesRegex(ValueError, "conflicts with robot-config dynamics"):
            LQRPolicy(
                AgiBotG1MobileBaseDynamic2Config(),
                None,
                model="single_integrator",
                state_dim=2,
                control_dim=1,
                position_dof_names=["LinearX"],
                control_names=["aLinearX"],
            )

    def test_ilqr_requires_and_uses_unicycle_config(self):
        robot_cfg = AgiBotG1MobileBaseUnicycleDynamic1Config()
        policy = ILQRPolicy(robot_cfg, None, N=2, max_iter=1, goal_state=[1.0, 0.0, 0.0])
        self.assertIs(policy.dynamics_model.robot_cfg, robot_cfg)
        np.testing.assert_allclose(policy._dynamics([0.0, 0.0, 0.0], [1.0, 0.2]), [1.0, 0.0, 0.2])


if __name__ == "__main__":
    unittest.main()
