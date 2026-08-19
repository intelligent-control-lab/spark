import unittest

import numpy as np

from spark_policy import (
    ExtendedKalmanFilterEstimator,
    FrequencyILCPolicy,
    GradientParameterEstimator,
    KalmanFilterEstimator,
    MRACPolicy,
    RecursiveLeastSquaresEstimator,
    SteadyStateKalmanFilterEstimator,
    TimeDomainILCPolicy,
    UnscentedKalmanFilterEstimator,
)
from spark_policy.core import (
    ComponentRole,
    ParameterEstimate,
    PortKind,
    StateEstimate,
    ExecutionContext,
    get_component_spec,
)
from spark_robot import LinearDiscreteDynamicsConfig


class EstimationTests(unittest.TestCase):
    def setUp(self):
        self.A = np.array([[0.9]])
        self.B = np.array([[1.0]])
        self.C = np.array([[1.0]])
        self.Bw = np.array([[1.0]])
        self.W = np.array([[0.1]])
        self.V = np.array([[0.2]])
        self.x0 = np.array([0.0])
        self.X0 = np.array([[1.0]])

    def test_linear_filters_support_numerical_and_component_apis(self):
        for estimator_type in (KalmanFilterEstimator, SteadyStateKalmanFilterEstimator):
            with self.subTest(estimator=estimator_type.__name__):
                estimator = estimator_type(
                    self.A,
                    self.B,
                    self.C,
                    self.Bw,
                    self.W,
                    self.V,
                    self.x0,
                    self.X0,
                )
                state, covariance = estimator.step([0.0], [1.0])
                self.assertEqual(state.shape, (1,))
                self.assertEqual(covariance.shape, (1, 1))
                estimate = estimator.estimate(
                    {"control": np.array([0.0]), "measurement": np.array([1.0])}
                )
                self.assertIsInstance(estimate, StateEstimate)
                estimator.reset()
                np.testing.assert_allclose(estimator.xhat, self.x0)

    def test_nonlinear_filters_step(self):
        transition = lambda state, control: self.A @ state + self.B @ control
        measurement = lambda state: self.C @ state
        jacobian_A = lambda _state, _control: self.A
        jacobian_C = lambda _state: self.C
        estimators = (
            ExtendedKalmanFilterEstimator(
                transition,
                measurement,
                jacobian_A,
                jacobian_C,
                self.Bw,
                self.W,
                self.V,
                self.x0,
                self.X0,
            ),
            UnscentedKalmanFilterEstimator(
                transition,
                measurement,
                self.Bw,
                self.W,
                self.V,
                self.x0,
                self.X0,
            ),
        )
        for estimator in estimators:
            with self.subTest(estimator=type(estimator).__name__):
                state, covariance = estimator.step([0.0], [1.0])
                self.assertTrue(np.all(np.isfinite(state)))
                self.assertTrue(np.all(np.isfinite(covariance)))

    def test_parameter_estimators_reset(self):
        rls = RecursiveLeastSquaresEstimator(p0hat=1.0, H0=0.5, lam=0.5)
        gradient = GradientParameterEstimator(p0hat=1.0, H0=0.5)
        self.assertEqual(len(rls.step(0.5, 1.0)), 2)
        self.assertIsInstance(gradient.step(0.5, 1.0), float)
        estimate = rls.estimate({"input": 0.5, "measurement": 1.0})
        self.assertIsInstance(estimate, ParameterEstimate)
        self.assertIn(
            PortKind.PARAMETER_ESTIMATE,
            get_component_spec(rls).outputs,
        )
        rls.reset()
        gradient.reset()
        self.assertEqual(rls.phat, 1.0)
        self.assertEqual(gradient.phat, 1.0)


class LearningControlTests(unittest.TestCase):
    def test_mrac_control_update_and_history(self):
        controller = MRACPolicy(
            Astar=np.eye(2) * 0.8,
            Ahat=np.eye(2) * 0.7,
            Bhat=np.eye(2),
            F=np.eye(4),
            x0=np.array([1.0, -1.0]),
            horizon=4,
            dt=1.0,
        )
        self.assertEqual(controller.control([1.0, -1.0], 0.0, 0).shape, (2,))
        controller.control([0.7, -0.7], 1.0, 1)
        self.assertEqual(len(controller.Ahat_history), len(controller.F_history))
        self.assertEqual(len(controller.error_history), len(controller.F_history))
        self.assertEqual(get_component_spec(controller).role, ComponentRole.CONTROLLER)

    def test_mrac_component_api_accepts_explicit_reference(self):
        controller = MRACPolicy(
            Astar=np.eye(2) * 0.8,
            Ahat=np.eye(2) * 0.7,
            Bhat=np.eye(2),
            F=np.eye(4),
        )
        first = controller.step(
            {"state": np.array([1.0, -1.0]), "reference": np.array([1.0, -1.0])},
            ExecutionContext(timestamp=0.0, dt=1.0, episode_step=0),
        )
        second = controller.step(
            {"state": np.array([0.7, -0.7]), "reference": np.array([0.8, -0.8])},
            ExecutionContext(timestamp=1.0, dt=1.0, episode_step=1),
        )
        self.assertEqual(first.value.values.shape, (2,))
        self.assertEqual(second.value.values.shape, (2,))
        self.assertIn("Ahat", second.diagnostics)
        self.assertIsNotNone(second.diagnostics["tracking_error"])

    def test_ilc_updates_have_expected_horizon(self):
        frequency = FrequencyILCPolicy(
            a=np.array([1.0, -2.0, 1.0]),
            b=np.array([1.0]),
            c=np.array([-0.5, 0.4]),
        )
        self.assertEqual(frequency.update(np.ones(6), np.zeros(6)).shape, (6,))

        time_domain = TimeDomainILCPolicy(
            np.array([[0.5]]),
            np.array([[1.0]]),
            np.array([[0.0]]),
            np.array([[1.0]]),
            horizon=4,
        )
        self.assertEqual(time_domain.update(np.ones(5), np.zeros(4)).shape, (4,))
        self.assertEqual(get_component_spec(time_domain).role, ComponentRole.CONTROLLER)

    def test_time_domain_ilc_builds_from_robot_dynamics_model(self):
        robot_cfg = LinearDiscreteDynamicsConfig([[0.5]], [[1.0]])
        controller = TimeDomainILCPolicy.from_dynamics_model(
            robot_cfg.create_dynamics_model(),
            K=np.array([[0.0]]),
            C=np.array([[1.0]]),
            horizon=4,
            dt=1.0,
            discretization="native",
        )
        result = controller.step({"error": np.ones(5), "previous_feedforward": np.zeros(4)})
        self.assertEqual(result.value.shape, (4,))
        self.assertIn("learning_matrix", result.diagnostics)


if __name__ == "__main__":
    unittest.main()
