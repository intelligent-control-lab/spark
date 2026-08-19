import unittest
import importlib
from pathlib import Path

import numpy as np

from spark_policy import BasePolicy, Policy
from spark_policy.control.pid.benchmark import BenchmarkPIDPolicy
from spark_policy.control.pid.teleop import TeleopPIDPolicy
from spark_policy.control.linear_quadratic import LQRPolicy
from spark_policy.control.pid import (
    CartesianPIDController,
    IKJointPIDController,
    JointPIDController,
)
from spark_policy.core import ComponentRole, JointReference, get_component_spec
from spark_policy.planning.motion import RRTConnectPolicy
from spark_policy.safety.filtering import BasicControlBarrierFunction, ByPassSafeControl
from spark_policy.safety.monitoring import FirstOrderCollisionSafetyIndex


class PolicyArchitectureTests(unittest.TestCase):
    def test_legacy_policy_namespaces_are_not_importable(self):
        for module_name in (
            "spark_policy.base",
            "spark_policy.model_based",
            "spark_policy.model_free",
            "spark_policy.safe",
        ):
            with self.subTest(module_name=module_name):
                with self.assertRaises(ModuleNotFoundError):
                    importlib.import_module(module_name)

    def test_implementations_resolve_inside_new_structure(self):
        policy_root = Path(importlib.import_module("spark_policy").__file__).resolve().parent
        for implementation in (
            LQRPolicy,
            RRTConnectPolicy,
            BasicControlBarrierFunction,
            FirstOrderCollisionSafetyIndex,
        ):
            module = importlib.import_module(implementation.__module__)
            module_path = Path(module.__file__).resolve()
            self.assertTrue(module_path.is_relative_to(policy_root))
            self.assertFalse({"base", "model_based", "model_free", "safe"} & set(module_path.parts))

    def test_legacy_policy_is_new_policy(self):
        self.assertTrue(issubclass(BasePolicy, Policy))

    def test_role_based_imports_resolve_to_existing_implementations(self):
        self.assertEqual(LQRPolicy.__name__, "LQRPolicy")
        self.assertEqual(RRTConnectPolicy.__name__, "RRTConnectPolicy")
        self.assertEqual(BasicControlBarrierFunction.__name__, "BasicControlBarrierFunction")
        self.assertEqual(ByPassSafeControl.__name__, "ByPassSafeControl")
        self.assertEqual(FirstOrderCollisionSafetyIndex.__name__, "FirstOrderCollisionSafetyIndex")
        self.assertTrue(issubclass(BenchmarkPIDPolicy, Policy))
        self.assertTrue(issubclass(TeleopPIDPolicy, Policy))

    def test_legacy_components_have_secondary_metadata(self):
        self.assertEqual(get_component_spec(LQRPolicy).role, ComponentRole.CONTROLLER)
        self.assertEqual(
            get_component_spec(FirstOrderCollisionSafetyIndex).role,
            ComponentRole.SAFETY_MONITOR,
        )

    def test_joint_pid(self):
        controller = JointPIDController(kp=2.0, kd=0.5)
        command = controller.compute(
            JointReference(position=np.array([1.0, -1.0])),
            position=np.array([0.5, -0.5]),
            velocity=np.array([0.2, -0.2]),
        )
        np.testing.assert_allclose(command, [0.9, -0.9])

    def test_cartesian_pid_returns_task_space_twist(self):
        controller = CartesianPIDController(1.0, 0.0, 1.0, 0.0)
        target = np.eye(4)
        target[0, 3] = 0.2
        command = controller.compute(target, np.eye(4))
        np.testing.assert_allclose(command, [0.2, 0.0, 0.0, 0.0, 0.0, 0.0])

    def test_ik_joint_pid_uses_robot_kinematics(self):
        class Kinematics:
            def inverse_kinematics(self, frames, seed):
                return np.asarray(seed) + 0.5, True

        controller = IKJointPIDController(
            Kinematics(),
            JointPIDController(kp=2.0, kd=0.0),
        )
        command, info = controller.compute(
            [np.eye(4)],
            position=np.array([0.0, 1.0]),
            velocity=np.zeros(2),
        )
        np.testing.assert_allclose(command, [1.0, 1.0])
        self.assertTrue(info["ik_success"])


if __name__ == "__main__":
    unittest.main()
