import unittest

import numpy as np

from spark_policy import Policy
from spark_policy.composed_policy.safety_filtered import SafetyFilteredPolicy


class _Control:
    vLinearX = 0

    @classmethod
    def __iter__(cls):
        return iter([0])


class _ControlEnum:
    __members__ = {"vLinearX": 0}

    def __iter__(self):
        return iter([0])


class _RobotConfig:
    Control = _ControlEnum()
    ControlLimit = {0: 1.0}


class _NominalPolicy(Policy):
    def act(self, agent_feedback, task_info):
        return np.array([2.0]), {"nominal": True}


class _SafetyFilter:
    def safe_control(self, x, u_ref, agent_feedback, task_info, action_info):
        return np.asarray(u_ref) * 0.25, {"trigger_safe": True}


class CompositionTests(unittest.TestCase):
    def test_safety_filtered_policy_preserves_diagnostics(self):
        policy = SafetyFilteredPolicy(
            _NominalPolicy(),
            _SafetyFilter(),
            robot_cfg=_RobotConfig(),
        )
        action, info = policy.act({"state": np.zeros(1)}, {})
        np.testing.assert_allclose(action, [0.5])
        self.assertTrue(info["nominal"])
        self.assertTrue(info["trigger_safe"])
        np.testing.assert_allclose(info["u_ref"], [2.0])
        np.testing.assert_allclose(info["u_safe"], [0.5])
        np.testing.assert_allclose(info["safe_base_u_delta"], [-1.5])


if __name__ == "__main__":
    unittest.main()
