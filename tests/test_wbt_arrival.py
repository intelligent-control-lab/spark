import unittest

import numpy as np

from spark_policy.control.whole_body.unitree_g1.wbt.policy import UnitreeG1WBTPolicy


class WBTArrivalTests(unittest.TestCase):
    def test_explicit_arrival_immediately_clears_planar_command_history(self):
        policy = object.__new__(UnitreeG1WBTPolicy)
        policy.base_position_kp = np.ones(3, dtype=float)
        policy.base_command_deadband = np.zeros(5, dtype=float)
        policy.base_command_rate_limit = np.full(5, 0.01, dtype=float)
        policy.base_xy_yaw_limit = np.ones(3, dtype=float)
        policy.base_z_pitch_limit = np.ones(2, dtype=float)
        policy._last_base_command = np.array([0.3, -0.2, 0.1, 0.0, 0.0])

        supplied_command = np.array([0.25, -0.15, 0.08, 0.0, 0.0])
        command = policy._base_command_from_goal(
            {"robot_base_frame": np.eye(4)},
            {
                "wbt_command": supplied_command,
                "wbt_goal_arrived": True,
            },
        )

        np.testing.assert_allclose(command[:3], 0.0)
        np.testing.assert_allclose(policy._last_base_command[:3], 0.0)
        # Policy processing must not mutate the composed policy's diagnostic.
        np.testing.assert_allclose(supplied_command, [0.25, -0.15, 0.08, 0.0, 0.0])


if __name__ == "__main__":
    unittest.main()
