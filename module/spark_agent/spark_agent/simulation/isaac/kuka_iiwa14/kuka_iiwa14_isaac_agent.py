"""Shared declarative Isaac adapter for KUKA iiwa 14 embodiments."""

import numpy as np

from spark_agent.simulation.isaac.isaac_agent import (
    ConfiguredIsaacAgent,
    _quaternion_wxyz_to_matrix,
    _rpy_to_quaternion_wxyz,
)


def remove_asset_alignment_from_base_frame(robot_base_frame, base_rpy):
    """Return the logical base pose after removing fixed URDF calibration."""

    calibration = np.eye(4, dtype=float)
    calibration[:3, :3] = _quaternion_wxyz_to_matrix(_rpy_to_quaternion_wxyz(base_rpy))
    return np.asarray(robot_base_frame, dtype=float) @ np.linalg.inv(calibration)


class KukaIIWA14IsaacAgent(ConfiguredIsaacAgent):
    """Keep KUKA's imported-asset alignment out of logical base feedback."""

    def get_feedback(self):
        feedback = super().get_feedback()
        # ``base_rpy`` calibrates the URDF axes to the KUKA MJCF/Pinocchio
        # convention. It is not a moving robot-base pose. Reporting that
        # calibration as ``robot_base_frame`` made the scalar teleop pipeline
        # rotate its analytical collision model a second time by -90 degrees.
        feedback["robot_base_frame"] = remove_asset_alignment_from_base_frame(
            feedback["robot_base_frame"],
            self.robot_cfg.isaac_articulation.base_rpy,
        )
        return feedback
