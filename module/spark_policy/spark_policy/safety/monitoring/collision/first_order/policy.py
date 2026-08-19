from spark_robot import RobotKinematics
import numpy as np

from spark_policy.safety.monitoring.collision.base import BasicCollisionSafetyIndex
import time


class FirstOrderCollisionSafetyIndex(BasicCollisionSafetyIndex):
    def __init__(self, robot_kinematics: RobotKinematics, **kwargs):

        super().__init__(robot_kinematics, **kwargs)

        assert self.robot_cfg.dynamics_order == 1, (
            "FirstOrderCollisionSafetyIndex requires first-order robot dynamics"
        )

        # Safety index parameters for the first order safety index
        self.n = 1.0
        self.k = 0

        # Safety specification in the Cartesian space
        self._phi0 = lambda d, normal: self.dmin - np.sum(-d * normal, axis=1)

        # Time derivative of the safety specification in the Cartesian space
        self._phi0dot = lambda v, normal: np.sum(v * normal, axis=1)

        # General implementation of first order safety index; normal direction is the distance decreasing direction
        self._phi = self._phi0

        # General implementation of the time derivative of the first order safety index in the Cartesian space
        self._phi_dot = self._phi0dot

        self.Cartesian_Lg = lambda normal: normal
        self.Cartesian_Lf = lambda normal: np.zeros((normal.shape[0], 1))

    def _dynamics_g_for_jacobian(self, gx: np.ndarray, jacobian_full: np.ndarray) -> np.ndarray:
        if jacobian_full.shape[-1] == gx.shape[0]:
            return gx
        if gx.shape[0] == jacobian_full.shape[-1] + 1:
            gx_vel = np.zeros((jacobian_full.shape[-1], gx.shape[1]), dtype=gx.dtype)
            gx_vel[:3, :] = gx[:3, :]
            gx_vel[6:, :] = gx[7:, :]
            return gx_vel
        raise ValueError(
            f"dynamics_g shape {gx.shape} is incompatible with jacobian shape {jacobian_full.shape}"
        )

    def phi(self, x: np.ndarray, task_info: dict):
        """
        [num_constraint,] Safety index function.

        Args:
            x (np.ndarray): [num_state,] robot state.
        """

        robot_collision_vol, obstacle_collision_vol = self.get_vol_info(x, task_info)
        d, v, normal, curv = self.compute_pairwise_vol_info(
            robot_collision_vol, obstacle_collision_vol
        )

        phi = self._phi(d, normal)
        phi0 = self._phi0(d, normal)
        phi0dot = self._phi0dot(v, normal)
        Cartesian_Lg = self.Cartesian_Lg(normal)
        Cartesian_Lf = self.Cartesian_Lf(normal)
        dof_pos = self.robot_cfg.decompose_state_to_dof_pos(x)
        jacobian_full = self.jacobian_full(dof_pos)
        gx = self.robot_cfg.dynamics_g(x)
        gx = self._dynamics_g_for_jacobian(gx, jacobian_full)

        Lg = np.einsum("ij,ijk->ik", Cartesian_Lg, jacobian_full @ gx)
        Lf = Cartesian_Lf
        return phi, Lg, Lf, phi0, phi0dot
