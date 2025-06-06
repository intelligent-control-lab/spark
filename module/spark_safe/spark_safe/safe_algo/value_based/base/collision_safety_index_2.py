from spark_robot import RobotKinematics
import numpy as np

from spark_safe.safe_algo.value_based.base.basic_collision_safety_index import BasicCollisionSafetyIndex

class SecondOrderCollisionSafetyIndex(BasicCollisionSafetyIndex):
    
    def __init__(self,
                 robot_kinematics : RobotKinematics,
                 **kwargs
                 ):
        
        super().__init__(robot_kinematics, **kwargs)
        
        assert "Dynamic2" in self.robot_cfg.__class__.__name__, "SecondOrderCollisionSafetyIndex is only supported for robot configuration with Dynamic2 (Acceleration Control)"
        
        self.n = kwargs["phi_n"]
        self.k = kwargs["phi_k"]
        eps = 1e-6 # Add small value to avoid division by zero
        
        # Safety specification in the Cartesian space
        self._phi0 = lambda d, normal: self.dmin - np.sum(- d * normal, axis=1)

        # Time derivative of the safety specification in the Cartesian space
        self._phi0dot = lambda v, normal: np.sum(v * normal, axis=1)

        # General implementation of first-order safety index
        # Normal direction is the distance-decreasing direction
        self._phi = lambda d, v, normal: (
            self.dmin**self.n - np.sum(- d * normal, axis=1)**self.n
            + self.k * np.sum(v * normal, axis=1)
        )

        # General implementation of the time derivative of the first-order safety index
        self._phi_dot = lambda d, v, a, normal, curv: (
            - self.n * (np.sum(- d * normal, axis=1) + eps)**(self.n - 1) * np.sum(v * normal, axis=1)
            + self.k * np.sum(a * normal, axis=1)
            - self.k * np.sum(v * (curv @ v[..., None]).squeeze(-1), axis=1)
        )

        # Control-related functions in Cartesian space
        self.Cartesian_Lg = lambda normal: self.k * normal

        self.Cartesian_Lf = lambda d, v, normal, curv: (
            - self.n * (np.sum(- d * normal, axis=1) + eps)**(self.n - 1) * np.sum(v * normal, axis=1)
            - self.k * np.sum(v * (curv @ v[..., None]).squeeze(-1), axis=1)
        )
        
    def phi(self,
            x        : np.ndarray,
            task_info: dict):
        '''
            [num_constraint,] Safety index function.

            Args:
                x (np.ndarray): [num_state,] robot state.
        '''
        
        robot_collision_vol, obstacle_collision_vol = self.get_vol_info(x, task_info)
        d, v, normal, curv = self.compute_pairwise_vol_info(robot_collision_vol, obstacle_collision_vol)
        
        phi = self._phi(d, v, normal)
        phi0 = self._phi0(d, normal)
        phi0dot = self._phi0dot(v, normal)
        Cartesian_Lg = self.Cartesian_Lg(normal)
        Cartesian_Lf = self.Cartesian_Lf(d, v, normal, curv)
        
        dof_pos = self.robot_cfg.decompose_state_to_dof_pos(x)
        dof_vel = self.robot_cfg.decompose_state_to_dof_vel(x)
        
        jacobian_full = self.jacobian_full(dof_pos)
        jacobian_dot_full = self.jacobian_dot_full(dof_pos, dof_vel)
        
        gx = self.robot_cfg.dynamics_g(x)
        fx = self.robot_cfg.dynamics_f(x)
        s_matrix = np.hstack([np.zeros((len(x) - self.num_dof,  self.num_dof)), np.eye(len(x) - self.num_dof)]) # only take the velocity part
        
        Lg = np.einsum('ij,ijk->ik', Cartesian_Lg, jacobian_full @ s_matrix @ gx)
        Lf = Cartesian_Lf + np.einsum('ij,ijk->ik', Cartesian_Lg, jacobian_dot_full) @ dof_vel + np.einsum('ij,ijk->ik', Cartesian_Lg, jacobian_full) @ s_matrix @ fx
        return phi, Lg, Lf, phi0, phi0dot
        
    