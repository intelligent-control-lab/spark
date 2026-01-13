from spark_robot import RobotKinematics
import numpy as np
import torch
import onnx
from onnx2torch import convert

from spark_policy.safe import SPARK_SAFE_ROOT
from spark_policy.safe.safe_algo.value_based.base.basic_collision_safety_index import BasicCollisionSafetyIndex

import os

class SecondOrderNNCollisionSafetyIndex(BasicCollisionSafetyIndex):
    
    def __init__(self,
                 robot_kinematics : RobotKinematics,
                 **kwargs
                 ):
        
        super().__init__(robot_kinematics, **kwargs)
        
        assert "Dynamic2" in self.robot_cfg.__class__.__name__, "SecondOrderNNCollisionSafetyIndex is only supported for robot configuration with Dynamic2 (Acceleration Control)"
        
        
        self.n = kwargs["phi_n"]
        self.k = kwargs["phi_k"]
        
        self.phi_nn_path = os.path.join(SPARK_SAFE_ROOT, "safe/safe_algo/value_based/base/onnx_model", kwargs["phi_nn_path"])  
        self.phi_nn_onnx = onnx.load(self.phi_nn_path)
        onnx.checker.check_model(self.phi_nn_onnx)
        print("Model successfully saved and verified in ONNX format.")
        self.phi_nn_torch = convert(self.phi_nn_onnx)
        self.phi_nn_torch.eval()
        
        # Safety specification in the Cartesian space
        self.phi0 = lambda d, normal: self.dmin - np.sum(- d * normal, axis=1)
        
        # Time derivative of the safety specification in the Cartesian space
        self.phi0dot = lambda v, normal: np.sum(v * normal, axis=1)

        # General implementation of second order safety index; normal direction is the distance decreasing direction
        self._phi = lambda d, v, normal, nn_term: self.dmin - np.sum(- d * normal, axis=1) + nn_term + self.k*np.sum(v * normal, axis=1)
        
        # General implementation of the time derivative of the second order safety index in the Cartesian space
        self._phi_dot = lambda d, v, a, normal, curv, nn_term_grad: np.sum(v * normal, axis=1) - np.sum(nn_term_grad[:, :3] * normal, axis=1) * np.sum(v * normal, axis=1) + self.k*np.sum(a * normal, axis=1) - self.k*np.sum(v * (curv@v[..., None]).squeeze(-1), axis=1)

        self.Cartesian_Lg = lambda normal: self.k * normal
        self.Cartesian_Lf = lambda d, v, normal, curv, nn_term_grad: np.sum(v * normal, axis=1) - np.sum(nn_term_grad[:, :3] * normal, axis=1) * np.sum(v * normal, axis=1) - self.k*np.sum(v * (curv@v[..., None]).squeeze(-1), axis=1)
    
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
        
        torch_input = torch.from_numpy(np.sum(d * normal, axis=1, dtype=np.float32).reshape(-1,1)).requires_grad_(True)
        torch_output = self.phi_nn_torch(torch_input)
        torch_output.backward(torch.ones_like(torch_output))
        torch_grad = torch_input.grad.numpy()
        
        phi = self._phi(d, v, normal, torch_output[:,0].detach().numpy().flatten())
        phi0 = self.phi0(d, normal)
        phi0dot = self.phi0dot(v, normal)
        Cartesian_Lg = self.Cartesian_Lg(normal)
        Cartesian_Lf = self.Cartesian_Lf(d, v, normal, curv, torch_grad)
        
        dof_pos = self.robot_cfg.decompose_state_to_dof_pos(x)
        dof_vel = self.robot_cfg.decompose_state_to_dof_vel(x)
        
        jacobian_full = self.jacobian_full(dof_pos)
        jacobian_dot_full = self.jacobian_dot_full(dof_pos, dof_vel)
        
        gx = self.robot_cfg.dynamics_g(x)
        fx = self.robot_cfg.dynamics_f(x)
        
        s_matrix = np.hstack([np.zeros((self.num_dof, self.num_dof)), np.eye(self.num_dof)]) # only take the velocity part
        
        Lg = np.einsum('ij,ijk->ik', Cartesian_Lg, jacobian_full @ s_matrix @ gx)
        Lf = Cartesian_Lf + np.einsum('ij,ijk->ik', Cartesian_Lg, jacobian_dot_full) @ dof_vel + np.einsum('ij,ijk->ik', Cartesian_Lg, jacobian_full) @ s_matrix @ fx
        
        return phi, Lg, Lf, phi0, phi0dot
        
    