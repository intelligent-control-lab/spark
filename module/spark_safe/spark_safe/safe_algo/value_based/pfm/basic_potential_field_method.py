import numpy as np

from spark_safe.safe_algo.value_based.base.value_based_safe_algo import ValueBasedSafeAlgorithm

class BasicPotentialFieldMethod(ValueBasedSafeAlgorithm):

    def __init__(self, safety_index, **kwargs):
        super().__init__(safety_index, **kwargs)
        print("Initializing BasicPotentialFieldMethod")
        # default values
        self.c_pfm_default = kwargs["c_pfm"]
    
    def safe_control(self,
                     x             : np.ndarray,
                     u_ref         : np.ndarray,
                     agent_feedback: dict,
                     task_info     : dict,
                     action_info   : dict
                     ):
        
        phi, Lg, Lf, phi0, phi0dot = self.safety_index.phi(x, task_info)        
        
        if (phi[self.safety_index.phi_mask>0].max() > 0):
            trigger_safe = True
            masked_indices_phi = np.argwhere(self.safety_index.env_collision_mask)  # Get the row and column indices of unmasked positions
            # Find the index of the maximum value in the masked array
            phi_env = phi[:self.safety_index.num_constraint_env].reshape(self.safety_index.num_robot_vol, self.safety_index.num_obstacle_vol)
            max_index_flat_env = np.argmax(phi_env[self.safety_index.env_collision_mask])
            
            s_matrix = np.zeros((self.safety_index.num_dof, self.safety_index.num_state))
            s_matrix[:, -self.safety_index.num_dof:] = np.eye(self.safety_index.num_dof)

            # Map the flat index back to row and column indices in the original array 
            max_row_col_env = tuple(masked_indices_phi[max_index_flat_env])
            max_phi_idx = np.ravel_multi_index(max_row_col_env, phi_env.shape)
            
            dof_pos = self.robot_cfg.decompose_state_to_dof_pos(x)
            jacobian_full = self.safety_index.jacobian_full(dof_pos)
            
            critical_jacobian = jacobian_full[max_phi_idx, :]
            Cartesian_Lg = np.einsum('ij,ijk->ik', Lg, np.linalg.pinv(jacobian_full @ s_matrix @ self.robot_cfg.dynamics_g(x)))
            
            # Calculate safe control in cartesian space for the critical robot frame
            u_ref_cartesian = critical_jacobian @ s_matrix @ self.robot_cfg.dynamics_g(x) @ u_ref
            u_safe_cartesian = u_ref_cartesian - self.c_pfm_default * Cartesian_Lg[max_phi_idx, :]
            
            # Convert the cartesian safe control back to the configuration space
            u_safe = np.linalg.pinv(critical_jacobian @ s_matrix @ self.robot_cfg.dynamics_g(x)) @ u_safe_cartesian
        else:
            trigger_safe = False
            u_safe = u_ref

        info = {
            "trigger_safe": trigger_safe,
        }

        # append decode info
        info.update(self.safety_index.decode_constraint_info(phi, 'phi_safe'))   
        info.update(self.safety_index.decode_constraint_info(phi0, 'phi0'))   
        info.update(self.safety_index.decode_constraint_info(phi0dot, 'phi0dot'))   

        return u_safe, info
    