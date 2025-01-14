import numpy as np

from spark_safe.safe_algo.value_based.base.value_based_safe_algo import ValueBasedSafeAlgorithm

class PotentialFieldMethod(ValueBasedSafeAlgorithm):

    def __init__(self, safety_index, **kwargs):
        super().__init__(safety_index, **kwargs)
        print("Initializing PotentialFieldMethod")
        # default values
        self.lambda_default = kwargs["lambda_pfm"]
        
    def compute_frame_jacobian(self, 
                               x        : np.ndarray,
                               frame_id : int,
                               task_info: dict,
                               eps      : float = 1e-4):
        
        def compute_frame(x, frame_id, task_info):
            trans_world2base          = task_info["robot_base_frame"]
            
            # compute all robot frames
            frames = self.robot_kinematics.forward_kinematics(self.robot_cfg.decompose_state_to_dof(x))
            trans_world2base = self.robot_kinematics.update_base_frame(trans_world2base, self.robot_cfg.decompose_state_to_dof(x))

            return trans_world2base @ frames[frame_id, :, :]
        
        jacobian = np.zeros((3, x.shape[0]))
        for i in range(x.shape[0]):
            x_forward = x.copy()
            x_forward[i] += eps
            x_backward = x.copy()
            x_backward[i] -= eps
            frame_forward = compute_frame(x_forward, frame_id, task_info)
            frame_backward = compute_frame(x_backward,  frame_id, task_info)
            jacobian[:, i] = (frame_forward[:3,3] - frame_backward[:3,3] ) / (2 * eps)
        return jacobian
    
    def trigger_PFM(self, x, task_info):
        
        phi = self.safety_index.phi(x, task_info)[:self.safety_index.num_collision_vol*task_info["obstacle"]["num"]].reshape(self.safety_index.num_collision_vol, task_info["obstacle"]["num"])
        phi = phi[self.safety_index.env_collision_mask]
        if (phi.max() < 0):
            return phi, False
        else:
            return phi, True
    
    def safe_control(self,
                     x             : np.ndarray,
                     u_ref         : np.ndarray,
                     agent_feedback: dict,
                     task_info     : dict,
                     action_info   : dict
                     ):
        
        # default values for now. Can adapt according to agent feedback, task info, etc.
        safe_control_args = {}

        trans_world2base = task_info["robot_base_frame"]
        if "lambda" in safe_control_args:
            assert safe_control_args["lambda"].shape == (self.safety_index.num_constraint,), \
                f"lambda shape mismatch: got {safe_control_args['lambda'].shape} != ({self.safety_index.num_constraint},)"
            assert (safe_control_args["lambda"] > 0).all(), "lambda must be positive"
        trans_world2base = self.robot_kinematics.update_base_frame(trans_world2base, self.robot_cfg.decompose_state_to_dof(x))

        
        phi_safe, trigger_safe = self.trigger_PFM(x, task_info)
        
        if not trigger_safe:   
            u_safe = u_ref
        else:
            masked_indices_phi = np.argwhere(self.safety_index.env_collision_mask)  # Get the row and column indices of unmasked positions
            # Find the index of the minimum value in the masked array
            min_index_flat_env = np.argmax(phi_safe)
            
            # Map the flat index back to row and column indices in the original array 
            self.min_row_col_env = tuple(masked_indices_phi[min_index_flat_env])

            frames = self.robot_kinematics.forward_kinematics(self.robot_cfg.decompose_state_to_dof(x))
            robot_frame_closest = frames[self.min_row_col_env[0]]
            phi_cartesian_closest = self.safety_index.phi_cartesian(robot_frame_closest, task_info,trans_world2base)
            grad_phi_cartesian_closest = self.safety_index.grad_phi_cartesian(robot_frame_closest, task_info,trans_world2base)
            jacobian = self.compute_frame_jacobian(x, self.min_row_col_env[0], task_info)
            u_ref_cartesian_closest = jacobian @ self.robot_cfg.dynamics_g(x) @ u_ref
            u_safe_cartesian_closest = u_ref_cartesian_closest - self.lambda_default * grad_phi_cartesian_closest[np.argmax(np.array(phi_cartesian_closest))]

            u_safe = np.linalg.pinv(jacobian @ self.robot_cfg.dynamics_g(x)) @ u_safe_cartesian_closest

        info = {
            "trigger_safe": trigger_safe,
            "phi_safe": phi_safe
        }


        return u_safe, info
    