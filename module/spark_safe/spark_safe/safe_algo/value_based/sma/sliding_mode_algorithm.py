import numpy as np
import osqp
from scipy import sparse

from spark_safe.safe_algo.value_based.base.value_based_safe_algo import ValueBasedSafeAlgorithm

class SlidingModeAlgorithm(ValueBasedSafeAlgorithm):

    def __init__(self, safety_index, **kwargs):
        super().__init__(safety_index, **kwargs)
        print("Initializing SlidingModeAlgorithm")
        # default values
        self.c_2_default = kwargs["c_2"]

    def trigger_SMA(self, x, task_info):
        phi = self.safety_index.phi(x, task_info) 

        if (phi[self.safety_index.phi_mask>0].max() < 0):
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

        if "c_2" in safe_control_args:
            assert safe_control_args["c_2"].shape == (self.safety_index.num_constraint,), \
                f"c_2 shape mismatch: got {safe_control_args['c_2'].shape} != ({self.safety_index.num_constraint},)"
            assert (safe_control_args["c_2"] > 0).all(), "c_2 must be positive"

        phi_safe, trigger_safe = self.trigger_SMA(x, task_info)
        
        if not trigger_safe:
            u_safe = u_ref
        else:
            grad_phi = self.safety_index.grad_phi(x = x, task_info = task_info)
            Lg = sparse.csc_matrix(grad_phi @ self.robot_cfg.dynamics_g(x))
            active_id = np.where((self.safety_index.phi_mask > 0) & (phi_safe >= 0))[0]
            
            max_phi_index = active_id[np.argmax(phi_safe[active_id])]
            Lg_max_phi = Lg.toarray()[max_phi_index, :].reshape(-1)
            u_safe = u_ref - Lg_max_phi * self.c_2_default
            
            
        info = {
            "trigger_safe": trigger_safe,
            "phi_safe": phi_safe
        }

        return u_safe, info
    