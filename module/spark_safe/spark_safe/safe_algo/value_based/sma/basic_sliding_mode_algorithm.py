import numpy as np
import osqp
from scipy import sparse

from spark_safe.safe_algo.value_based.base.value_based_safe_algo import ValueBasedSafeAlgorithm

class BasicSlidingModeAlgorithm(ValueBasedSafeAlgorithm):

    def __init__(self, safety_index, **kwargs):
        super().__init__(safety_index, **kwargs)
        print("Initializing BasicSlidingModeAlgorithm")
        # default values
        self.c_sma_default = kwargs["c_sma"]
        
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
            
            active_id = np.where((self.safety_index.phi_mask > 0) & (phi >= 0))[0]
            
            max_phi_index = active_id[np.argmax(phi[active_id])]
            Lg_max_phi = Lg[max_phi_index, :].reshape(-1)
            u_safe = u_ref - self.c_sma_default* Lg_max_phi
            
        else:
            trigger_safe = False
            u_safe = u_ref
     
        for control_id in self.robot_cfg.Control:
            u_safe[control_id] = np.clip(u_safe[control_id], -self.robot_cfg.ControlLimit[control_id], self.robot_cfg.ControlLimit[control_id])

            
        info = {
            "trigger_safe": trigger_safe,
        }
        
        # append decode info
        info.update(self.safety_index.decode_constraint_info(phi, 'phi_safe'))   
        info.update(self.safety_index.decode_constraint_info(phi0, 'phi0'))   
        info.update(self.safety_index.decode_constraint_info(phi0dot, 'phi0dot'))   
        

        return u_safe, info
    