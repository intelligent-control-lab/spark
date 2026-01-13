from .base_safe_algo import BaseSafeAlgorithm
import numpy as np

class ByPassSafeControl(BaseSafeAlgorithm):
    
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
    
    def safe_control(self,
                     x             : np.ndarray,
                     u_ref         : np.ndarray,
                     agent_feedback: dict,
                     task_info     : dict,
                     action_info   : dict
                     ):
        
        info = {}
        
        return u_ref, info