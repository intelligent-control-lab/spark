import numpy as np
import osqp
from scipy import sparse

from spark_safe.safe_algo.value_based.base.value_based_safe_algo import ValueBasedSafeAlgorithm

class MySafeControlAlgorithm(ValueBasedSafeAlgorithm):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        print("Initializing MySafeControlAlgorithm")
        # default values
        self.eta_default = kwargs["eta"]
        self.control_weight = np.array(kwargs["control_weight"])
        
    def eta(self, phi): # This is a place holder
        return np.zeros_like(phi)

    def qp_solver(self,
                  phi, phi_mask,
                  u_ref, Q,
                  Lg, Lf,
                  eps_=1.00e-2, abs_=1.00e-2):
        """
        The qp solver min||u-uref|| s.t. Lg u + Lf <=0
        
        :param u_ref: Reference robot control.
        :param Q: Quadratic cost matrix (Hessian).
        :param Lg: Gradient of constraints with respect to control input.
        :param Lf: Constraint violation (SSA condition).
        """
        n = u_ref.shape[0]
        m = Lf.shape[0]
        
        Q = sparse.diags(self.control_weight)
        # Construct q
        q = -(Q.T @ u_ref.reshape(-1, 1)).reshape(-1)
        Q = sparse.csc_matrix(Q)
        
        C_upper = sparse.csc_matrix(Lg)
        C_lower = sparse.eye(n)
        Cmat = sparse.vstack([C_upper, C_lower]).tocsc()

        # Construct l and u
        l = np.concatenate([-np.inf * np.ones(m), self.control_min.flatten()])
        u = np.concatenate([-Lf.reshape(-1), self.control_max.flatten()])
        
        # Setup and solve the problem
        prob = osqp.OSQP()
        prob.setup(Q, q, Cmat, l, u, alpha=1.0, eps_abs = abs_, eps_rel = eps_, verbose=False)      
        result = prob.solve()
        if result.info.status == 'solved':
            u_sol = result.x[:n].flatten()
            print("Solution found:", u_sol)
            return u_sol
        else:
            print("No Solution")
            return u_ref

    def safe_control(self,
                     x             : np.ndarray,
                     u_ref         : np.ndarray,
                     agent_feedback: dict,
                     task_info     : dict,
                     action_info   : dict
                     ):
        
        safe_control_args = {}
        phi, Lg, Lf = self.safety_index.phi(x, task_info)

        if (phi[self.safety_index.phi_mask>0].max() > 0):
            trigger_safe = True
            # for QP solver
            u_safe = self.qp_solver(
                phi=phi,
                phi_mask=self.safety_index.phi_mask,
                u_ref = u_ref,
                Q = np.diag(self.control_weight),
                Lg = Lg,
                Lf = Lf.reshape(-1,1) + self.eta(phi)
            )
        else:
            trigger_safe = False
            u_safe = u_ref

        info = {
            "trigger_safe": True, # error when assigning trigger_safe
            "phi_safe": phi,
        }
        
        # append decode info
        info.update(self.safety_index.decode_constraint_info(phi, 'phi_safe'))        
        return u_safe, info
    