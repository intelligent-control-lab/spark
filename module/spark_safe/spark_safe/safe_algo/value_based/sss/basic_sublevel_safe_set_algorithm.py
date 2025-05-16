import numpy as np
import osqp
from scipy import sparse

from spark_safe.safe_algo.value_based.base.value_based_safe_algo import ValueBasedSafeAlgorithm

class BasicSublevelSafeSetAlgorithm(ValueBasedSafeAlgorithm):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        print("Initializing BasicSublevelSafeSetAlgorithm")
        # default values
        self.lambda_default = kwargs["lambda_sss"]
        self.control_weight = np.array(kwargs["control_weight"])
        assert self.control_weight.shape == (self.num_control,), f"control_weight shape {self.control_weight.shape} does not match num_control {self.num_control}"
        
    def lambda_fn(self, phi):
        lambda_values = np.where(
                        (self.safety_index.phi_mask.reshape(-1,1) > 0) & (phi.reshape(-1,1) >= 0),
                        np.ones((self.safety_index.num_constraint, 1)) * self.lambda_default * phi.reshape(-1, 1),
                        np.ones((self.safety_index.num_constraint, 1)) * -np.inf
                    )
        return lambda_values.reshape(-1, 1)

    def qp_solver(self,
                  u_ref, Q_u,
                  Lg, Lf,
                  eps_=1.00e-2, abs_=1.00e-2):
        """
        The qp solver min||u - u_ref|| s.t. Lg * u + Lf <= 0
        
        :param u_ref: Reference robot control.
        :param Q: Quadratic cost matrix (Hessian).
        :param Lg: Gradient of constraints with respect to control input.
        :param Lf: Constraint violation (SSS condition).
        """
        n = u_ref.shape[0]
        m = Lf.shape[0]
        
        # Construct q
        
        Q = sparse.csc_matrix(Q_u)
        q = -(Q_u.T @ u_ref.reshape(-1, 1)).reshape(-1)
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
            violation = np.zeros(m)
            return u_sol, violation
        else:
            violation = Lf + Lg @ u_ref.reshape(-1, 1)
            violation = violation.reshape(-1)
            violation[violation < 0] = 0
            return u_ref, violation

    def safe_control(self,
                     x             : np.ndarray,
                     u_ref         : np.ndarray,
                     agent_feedback: dict,
                     task_info     : dict,
                     action_info   : dict
                     ):
        
        phi, Lg, Lf, phi0, phi0dot = self.safety_index.phi(x, task_info)
        
        Q_u = np.diag(self.control_weight)
        if (phi[self.safety_index.phi_mask>0].max() > 0):
            trigger_safe = True
            # for QP solver
            u_safe, violation = self.qp_solver(
                u_ref = u_ref,
                Q_u = Q_u,
                Lg = Lg,
                Lf = Lf.reshape(-1,1) + self.lambda_fn(phi).reshape(-1,1)
            )
        else:
            trigger_safe = False
            u_safe = u_ref
            violation = np.zeros(Lf.shape[0])

        info = {
            "trigger_safe": trigger_safe,
        }
        
        # append decode info
        info.update(self.safety_index.decode_constraint_info(phi, 'phi_safe'))   
        info.update(self.safety_index.decode_constraint_info(phi0, 'phi0'))   
        info.update(self.safety_index.decode_constraint_info(phi0dot, 'phi0dot'))
        info.update(self.safety_index.decode_constraint_info(violation, 'violation'))
            
        return u_safe, info
    