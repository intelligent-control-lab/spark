import numpy as np
import osqp
from scipy import sparse

from spark_safe.safe_algo.value_based.sss.basic_sublevel_safe_set_algorithm import BasicSublevelSafeSetAlgorithm

class RelaxedSublevelSafeSetAlgorithm(BasicSublevelSafeSetAlgorithm):
    '''
        Relaxed-SSS
        
        SSS with multiple phi_dot <= -lambda * phi constraints and slack variables for each constraint
        Slack variables are penalized quadratically in the QP solver with manual weights
    '''

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        print("Initializing RelaxedSublevelSafeSetAlgorithm")
        # default values
        self.lambda_default = kwargs["lambda_sss"]
        self.control_weight = np.array(kwargs["control_weight"])
        self.slack_weight_default = kwargs["slack_weight"]
        self.slack_regularization_order = kwargs.get("slack_regularization_order", 2)

    def qp_solver(self,
                  u_ref, Q_u,
                  Lg, Lf,
                  eps_=1.00e-2, abs_=1.00e-2):
        """
        The qp solver min||u - u_ref|| s.t. Lg * u + Lf <= 0
        
        :param u_ref: Reference robot control.
        :param Q: Quadratic cost matrix (Hessian).
        :param Lg: Gradient of constraints with respect to control input.
        :param Lf: Constraint violation (SSA condition).
        """
        n = u_ref.shape[0]
        m = Lf.shape[0]
        
        # Construct q
        
        Q_u = sparse.csc_matrix(Q_u)
        q_u = -(Q_u.T @ u_ref.reshape(-1, 1)).reshape(-1)
        if self.slack_regularization_order == 1:
            """
            min||u-uref|| + ||s|| 
            s.t. Lg u + Lf <= 0
                         s >= 0
            """
            Q_s = sparse.csc_matrix( np.zeros((m, m)))
            q_s = self.slack_weight_default * np.ones(self.safety_index.num_constraint)
            
        elif self.slack_regularization_order == 2: 
            """
            min||u-uref|| + ||s||^2
            s.t. Lg u + Lf <= 0
                         s >= 0
            """
            Q_s = sparse.diags(self.slack_weight_default * np.ones(self.safety_index.num_constraint))
            q_s = np.zeros(m)
        else:
            raise ValueError(f"slack_regularization_order {self.slack_regularization_order} not supported, must be 1 or 2")

        
        # Construct Q matrix
        Q = sparse.block_diag([Q_u, Q_s]).tocsc()
        # Construct q vector
        q = np.concatenate([q_u, q_s])
        
        # Construct C matrix
        C_upper = sparse.hstack([Lg, -sparse.eye(m)])
        C_lower = sparse.eye(m + n)
        Cmat = sparse.vstack([C_upper, C_lower]).tocsc()

        # Construct l and u
        l = np.concatenate([-np.inf * np.ones(m), self.control_min.flatten(), np.zeros(m)])
        u = np.concatenate([-Lf.reshape(-1), self.control_max.flatten(), np.inf * np.ones(m)])
        
        # Setup and solve the problem
        prob = osqp.OSQP()
        prob.setup(Q, q, Cmat, l, u, alpha=1.0, eps_abs = abs_, eps_rel = eps_, verbose=False)      
        result = prob.solve()
        if result.info.status == 'solved':
            # print("Solution found:", u_sol)
            u_sol = result.x[:n].flatten()
            s_sol = result.x[n:].flatten()
            return u_sol, s_sol
        else:
            print("No Solution")
            violation = Lf + Lg @ u_ref.reshape(-1, 1)
            violation[violation < 0] = 0
            return u_ref, np.asarray(violation).reshape(-1)

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
            violation = np.zeros_like(phi)

        info = {
            "trigger_safe": trigger_safe, 
            "violation": violation,
            "env_collision_mask": self.safety_index.env_collision_mask
        }
        
        # append decode info
        info.update(self.safety_index.decode_constraint_info(phi, 'phi_safe'))   
        info.update(self.safety_index.decode_constraint_info(phi0, 'phi0'))   
        info.update(self.safety_index.decode_constraint_info(phi0dot, 'phi0dot'))
        info.update(self.safety_index.decode_constraint_info(violation, 'violation'))
            
        return u_safe, info
    