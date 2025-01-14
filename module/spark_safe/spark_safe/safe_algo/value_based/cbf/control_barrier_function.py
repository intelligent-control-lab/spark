import numpy as np
import osqp
from scipy import sparse

from spark_safe.safe_algo.value_based.base.value_based_safe_algo import ValueBasedSafeAlgorithm

class ControlBarrierFunction(ValueBasedSafeAlgorithm):

    def __init__(self, safety_index, **kwargs):
        super().__init__(safety_index, **kwargs)
        print("Initializing ControlBarrierFunction")
        # default values
        self.lambda_cbf_default = kwargs["lambda_cbf"]
        self.slack_weight_default = kwargs["slack_weight"]
        self.control_weight = np.array(kwargs["control_weight"])
        assert self.control_weight.shape == (self.num_control,), f"control_weight shape {self.control_weight.shape} does not match num_control {self.num_control}"
        
    
    def kappa(self, x):
        '''
            k(x) = lambda_cbf * x

            phi_dot <= -lambda_cbf * x
        '''
        return x * self.lambda_cbf_default

    def qp_solver(self,
                  x, u_ref,
                  weight_u, weight_s,
                  grad_phi, phi, phi_mask,
                  k_fn,
                  eps_=1.00e-2, abs_=1.00e-2):
        """
        The qp solver used in safe control
        :param x: current robot state (dof pos, vel, etc.)
        :param weight_u: Weight of decision vars (robot control).
                        Upper-Left corner of P matrix with shape (n,). [wx1, wx2, ... wxn]
        :param weight_s: Weight of slack variables for safety constraints.
                        Lower-Right corner of P matrix with shape (m,). [ws1, ws2, ... wsn]
        :param u_ref: Reference robot control
        :param grad_phi: First order derivative of phi
        :param phi: current SI values
        :param phi_mask: mask of phi
        :return: Matrix X, S
        """
        
        n = weight_u.shape[0]
        m = weight_s.shape[0]

        Qx = sparse.diags(weight_u)
        Qs = sparse.diags(weight_s)

        # Construct P matrix directly as sparse
        Pmat = sparse.block_diag([Qx, Qs]).tocsc()

        # Construct q
        q = -np.concatenate([(Qx.T @ u_ref.reshape(-1, 1)).reshape(-1), np.zeros(m)])

        # Construct Lf, Lg
        Lf = sparse.csc_matrix(grad_phi @ self.robot_cfg.dynamics_f(x))
        Lg = sparse.csc_matrix(grad_phi @ self.robot_cfg.dynamics_g(x))

        # Construct C matrix directly as sparse
        C_upper = sparse.hstack([Lg, -sparse.eye(m)])
        C_lower = sparse.eye(m + n)
        Cmat = sparse.vstack([C_upper, C_lower]).tocsc()

        # Construct l and u
        l = np.concatenate([-np.inf * np.ones(m), self.control_min.flatten(), np.zeros(m)])
        # ssa condition
        val = np.where(
            (phi_mask > 0),
            np.asarray(-Lf - k_fn(phi.reshape(-1, 1))).flatten(),
            np.ones_like(phi_mask) * np.inf
        )
        u = np.concatenate([ val, self.control_max.flatten(), np.inf * np.ones(m)])

        # Setup and solve the problem
        prob = osqp.OSQP()
        prob.setup(Pmat, q, Cmat, l, u, alpha=1.0, eps_abs = abs_, eps_rel = eps_, verbose=False)
        result = prob.solve()

        u_sol = result.x[:n].flatten()
        s_sol = result.x[n:].flatten()
        return u_sol, s_sol

    def safe_control(self,
                     x             : np.ndarray,
                     u_ref         : np.ndarray,
                     agent_feedback: dict,
                     task_info     : dict,
                     action_info   : dict
                     ):
        
        # default values for now. Can adapt according to agent feedback, task info, etc.
        safe_control_args = {}
        
        if "lambda_cbf" in safe_control_args:
            assert safe_control_args["lambda_cbf"].shape == (self.safety_index.num_constraint,), \
                f"lambda_cbf shape mismatch: got {safe_control_args['lambda_cbf'].shape} != ({self.safety_index.num_constraint},)"
            assert (safe_control_args["lambda_cbf"] > 0).all(), "lambda_cbf must be positive"

        if "slack_weight" in safe_control_args:
            assert safe_control_args["slack_weight"].shape == (self.safety_index.num_constraint,), \
                f"slack_weight shape mismatch: got {safe_control_args['slack_weight'].shape} != ({self.safety_index.num_constraint},)"
        
        phi = self.safety_index.phi(x, task_info)
        
        # for QP solver
        u_safe, slack_vars = self.qp_solver(
            x = x,
            u_ref = u_ref,
            weight_u = np.where(self.control_weight > 0, self.control_weight, np.ones_like(self.control_weight)*1e8),
            weight_s = safe_control_args.get("slack_weight", self.slack_weight_default * np.ones(self.safety_index.num_constraint)),
            grad_phi = self.safety_index.grad_phi(x = x, task_info = task_info),
            phi = phi,
            phi_mask = self.safety_index.phi_mask,
            k_fn = self.kappa if 'lambda_cbf' not in safe_control_args else lambda x: safe_control_args["lambda_cbf"].reshape(-1, 1)
        )

        info = {
            "trigger_safe": True,
        }

        return u_safe, info
    