import numpy as np
import osqp
from scipy import sparse

from spark_safe.safe_algo.value_based.base.value_based_safe_algo import ValueBasedSafeAlgorithm

class SafeSetAlgorithm(ValueBasedSafeAlgorithm):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        print("Initializing SafeSetAlgorithm")
        # default values
        self.eta_default = kwargs["eta"]
        self.safety_buffer_default = kwargs["safety_buffer"]
        self.slack_weight_default = kwargs["slack_weight"]
        self.control_weight = np.array(kwargs["control_weight"])
        assert self.control_weight.shape == (self.num_control,), f"control_weight shape {self.control_weight.shape} does not match num_control {self.num_control}"
        
    
    def kappa(self, x):
        '''
            k(x) = eta

            phi_dot <= -eta
        '''
        return np.ones((self.safety_index.num_constraint, 1)) * self.eta_default

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
            (phi_mask > 0) & (phi >= 0),
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
    
    def safety_buffer_solver(self,
                  x, u_ref,control_weight,
                  grad_phi, phi, phi_mask,
                  k_fn):
        """
        The safety buffer solver used in safe control
        
        
        
        :param x: current robot state (dof pos, vel, etc.)
        :param u_ref: Reference robot control
        :param grad_phi: First order derivative of phi
        :param phi: current SI values
        :param phi_mask: mask of phi
        :return: Matrix X, S
        """

        u_ref = np.clip(u_ref, self.control_min, self.control_max)
        
        # Construct Lf, Lg
        Lf = sparse.csc_matrix(grad_phi @ self.robot_cfg.dynamics_f(x))
        Lg = sparse.csc_matrix(grad_phi @ self.robot_cfg.dynamics_g(x))
        
        # !this makes motion smooth in safety buffer for a while,
        # !but eventually still triggers safe again, bounce back and repeat
        # clamp positive contribution of u_ref to the largest phi
        active_id = np.where((phi_mask > 0) & (phi >= 0))[0]
        max_phi_index = active_id[np.argmax(phi[active_id])]
        Lg_max_phi = Lg.toarray()[max_phi_index, :].reshape(-1)
        # if np.dot(u_ref, Lg_max_phi) < 0:
        #     u_safe = u_ref
        # else:
            # for id in [max_phi_index]:
            #     Lg_phi = Lg.toarray()[id, :].reshape(-1)
            #     if np.dot(u_ref, Lg_phi) < 0:
            #         print(np.dot(u_ref, Lg_phi) )
            #     u_ref_Lg = np.dot(u_ref, Lg_phi) / (np.linalg.norm(Lg_phi) ** 2)  * Lg_phi
            #     u_ref_residual = u_ref - u_ref_Lg
            #     u_ref = u_ref_residual
            
            # u_ref_Lg = np.dot(u_ref, Lg_max_phi) / (np.linalg.norm(Lg_max_phi) ** 2)  * Lg_max_phi
            # u_ref_residual = u_ref - u_ref_Lg
            # u_safe = u_ref_residual * 0.0
            
            
            
            
            

        # !this still causes jittering
        # # compute phi_dot
        # phi_dot = Lf + Lg @ u_ref.reshape(-1, 1)
        # phi_dot = phi_dot.A.reshape(-1)

        # # if phi_dot is negative for positive and considered phi, use uref
        # active_id = np.where((phi_mask > 0) & (phi >= 0))
        
        # if phi_dot[active_id].min() < 0:
        #     u_safe = u_ref
        # else:
        #     u_safe = u_ref * 0.0

        # !this doesn't solve stuck issue
        active_id = np.where((phi_mask > 0) & (phi >= 0))[0]
        Lg_mask = np.abs(Lg[active_id,:]).sum(axis=0) # pick control that contributes to the safety index
        phi_new = phi + Lg * u_ref * 0.02
        
        if phi_new[self.safety_index.phi_mask>0].max() - phi[self.safety_index.phi_mask>0].max() <= 0:
            u_safe = u_ref
        else:
            # u_safe = u_ref * 0.0
            
            # allow the control that doesn't contribute to the safety index
            u_safe = np.where(Lg_mask > 0, 0, u_ref)

        return u_safe
    
    def trigger_SSA(self, x, task_info, phi_offset = 0):
        
        phi = self.safety_index.phi(x, task_info) 
        phi[:self.safety_index.num_constraint_env] += phi_offset

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
        
        # if "safety_buffer" in safe_control_args:
        #     assert safe_control_args["safety_buffer"].shape == (self.safety_index.num_constraint,), \
        #         f"safety_buffer shape mismatch: got {safe_control_args['safety_buffer'].shape} != ({self.safety_index.num_constraint},)"

        # if "eta" in safe_control_args:
        #     assert safe_control_args["eta"].shape == (self.safety_index.num_constraint,), \
        #         f"eta shape mismatch: got {safe_control_args['eta'].shape} != ({self.safety_index.num_constraint},)"
        #     assert (safe_control_args["eta"] > 0).all(), "eta must be positive"

        # if "slack_weight" in safe_control_args:
        #     assert safe_control_args["slack_weight"].shape == (self.safety_index.num_constraint,), \
        #         f"slack_weight shape mismatch: got {safe_control_args['slack_weight'].shape} != ({self.safety_index.num_constraint},)"

        phi_hold, trigger_hold = self.trigger_SSA(x, task_info,
                                                  phi_offset = safe_control_args.get("safety_buffer", self.safety_buffer_default))
        phi_safe, trigger_safe = self.trigger_SSA(x, task_info)
        slack_vars = np.zeros_like(phi_safe)
        
        if not trigger_hold:
            
            u_safe = u_ref

        elif not trigger_safe:
            
            # u_safe = u_ref * 0.0
            u_safe = self.safety_buffer_solver(
                                            x = x,
                                            u_ref = u_ref,
                                            control_weight = safe_control_args.get("control_weight", self.control_weight),
                                            grad_phi = self.safety_index.grad_phi(x = x, task_info = task_info),
                                            phi = phi_hold,
                                            phi_mask = self.safety_index.phi_mask,
                                            k_fn = self.kappa if 'eta' not in safe_control_args else lambda x: safe_control_args["eta"].reshape(-1, 1)
                                        )
            
        else:

            control_weight = safe_control_args.get("control_weight", self.control_weight)
            # for QP solver
            u_safe, slack_vars = self.qp_solver(
                x = x,
                u_ref = u_ref,
                weight_u = np.where(control_weight > 0, control_weight, np.ones_like(control_weight)*1e8),
                weight_s = safe_control_args.get("slack_weight", self.slack_weight_default * np.ones(self.safety_index.num_constraint)),
                grad_phi = self.safety_index.grad_phi(x = x, task_info = task_info),
                phi = phi_safe,
                phi_mask = self.safety_index.phi_mask,
                k_fn = self.kappa if 'eta' not in safe_control_args else lambda x: safe_control_args["eta"].reshape(-1, 1)
            )

        info = {
            "trigger_hold": trigger_hold,
            "trigger_safe": trigger_safe,
            "phi_hold": phi_hold,
            "phi_safe": phi_safe,
            "slack_vars": slack_vars
        }
        
        # append decode info
        info.update(self.safety_index.decode_constraint_info(phi_hold, 'phi_hold'))
        info.update(self.safety_index.decode_constraint_info(phi_safe, 'phi_safe'))
        info.update(self.safety_index.decode_constraint_info(slack_vars, 'slack_vars'))
        
        return u_safe, info
    