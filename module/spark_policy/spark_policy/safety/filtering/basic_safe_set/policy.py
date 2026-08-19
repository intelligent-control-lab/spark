import numpy as np
import osqp
from scipy import sparse

from spark_policy.safety.filtering.value_based import ValueBasedSafeAlgorithm


class BasicSafeSetAlgorithm(ValueBasedSafeAlgorithm):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        # Derived filters print their own concrete initialization once.
        if self.__class__ is BasicSafeSetAlgorithm:
            print(f"Initializing {self.__class__.__name__}")
        # default values
        eta_ssa = kwargs.get("eta_ssa", 0.1)
        self.eta_default = float(0.1 if eta_ssa is None else eta_ssa)
        self.control_weight = np.array(kwargs["control_weight"])
        assert self.control_weight.shape == (self.num_control,), (
            f"control_weight shape {self.control_weight.shape} does not match num_control {self.num_control}"
        )

    def eta_fn(self, phi):
        eta_values = np.where(
            (self.safety_index.phi_mask.reshape(-1, 1) > 0) & (phi.reshape(-1, 1) >= 0),
            np.ones((self.safety_index.num_constraint, 1)) * self.eta_default,
            np.ones((self.safety_index.num_constraint, 1)) * -np.inf,
        )
        return eta_values.reshape(-1, 1)

    def _base_control_indices(self):
        names = ("vLinearX", "vLinearY", "vRotYaw", "vLinearZ")
        controls = self.robot_cfg.Control.__members__
        return [int(controls[name]) for name in names if name in controls]

    def _base_safe_control_info(self, u_ref, u_safe, Lg, phi):
        base_indices = self._base_control_indices()
        if not base_indices:
            return {}

        Lg = np.asarray(Lg, dtype=float)
        base_Lg = Lg[:, base_indices]
        active_mask = np.asarray(self.safety_index.phi_mask, dtype=bool).reshape(-1)
        if phi is not None:
            active_mask &= np.asarray(phi, dtype=float).reshape(-1) >= 0.0
        active_base_Lg = (
            base_Lg[active_mask] if np.any(active_mask) else np.zeros((0, len(base_indices)))
        )

        u_ref = np.asarray(u_ref, dtype=float).reshape(-1)
        u_safe = np.asarray(u_safe, dtype=float).reshape(-1)
        base_ref = u_ref[base_indices]
        base_safe = u_safe[base_indices]

        return {
            "safe_base_control_indices": np.asarray(base_indices, dtype=int),
            "safe_base_u_ref": base_ref,
            "safe_base_u_safe": base_safe,
            "safe_base_u_delta": base_safe - base_ref,
            "safe_base_motion_cmd": base_safe,
            "safe_base_motion_cmd_norm": float(np.linalg.norm(base_safe)),
            "safe_base_lg_max_abs": float(np.max(np.abs(base_Lg))) if base_Lg.size else 0.0,
            "safe_active_base_lg_max_abs": (
                float(np.max(np.abs(active_base_Lg))) if active_base_Lg.size else 0.0
            ),
        }

    def qp_solver(self, u_ref, Q_u, Lg, Lf, eps_=1.00e-2, abs_=1.00e-2):
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
        prob.setup(Q, q, Cmat, l, u, alpha=1.0, eps_abs=abs_, eps_rel=eps_, verbose=False)
        result = prob.solve()
        if result.info.status == "solved":
            u_sol = result.x[:n].flatten()
            violation = np.zeros(m)
            return u_sol, violation
        else:
            violation = Lf + Lg @ u_ref.reshape(-1, 1)
            violation = violation.reshape(-1)
            violation[violation < 0] = 0
            return u_ref, violation

    def safe_control(
        self,
        x: np.ndarray,
        u_ref: np.ndarray,
        agent_feedback: dict,
        task_info: dict,
        action_info: dict,
    ):

        phi, Lg, Lf, phi0, phi0dot = self.safety_index.phi(x, task_info)
        # dof_pos = self.robot_cfg.decompose_state_to_dof_pos(x)
        # self.robot_kinematics.pre_computation(dof_pos)
        # Use this for cartesian tracking of hands
        # jacobian_left = self.robot_kinematics.get_jacobian(self.robot_cfg.Frames.L_ee)[:3, :]
        # jacobian_right = self.robot_kinematics.get_jacobian(self.robot_cfg.Frames.R_ee)[:3, :]
        # jacobian_full = np.vstack([jacobian_left, jacobian_right])
        # Q_u = jacobian_full.T @ jacobian_full

        Q_u = np.diag(self.control_weight)
        if self.has_active_violation(phi):
            trigger_safe = True
            # for QP solver
            u_safe, violation = self.qp_solver(
                u_ref=u_ref, Q_u=Q_u, Lg=Lg, Lf=Lf.reshape(-1, 1) + self.eta_fn(phi).reshape(-1, 1)
            )
        else:
            trigger_safe = False
            u_safe = u_ref
            violation = np.zeros(Lf.shape[0])

        info = {
            "trigger_safe": trigger_safe,
        }
        info.update(self._base_safe_control_info(u_ref, u_safe, Lg, phi))

        # append decode info
        info.update(self.safety_index.decode_constraint_info(phi, "phi_safe"))
        info.update(self.safety_index.decode_constraint_info(phi0, "phi0"))
        info.update(self.safety_index.decode_constraint_info(phi0dot, "phi0dot"))
        info.update(self.safety_index.decode_constraint_info(violation, "violation"))

        return u_safe, info
