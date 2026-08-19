"""Fixed-shape batched ADMM safety QP filters."""

from __future__ import annotations


class BatchedQPSafetyFilter:
    """Solve independent hard or relaxed linear safety QPs on one device.

    The objective is a weighted projection of ``reference``.  Relaxed mode
    adds one nonnegative quadratic slack per collision constraint. All matrix
    operations retain a leading environment dimension.
    """

    def __init__(
        self,
        *,
        iterations: int = 40,
        rho: float = 40.0,
        slack_weight: float | None = None,
        tolerance: float = 1.0e-3,
        regularization: float = 1.0e-6,
        warm_start: bool = False,
    ) -> None:
        self.iterations = int(iterations)
        self.rho = float(rho)
        self.slack_weight = None if slack_weight is None else float(slack_weight)
        self.tolerance = float(tolerance)
        self.regularization = float(regularization)
        self.warm_start = bool(warm_start)
        self._warm_state = None

    def reset(self, env_ids=None) -> None:
        """Clear all or selected persistent ADMM state rows."""
        if self._warm_state is None or env_ids is None:
            self._warm_state = None
            return
        import torch

        ids = torch.as_tensor(env_ids, device=self._warm_state["dual"].device, dtype=torch.long)
        for value in self._warm_state.values():
            value[ids] = 0.0

    def filter(
        self,
        reference,
        constraints,
        *,
        lower_limit=None,
        upper_limit=None,
        control_weight=None,
    ):
        import torch

        batch, controls = reference.shape
        count = constraints.A.shape[1]
        if count == 0:
            zeros = reference.new_zeros(batch)
            return reference.clone(), {
                "triggered": torch.zeros(batch, device=reference.device, dtype=torch.bool),
                "active_constraints": torch.zeros(batch, device=reference.device, dtype=torch.long),
                "max_violation": zeros,
                "converged": torch.ones(batch, device=reference.device, dtype=torch.bool),
                "control_delta_norm": zeros,
            }
        mask = constraints.active_mask
        if not bool(mask.any().item()):
            self._warm_state = None
            zeros = reference.new_zeros(batch)
            return reference.clone(), {
                "triggered": torch.zeros(batch, device=reference.device, dtype=torch.bool),
                "active_constraints": torch.zeros(batch, device=reference.device, dtype=torch.long),
                "max_violation": zeros,
                "converged": torch.ones(batch, device=reference.device, dtype=torch.bool),
                "control_delta_norm": zeros,
                "slack": reference.new_zeros(batch, count),
            }
        A = constraints.A * mask[:, :, None]
        lower = torch.where(mask, constraints.lower, torch.full_like(constraints.lower, -1.0e4))
        if control_weight is None:
            weight = torch.ones(controls, device=reference.device, dtype=reference.dtype)
        else:
            weight = torch.as_tensor(
                control_weight, device=reference.device, dtype=reference.dtype
            ).reshape(controls)
        eye = torch.eye(controls, device=reference.device, dtype=reference.dtype)
        hessian = torch.diag(weight)[None] + self.rho * (A.mT @ A)
        hessian = hessian + self.regularization * eye[None]
        # The Hessian is symmetric positive definite and remains unchanged
        # throughout ADMM. Factor it once for the whole environment batch;
        # calling ``torch.linalg.solve`` in every iteration redundantly
        # repeated an LU factorization 40 times.
        cholesky = torch.linalg.cholesky(hessian)
        weighted_reference = weight[None] * reference
        Au = torch.einsum("bcu,bu->bc", A, reference)
        warm = self._warm_state
        use_warm = bool(
            self.warm_start
            and warm is not None
            and warm["dual"].shape == Au.shape
            and warm["dual"].device == Au.device
            and warm["dual"].dtype == Au.dtype
        )
        if use_warm:
            slack = warm["slack"].clone() * mask
            dual = warm["dual"].clone() * mask
            z = torch.maximum(lower, Au + slack + dual)
        else:
            slack = torch.zeros_like(Au)
            z = torch.maximum(lower, Au)
            dual = torch.zeros_like(Au)
        for _ in range(self.iterations):
            rhs = weighted_reference + self.rho * torch.einsum("buc,bc->bu", A.mT, z - slack - dual)
            control = torch.cholesky_solve(rhs.unsqueeze(-1), cholesky).squeeze(-1)
            if lower_limit is not None:
                control = torch.maximum(control, lower_limit)
            if upper_limit is not None:
                control = torch.minimum(control, upper_limit)
            Au = torch.einsum("bcu,bu->bc", A, control)
            if self.slack_weight is not None:
                slack = torch.clamp_min(
                    self.rho * (z - dual - Au) / (self.slack_weight + self.rho),
                    0.0,
                )
                slack = slack * mask
            z = torch.maximum(lower, Au + slack + dual)
            dual = (dual + Au + slack - z) * mask

        residual = torch.where(
            mask,
            torch.clamp_min(lower - Au - slack, 0.0),
            torch.zeros_like(lower),
        )
        max_violation = residual.max(dim=1).values
        delta = torch.linalg.vector_norm(control - reference, dim=1)
        if self.warm_start:
            self._warm_state = {
                "dual": dual.detach().clone(),
                "slack": slack.detach().clone(),
            }
        return control, {
            "triggered": delta > self.tolerance,
            "active_constraints": mask.sum(dim=1),
            "max_violation": max_violation,
            "converged": max_violation <= self.tolerance,
            "control_delta_norm": delta,
            "slack": slack,
        }


class BatchedRelaxedQPSafetyFilter(BatchedQPSafetyFilter):
    def __init__(self, *, slack_weight: float = 1000.0, **kwargs):
        super().__init__(slack_weight=slack_weight, **kwargs)
