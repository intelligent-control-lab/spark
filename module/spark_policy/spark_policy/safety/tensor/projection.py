"""Fixed-shape batched projection filter for small real-time safety QPs."""

from __future__ import annotations


class BatchedProjectionSafetyFilter:
    """Project controls onto active halfspaces with batched Hildreth updates.

    The solver selects the most violated inequality in every environment per
    iteration.  This keeps work independent across environments and avoids a
    Python loop over collision constraints.  Residuals remain observable so a
    later optimized QP backend can use the same result contract.
    """

    def __init__(self, *, iterations: int = 24, tolerance: float = 1.0e-4):
        self.iterations = int(iterations)
        self.tolerance = float(tolerance)

    def filter(self, reference, constraints, *, lower_limit=None, upper_limit=None):
        import torch

        control = reference.clone()
        batch = reference.shape[0]
        if constraints.A.shape[0] != batch:
            raise ValueError("constraint and reference batch sizes must match")
        if constraints.A.shape[1] == 0:
            return control, self._result(reference, control, constraints)
        batch_ids = torch.arange(batch, device=reference.device)
        for _ in range(self.iterations):
            violation = constraints.lower - torch.einsum("bcu,bu->bc", constraints.A, control)
            violation = torch.where(
                constraints.active_mask,
                violation,
                torch.full_like(violation, -torch.inf),
            )
            worst_value, worst_id = violation.max(dim=1)
            direction = constraints.A[batch_ids, worst_id]
            scale = torch.clamp_min(worst_value, 0.0) / direction.square().sum(dim=1).clamp_min(
                1.0e-9
            )
            control = control + scale[:, None] * direction
            if lower_limit is not None:
                control = torch.maximum(control, lower_limit)
            if upper_limit is not None:
                control = torch.minimum(control, upper_limit)
        return control, self._result(reference, control, constraints)

    def _result(self, reference, control, constraints):
        import torch

        if constraints.A.shape[1] == 0:
            residual = reference.new_zeros((reference.shape[0], 0))
        else:
            residual = constraints.lower - torch.einsum("bcu,bu->bc", constraints.A, control)
        positive = torch.where(
            constraints.active_mask, torch.clamp_min(residual, 0.0), torch.zeros_like(residual)
        )
        max_violation = (
            positive.max(dim=1).values
            if positive.shape[1]
            else reference.new_zeros(reference.shape[0])
        )
        triggered = (control - reference).abs().amax(dim=1) > self.tolerance
        return {
            "triggered": triggered,
            "active_constraints": constraints.active_mask.sum(dim=1),
            "max_violation": max_violation,
            "converged": max_violation <= self.tolerance,
            "control_delta_norm": torch.linalg.vector_norm(control - reference, dim=1),
        }
