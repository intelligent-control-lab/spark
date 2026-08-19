"""Batched reactive safety filters matching SPARK's PFM/SMA families."""

from __future__ import annotations


class BatchedReactiveSafetyFilter:
    """Modify control along the most critical active constraint gradient."""

    def __init__(self, *, gain: float = 0.5, tolerance: float = 1.0e-4):
        self.gain = float(gain)
        self.tolerance = float(tolerance)

    def filter(self, reference, constraints, *, lower_limit=None, upper_limit=None):
        import torch

        batch = reference.shape[0]
        if constraints.A.shape[1] == 0:
            zeros = reference.new_zeros(batch)
            return reference.clone(), {
                "triggered": torch.zeros(batch, device=reference.device, dtype=torch.bool),
                "active_constraints": torch.zeros(batch, device=reference.device, dtype=torch.long),
                "max_violation": zeros,
                "converged": torch.ones(batch, device=reference.device, dtype=torch.bool),
                "control_delta_norm": zeros,
            }
        criticality = torch.where(
            constraints.active_mask,
            -constraints.distance,
            torch.full_like(constraints.distance, -torch.inf),
        )
        _, index = criticality.max(dim=1)
        batch_ids = torch.arange(batch, device=reference.device)
        direction = constraints.A[batch_ids, index]
        triggered = constraints.active_mask.any(dim=1)
        # PFM/SMA both apply a repulsive correction in the safety-gradient
        # direction. Normalization makes the configured gain independent of
        # Jacobian scale and retains fully independent batch rows.
        correction = (
            self.gain
            * direction
            / torch.linalg.vector_norm(direction, dim=1, keepdim=True).clamp_min(1.0e-8)
        )
        control = reference + torch.where(triggered[:, None], correction, 0.0)
        if lower_limit is not None:
            control = torch.maximum(control, lower_limit)
        if upper_limit is not None:
            control = torch.minimum(control, upper_limit)
        residual = constraints.lower - torch.einsum("bcu,bu->bc", constraints.A, control)
        residual = torch.where(
            constraints.active_mask,
            torch.clamp_min(residual, 0.0),
            torch.zeros_like(residual),
        )
        maximum = residual.max(dim=1).values
        return control, {
            "triggered": triggered,
            "active_constraints": constraints.active_mask.sum(dim=1),
            "max_violation": maximum,
            "converged": maximum <= self.tolerance,
            "control_delta_norm": torch.linalg.vector_norm(control - reference, dim=1),
        }
