"""First-order collision constraints over batched signed-distance results."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any


@dataclass
class TensorSafetyConstraints:
    """Linear constraints ``A @ control >= lower`` for every environment."""

    A: Any
    lower: Any
    distance: Any
    active_mask: Any
    source: Any | None = None


class FirstOrderTensorSafetyIndex:
    def __init__(
        self,
        *,
        minimum_distance: float = 0.05,
        activation_distance: float = 0.15,
        alpha: float = 4.0,
        mode: str = "sublevel",
    ) -> None:
        if activation_distance < minimum_distance:
            raise ValueError("activation_distance must be >= minimum_distance")
        self.minimum_distance = float(minimum_distance)
        self.activation_distance = float(activation_distance)
        self.alpha = float(alpha)
        if mode not in {"safe_set", "sublevel"}:
            raise ValueError("mode must be 'safe_set' or 'sublevel'")
        self.mode = mode

    def build(self, query, point_jacobian, *, other_point_jacobian=None):
        """Construct CBF-like first-order inequalities from query normals."""
        import torch

        if point_jacobian.ndim != 4:
            raise ValueError("point_jacobian must have shape [B, C, 3, U]")
        relative_jacobian = point_jacobian
        if other_point_jacobian is not None:
            relative_jacobian = relative_jacobian - other_point_jacobian
        A = torch.einsum("bci,bciu->bcu", query.normal, relative_jacobian)
        if self.mode == "safe_set":
            # SSA requires outward motion once its activation boundary is
            # crossed, matching the original positive eta correction.
            lower = self.alpha * (self.activation_distance - query.distance)
        else:
            # SSS/CBF use an exponential distance-barrier condition.
            lower = -self.alpha * (query.distance - self.minimum_distance)
        active = query.valid_mask & (query.distance < self.activation_distance)
        return TensorSafetyConstraints(
            A=A,
            lower=lower,
            distance=query.distance,
            active_mask=active,
            source=query.source,
        )
