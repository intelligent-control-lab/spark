"""Relative-degree-two collision constraints for batched acceleration control."""

from __future__ import annotations

from .first_order import TensorSafetyConstraints


class SecondOrderTensorSafetyIndex:
    """Build exponential-CBF constraints ``A @ acceleration >= lower``.

    Point velocity is evaluated with the instantaneous control Jacobian. The
    The caller supplies the point-Jacobian rate evaluated along the commanded
    velocity. This mirrors the scalar fixed-base D2 monitor while retaining a
    batched acceleration result for velocity-command locomotion policies.
    """

    def __init__(
        self,
        *,
        minimum_distance: float = 0.05,
        activation_distance: float = 0.15,
        position_gain: float = 16.0,
        velocity_gain: float = 8.0,
        curvature_gain: float = 1.0,
    ) -> None:
        if activation_distance < minimum_distance:
            raise ValueError("activation_distance must be >= minimum_distance")
        self.minimum_distance = float(minimum_distance)
        self.activation_distance = float(activation_distance)
        self.position_gain = float(position_gain)
        self.velocity_gain = float(velocity_gain)
        self.curvature_gain = float(curvature_gain)

    def build(
        self,
        query,
        point_jacobian,
        *,
        control_velocity,
        point_jacobian_dot,
        other_point_jacobian=None,
        other_point_jacobian_dot=None,
    ):
        import torch

        if point_jacobian.ndim != 4:
            raise ValueError("point_jacobian must have shape [B, C, 3, U]")
        if control_velocity.ndim != 2:
            raise ValueError("control_velocity must have shape [B, U]")
        relative_jacobian = point_jacobian
        relative_jacobian_dot = point_jacobian_dot
        if other_point_jacobian is not None:
            relative_jacobian = relative_jacobian - other_point_jacobian
        if other_point_jacobian_dot is not None:
            relative_jacobian_dot = relative_jacobian_dot - other_point_jacobian_dot
        A = torch.einsum("bci,bciu->bcu", query.normal, relative_jacobian)
        relative_velocity = torch.einsum("bciu,bu->bci", relative_jacobian, control_velocity)
        if query.relative_velocity is not None:
            relative_velocity = relative_velocity + query.relative_velocity
        distance_rate = torch.einsum("bci,bci->bc", query.normal, relative_velocity)
        jacobian_rate_acceleration = torch.einsum(
            "bciu,bu->bci", relative_jacobian_dot, control_velocity
        )
        normal_jdot_velocity = torch.einsum("bci,bci->bc", query.normal, jacobian_rate_acceleration)
        # For sphere/point distance, the normal-direction curvature is the
        # tangential relative speed divided by center distance. The exact
        # combined radii are backend-specific, so use the activation-scale
        # lower bound; Jdot remains exact with respect to the supplied
        # Jacobian sequence, while this term stays conservative near contact.
        tangential_velocity = relative_velocity - query.normal * distance_rate[..., None]
        center_distance = torch.clamp(query.distance + self.activation_distance, min=1.0e-4)
        curvature_acceleration = tangential_velocity.square().sum(dim=-1) / center_distance
        lower = (
            -self.position_gain * (query.distance - self.minimum_distance)
            - self.velocity_gain * distance_rate
            - normal_jdot_velocity
            - self.curvature_gain * curvature_acceleration
        )
        active = query.valid_mask & (query.distance < self.activation_distance)
        return TensorSafetyConstraints(
            A=A,
            lower=lower,
            distance=query.distance,
            active_mask=active,
            source=query.source,
        )
