"""Order-1 unicycle model for the physical AgiBot G1 mobile embodiment.

``Dynamic1`` means the complete standard robot configuration is first order:
positions are state and velocity-like quantities are controls.  The inherited
upper-body and simulator contracts remain those of
``AgiBotG1MobileBaseDynamic1Config``.  The default analytical view is reduced
to the planar state ``(x, y, yaw)`` and controls ``(forward speed, yaw rate)``;
lateral base speed is omitted, which produces the nonholonomic kinematic
unicycle equations.  A policy may request that reduced view without defining a
different physical robot.

This class does not implement a second-order/dynamic unicycle.  Such a model
would require explicit velocity states and acceleration, wheel-force, or
wheel-torque inputs and should be introduced as a separate ``Dynamic2``
configuration.
"""

from .agibot_g1_mobile_base_dynamic_1_config import AgiBotG1MobileBaseDynamic1Config


class AgiBotG1MobileBaseUnicycleDynamic1Config(AgiBotG1MobileBaseDynamic1Config):
    """First-order mobile AgiBot with kinematic-unicycle base equations."""

    dynamics_variant = "unicycle"
    dynamics_order = 1
    dynamics_is_linear = False
    default_dynamics_state_dof_names = ("LinearX", "LinearY", "RotYaw")
    default_dynamics_control_names = ("vLinearX", "vRotYaw")
