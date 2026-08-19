"""Shared declarative Isaac adapter for AgiBot G1 embodiments."""

from __future__ import annotations

import warnings

from spark_agent.simulation.isaac.isaac_agent import ConfiguredIsaacAgent


def warn_if_custom_simulator_dynamics(robot_cfg, kwargs) -> None:
    """Explain the physical-agent contract for an analytical base model."""

    dynamics_backend = str(kwargs.get("dynamics_backend", "simulator")).lower()
    use_sim_dynamics = kwargs.get("use_sim_dynamics")
    if use_sim_dynamics is not None:
        dynamics_backend = "simulator" if use_sim_dynamics else "model"
    variant = str(getattr(robot_cfg, "dynamics_variant", ""))
    if dynamics_backend == "simulator" and variant not in {
        "single_integrator",
        "double_integrator",
    }:
        message = (
            f"{type(robot_cfg).__name__} selects the custom {variant!r} analytical "
            "dynamics with simulator-owned execution. The shared AgiBot mobile-base "
            "agent will execute its physical planar-drive contract, not reproduce the "
            "custom analytical equations exactly. Use --dynamics-backend model when "
            "the analytical model must own state evolution."
        )
        try:
            import carb
        except ImportError:
            warnings.warn(message, UserWarning, stacklevel=3)
        else:
            carb.log_warn(message)


class AgiBotG1IsaacAgent(ConfiguredIsaacAgent):
    """Select scalar or tensor Isaac execution from AgiBot config metadata."""

    def __new__(cls, robot_cfg, *args, **kwargs):
        tensor_api = bool(kwargs.pop("tensor_api", False))
        if int(kwargs.get("num_envs", 1)) > 1 or tensor_api:
            warn_if_custom_simulator_dynamics(robot_cfg, kwargs)
            from spark_agent.simulation.isaac.configured_tensor_agent import (
                ConfiguredIsaacTensorAgent,
            )

            return ConfiguredIsaacTensorAgent(robot_cfg, *args, **kwargs)
        return super().__new__(cls, robot_cfg, *args, **kwargs)

    def __init__(self, robot_cfg, **kwargs):
        warn_if_custom_simulator_dynamics(robot_cfg, kwargs)
        super().__init__(robot_cfg, **kwargs)
