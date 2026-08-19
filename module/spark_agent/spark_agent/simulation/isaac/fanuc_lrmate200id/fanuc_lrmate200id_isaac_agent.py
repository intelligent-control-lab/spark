"""Shared declarative Isaac adapter for FANUC LR Mate 200iD embodiments."""

from spark_agent.simulation.isaac.isaac_agent import ConfiguredIsaacAgent


class FanucLRMate200iDIsaacAgent(ConfiguredIsaacAgent):
    """FANUC adapter with contact-stable articulation solver defaults."""

    def __init__(self, robot_cfg, *args, **kwargs):
        # The articulated three-finger hand creates more simultaneous contact
        # constraints than a bare arm.  Four/default position iterations can
        # show visible penetration before the solver converges.
        kwargs.setdefault("solver_position_iteration_count", 8)
        kwargs.setdefault("solver_velocity_iteration_count", 4)
        kwargs.setdefault("sim_position_error_limit", 0.025)
        super().__init__(robot_cfg, *args, **kwargs)
