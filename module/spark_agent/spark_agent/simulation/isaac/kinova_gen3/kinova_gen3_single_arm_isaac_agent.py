"""Single-arm Kinova Gen3 Isaac adapter."""

from .kinova_gen3_isaac_agent import KinovaGen3IsaacAgent


class KinovaGen3SingleArmIsaacAgent(KinovaGen3IsaacAgent):
    """Run one Gen3 articulation from RobotConfig metadata."""
