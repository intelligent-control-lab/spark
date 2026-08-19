"""Dual-arm Kinova Gen3 Isaac adapter."""

from .kinova_gen3_isaac_agent import KinovaGen3IsaacAgent


class KinovaGen3DualArmIsaacAgent(KinovaGen3IsaacAgent):
    """Run the composed dual Gen3 articulation."""
