"""Dual-arm KUKA iiwa 14 Isaac adapter."""

from .kuka_iiwa14_isaac_agent import KukaIIWA14IsaacAgent


class KukaIIWA14DualArmIsaacAgent(KukaIIWA14IsaacAgent):
    """Run the composed dual iiwa 14 articulation."""
