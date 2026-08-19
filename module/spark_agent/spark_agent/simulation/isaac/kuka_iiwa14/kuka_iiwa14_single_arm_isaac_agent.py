"""Single-arm KUKA iiwa 14 Isaac adapter."""

from .kuka_iiwa14_isaac_agent import KukaIIWA14IsaacAgent


class KukaIIWA14SingleArmIsaacAgent(KukaIIWA14IsaacAgent):
    """Run one iiwa 14 articulation from RobotConfig metadata."""
