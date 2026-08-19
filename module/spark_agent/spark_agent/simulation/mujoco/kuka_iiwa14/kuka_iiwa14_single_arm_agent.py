"""Single-arm KUKA iiwa 14 MuJoCo adapter."""

from .kuka_iiwa14_base_agent import _KukaIIWA14AgentBase


class KukaIIWA14SingleArmAgent(_KukaIIWA14AgentBase):
    """Run one iiwa 14 with its articulated Robotiq 2F-85."""
