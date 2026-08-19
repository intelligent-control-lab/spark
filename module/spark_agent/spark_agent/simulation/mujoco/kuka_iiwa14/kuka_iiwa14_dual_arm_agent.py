"""Dual-arm KUKA iiwa 14 MuJoCo adapter."""

from .kuka_iiwa14_base_agent import _KukaIIWA14AgentBase


class KukaIIWA14DualArmAgent(_KukaIIWA14AgentBase):
    """Run the paired iiwa 14 articulation and both Robotiq grippers."""
