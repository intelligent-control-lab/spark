"""Dual-arm FANUC LR Mate 200iD Isaac adapter."""

from .fanuc_lrmate200id_isaac_agent import FanucLRMate200iDIsaacAgent


class FanucLRMate200iDDualArmIsaacAgent(FanucLRMate200iDIsaacAgent):
    """Run the composed dual LR Mate articulation."""
