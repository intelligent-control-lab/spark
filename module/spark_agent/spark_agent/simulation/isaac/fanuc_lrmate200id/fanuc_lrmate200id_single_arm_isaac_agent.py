"""Single-arm FANUC LR Mate 200iD Isaac adapter."""

from .fanuc_lrmate200id_isaac_agent import FanucLRMate200iDIsaacAgent


class FanucLRMate200iDSingleArmIsaacAgent(FanucLRMate200iDIsaacAgent):
    """Run a single LR Mate articulation from RobotConfig metadata."""
