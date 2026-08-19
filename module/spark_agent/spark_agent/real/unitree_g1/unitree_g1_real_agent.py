"""Single public hardware interface for all Unitree G1 control modes."""

from .unitree_g1_real_whole_body_agent import UnitreeG1RealWholeBodyAgent


class UnitreeG1RealAgent(UnitreeG1RealWholeBodyAgent):
    """Unified G1 hardware agent.

    The inherited implementation already contains the high-level locomotion
    interface and the low-level whole-body path.  Keeping one public class
    ensures that only one object owns DDS state and command channels.
    """
