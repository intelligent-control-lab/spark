"""Isaac agent for the Unitree G1 abstract mobile-base embodiment."""

from .unitree_g1_isaac_agent import UnitreeG1IsaacAgent


class UnitreeG1MobileBaseIsaacAgent(UnitreeG1IsaacAgent):
    def __new__(cls, robot_cfg, *args, **kwargs):
        if "num_envs" in kwargs:
            from .unitree_g1_isaac_tensor_backend import _UnitreeG1IsaacTensorBackend

            return _UnitreeG1IsaacTensorBackend(robot_cfg, *args, **kwargs)
        return super().__new__(cls)

    def __init__(self, robot_cfg, **kwargs):
        kwargs.setdefault("fixed_base", False)
        super().__init__(robot_cfg, **kwargs)
