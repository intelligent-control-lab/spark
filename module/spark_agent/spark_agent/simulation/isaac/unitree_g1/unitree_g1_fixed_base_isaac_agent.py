"""Isaac agent for the Unitree G1 fixed-base embodiment."""

from .unitree_g1_isaac_agent import UnitreeG1IsaacAgent


class UnitreeG1FixedBaseIsaacAgent(UnitreeG1IsaacAgent):
    def __new__(cls, robot_cfg, *args, **kwargs):
        if "num_envs" in kwargs:
            from .unitree_g1_isaac_tensor_backend import _UnitreeG1IsaacTensorBackend

            kwargs.setdefault("fixed_base", True)
            return _UnitreeG1IsaacTensorBackend(robot_cfg, *args, **kwargs)
        return super().__new__(cls)

    def __init__(self, robot_cfg, **kwargs):
        kwargs.setdefault("fixed_base", True)
        super().__init__(robot_cfg, **kwargs)
