"""Isaac agent for the Unitree G1 articulated whole-body embodiment."""

from .unitree_g1_isaac_agent import UnitreeG1IsaacAgent


class UnitreeG1WholeBodyIsaacAgent(UnitreeG1IsaacAgent):
    def __new__(cls, robot_cfg, *args, **kwargs):
        # The same public concrete agent selects the tensorized Isaac Lab
        # implementation whenever an environment count is supplied. Standard
        # scalar pipelines omit num_envs and retain their NumPy boundary.
        if "num_envs" in kwargs:
            from .unitree_g1_isaac_tensor_backend import _UnitreeG1IsaacTensorBackend

            return _UnitreeG1IsaacTensorBackend(robot_cfg, *args, **kwargs)
        return super().__new__(cls)

    def __init__(self, robot_cfg, **kwargs):
        kwargs.setdefault("fixed_base", False)
        super().__init__(robot_cfg, **kwargs)
