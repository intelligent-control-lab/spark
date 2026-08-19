"""MuJoCo agent for the Unitree G1 dual-arm embodiment."""

from spark_robot import RobotConfig

from .unitree_g1_fixed_base_mujoco_agent import UnitreeG1FixedBaseMujocoAgent


class UnitreeG1DualArmMujocoAgent(UnitreeG1FixedBaseMujocoAgent):
    """Fourteen-DoF dual-arm specialization of the fixed-base agent."""

    def __init__(self, robot_cfg: RobotConfig, **kwargs) -> None:
        super().__init__(robot_cfg, **kwargs)
