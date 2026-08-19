from spark_agent.simulation.mujoco.agibot_g1.agibot_g1_fixed_base_agent import (
    AgiBotG1FixedBaseAgent,
)
from spark_robot import RobotConfig


class AgiBotG1DualArmAgent(AgiBotG1FixedBaseAgent):
    """AgiBot G1 fixed-base MuJoCo agent exposing both arms with torso joints locked."""

    def __init__(self, robot_cfg: RobotConfig, **kwargs) -> None:
        super().__init__(robot_cfg, **kwargs)
