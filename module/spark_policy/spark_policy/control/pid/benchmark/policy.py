from spark_policy.control.pid.base import BasePIDPolicy
from spark_robot import RobotKinematics, RobotConfig


class BenchmarkPIDPolicy(BasePIDPolicy):
    def __init__(
        self,
        robot_cfg: RobotConfig,
        robot_kinematics: RobotKinematics,
        **kwargs,
    ) -> None:
        super().__init__(robot_cfg, robot_kinematics, **kwargs)
