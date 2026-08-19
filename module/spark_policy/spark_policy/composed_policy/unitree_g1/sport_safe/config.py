from dataclasses import dataclass

from spark_policy.core import PolicyConfig


@dataclass
class UnitreeG1SportSafePolicyConfig(PolicyConfig):
    """Policy-owned defaults for Sport locomotion plus whole-body safety."""

    physics_dt: float = 0.005
    control_decimation: int = 4
    control_dt: float = 0.02
    use_wbt_motor_gains: bool = False
    max_planar_speed: float = 0.150
    min_planar_speed: float = 0.030
    max_yaw_rate: float = 1.00
    max_acceleration: float = 0.20
    max_yaw_acceleration: float = 2.00
    pre_safe_loco_command_limit: tuple[float, float, float] = (0.150, 0.120, 1.00)
    pre_safe_loco_command_rate_limit: tuple[float, float, float] = (0.010, 0.010, 0.080)
    goal_tracking_type: str = "pid"

    def __post_init__(self):
        period = self.physics_dt * self.control_decimation
        if abs(period - self.control_dt) > 1.0e-9:
            raise ValueError(
                "Sport timing is inconsistent: "
                f"physics_dt*control_decimation={period}, control_dt={self.control_dt}"
            )

    def teleop_overrides(self) -> dict:
        return {
            "dt": self.physics_dt,
            "control_decimation": self.control_decimation,
            "control_dt": self.control_dt,
            "use_wbt_motor_gains": self.use_wbt_motor_gains,
            "max_planar_speed": self.max_planar_speed,
            "min_planar_speed": self.min_planar_speed,
            "max_yaw_rate": self.max_yaw_rate,
            "max_acceleration": self.max_acceleration,
            "max_yaw_acceleration": self.max_yaw_acceleration,
            "pre_safe_loco_command_limit": list(self.pre_safe_loco_command_limit),
            "pre_safe_loco_command_rate_limit": list(self.pre_safe_loco_command_rate_limit),
            "goal_tracking_type": self.goal_tracking_type,
        }
