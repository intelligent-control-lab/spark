"""Qualified simulator timing shared by every R1 Lite embodiment."""

from spark_robot.base.backend_capability import SimulatorDynamicsSpec


# Four 5 ms physics steps retain SPARK's 20 ms control period while improving
# drive/contact resolution and aligning every R1 Lite embodiment to the shared
# 50 Hz simulation clock.
R1_LITE_SIMULATOR_DYNAMICS = SimulatorDynamicsSpec(
    physics_dt=0.005,
    control_decimation=4,
)
