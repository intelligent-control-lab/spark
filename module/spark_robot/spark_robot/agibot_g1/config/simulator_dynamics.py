"""Qualified simulator timing shared by physical AgiBot G1 embodiments."""

from spark_robot.base.backend_capability import SimulatorDynamicsSpec


# Four 5 ms physics steps preserve the toolbox-wide 20 ms control period and
# align AgiBot with the shared humanoid simulation clock. The finer physics
# step improves drive/contact resolution without changing the 50 Hz policy.
AGIBOT_G1_SIMULATOR_DYNAMICS = SimulatorDynamicsSpec(
    physics_dt=0.005,
    control_decimation=4,
)
