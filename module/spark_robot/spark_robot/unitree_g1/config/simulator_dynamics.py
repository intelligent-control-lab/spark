"""Qualified simulator timing shared by every Unitree G1 embodiment."""

from spark_robot.base.backend_capability import SimulatorDynamicsSpec


# Unitree simulation uses the same 50 Hz command contract as the real agent.
# Four 5 ms physics steps retain the substep resolution required by the learned
# whole-body controllers while giving reduced arm/base agents the same clock.
UNITREE_G1_SIMULATOR_DYNAMICS = SimulatorDynamicsSpec(
    physics_dt=0.005,
    control_decimation=4,
)
