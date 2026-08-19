"""Shared declarative Isaac adapter for Kinova Gen3 embodiments."""

from spark_agent.simulation.isaac.isaac_agent import ConfiguredIsaacAgent


class KinovaGen3IsaacAgent(ConfiguredIsaacAgent):
    """Family extension point for Kinova-specific Isaac behavior."""
