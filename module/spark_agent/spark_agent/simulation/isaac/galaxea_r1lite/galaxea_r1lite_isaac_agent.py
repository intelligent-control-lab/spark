"""Shared declarative Isaac adapter for Galaxea R1 Lite embodiments."""

from spark_agent.simulation.isaac.isaac_agent import ConfiguredIsaacAgent


class GalaxeaR1LiteIsaacAgent(ConfiguredIsaacAgent):
    """Family extension point for R1 Lite-specific Isaac behavior."""
