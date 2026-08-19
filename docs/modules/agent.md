# Agent

`spark_agent` translates the common SPARK action and feedback contracts to
simulation and physical-robot backends.

## Submodules

`simulation.mujoco`
: MuJoCo stepping, visualization, keyboard interaction, cameras, and
  embodiment-specific adapters.

`simulation.isaac`
: Isaac Sim lifecycle, articulation control, tensor feedback, rendering, and
  Unitree G1 adapters.

`simulation.dynamics_model_agent`
: Direct execution of a `RobotDynamicsModel` for low-dimensional controller
  and planning experiments.

`real`
: Hardware communication and command adapters for supported robots.

## Contract

An agent owns backend state and timing. It resets the backend, accepts actions,
advances execution, and returns feedback including joint state and robot-frame
information. Task logic and policy selection remain outside the agent.

```{toctree}
:maxdepth: 1

agent/overview
agent/mujoco
agent/isaac
agent/real
agent/dynamics_model
```
