# Agent and environment modules

## `spark_agent`

Agents translate the common SPARK command and feedback dictionaries to an
execution backend.

`base`
: Defines agent configuration, reset, step, lifecycle, and feedback contracts.

`simulation.mujoco`
: MuJoCo stepping, viewer interaction, keyboard-controlled debug goals and
  obstacles, cameras, and robot-family adapters. The interactive viewer shows
  the selected robot configuration, dynamics model, dynamics order and
  dimensions, propagation backend, control period, and simulation time in a
  top-left overlay when `--show-simulation-info` is requested. The overlay is
  disabled by default for both simulation backends; custom integrations can
  opt in with `viewer_show_simulation_info=True` in the agent configuration.

`simulation.isaac`
: Isaac Sim application lifecycle, articulation control, tensor feedback,
  rendering, and Unitree G1 adapters.

`simulation.dynamics_model_agent`
: Executes a `RobotDynamicsModel` directly for low-dimensional control and
  planning experiments.

`real`
: Hardware adapters, including Unitree feedback, command transport, and
  physical gripper/whole-body integration.

## `spark_env`

`SparkEnvironment` owns one agent and one task. `reset()` resets both sides and
returns the first `(agent_feedback, task_info)`. `step(action, action_info)`
applies a policy command, advances the backend, updates the task, and returns
the next pair. This keeps policies independent of whether the state came from
MuJoCo, Isaac, a dynamics model, or hardware.
