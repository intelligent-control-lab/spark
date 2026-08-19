# MuJoCo Agent

The MuJoCo backend loads MJCF models, maps configured joints, applies position,
velocity, or effort commands, advances simulation, and returns joint and frame
feedback. It also supplies cameras, debug geometry, rendering, and interactive
keyboard control.

MuJoCo 3.4 or newer can display a top-left SPARK information overlay. Add
`--show-simulation-info` to any released teleoperation or benchmark command to
enable it; the default viewer has no information overlay. The overlay is
refreshed every rendered frame and reports the selected robot
configuration, configured dynamics variant, dynamics order and dimensions,
state-propagation backend, control period, and simulation time. This makes the
important distinction between `MuJoCo physics` and `SPARK model` propagation
visible while a run is active.

`SPARK model (Euler)` means the selected analytical robot model owns state
propagation and Euler is its numerical integrator. It does not describe
MuJoCo's engine integrator. Conversely, `MuJoCo physics` means the physics
engine owns state; any MuJoCo `Euler`, `implicit`, or `implicit-fast` setting
belongs to that simulator backend. See {doc}`../../architecture/dynamics_execution`
for the model, order, backend, integrator, and timing selection tables.

Custom integrations can set `viewer_show_simulation_info=True` on the agent
configuration to opt in without using the CLI. This setting is independent of
MuJoCo's left and right native UI panels.

When a MuJoCo agent is used only to replay a trajectory produced by another
environment, call `agent.set_viewer_simulation_info({...}, replace=True)` to
override the displayed model, propagation method, dimensions, or timing. This
changes viewer metadata only; it does not replace the agent's dynamics model.

## RGB-D camera views

Every released MuJoCo robot agent supports RGB and metric-depth capture. Add
`--enable-camera` to a teleoperation command to open one **SPARK RGB Camera**
window and one **SPARK Depth Camera** window alongside the normal viewer.
Unitree G1 WBT uses its torso-mounted head-camera definition. Other robots use
an overview sensor derived from that robot's backend-neutral viewer pose, so
no robot-specific MuJoCo camera name is required. The resulting arrays are
also returned through `camera_feedback`.

The defaults are 640 by 480 pixels at 30 Hz. Use `--camera-width`,
`--camera-height`, and `--camera-rate-hz` to change them. Add
`--no-camera-display` to retain camera feedback without creating the two
OpenCV windows; this is the appropriate mode for data collection.

For the task-oriented key table, see {doc}`../task/teleoperation`.
