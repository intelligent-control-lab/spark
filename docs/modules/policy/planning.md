# Planning

The `planning.motion` package provides RRT-Connect joint-space planning with
collision-checked interpolation. `planning.trajectory` provides whole-horizon
trajectory optimization and iLQR.

Trajectory optimization balances pose error, nominal posture, velocity
smoothness, acceleration smoothness, and initial-state anchoring. The current
iLQR policy performs iterative backward/forward optimization for the reduced
AgiBot unicycle model.
