# Other robot embodiments

The repository contains examples and configuration packages for several robot
families, including:

- AgiBot G1
- FANUC LR Mate 200iD
- Galaxea R1 Lite
- Kinova Gen3
- KUKA iiwa 14
- Unitree G1

Each example under `example/<robot>/` is the executable source of truth. Use
the command's `--help` output to confirm options for your checkout.

```bash
python example/kinova_gen3/run_kinova_gen3_teleop.py --help
python example/kuka_iiwa14/run_kuka_iiwa14_teleop.py --help
```

The common simulator interaction contract applies across robot families:

- MuJoCo and Isaac viewers are enabled by default; pass `--headless` to disable
  them.
- Press `V` to hide or show collision volumes. Collision checking remains
  active when the overlay is hidden.
- In MuJoCo, `[` opens and `]` closes the selected articulated gripper. The
  keyboard override remains active until the task/policy acknowledges the same
  state, so a stale action value cannot undo the key press immediately.
- Simulator-owned dynamics integrate commands from measured joint state to
  prevent target windup. Model-owned dynamics continue from analytical command
  state.
- One Isaac environment defaults to CPU; cloned environments default to
  `cuda:0`. An explicit `--device` always takes precedence.

Gripper motion is capability-based. R1 Lite, FANUC, Kinova, KUKA, and supported
Unitree configurations expose articulated grippers in both backends. AgiBot
exposes articulated grippers in MuJoCo, but its Isaac articulation does not
currently declare gripper actuation. FANUC's Isaac Robotiq 3F uses nine driven
finger joints; Kinova and KUKA use their two-finger articulation contracts.
