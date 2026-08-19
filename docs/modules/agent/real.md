# Real-Robot Agent

Real agents translate SPARK commands to hardware transports and convert device
state into the common feedback contract. Deployment must validate control
limits, communication loss behavior, initialization state, gripper behavior,
and emergency stopping on the target system.

For Unitree G1, `--use-real true` selects the hardware agent. The optional
MuJoCo mirror is controlled independently with `--enable-digital-twin true` or
`--enable-digital-twin false`; `--digital-twin-sync-hz` controls presentation
updates. The digital twin never replaces hardware feedback or command
transport. `--backend` selects the primary simulator only when
`--use-real=false`; it does not select the twin. In real mode the current
launcher retains its default `mujoco` routing value, so the argument can be
omitted. Use `--digital-twin-backend` to select the mirror implementation. See
{doc}`../robot/unitree_g1` for complete commands.
