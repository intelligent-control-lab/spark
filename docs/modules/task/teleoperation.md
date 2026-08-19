# Teleoperation Task

`TeleopTask` supports Cartesian end-effector targets, upper-body joint targets,
gripper state, debug obstacles, ROS 1, and ROS 2.

```bash
python example/unitree_g1/run_unitree_g1_teleop.py
```

| Key | Command |
|---|---|
| Arrow up/down | Move selected object in −X/+X |
| Arrow left/right | Move selected object in −Y/+Y |
| `E` / `Q` | Move in +Z / −Z |
| `2` / `3` | Rotate +Z / −Z |
| `4` / `5` | Rotate +Y / −Y |
| `6` / `7` | Rotate +X / −X |
| `O` / `P` / `B` | Select right-hand / left-hand / base goal |
| `[` / `]` | Open / close the selected articulated gripper |
| `Space` | Select next obstacle |
| `Page Up` / `Page Down` | Add / remove obstacle |
| `N` | Toggle world/local translation |
| `-` / `+` | Change translation step |
| `V` | Toggle robot collision-volume display |

Click the viewer first. A useful safety example is to select an obstacle,
move it toward an arm, then select and move the corresponding hand goal.

All robot-family safe-teleoperation entry points use a 0.05 m environment
clearance by default. Collision volumes fade to a faint transparent overlay
beyond 0.30 m, become blue as their closest obstacle approaches the configured
clearance, and turn red only at contact (signed surface clearance at or below
0.001 m). The `V`
toggle changes visualization only; it never removes volumes from collision
checking. Use `--minimum-distance` on the AgiBot, Galaxea, Kinova, KUKA, and
FANUC entry points, or `--min-distance` on Unitree, to override the default.

The gripper shortcut is effective only when the selected backend asset exposes
an articulated gripper. The shared keyboard override remains active until the
task/policy acknowledges the same state, preventing a stale action value from
immediately undoing the key press.
