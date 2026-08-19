# Task

Tasks convert backend feedback into goals, environment descriptions, and
episode state. Common output fields include `goal_teleop`, `obstacle`,
`robot_base_frame`, `done`, and `done_info`.

## Shared goal-task parameters

| Parameter group | Important fields |
|---|---|
| Episode | `max_episode_length`, `fall_height_threshold`, `reset_on_success`, `reset_on_timeout`, `completion_mode` |
| Obstacles | `num_obstacle_task`, `obstacle_range`, `obstacle_size`, `obstacle_keepout`, `obstacle_velocity`, `mode` |
| Arm goals | `arm_goal_enable`, `use_dual_arm`, left/right ranges and initial poses, `arm_goal_reach_done` |
| Base goal | `base_goal_enable`, position/rotation range, initial pose, velocity, `base_goal_reach_done` |
| Randomization | `seed`, `seed_list`, goal/obstacle resampling flags |
| Integration | `dt`, `enable_ros`, `ros_version`, `ros_params` |

`completion_mode` is either `all_enabled_goals` or `any_enabled_goal`. Goal and
obstacle motion modes can be configured independently in `BenchmarkTask`.

## TeleopTask

`TeleopTask` supports Cartesian end-effector goals, upper-body joint targets,
gripper goals, debug obstacles, and ROS 1/ROS 2 topics. Set
`teleop_upper_body_mode` to `cartesian` or the supported joint-target mode;
`cart_traj` supplies a pose trajectory when no external device is used.

### Keyboard teleoperation example

```bash
python example/unitree_g1/run_unitree_g1_teleop.py
```

Click the MuJoCo viewer before using the keyboard.

| Key | Command |
|---|---|
| Arrow up/down | Translate selected object in −X/+X |
| Arrow left/right | Translate selected object in −Y/+Y |
| `E` / `Q` | Translate in +Z / −Z |
| `2` / `3` | Rotate +Z / −Z |
| `4` / `5` | Rotate +Y / −Y |
| `6` / `7` | Rotate +X / −X |
| `O` / `P` / `B` | Select right-hand / left-hand / base goal |
| `Space` | Select the next obstacle |
| `Page Up` / `Page Down` | Add / remove a debug obstacle |
| `N` | Toggle world-frame and selected-object local-frame translation |
| `-` / `+` | Decrease / increase translation step size |
| `V` | Toggle robot collision-volume display |

Example safety interaction: press `Space` to select an obstacle, move it toward
the controlled arm with the arrow and `E`/`Q` keys, press `O` or `P` to select
a hand goal, and move the goal. The viewer shows how the configured policy
responds as the obstacle approaches the robot collision volumes.

## BenchmarkTask

Adds reproducible goal-reaching benchmarks and environment representations.
Important parameters are `robot_keepout`, per-object motion modes,
`environment_representation` (`sphere`, `point_cloud`, or `mesh`),
`points_per_obstacle`, `minimum_points_per_obstacle`, `dynamic_point_count`,
`point_radius`, `object_mesh_path`, and `object_mesh_scale`.

## CartesianTrajectoryTrackingTask

Loads right, left, object, and distribution trajectories from arrays or a
trajectory file. Key parameters include `trajectory_frame`, `step_mode`,
`waypoint_hold_steps`, `initial_hold_steps`, `repeat`, debug rendering,
grasp-target names/radii, and optional object-follow behavior.

## DynamicsTask

Provides a minimal task around `RobotDynamicsModel` with `initial_state`,
`max_steps`, and `seed`. It is intended for controllers and planners that do
not require a full physics scene.

```{toctree}
:maxdepth: 1

task/overview
task/goal
task/teleoperation
task/benchmark
task/cartesian_trajectory
task/dynamics
```
