# [Pipeline] Safe teleoperation for G1

`python example/run_g1_safe_teleop_sim.py`

Configure the pipeline by modifying the `cfg` object in the python script or modify the config file (`pipeline/spark_pipeline/g1_safe_teleop/g1_safe_teleop_sim_pipeline_config.py`)

There are several use cases described below.

## 1. Safety with teleoperation, both goal and obstacles from AVP via ROS

Three obstalces are defined by the head and two hands of a human (human obstacle).
Teleoperation goals are from a human as well (human teleoperator).
Human obstacle and human teleoperator can be the same person of different people, depending on how ros topics in task config are sent.

## 2. Safety in idle position, obstacle moved via ROS topic
When runnig with ROS, send obstacle positions via
```
rostopic pub /g1_29dof/human_state std_msgs/Float64MultiArray "data: [0.0, 0.35, 0.2, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0]"
```
Safety behavior should be triggered.
The data format is `[radius, x1, y1, z1, x2, y2, z2]` for each obstacle.
Currently, each obstacle is a sphere at `x1y1z1` in robot base frame with fixed radius of 0.05.
Hence, the above safety behavior is triggered by the 1st obstacle at `[0.35, 0.2, 0.0]`.
Tweak `x1y1z1` to see the effect.

## 3. Safety in idle position, obstacle moved via keyboard.
When running without ROS, move debug obstacle (grey) positions via keyboard. Available operations are
```
Arrow keys (up/down/left/right): movement in X-Y plane
E        : move in Z+
Q        : move in Z-
Space    : switch to next obstacle
PAGE_UP  : add obstacle
PAGE_DOWN: remove obstacle
```
Currently controlled obstacle is marked with grey arrow.

![g1_benchmark](../../../docs/img/g1_safe_teleop_sim.gif)

In the above video:
```
Black spheres: robot collision volume
Green spheres: tracking goal for hands
Grey spheres : obstacle
```

The color of robot collision volumes will change if certain constraints are active (yellow: ssa hold state, red: ssa unsafe).
Critical paris of robot collision volumes and obstacles will also be connected with lines with corresponding color when constraints are active.