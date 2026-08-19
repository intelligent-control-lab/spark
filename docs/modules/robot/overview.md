# Robot Module Overview

`spark_robot` defines robot state and control coordinates, limits, continuous
dynamics, kinematics, frames, collision volumes, and backend compatibility.
First-order configurations generally use `x=q`; second-order configurations
use position and velocity state. Always use the dimensions reported by the
selected configuration.
