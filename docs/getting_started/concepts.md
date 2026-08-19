# Core concepts

SPARK separates a robotics application into components that can be combined
without rewriting the entire execution loop.

Robot
: Describes kinematics, dynamics, joint conventions, and embodiment-specific
  configuration.

Agent
: Connects SPARK to a simulator or a physical robot and exchanges observations
  and commands.

Task
: Defines goals, task state, observations, and termination conditions.

Policy
: Maps the current state and goal into an action. Control and safety policies
  can be composed.

Pipeline
: Wires the robot, agent, task, and policies together and owns the runtime
  lifecycle.

The {doc}`../architecture/overview` page shows how these pieces interact.
