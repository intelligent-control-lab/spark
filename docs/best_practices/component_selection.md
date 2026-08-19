# Component Selection

Choose the smallest robot configuration that contains the required task
coordinates. Select an agent that supports that configuration, then select a
task and policy with matching state, goal, and action contracts.

Validate dimension compatibility at construction time and prefer explicit
composed policies when one policy consumes another policy's output.
