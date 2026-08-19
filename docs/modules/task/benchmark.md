# Benchmark Task

`BenchmarkTask` adds reproducible goal-reaching evaluation, independent motion
modes for obstacles and goals, and sphere, point-cloud, or mesh environment
representations. Important parameters include point density, dynamic point
counts, point radius, mesh path/scale, completion mode, and deterministic seeds.

Canonical test cases store robot-independent goal ranges. A robot-specific
benchmark entry point may copy and translate those ranges while building its
pipeline configuration. Such presentation/task grounding stays in the runner,
not in `RobotConfig` or the shared test-case registry.
