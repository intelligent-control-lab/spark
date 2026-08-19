# Unitree G1

SPARK includes G1 configurations for right-arm, dual-arm, fixed-base,
mobile-base, and whole-body workflows, with optional articulated hands.

## Teleoperation simulation

Start the unified teleoperation example:

```bash
python example/unitree_g1/run_unitree_g1_teleop.py
```

Use `--help` to inspect backend, policy, viewer, and logging options supported
by the current checkout:

```bash
python example/unitree_g1/run_unitree_g1_teleop.py --help
```

## Benchmark

Run the benchmark entry point with:

```bash
python example/unitree_g1/run_unitree_g1_benchmark.py --help
```

Begin with a short, headless simulation before increasing the duration or
moving a configuration to physical hardware.

## Whole-body policies

The example directory also contains SONIC and WBT integration entry points.
Their external models and runtime dependencies must be installed separately;
consult each command's help and the policy configuration before running it.
