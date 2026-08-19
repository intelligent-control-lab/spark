# Benchmark Pipeline

Benchmark pipelines select reproducible goals and environments, execute until
completion or timeout, and report goal and safety metrics. Robot-family
configurations provide the appropriate embodiment, agent, and policy choices.

The Unitree G1 benchmark uses four independent selectors: robot configuration,
policy configuration, self-contained task case, and simulator backend. Task
levels are `v0` (no obstacles), `v1` (sparse), and `v2` (dense). The catalog
also distinguishes static and dynamic goals from static and dynamic obstacles.

MuJoCo WBT uses isolated processes for parallel execution. Isaac WBT, Sport,
and SONIC always use the same tensor runner, CUDA device, policy adapter,
50 Hz control loop, arrival logic, and safety implementation for both one and
many environments. This makes a one-environment benchmark the reference for
scaling the same configuration to `--num-envs N`.

Benchmark defaults are 1,000 control steps, at most 10 scalar resets, a
1,000-step episode horizon, and PID base-goal tracking. SONIC additionally
defaults to automatic server launch and FP16 planner/policy inference. Its
deployment root is discovered from the standard sibling checkout or from
`SPARK_SONIC_DEPLOY_ROOT`.

The patched SONIC deployment uses one shared endpoint with independent session
state and one shared TensorRT model set. The released planner/encoder/decoder
exports are batch-one graphs, so the stable shared server executes those
stateful sessions sequentially inside one process. Apply
`third_party/sonic/patches/spark_batched_policy_server.patch` and then
`third_party/sonic/patches/spark_shared_session_stability.patch` to the SONIC
checkout with `git apply --ignore-space-change`, then rebuild it. See
{doc}`../../tutorials/benchmarking_and_recording` for commands and media.

A complete parallel SONIC invocation now needs only selectors:

```bash
python example/unitree_g1/run_unitree_g1_benchmark.py \
  --backend isaac \
  --policy-config UnitreeG1SonicSafePolicy \
  --robot-config UnitreeG1WholeBodyWithHandDynamic1Config \
  --num-envs 4 \
  --test-case base_goal_static_v1 \
  --headless
```

SONIC whole-goal completion remains a research configuration because its
released upper-body controller is not a strict Cartesian IK tracker. Base-goal
cases are the supported locomotion and safety benchmark.
