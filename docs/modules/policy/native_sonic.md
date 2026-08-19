# Planner-free batched SONIC

`UnitreeG1NativeBatchedSonicPolicy` runs the released SONIC encoder and
decoder directly on CUDA tensors. It does not start the SONIC C++ server and
does not invoke SONIC's locomotion planner or use JSON/ZMQ transport. This
makes one model instance suitable for large vectorized training batches.

The interface is intentionally narrower than the deployment-backed SONIC
policy. An upstream motion generator, trajectory optimizer, dataset, or task
policy must provide an absolute ten-frame G1 joint reference in SPARK/hardware
order. The native adapter converts that reference into SONIC's 64-dimensional
motion token, combines it with measured robot history, and returns 29 joint
targets. It is not a velocity-command-to-locomotion replacement for the
complete SONIC stack.

## Installation and models

Install the learned-policy dependencies, or use the Isaac profile:

```bash
./install.sh --profile mujoco --learned
# or
./install.sh --profile isaac
```

The SONIC models remain external and are not distributed with SPARK. Batched
execution requires encoder and decoder ONNX exports whose first dimension is
dynamic. The export changes are included in
`third_party/sonic/patches/spark_batched_policy_server.patch`; re-export the
models from the patched SONIC checkout. SPARK's benchmark looks first for
`model_encoder_spark_dynamic.onnx` and `model_decoder_spark_dynamic.onnx`, but
explicit paths can be supplied.

## Policy API

```python
from spark_policy.control.whole_body.unitree_g1 import (
    UnitreeG1NativeBatchedSonicPolicy,
)

policy = UnitreeG1NativeBatchedSonicPolicy(
    batch_size=num_envs,
    device="cuda:0",
    encoder_path=encoder_path,
    decoder_path=decoder_path,
    encoder_profile="release",
)

joint_target, info = policy.infer_tensor(
    body_joint_pos=joint_position,  # [B, 29]
    body_joint_vel=joint_velocity,  # [B, 29]
    root_pose_w=root_pose_xyzw,  # [B, 7]
    root_angular_velocity=root_angular_velocity,  # [B, 3]
    reference_joint_position=reference_window,  # [B, 10, 29]
    reference_joint_velocity=reference_velocity,  # [B, 10, 29]
)
```

`infer_token_tensor` accepts a precomputed `[B, 64]` SONIC token.
`encode_reference_tensor` exposes reference encoding without advancing the
controller history. `preview_tensor` evaluates a candidate reference and then
restores recurrent state, which is useful for batched reference governors.

Call `reset(env_ids=...)` at episode boundaries. Every environment owns an
independent history row even though the neural networks are shared.

## Throughput gate

The standalone gate measures policy inference, not physics simulation or
closed-loop locomotion quality:

```bash
SPARK_SONIC_DEPLOY_ROOT=/path/to/gear_sonic_deploy \
python tools/benchmark_native_sonic.py \
  --batch-size 4096 --warmup-steps 10 --steps 100
```

Report both `batch_hz` and `policy_rows_per_second`. A large aggregate row rate
does not imply that every simulated robot is controlled at 50 Hz. Full Isaac
qualification must separately execute the policy inside the intended task and
verify stability, contacts, resets, and task metrics.
