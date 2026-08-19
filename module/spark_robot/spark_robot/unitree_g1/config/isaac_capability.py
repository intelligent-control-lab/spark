"""Shared Isaac capability metadata for Unitree G1 configurations."""

from spark_robot.base.backend_capability import BackendCapability


def unitree_g1_isaac_capability(agent_class_name: str) -> BackendCapability:
    """Describe a concrete G1 adapter's scalar/tensor execution contract."""

    return BackendCapability(
        agent_class_name,
        batched=True,
        rendering=True,
        tensor_io=True,
        validation="validated",
        notes=(
            "Scalar execution uses the Unitree Isaac adapter; supplying num_envs selects "
            "the CUDA-resident Isaac Lab tensor backend used by WBT, Sport, and SONIC workflows."
        ),
    )
