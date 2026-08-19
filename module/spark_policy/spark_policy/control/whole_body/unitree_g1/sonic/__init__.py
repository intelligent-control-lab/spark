from .config import UnitreeG1SonicPolicyConfig
from .policy import UnitreeG1SonicPolicy
from .batched_policy import UnitreeG1BatchedSonicPolicy

__all__ = [
    "UnitreeG1SonicPolicy",
    "UnitreeG1BatchedSonicPolicy",
    "UnitreeG1NativeBatchedSonicPolicy",
    "UnitreeG1SonicPolicyConfig",
]


def __getattr__(name):
    """Load the CUDA-native adapter only when its optional stack is requested."""
    if name == "UnitreeG1NativeBatchedSonicPolicy":
        from .native_batched_policy import UnitreeG1NativeBatchedSonicPolicy

        return UnitreeG1NativeBatchedSonicPolicy
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
