from . import sonic as _sonic
from .sonic import (
    UnitreeG1BatchedSonicPolicy,
    UnitreeG1SonicPolicy,
    UnitreeG1SonicPolicyConfig,
)
from .sport import UnitreeG1SportPolicy, UnitreeG1SportPolicyConfig
from .wbc import (
    UnitreeG1WBCPIDPolicy,
    UnitreeG1WBCPIDPolicyConfig,
    UnitreeG1WBCPolicy,
    UnitreeG1WBCPolicyConfig,
)
from .wbt import UnitreeG1BatchedWBTPolicy, UnitreeG1WBTPolicy, WBTPolicyConfig

__all__ = [name for name in globals() if not name.startswith("_")]
__all__.append("UnitreeG1NativeBatchedSonicPolicy")


def __getattr__(name):
    """Forward optional SONIC exports without importing Torch eagerly."""
    if name == "UnitreeG1NativeBatchedSonicPolicy":
        return getattr(_sonic, name)
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
