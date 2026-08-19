from importlib import import_module

from .base import BaseSafetyIndex
from .collision import (
    BasicCollisionSafetyIndex,
    FirstOrderCollisionSafetyIndex,
    FirstOrderCollisionSafetyIndexApprox,
    SecondOrderCollisionSafetyIndex,
    SecondOrderCollisionSafetyIndexApprox,
)

__all__ = [
    "BaseSafetyIndex",
    "BasicCollisionSafetyIndex",
    "FirstOrderCollisionSafetyIndex",
    "FirstOrderCollisionSafetyIndexApprox",
    "SecondOrderCollisionSafetyIndex",
    "SecondOrderCollisionSafetyIndexApprox",
]


def __getattr__(name):
    if name != "SecondOrderNNCollisionSafetyIndex":
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    value = getattr(import_module(".collision", __name__), name)
    globals()[name] = value
    return value


def __dir__():
    return sorted(set(globals()) | {"SecondOrderNNCollisionSafetyIndex"})
