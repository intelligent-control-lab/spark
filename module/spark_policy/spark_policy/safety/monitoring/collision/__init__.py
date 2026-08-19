from importlib import import_module

from .base import BasicCollisionSafetyIndex
from .first_order import FirstOrderCollisionSafetyIndex
from .first_order_approx import FirstOrderCollisionSafetyIndexApprox
from .second_order import SecondOrderCollisionSafetyIndex
from .second_order_approx import SecondOrderCollisionSafetyIndexApprox

__all__ = [
    "BasicCollisionSafetyIndex",
    "FirstOrderCollisionSafetyIndex",
    "FirstOrderCollisionSafetyIndexApprox",
    "SecondOrderCollisionSafetyIndex",
    "SecondOrderCollisionSafetyIndexApprox",
    "SecondOrderNNCollisionSafetyIndex",
]


def __getattr__(name):
    if name != "SecondOrderNNCollisionSafetyIndex":
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    value = getattr(import_module(".learned", __name__), name)
    globals()[name] = value
    return value


def __dir__():
    return sorted(set(globals()) | {"SecondOrderNNCollisionSafetyIndex"})
