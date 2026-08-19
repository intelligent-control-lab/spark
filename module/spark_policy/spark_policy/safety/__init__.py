"""Safety assessment and nominal-command filtering."""

from pathlib import Path

SPARK_SAFETY_ROOT = str(Path(__file__).resolve().parent)

from .geometry import (  # noqa: E402
    CollisionQueryResult,
    DynamicPointCloudBuffer,
    DenseESDFGrid,
    LinkSphereModel,
    PointCloudBatch,
    TorchSphereCollisionBackend,
    TorchDenseESDFBackend,
    TorchMeshCollisionBackend,
    TriangleMeshBatch,
    build_link_sphere_model,
)
from .tensor import (  # noqa: E402
    BatchedProjectionSafetyFilter,
    BatchedQPSafetyFilter,
    BatchedRelaxedQPSafetyFilter,
    BatchedReactiveSafetyFilter,
    FirstOrderTensorSafetyIndex,
    SecondOrderTensorSafetyIndex,
    TensorSafetyConstraints,
)

from .monitoring import *
from .filtering import *


def __getattr__(name):
    if name != "SecondOrderNNCollisionSafetyIndex":
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    from .monitoring import SecondOrderNNCollisionSafetyIndex

    globals()[name] = SecondOrderNNCollisionSafetyIndex
    return SecondOrderNNCollisionSafetyIndex


def __dir__():
    return sorted(set(globals()) | {"SecondOrderNNCollisionSafetyIndex"})
