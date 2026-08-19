"""R1 Lite dual-arm first-order configuration with sparse body spheres."""

from .arm_collision_geometry import sparse_arm_collision_definition
from .galaxea_r1lite_dual_arm_dynamic_1_config import GalaxeaR1LiteDualArmDynamic1Config


class GalaxeaR1LiteDualArmDynamic1CollisionConfig(GalaxeaR1LiteDualArmDynamic1Config):
    Frames, CollisionVol, CollisionVolBodyNames, CollisionVolLocalOffsets = (
        sparse_arm_collision_definition(dual_arm=True)
    )
    AdjacentCollisionVolPairs = []
    SelfCollisionVolIgnored = []
    EnvCollisionVolIgnored = []
    VisualizeSafeZone = list(CollisionVol)
    VisualizePhiTraj = [Frames.R_ee, Frames.L_ee]
