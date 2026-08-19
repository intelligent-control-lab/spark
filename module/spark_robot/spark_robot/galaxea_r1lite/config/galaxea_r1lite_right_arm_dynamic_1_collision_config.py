"""R1 Lite right-arm first-order configuration with sparse body spheres."""

from .arm_collision_geometry import sparse_arm_collision_definition
from .galaxea_r1lite_right_arm_dynamic_1_config import GalaxeaR1LiteRightArmDynamic1Config


class GalaxeaR1LiteRightArmDynamic1CollisionConfig(GalaxeaR1LiteRightArmDynamic1Config):
    Frames, CollisionVol, CollisionVolBodyNames, CollisionVolLocalOffsets = (
        sparse_arm_collision_definition(dual_arm=False)
    )
    AdjacentCollisionVolPairs = []
    SelfCollisionVolIgnored = []
    EnvCollisionVolIgnored = []
    VisualizeSafeZone = list(CollisionVol)
    VisualizePhiTraj = [Frames.R_ee]
