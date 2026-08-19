"""R1 Lite mobile-base first-order configuration with sparse body spheres."""

from .arm_collision_geometry import sparse_whole_body_collision_definition
from .galaxea_r1lite_mobile_base_dynamic_1_config import (
    GalaxeaR1LiteMobileBaseDynamic1Config,
)


class GalaxeaR1LiteMobileBaseDynamic1CollisionConfig(GalaxeaR1LiteMobileBaseDynamic1Config):
    Frames, CollisionVol, CollisionVolBodyNames, CollisionVolLocalOffsets = (
        sparse_whole_body_collision_definition()
    )
    AdjacentCollisionVolPairs = []
    SelfCollisionVolIgnored = []
    EnvCollisionVolIgnored = []
    VisualizeSafeZone = list(CollisionVol)
    VisualizePhiTraj = [Frames.R_ee, Frames.L_ee]
