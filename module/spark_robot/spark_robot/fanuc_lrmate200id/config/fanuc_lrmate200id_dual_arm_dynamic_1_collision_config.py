"""FANUC LR Mate dual-arm first-order config with sparse body spheres."""

from .arm_collision_geometry import arm_collision_definition, self_collision_ignored_pairs
from .fanuc_lrmate200id_dual_arm_dynamic_1_config import (
    FanucLRMate200iDDualArmDynamic1Config,
)


class FanucLRMate200iDDualArmDynamic1CollisionConfig(FanucLRMate200iDDualArmDynamic1Config):
    collision_spheres_json_path = (
        "fanuc_lrmate200id/config/fanuc_lrmate200id_single_arm_collision_spheres.json"
    )
    Frames, CollisionVol, CollisionVolBodyNames, CollisionVolLocalOffsets = (
        arm_collision_definition(dual_arm=True)
    )
    AdjacentCollisionVolPairs = self_collision_ignored_pairs(Frames, CollisionVolBodyNames)
    SelfCollisionVolIgnored = []
    EnvCollisionVolIgnored = []
    VisualizeSafeZone = list(CollisionVol)
    VisualizePhiTraj = [Frames.R_ee, Frames.L_ee]
