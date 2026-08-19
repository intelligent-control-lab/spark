"""KUKA iiwa 14 dual-arm first-order config with sparse body spheres."""

from .arm_collision_geometry import arm_collision_definition, self_collision_ignored_pairs
from .kuka_iiwa14_dual_arm_dynamic_1_config import KukaIIWA14DualArmDynamic1Config


class KukaIIWA14DualArmDynamic1CollisionConfig(KukaIIWA14DualArmDynamic1Config):
    collision_spheres_json_path = "kuka_iiwa14/config/kuka_iiwa14_single_arm_collision_spheres.json"
    Frames, CollisionVol, CollisionVolBodyNames, CollisionVolLocalOffsets = (
        arm_collision_definition(dual_arm=True)
    )
    AdjacentCollisionVolPairs = self_collision_ignored_pairs(Frames, CollisionVolBodyNames)
    SelfCollisionVolIgnored = []
    EnvCollisionVolIgnored = []
    VisualizeSafeZone = list(CollisionVol)
    VisualizePhiTraj = [Frames.R_ee, Frames.L_ee]
