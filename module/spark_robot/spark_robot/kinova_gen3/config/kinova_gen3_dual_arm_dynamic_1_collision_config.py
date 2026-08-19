"""Kinova Gen3 dual-arm first-order configuration with sparse body spheres."""

from .arm_collision_geometry import arm_collision_definition
from .kinova_gen3_dual_arm_dynamic_1_config import KinovaGen3DualArmDynamic1Config


class KinovaGen3DualArmDynamic1CollisionConfig(KinovaGen3DualArmDynamic1Config):
    collision_spheres_json_path = "kinova_gen3/config/kinova_gen3_single_arm_collision_spheres.json"
    Frames, CollisionVol, CollisionVolBodyNames, CollisionVolLocalOffsets = (
        arm_collision_definition(dual_arm=True)
    )
    AdjacentCollisionVolPairs = []
    SelfCollisionVolIgnored = []
    EnvCollisionVolIgnored = []
    VisualizeSafeZone = list(CollisionVol)
    VisualizePhiTraj = [Frames.R_ee, Frames.L_ee]
