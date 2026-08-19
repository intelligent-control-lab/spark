import os
import sys

SPARK_ROBOT_ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
_SOURCE_RESOURCE_DIR = os.path.join(SPARK_ROBOT_ROOT, "resources")
_TARGET_RESOURCE_DIR = os.path.join(SPARK_ROBOT_ROOT, "share", "spark_robot", "resources")
_INSTALLED_RESOURCE_DIR = os.path.join(sys.prefix, "share", "spark_robot", "resources")
_RESOURCE_CANDIDATES = (
    os.environ.get("SPARK_ROBOT_RESOURCE_DIR"),
    _SOURCE_RESOURCE_DIR,
    _TARGET_RESOURCE_DIR,
    _INSTALLED_RESOURCE_DIR,
)
SPARK_ROBOT_RESOURCE_DIR = next(
    (path for path in _RESOURCE_CANDIDATES if path and os.path.isdir(path)),
    _SOURCE_RESOURCE_DIR,
)


def get_agent_class_name(robot_config_class_name, backend="mujoco"):
    """Resolve the default agent declared by a robot configuration class."""
    robot_config_class = globals().get(robot_config_class_name)
    if robot_config_class is None:
        raise ValueError(f"Unknown robot configuration: {robot_config_class_name!r}")
    return robot_config_class.agent_class_name(backend)


# ======================================================================
# Base
# ======================================================================
from .base.base_robot_config import RobotConfig
from .base.backend_capability import (
    ArticulatedGripperSpec,
    BackendCapability,
    IsaacArticulationInstanceSpec,
    IsaacPlanarBaseSpec,
    IsaacArticulationSpec,
    SimulatorDynamicsSpec,
)
from .base.base_robot_kinematics import RobotKinematics
from .base.dynamics_model import DynamicsDimensions, DynamicsStepContext, RobotDynamicsModel
from .kinematics import BoundedLeastSquaresIK, IKProblem, IKResult, IKSolverConfig, IKTarget
from .numerical import (
    DiscreteTimeDynamicsConfig,
    DiscreteTimeDynamicsModel,
    LinearDiscreteDynamicsConfig,
)

# ======================================================================
# Unitree G1
# ======================================================================
from .unitree_g1.kinematics.unitree_g1_right_arm_kinematics import UnitreeG1RightArmKinematics
from .unitree_g1.kinematics.unitree_g1_fixed_base_kinematics import UnitreeG1FixedBaseKinematics
from .unitree_g1.kinematics.unitree_g1_dual_arm_kinematics import UnitreeG1DualArmKinematics
from .unitree_g1.kinematics.unitree_g1_mobile_base_kinematics import UnitreeG1MobileBaseKinematics
from .unitree_g1.kinematics.unitree_g1_whole_body_kinematics import UnitreeG1WholeBodyKinematics
from .unitree_g1.config.unitree_g1_fixed_base_dynamic_1_config import (
    UnitreeG1FixedBaseDynamic1Config,
)
from .unitree_g1.config.unitree_g1_dual_arm_dynamic_1_config import UnitreeG1DualArmDynamic1Config
from .unitree_g1.config.unitree_g1_dual_arm_dynamic_2_config import UnitreeG1DualArmDynamic2Config
from .unitree_g1.config.unitree_g1_mobile_base_dynamic_1_config import (
    UnitreeG1MobileBaseDynamic1Config,
)
from .unitree_g1.config.unitree_g1_right_arm_dynamic_1_config import UnitreeG1RightArmDynamic1Config
from .unitree_g1.config.unitree_g1_whole_body_dynamic_1_config import (
    UnitreeG1WholeBodyDynamic1Config,
)
from .unitree_g1.config.unitree_g1_fixed_base_dynamic_2_config import (
    UnitreeG1FixedBaseDynamic2Config,
)
from .unitree_g1.config.unitree_g1_mobile_base_dynamic_2_config import (
    UnitreeG1MobileBaseDynamic2Config,
)
from .unitree_g1.config.unitree_g1_right_arm_dynamic_2_config import UnitreeG1RightArmDynamic2Config
from .unitree_g1.config.unitree_g1_whole_body_dynamic_2_config import (
    UnitreeG1WholeBodyDynamic2Config,
)
from .unitree_g1.config.unitree_g1_dual_arm_with_hand_dynamic_1_config import (
    UnitreeG1DualArmWithHandDynamic1Config,
)
from .unitree_g1.config.unitree_g1_dual_arm_with_hand_dynamic_2_config import (
    UnitreeG1DualArmWithHandDynamic2Config,
)
from .unitree_g1.config.unitree_g1_fixed_base_with_hand_dynamic_1_config import (
    UnitreeG1FixedBaseWithHandDynamic1Config,
)
from .unitree_g1.config.unitree_g1_fixed_base_with_hand_dynamic_2_config import (
    UnitreeG1FixedBaseWithHandDynamic2Config,
)
from .unitree_g1.config.unitree_g1_mobile_base_with_hand_dynamic_1_config import (
    UnitreeG1MobileBaseWithHandDynamic1Config,
)
from .unitree_g1.config.unitree_g1_mobile_base_with_hand_dynamic_2_config import (
    UnitreeG1MobileBaseWithHandDynamic2Config,
)
from .unitree_g1.config.unitree_g1_right_arm_with_hand_dynamic_1_config import (
    UnitreeG1RightArmWithHandDynamic1Config,
)
from .unitree_g1.config.unitree_g1_right_arm_with_hand_dynamic_2_config import (
    UnitreeG1RightArmWithHandDynamic2Config,
)
from .unitree_g1.config.unitree_g1_whole_body_with_hand_dynamic_1_config import (
    UnitreeG1WholeBodyWithHandDynamic1Config,
)
from .unitree_g1.config.unitree_g1_whole_body_with_hand_dynamic_2_config import (
    UnitreeG1WholeBodyWithHandDynamic2Config,
)

# Deprecated policy-named robot contracts.  Active code uses MobileBase
# configurations; these aliases keep older user configuration files importable
# for one migration cycle without retaining duplicate implementations.
UnitreeG1SportModeDynamic1Config = UnitreeG1MobileBaseDynamic1Config
UnitreeG1SportModeDynamic2Config = UnitreeG1MobileBaseDynamic2Config
UnitreeG1SportModeWithHandDynamic1Config = UnitreeG1MobileBaseWithHandDynamic1Config
UnitreeG1SportModeWithHandDynamic2Config = UnitreeG1MobileBaseWithHandDynamic2Config
UnitreeG1SonicDynamic1Config = UnitreeG1MobileBaseDynamic1Config
UnitreeG1WBTDynamic1Config = UnitreeG1MobileBaseDynamic1Config

# ======================================================================
# KUKA IIWA14
# ======================================================================
from .kuka_iiwa14.kinematics.kuka_iiwa14_single_arm_kinematics import KukaIIWA14SingleArmKinematics
from .kuka_iiwa14.kinematics.kuka_iiwa14_dual_arm_kinematics import KukaIIWA14DualArmKinematics
from .kuka_iiwa14.config.kuka_iiwa14_single_arm_dynamic_1_config import (
    KukaIIWA14SingleArmDynamic1Config,
)
from .kuka_iiwa14.config.kuka_iiwa14_single_arm_dynamic_1_collision_config import (
    KukaIIWA14SingleArmDynamic1CollisionConfig,
)
from .kuka_iiwa14.config.kuka_iiwa14_single_arm_dynamic_2_config import (
    KukaIIWA14SingleArmDynamic2Config,
)
from .kuka_iiwa14.config.kuka_iiwa14_single_arm_dynamic_2_collision_config import (
    KukaIIWA14SingleArmDynamic2CollisionConfig,
)
from .kuka_iiwa14.config.kuka_iiwa14_dual_arm_dynamic_1_config import (
    KukaIIWA14DualArmDynamic1Config,
)
from .kuka_iiwa14.config.kuka_iiwa14_dual_arm_dynamic_1_collision_config import (
    KukaIIWA14DualArmDynamic1CollisionConfig,
)

# ======================================================================
# Galaxea R1 Lite
# ======================================================================
from .galaxea_r1lite.kinematics.galaxea_r1lite_dual_arm_kinematics import (
    GalaxeaR1LiteDualArmKinematics,
)
from .galaxea_r1lite.kinematics.galaxea_r1lite_fixed_base_kinematics import (
    GalaxeaR1LiteFixedBaseKinematics,
)
from .galaxea_r1lite.kinematics.galaxea_r1lite_mobile_base_kinematics import (
    GalaxeaR1LiteMobileBaseKinematics,
)
from .galaxea_r1lite.kinematics.galaxea_r1lite_right_arm_kinematics import (
    GalaxeaR1LiteRightArmKinematics,
)
from .galaxea_r1lite.kinematics.galaxea_r1lite_flat_kinematics import GalaxeaR1LiteFlatKinematics
from .galaxea_r1lite.config.galaxea_r1lite_mobile_base_dynamic_1_config import (
    GalaxeaR1LiteMobileBaseDynamic1Config,
)
from .galaxea_r1lite.config.galaxea_r1lite_mobile_base_dynamic_1_collision_config import (
    GalaxeaR1LiteMobileBaseDynamic1CollisionConfig,
)
from .galaxea_r1lite.config.galaxea_r1lite_dual_arm_dynamic_1_config import (
    GalaxeaR1LiteDualArmDynamic1Config,
)
from .galaxea_r1lite.config.galaxea_r1lite_dual_arm_dynamic_1_collision_config import (
    GalaxeaR1LiteDualArmDynamic1CollisionConfig,
)
from .galaxea_r1lite.config.galaxea_r1lite_right_arm_dynamic_1_config import (
    GalaxeaR1LiteRightArmDynamic1Config,
)
from .galaxea_r1lite.config.galaxea_r1lite_right_arm_dynamic_1_collision_config import (
    GalaxeaR1LiteRightArmDynamic1CollisionConfig,
)
from .galaxea_r1lite.config.galaxea_r1lite_fixed_base_dynamic_1_config import (
    GalaxeaR1LiteFixedBaseDynamic1Config,
)
from .galaxea_r1lite.config.galaxea_r1lite_fixed_base_dynamic_1_collision_config import (
    GalaxeaR1LiteFixedBaseDynamic1CollisionConfig,
)
from .galaxea_r1lite.config.galaxea_r1lite_fixed_base_dynamic_2_config import (
    GalaxeaR1LiteFixedBaseDynamic2Config,
)
from .galaxea_r1lite.config.galaxea_r1lite_fixed_base_dynamic_2_collision_config import (
    GalaxeaR1LiteFixedBaseDynamic2CollisionConfig,
)

# ======================================================================
# FANUC LR Mate 200iD
# ======================================================================
from .fanuc_lrmate200id.kinematics.fanuc_lrmate200id_dual_arm_kinematics import (
    FanucLRMate200iDDualArmKinematics,
)
from .fanuc_lrmate200id.kinematics.fanuc_lrmate200id_single_arm_kinematics import (
    FanucLRMate200iDSingleArmKinematics,
)
from .fanuc_lrmate200id.config.fanuc_lrmate200id_dual_arm_dynamic_1_config import (
    FanucLRMate200iDDualArmDynamic1Config,
)
from .fanuc_lrmate200id.config.fanuc_lrmate200id_dual_arm_dynamic_1_collision_config import (
    FanucLRMate200iDDualArmDynamic1CollisionConfig,
)
from .fanuc_lrmate200id.config.fanuc_lrmate200id_single_arm_dynamic_1_config import (
    FanucLRMate200iDSingleArmDynamic1Config,
)
from .fanuc_lrmate200id.config.fanuc_lrmate200id_single_arm_dynamic_1_collision_config import (
    FanucLRMate200iDSingleArmDynamic1CollisionConfig,
)
from .fanuc_lrmate200id.config.fanuc_lrmate200id_single_arm_dynamic_2_config import (
    FanucLRMate200iDSingleArmDynamic2Config,
)
from .fanuc_lrmate200id.config.fanuc_lrmate200id_single_arm_dynamic_2_collision_config import (
    FanucLRMate200iDSingleArmDynamic2CollisionConfig,
)

# ======================================================================
# Kinova Gen3
# ======================================================================
from .kinova_gen3.kinematics.kinova_gen3_dual_arm_kinematics import KinovaGen3DualArmKinematics
from .kinova_gen3.kinematics.kinova_gen3_single_arm_kinematics import KinovaGen3SingleArmKinematics
from .kinova_gen3.config.kinova_gen3_dual_arm_dynamic_1_config import (
    KinovaGen3DualArmDynamic1Config,
)
from .kinova_gen3.config.kinova_gen3_dual_arm_dynamic_1_collision_config import (
    KinovaGen3DualArmDynamic1CollisionConfig,
)
from .kinova_gen3.config.kinova_gen3_single_arm_dynamic_1_config import (
    KinovaGen3SingleArmDynamic1Config,
)
from .kinova_gen3.config.kinova_gen3_single_arm_dynamic_1_collision_config import (
    KinovaGen3SingleArmDynamic1CollisionConfig,
)
from .kinova_gen3.config.kinova_gen3_single_arm_dynamic_2_config import (
    KinovaGen3SingleArmDynamic2Config,
)
from .kinova_gen3.config.kinova_gen3_single_arm_dynamic_2_collision_config import (
    KinovaGen3SingleArmDynamic2CollisionConfig,
)

# ======================================================================
# AgiBot G1
# ======================================================================
from .agibot_g1.kinematics.agibot_g1_fixed_base_kinematics import AgiBotG1FixedBaseKinematics
from .agibot_g1.kinematics.agibot_g1_mobile_base_kinematics import AgiBotG1MobileBaseKinematics
from .agibot_g1.kinematics.agibot_g1_dual_arm_kinematics import AgiBotG1DualArmKinematics
from .agibot_g1.kinematics.agibot_g1_right_arm_kinematics import AgiBotG1RightArmKinematics
from .agibot_g1.config.agibot_g1_fixed_base_dynamic_1_config import AgiBotG1FixedBaseDynamic1Config
from .agibot_g1.config.agibot_g1_mobile_base_dynamic_1_config import (
    AgiBotG1MobileBaseDynamic1Config,
)
from .agibot_g1.config.agibot_g1_mobile_base_dynamic_2_config import (
    AgiBotG1MobileBaseDynamic2Config,
)
from .agibot_g1.config.agibot_g1_mobile_base_unicycle_dynamic_1_config import (
    AgiBotG1MobileBaseUnicycleDynamic1Config,
)
from .agibot_g1.config.agibot_g1_mobile_base_bicycle_dynamic_2_config import (
    AgiBotG1MobileBaseBicycleDynamic2Config,
    BicycleParams,
)
from .agibot_g1.config.agibot_g1_dual_arm_dynamic_1_config import AgiBotG1DualArmDynamic1Config
from .agibot_g1.config.agibot_g1_right_arm_dynamic_1_config import AgiBotG1RightArmDynamic1Config
