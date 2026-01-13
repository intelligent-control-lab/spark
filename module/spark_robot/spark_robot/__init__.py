import os

SPARK_ROBOT_ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
SPARK_ROBOT_RESOURCE_DIR = os.path.join(SPARK_ROBOT_ROOT, "resources")

# ======================================================================
# Base
# ======================================================================
from .base.base_robot_config import RobotConfig
from .base.base_robot_kinematics import RobotKinematics

# ======================================================================
# G1
# ======================================================================
from .g1.kinematics.g1_right_arm_kinematics import G1RightArmKinematics
from .g1.kinematics.g1_fixed_base_kinematics import G1FixedBaseKinematics
from .g1.kinematics.g1_dual_arm_kinematics import G1DualArmBaseKinematics
from .g1.kinematics.g1_mobile_base_kinematics import G1MobileBaseKinematics
from .g1.kinematics.g1_whole_body_kinematics import G1WholeBodyKinematics
from .g1.config.g1_fixed_base_dynamic_1_config import G1FixedBaseDynamic1Config
from .g1.config.g1_dual_arm_dynamic_1_config import G1DualArmDynamic1Config
from .g1.config.g1_mobile_base_dynamic_1_config import G1MobileBaseDynamic1Config
from .g1.config.g1_right_arm_dynamic_1_config import G1RightArmDynamic1Config
from .g1.config.g1_sport_mode_dynamic_1_config import G1SportModeDynamic1Config
from .g1.config.g1_whole_body_dynamic_1_config import G1WholeBodyDynamic1Config
from .g1.config.g1_fixed_base_dynamic_2_config import G1FixedBaseDynamic2Config
from .g1.config.g1_mobile_base_dynamic_2_config import G1MobileBaseDynamic2Config
from .g1.config.g1_right_arm_dynamic_2_config import G1RightArmDynamic2Config
from .g1.config.g1_sport_mode_dynamic_2_config import G1SportModeDynamic2Config
from .g1.config.g1_whole_body_dynamic_2_config import G1WholeBodyDynamic2Config

# ======================================================================
# IIWA14
# ======================================================================
from .iiwa14.kinematics.iiwa14_single_kinematics import IIWA14SingleKinematics
from .iiwa14.kinematics.iiwa14_dual_kinematics import IIWA14DualKinematics
from .iiwa14.config.iiwa14_single_dynamic_1_config import IIWA14SingleDynamic1Config
from .iiwa14.config.iiwa14_single_dynamic_1_collision_config import IIWA14SingleDynamic1CollisionConfig
from .iiwa14.config.iiwa14_single_dynamic_2_config import IIWA14SingleDynamic2Config
from .iiwa14.config.iiwa14_single_dynamic_2_collision_config import IIWA14SingleDynamic2CollisionConfig
from .iiwa14.config.iiwa14_dual_dynamic_1_config import IIWA14DualDynamic1Config

# ======================================================================
# R1Lite
# ======================================================================
from .r1lite.kinematics.r1lite_dual_kinematics import R1LiteDualArmKinematics
from .r1lite.kinematics.r1lite_upper_kinematics import R1LiteUpperKinematics
from .r1lite.kinematics.r1lite_mobile_kinematics import R1LiteMobileKinematics
from .r1lite.kinematics.r1lite_flat_kinematics import R1LiteFlatKinematics
from .r1lite.config.r1lite_mobile_dynamic_1_config import R1LiteMobileDynamic1Config
from .r1lite.config.r1lite_dual_dynamic_1_config import R1LiteDualArmDynamic1Config
from .r1lite.config.r1lite_upper_dynamic_1_config import R1LiteUpperDynamic1Config
from .r1lite.config.r1lite_upper_dynamic_1_collision_config import R1LiteUpperDynamic1CollisionConfig
from .r1lite.config.r1lite_upper_dynamic_2_config import R1LiteUpperDynamic2Config
from .r1lite.config.r1lite_upper_dynamic_2_collision_config import R1LiteUpperDynamic2CollisionConfig

# ======================================================================
# LRMate200iD
# ======================================================================
from .lrmate200id.kinematics.lrmate200id_3f_dual_kinematic import LRMate200iD3fDualKinematics
from .lrmate200id.kinematics.lrmate200id_3f_single_kinematics import LRMate200iD3fSingleKinematics
from .lrmate200id.config.lrmate200id_3f_single_dynamic_1_config import LRMate200iD3fSingleDynamic1Config
from .lrmate200id.config.lrmate200id_3f_single_dynamic_1_collision_config import LRMate200iD3fSingleDynamic1CollisionConfig
from .lrmate200id.config.lrmate200id_3f_single_dynamic_2_config import LRMate200iD3fSingleDynamic2Config
from .lrmate200id.config.lrmate200id_3f_single_dynamic_2_collision_config import LRMate200iD3fSingleDynamic2CollisionConfig

# ======================================================================
# Kinova Gen3
# ======================================================================
from .gen3.kinematics.gen3_single_kinematics import Gen3SingleKinematics
from .gen3.config.gen3_single_dynamic_1_config import Gen3SingleDynamic1Config
from .gen3.config.gen3_single_dynamic_1_collision_config import Gen3SingleDynamic1CollisionConfig
from .gen3.config.gen3_single_dynamic_2_config import Gen3SingleDynamic2Config
from .gen3.config.gen3_single_dynamic_2_collision_config import Gen3SingleDynamic2CollisionConfig
