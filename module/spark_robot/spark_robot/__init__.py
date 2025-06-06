import os

SPARK_ROBOT_ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
SPARK_ROBOT_RESOURCE_DIR = os.path.join(SPARK_ROBOT_ROOT, "resources")

from .base.base_robot_config import RobotConfig
from .base.base_robot_kinematics import RobotKinematics
from .g1.kinematics.g1_right_arm_kinematics import G1RightArmKinematics
from .g1.kinematics.g1_fixed_base_kinematics import G1FixedBaseKinematics
from .g1.kinematics.g1_mobile_base_kinematics import G1MobileBaseKinematics
from .g1.kinematics.g1_whole_body_kinematics import G1WholeBodyKinematics

# Single Integrator
from .g1.config.g1_fixed_base_dynamic_1_config import G1FixedBaseDynamic1Config
from .g1.config.g1_mobile_base_dynamic_1_config import G1MobileBaseDynamic1Config
from .g1.config.g1_right_arm_dynamic_1_config import G1RightArmDynamic1Config
from .g1.config.g1_sport_mode_dynamic_1_config import G1SportModeDynamic1Config
from .g1.config.g1_whole_body_dynamic_1_config import G1WholeBodyDynamic1Config

# Double Integrator
from .g1.config.g1_fixed_base_dynamic_2_config import G1FixedBaseDynamic2Config
from .g1.config.g1_mobile_base_dynamic_2_config import G1MobileBaseDynamic2Config
from .g1.config.g1_right_arm_dynamic_2_config import G1RightArmDynamic2Config
from .g1.config.g1_sport_mode_dynamic_2_config import G1SportModeDynamic2Config
from .g1.config.g1_whole_body_dynamic_2_config import G1WholeBodyDynamic2Config