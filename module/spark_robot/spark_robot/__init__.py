import os

SPARK_ROBOT_ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
SPARK_ROBOT_RESOURCE_DIR = os.path.join(SPARK_ROBOT_ROOT, "resources")

from .base.base_robot_config import RobotConfig
from .base.base_robot_kinematics import RobotKinematics
from .g1.g1_basic_config import G1BasicConfig
from .g1.g1_right_arm_config import G1RightArmConfig
from .g1.g1_basic_kinematics import G1BasicKinematics