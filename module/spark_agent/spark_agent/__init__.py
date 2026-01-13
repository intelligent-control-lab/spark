import os

SPARK_AGENT_ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

# ======================================================================
# Base
# ======================================================================
from .base.base_agent_config import BaseAgentConfig
from .base.base_agent import BaseAgent
from .simulation.mujoco.mujoco_agent import MujocoAgent

# ======================================================================
# G1
# ======================================================================
from .simulation.mujoco.g1.g1_mujoco_fixed_base_agent import G1MujocoFixedBaseAgent
from .simulation.mujoco.g1.g1_mujoco_right_arm_agent import G1MujocoRightArmAgent
from .simulation.mujoco.g1.g1_mujoco_mobile_base_agent import G1MujocoMobileBaseAgent
from .simulation.mujoco.g1.g1_mujoco_sport_mode_agent import G1MujocoSportModeAgent
from .simulation.mujoco.g1.g1_mujoco_whole_body_agent import G1MujocoWholeBodyAgent
from .real.g1.g1_real_agent import G1RealAgent

# ======================================================================
# IIWA14
# ======================================================================
from .simulation.mujoco.iiwa14.iiwa14_mujoco_agent import IIWA14MujocoFixedBaseAgent

# ======================================================================
# Gen3
# ======================================================================
from .simulation.mujoco.gen3.gen3_mujoco_agent import Gen3MujocoFixedBaseAgent

# ======================================================================
# LRMate200iD
# =====================================================================

from .simulation.mujoco.lrmate200id.lrmate200id_3f_mujoco_single_agent import LRMate200iD3fMujocoSingleAgent

# ======================================================================
# R1Lite
# ======================================================================
from .simulation.mujoco.r1lite.r1lite_mujoco_agent import R1LiteMujocoFixedBaseAgent
from .real.r1lite.r1lite_real_agent import R1LiteRealAgent