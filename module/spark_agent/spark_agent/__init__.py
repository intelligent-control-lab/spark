import os
from importlib import import_module

SPARK_AGENT_ROOT = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))

# ======================================================================
# Base
# ======================================================================
from .base.base_agent_config import BaseAgentConfig
from .base.base_agent import BaseAgent
from .dynamics import DynamicsExecutor
from .simulation.dynamics_model_agent import DynamicsModelAgent


# Native backends are deliberately lazy. Importing spark_agent must not load
# MuJoCo/OpenCV before an Isaac child process initializes Kit, and lightweight
# model-only workflows should not require either simulator.
_LAZY_IMPORTS = {
    "MujocoAgent": (".simulation.mujoco.mujoco_agent", "MujocoAgent"),
    "IsaacAgent": (".simulation.isaac", "IsaacAgent"),
    "ConfiguredIsaacAgent": (".simulation.isaac", "ConfiguredIsaacAgent"),
    "ConfiguredIsaacTensorAgent": (
        ".simulation.isaac",
        "ConfiguredIsaacTensorAgent",
    ),
    "AgiBotG1IsaacAgent": (
        ".simulation.isaac.agibot_g1.agibot_g1_isaac_agent",
        "AgiBotG1IsaacAgent",
    ),
    "AgiBotG1RightArmIsaacAgent": (
        ".simulation.isaac.agibot_g1.agibot_g1_right_arm_isaac_agent",
        "AgiBotG1RightArmIsaacAgent",
    ),
    "AgiBotG1DualArmIsaacAgent": (
        ".simulation.isaac.agibot_g1.agibot_g1_dual_arm_isaac_agent",
        "AgiBotG1DualArmIsaacAgent",
    ),
    "AgiBotG1FixedBaseIsaacAgent": (
        ".simulation.isaac.agibot_g1.agibot_g1_fixed_base_isaac_agent",
        "AgiBotG1FixedBaseIsaacAgent",
    ),
    "AgiBotG1MobileBaseIsaacAgent": (
        ".simulation.isaac.agibot_g1.agibot_g1_mobile_base_isaac_agent",
        "AgiBotG1MobileBaseIsaacAgent",
    ),
    "FanucLRMate200iDIsaacAgent": (
        ".simulation.isaac.fanuc_lrmate200id.fanuc_lrmate200id_isaac_agent",
        "FanucLRMate200iDIsaacAgent",
    ),
    "FanucLRMate200iDSingleArmIsaacAgent": (
        ".simulation.isaac.fanuc_lrmate200id.fanuc_lrmate200id_single_arm_isaac_agent",
        "FanucLRMate200iDSingleArmIsaacAgent",
    ),
    "FanucLRMate200iDDualArmIsaacAgent": (
        ".simulation.isaac.fanuc_lrmate200id.fanuc_lrmate200id_dual_arm_isaac_agent",
        "FanucLRMate200iDDualArmIsaacAgent",
    ),
    "GalaxeaR1LiteIsaacAgent": (
        ".simulation.isaac.galaxea_r1lite.galaxea_r1lite_isaac_agent",
        "GalaxeaR1LiteIsaacAgent",
    ),
    "GalaxeaR1LiteFixedBaseIsaacAgent": (
        ".simulation.isaac.galaxea_r1lite.galaxea_r1lite_fixed_base_isaac_agent",
        "GalaxeaR1LiteFixedBaseIsaacAgent",
    ),
    "GalaxeaR1LiteDualArmIsaacAgent": (
        ".simulation.isaac.galaxea_r1lite.galaxea_r1lite_dual_arm_isaac_agent",
        "GalaxeaR1LiteDualArmIsaacAgent",
    ),
    "GalaxeaR1LiteMobileBaseIsaacAgent": (
        ".simulation.isaac.galaxea_r1lite.galaxea_r1lite_mobile_base_isaac_agent",
        "GalaxeaR1LiteMobileBaseIsaacAgent",
    ),
    "GalaxeaR1LiteRightArmIsaacAgent": (
        ".simulation.isaac.galaxea_r1lite.galaxea_r1lite_right_arm_isaac_agent",
        "GalaxeaR1LiteRightArmIsaacAgent",
    ),
    "KinovaGen3IsaacAgent": (
        ".simulation.isaac.kinova_gen3.kinova_gen3_isaac_agent",
        "KinovaGen3IsaacAgent",
    ),
    "KinovaGen3SingleArmIsaacAgent": (
        ".simulation.isaac.kinova_gen3.kinova_gen3_single_arm_isaac_agent",
        "KinovaGen3SingleArmIsaacAgent",
    ),
    "KinovaGen3DualArmIsaacAgent": (
        ".simulation.isaac.kinova_gen3.kinova_gen3_dual_arm_isaac_agent",
        "KinovaGen3DualArmIsaacAgent",
    ),
    "KukaIIWA14IsaacAgent": (
        ".simulation.isaac.kuka_iiwa14.kuka_iiwa14_isaac_agent",
        "KukaIIWA14IsaacAgent",
    ),
    "KukaIIWA14SingleArmIsaacAgent": (
        ".simulation.isaac.kuka_iiwa14.kuka_iiwa14_single_arm_isaac_agent",
        "KukaIIWA14SingleArmIsaacAgent",
    ),
    "KukaIIWA14DualArmIsaacAgent": (
        ".simulation.isaac.kuka_iiwa14.kuka_iiwa14_dual_arm_isaac_agent",
        "KukaIIWA14DualArmIsaacAgent",
    ),
    "UnitreeG1IsaacAgent": (".simulation.isaac", "UnitreeG1IsaacAgent"),
    "UnitreeG1DynamicsVisualizationAgent": (
        ".simulation.isaac.unitree_g1.unitree_g1_dynamics_visualization_agent",
        "UnitreeG1DynamicsVisualizationAgent",
    ),
    "UnitreeG1RightArmIsaacAgent": (
        ".simulation.isaac.unitree_g1.unitree_g1_right_arm_isaac_agent",
        "UnitreeG1RightArmIsaacAgent",
    ),
    "UnitreeG1DualArmIsaacAgent": (
        ".simulation.isaac.unitree_g1.unitree_g1_dual_arm_isaac_agent",
        "UnitreeG1DualArmIsaacAgent",
    ),
    "UnitreeG1FixedBaseIsaacAgent": (
        ".simulation.isaac.unitree_g1.unitree_g1_fixed_base_isaac_agent",
        "UnitreeG1FixedBaseIsaacAgent",
    ),
    "UnitreeG1MobileBaseIsaacAgent": (
        ".simulation.isaac.unitree_g1.unitree_g1_mobile_base_isaac_agent",
        "UnitreeG1MobileBaseIsaacAgent",
    ),
    "UnitreeG1WholeBodyIsaacAgent": (
        ".simulation.isaac.unitree_g1.unitree_g1_whole_body_isaac_agent",
        "UnitreeG1WholeBodyIsaacAgent",
    ),
    "UnitreeG1RightArmMujocoAgent": (
        ".simulation.mujoco.unitree_g1.unitree_g1_right_arm_mujoco_agent",
        "UnitreeG1RightArmMujocoAgent",
    ),
    "UnitreeG1DualArmMujocoAgent": (
        ".simulation.mujoco.unitree_g1.unitree_g1_dual_arm_mujoco_agent",
        "UnitreeG1DualArmMujocoAgent",
    ),
    "UnitreeG1FixedBaseMujocoAgent": (
        ".simulation.mujoco.unitree_g1.unitree_g1_fixed_base_mujoco_agent",
        "UnitreeG1FixedBaseMujocoAgent",
    ),
    "UnitreeG1MobileBaseMujocoAgent": (
        ".simulation.mujoco.unitree_g1.unitree_g1_mobile_base_mujoco_agent",
        "UnitreeG1MobileBaseMujocoAgent",
    ),
    "UnitreeG1WholeBodyMujocoAgent": (
        ".simulation.mujoco.unitree_g1.unitree_g1_whole_body_mujoco_agent",
        "UnitreeG1WholeBodyMujocoAgent",
    ),
    "UnitreeG1RealAgent": (".real.unitree_g1.unitree_g1_real_agent", "UnitreeG1RealAgent"),
    "KukaIIWA14SingleArmAgent": (
        ".simulation.mujoco.kuka_iiwa14.kuka_iiwa14_single_arm_agent",
        "KukaIIWA14SingleArmAgent",
    ),
    "KukaIIWA14DualArmAgent": (
        ".simulation.mujoco.kuka_iiwa14.kuka_iiwa14_dual_arm_agent",
        "KukaIIWA14DualArmAgent",
    ),
    "KinovaGen3SingleArmAgent": (
        ".simulation.mujoco.kinova_gen3.kinova_gen3_single_arm_agent",
        "KinovaGen3SingleArmAgent",
    ),
    "KinovaGen3DualArmAgent": (
        ".simulation.mujoco.kinova_gen3.kinova_gen3_dual_arm_agent",
        "KinovaGen3DualArmAgent",
    ),
    "FanucLRMate200iDSingleArmAgent": (
        ".simulation.mujoco.fanuc_lrmate200id.fanuc_lrmate200id_single_arm_agent",
        "FanucLRMate200iDSingleArmAgent",
    ),
    "FanucLRMate200iDDualArmAgent": (
        ".simulation.mujoco.fanuc_lrmate200id.fanuc_lrmate200id_dual_arm_agent",
        "FanucLRMate200iDDualArmAgent",
    ),
    "GalaxeaR1LiteFixedBaseAgent": (
        ".simulation.mujoco.galaxea_r1lite.galaxea_r1lite_fixed_base_agent",
        "GalaxeaR1LiteFixedBaseAgent",
    ),
    "GalaxeaR1LiteDualArmAgent": (
        ".simulation.mujoco.galaxea_r1lite.galaxea_r1lite_dual_arm_agent",
        "GalaxeaR1LiteDualArmAgent",
    ),
    "GalaxeaR1LiteMobileBaseAgent": (
        ".simulation.mujoco.galaxea_r1lite.galaxea_r1lite_mobile_base_agent",
        "GalaxeaR1LiteMobileBaseAgent",
    ),
    "GalaxeaR1LiteRightArmAgent": (
        ".simulation.mujoco.galaxea_r1lite.galaxea_r1lite_right_arm_agent",
        "GalaxeaR1LiteRightArmAgent",
    ),
    "GalaxeaR1LiteRealAgent": (
        ".real.galaxea_r1lite.galaxea_r1lite_real_agent",
        "GalaxeaR1LiteRealAgent",
    ),
    "AgiBotG1FixedBaseAgent": (
        ".simulation.mujoco.agibot_g1.agibot_g1_fixed_base_agent",
        "AgiBotG1FixedBaseAgent",
    ),
    "AgiBotG1MobileBaseAgent": (
        ".simulation.mujoco.agibot_g1.agibot_g1_mobile_base_agent",
        "AgiBotG1MobileBaseAgent",
    ),
    "AgiBotG1DualArmAgent": (
        ".simulation.mujoco.agibot_g1.agibot_g1_dual_arm_agent",
        "AgiBotG1DualArmAgent",
    ),
    "AgiBotG1RightArmAgent": (
        ".simulation.mujoco.agibot_g1.agibot_g1_right_arm_agent",
        "AgiBotG1RightArmAgent",
    ),
}


def __getattr__(name):
    target = _LAZY_IMPORTS.get(name)
    if target is None:
        raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
    module_name, attribute_name = target
    value = getattr(import_module(module_name, __name__), attribute_name)
    globals()[name] = value
    return value


def __dir__():
    return sorted(set(globals()) | set(_LAZY_IMPORTS))
