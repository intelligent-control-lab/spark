from enum import IntEnum
from spark_robot.unitree_g1.config.unitree_g1_mobile_base_dynamic_1_config import (
    UnitreeG1MobileBaseDynamic1Config,
)


def _copy_mobile_base_dof_maps(base_cls, dof_cls):
    mj_to_dof = {mj_dof: getattr(base_cls.DoFs, mj_dof.name) for mj_dof in dof_cls}
    return mj_to_dof, {dof: mj_dof for mj_dof, dof in mj_to_dof.items()}


class UnitreeG1MobileBaseWithHandDynamic1Config(UnitreeG1MobileBaseDynamic1Config):
    kinematics_class_name = UnitreeG1MobileBaseDynamic1Config.kinematics_class_name
    mujoco_model_path = "unitree_g1/mjcf/g1_29dof_mobile_base_with_hand.xml"

    class MujocoDoFs(IntEnum):
        LinearX = 0
        LinearY = 1
        RotYaw = 2

        WaistYaw = 3
        WaistRoll = 4
        WaistPitch = 5

        LeftShoulderPitch = 6
        LeftShoulderRoll = 7
        LeftShoulderYaw = 8
        LeftElbow = 9
        LeftWristRoll = 10
        LeftWristPitch = 11
        LeftWristYaw = 12

        RightShoulderPitch = 20
        RightShoulderRoll = 21
        RightShoulderYaw = 22
        RightElbow = 23
        RightWristRoll = 24
        RightWristPitch = 25
        RightWristYaw = 26

    MujocoDoF_to_DoF, DoF_to_MujocoDoF = _copy_mobile_base_dof_maps(
        UnitreeG1MobileBaseDynamic1Config,
        MujocoDoFs,
    )
