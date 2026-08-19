from enum import IntEnum
from spark_robot.unitree_g1.config.unitree_g1_fixed_base_dynamic_2_config import (
    UnitreeG1FixedBaseDynamic2Config,
)


def _enum_from_pairs(name, pairs):
    return IntEnum(name, {key: value for key, value in pairs})


def _copy_motor_gains(base_cls, motor_cls):
    kps = {}
    kds = {}
    for motor in motor_cls:
        base_motor = getattr(base_cls.MujocoMotors, motor.name)
        kps[motor] = base_cls.MujocoMotorKps[base_motor]
        kds[motor] = base_cls.MujocoMotorKds[base_motor]
    return kps, kds


def _copy_motor_control_map(base_cls, motor_cls):
    return {
        motor: getattr(
            base_cls.Control,
            base_cls.MujocoMotor_to_Control[getattr(base_cls.MujocoMotors, motor.name)].name,
        )
        for motor in motor_cls
        if getattr(base_cls.MujocoMotors, motor.name) in base_cls.MujocoMotor_to_Control
    }


def _copy_dof_maps(base_cls, dof_cls):
    mj_to_dof = {
        mj_dof: getattr(base_cls.DoFs, mj_dof.name)
        for mj_dof in dof_cls
        if hasattr(base_cls.DoFs, mj_dof.name)
    }
    return mj_to_dof, {dof: mj_dof for mj_dof, dof in mj_to_dof.items()}


class UnitreeG1FixedBaseWithHandDynamic2Config(UnitreeG1FixedBaseDynamic2Config):
    kinematics_class_name = UnitreeG1FixedBaseDynamic2Config.kinematics_class_name
    kinematics_model_path = UnitreeG1FixedBaseDynamic2Config.mujoco_model_path
    mujoco_model_path = "unitree_g1/mjcf/g1_29dof_fixed_base_with_hand.xml"

    MujocoDoFs = _enum_from_pairs(
        "MujocoDoFs",
        [
            ("WaistYaw", 12),
            ("WaistRoll", 13),
            ("WaistPitch", 14),
            ("LeftShoulderPitch", 15),
            ("LeftShoulderRoll", 16),
            ("LeftShoulderYaw", 17),
            ("LeftElbow", 18),
            ("LeftWristRoll", 19),
            ("LeftWristPitch", 20),
            ("LeftWristYaw", 21),
            ("RightShoulderPitch", 29),
            ("RightShoulderRoll", 30),
            ("RightShoulderYaw", 31),
            ("RightElbow", 32),
            ("RightWristRoll", 33),
            ("RightWristPitch", 34),
            ("RightWristYaw", 35),
        ],
    )
    MujocoMotors = _enum_from_pairs(
        "MujocoMotors",
        [
            ("WaistYaw", 12),
            ("WaistRoll", 13),
            ("WaistPitch", 14),
            ("LeftShoulderPitch", 15),
            ("LeftShoulderRoll", 16),
            ("LeftShoulderYaw", 17),
            ("LeftElbow", 18),
            ("LeftWristRoll", 19),
            ("LeftWristPitch", 20),
            ("LeftWristYaw", 21),
            ("RightShoulderPitch", 29),
            ("RightShoulderRoll", 30),
            ("RightShoulderYaw", 31),
            ("RightElbow", 32),
            ("RightWristRoll", 33),
            ("RightWristPitch", 34),
            ("RightWristYaw", 35),
        ],
    )
    MujocoDoF_to_DoF, DoF_to_MujocoDoF = _copy_dof_maps(
        UnitreeG1FixedBaseDynamic2Config, MujocoDoFs
    )
    DoF_to_MujocoVel = DoF_to_MujocoDoF
    MujocoMotor_to_Control = _copy_motor_control_map(UnitreeG1FixedBaseDynamic2Config, MujocoMotors)
    MujocoMotorKps, MujocoMotorKds = _copy_motor_gains(
        UnitreeG1FixedBaseDynamic2Config, MujocoMotors
    )
