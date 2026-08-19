from enum import IntEnum
from spark_robot.unitree_g1.config.unitree_g1_dual_arm_dynamic_1_config import (
    UnitreeG1DualArmDynamic1Config,
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


class UnitreeG1DualArmWithHandDynamic1Config(UnitreeG1DualArmDynamic1Config):
    kinematics_class_name = UnitreeG1DualArmDynamic1Config.kinematics_class_name
    mujoco_model_path = "unitree_g1/mjcf/g1_29dof_dual_arm_with_hand.xml"

    # This reduced MJCF contains only the fourteen arm joints. The hand meshes
    # are visual/fixed geometry, so its qpos and actuator indices remain 0..13
    # just like the non-hand dual-arm model.
    MujocoDoFs = _enum_from_pairs(
        "MujocoDoFs", [(dof.name, int(dof)) for dof in UnitreeG1DualArmDynamic1Config.MujocoDoFs]
    )
    MujocoMotors = _enum_from_pairs(
        "MujocoMotors",
        [(motor.name, int(motor)) for motor in UnitreeG1DualArmDynamic1Config.MujocoMotors],
    )
    MujocoDoF_to_DoF, DoF_to_MujocoDoF = _copy_dof_maps(UnitreeG1DualArmDynamic1Config, MujocoDoFs)
    DoF_to_MujocoVel = DoF_to_MujocoDoF
    MujocoMotor_to_Control = _copy_motor_control_map(UnitreeG1DualArmDynamic1Config, MujocoMotors)
    MujocoMotorKps, MujocoMotorKds = _copy_motor_gains(UnitreeG1DualArmDynamic1Config, MujocoMotors)
