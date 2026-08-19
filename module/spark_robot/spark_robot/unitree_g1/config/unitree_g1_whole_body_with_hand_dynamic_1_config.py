from enum import IntEnum
import numpy as np
from spark_robot.unitree_g1.config.unitree_g1_whole_body_dynamic_1_config import (
    UnitreeG1WholeBodyDynamic1Config,
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


_BODY_DOF_NAMES = [
    "LeftHipPitch",
    "LeftHipRoll",
    "LeftHipYaw",
    "LeftKnee",
    "LeftAnklePitch",
    "LeftAnkleRoll",
    "RightHipPitch",
    "RightHipRoll",
    "RightHipYaw",
    "RightKnee",
    "RightAnklePitch",
    "RightAnkleRoll",
    "WaistYaw",
    "WaistRoll",
    "WaistPitch",
    "LeftShoulderPitch",
    "LeftShoulderRoll",
    "LeftShoulderYaw",
    "LeftElbow",
    "LeftWristRoll",
    "LeftWristPitch",
    "LeftWristYaw",
    "RightShoulderPitch",
    "RightShoulderRoll",
    "RightShoulderYaw",
    "RightElbow",
    "RightWristRoll",
    "RightWristPitch",
    "RightWristYaw",
]

_HAND_DOF_NAMES = [
    "LeftHandThumb0",
    "LeftHandThumb1",
    "LeftHandThumb2",
    "LeftHandIndex0",
    "LeftHandIndex1",
    "LeftHandMiddle0",
    "LeftHandMiddle1",
    "RightHandThumb0",
    "RightHandThumb1",
    "RightHandThumb2",
    "RightHandIndex0",
    "RightHandIndex1",
    "RightHandMiddle0",
    "RightHandMiddle1",
]

_MODEL_DOF_PAIRS = [
    ("LinearX", 0),
    ("LinearY", 1),
    ("LinearZ", 2),
    ("QuaternionW", 3),
    ("QuaternionX", 4),
    ("QuaternionY", 5),
    ("QuaternionZ", 6),
    ("LeftHipPitch", 7),
    ("LeftHipRoll", 8),
    ("LeftHipYaw", 9),
    ("LeftKnee", 10),
    ("LeftAnklePitch", 11),
    ("LeftAnkleRoll", 12),
    ("RightHipPitch", 13),
    ("RightHipRoll", 14),
    ("RightHipYaw", 15),
    ("RightKnee", 16),
    ("RightAnklePitch", 17),
    ("RightAnkleRoll", 18),
    ("WaistYaw", 19),
    ("WaistRoll", 20),
    ("WaistPitch", 21),
    ("LeftShoulderPitch", 22),
    ("LeftShoulderRoll", 23),
    ("LeftShoulderYaw", 24),
    ("LeftElbow", 25),
    ("LeftWristRoll", 26),
    ("LeftWristPitch", 27),
    ("LeftWristYaw", 28),
    ("LeftHandThumb0", 29),
    ("LeftHandThumb1", 30),
    ("LeftHandThumb2", 31),
    ("LeftHandIndex0", 32),
    ("LeftHandIndex1", 33),
    ("LeftHandMiddle0", 34),
    ("LeftHandMiddle1", 35),
    ("RightShoulderPitch", 36),
    ("RightShoulderRoll", 37),
    ("RightShoulderYaw", 38),
    ("RightElbow", 39),
    ("RightWristRoll", 40),
    ("RightWristPitch", 41),
    ("RightWristYaw", 42),
    ("RightHandThumb0", 43),
    ("RightHandThumb1", 44),
    ("RightHandThumb2", 45),
    ("RightHandIndex0", 46),
    ("RightHandIndex1", 47),
    ("RightHandMiddle0", 48),
    ("RightHandMiddle1", 49),
]

_MODEL_DOFS = _enum_from_pairs("DoFs", _MODEL_DOF_PAIRS)


class UnitreeG1WholeBodyWithHandDynamic1Config(UnitreeG1WholeBodyDynamic1Config):
    kinematics_class_name = UnitreeG1WholeBodyDynamic1Config.kinematics_class_name
    mujoco_model_path = "unitree_g1/mjcf/g1_29dof_whole_body_with_hand.xml"

    DoFs = _MODEL_DOFS
    DefaultDoFVal = {
        getattr(_MODEL_DOFS, name): UnitreeG1WholeBodyDynamic1Config.DefaultDoFVal[
            getattr(UnitreeG1WholeBodyDynamic1Config.DoFs, name)
        ]
        for name in (
            "LinearX",
            "LinearY",
            "LinearZ",
            "QuaternionW",
            "QuaternionX",
            "QuaternionY",
            "QuaternionZ",
            *_BODY_DOF_NAMES,
        )
    }
    DefaultDoFVal.update({getattr(_MODEL_DOFS, name): 0.0 for name in _HAND_DOF_NAMES})

    MujocoDoFs = _enum_from_pairs("MujocoDoFs", _MODEL_DOF_PAIRS)
    MujocoMotors = _enum_from_pairs(
        "MujocoMotors",
        [
            ("LeftHipPitch", 0),
            ("LeftHipRoll", 1),
            ("LeftHipYaw", 2),
            ("LeftKnee", 3),
            ("LeftAnklePitch", 4),
            ("LeftAnkleRoll", 5),
            ("RightHipPitch", 6),
            ("RightHipRoll", 7),
            ("RightHipYaw", 8),
            ("RightKnee", 9),
            ("RightAnklePitch", 10),
            ("RightAnkleRoll", 11),
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
    MujocoDoF_to_DoF = {dof: getattr(_MODEL_DOFS, dof.name) for dof in MujocoDoFs}
    DoF_to_MujocoDoF = {dof: mj_dof for mj_dof, dof in MujocoDoF_to_DoF.items()}
    DoF_to_MujocoVel = DoF_to_MujocoDoF
    MujocoMotor_to_Control = _copy_motor_control_map(UnitreeG1WholeBodyDynamic1Config, MujocoMotors)
    MujocoMotorKps, MujocoMotorKds = _copy_motor_gains(
        UnitreeG1WholeBodyDynamic1Config, MujocoMotors
    )

    @property
    def num_state(self):
        return len(self.DoFs)

    def compose_state_from_dof(self, dof_pos, dof_vel):
        dof_pos = np.asarray(dof_pos, dtype=float).reshape(-1)
        return dof_pos.reshape(self.num_state)

    def decompose_state_to_dof_pos(self, state):
        return np.asarray(state, dtype=float).reshape(-1)

    def decompose_state_to_dof_vel(self, state):
        return np.zeros(self.num_dof - 1)
