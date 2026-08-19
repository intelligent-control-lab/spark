from enum import IntEnum

import numpy as np

from spark_robot.base.base_robot_config import RobotConfig
from .collision_geometry import (
    AGIBOT_G1_COLLISION_BODY_NAMES,
    AGIBOT_G1_COLLISION_LOCAL_OFFSETS,
    AGIBOT_G1_COLLISION_VOLUMES,
    AgiBotG1Frames,
)

from .isaac_articulation import (
    AGIBOT_G1_VISUAL_LINK_COLORS,
    DUAL_ARM_ISAAC_ARTICULATION,
    agibot_g1_isaac_capability,
)
from .joint_defaults import _AGIBOT_G1_LEFT_DEFAULTS, _AGIBOT_G1_RIGHT_DEFAULTS
from .simulator_dynamics import AGIBOT_G1_SIMULATOR_DYNAMICS


_AGIBOT_G1_GRIPPER_JOINTS = [
    "left_narrow1_joint",
    "left_narrow3_joint",
    "left_narrow4_joint",
    "left_narrow2_joint",
    "left_wide1_joint",
    "left_wide3_joint",
    "left_wide4_joint",
    "left_wide2_joint",
    "right_narrow1_joint",
    "right_narrow3_joint",
    "right_narrow4_joint",
    "right_narrow2_joint",
    "right_wide1_joint",
    "right_wide3_joint",
    "right_wide4_joint",
    "right_wide2_joint",
]

_AGIBOT_G1_TORSO_JOINTS = [
    "joint_lift_body",
    "joint_body_pitch",
]

_AGIBOT_G1_LEFT_ARM_JOINTS = [
    "Joint1_l",
    "Joint2_l",
    "Joint3_l",
    "Joint4_l",
    "Joint5_l",
    "Joint6_l",
    "Joint7_l",
]

_AGIBOT_G1_RIGHT_ARM_JOINTS = [
    "Joint1_r",
    "Joint2_r",
    "Joint3_r",
    "Joint4_r",
    "Joint5_r",
    "Joint6_r",
    "Joint7_r",
]

_AGIBOT_G1_LEFT_LIMITS = [
    (-3.14, 3.14),
    (-2.09, 2.09),
    (-3.14, 3.14),
    (-1.832595715, 1.483529864),
    (-3.14, 3.14),
    (-1.745329252, 1.745329252),
    (-3.14, 3.14),
]

_AGIBOT_G1_RIGHT_LIMITS = [
    (-3.14, 3.14),
    (-2.09, 2.09),
    (-3.14, 3.14),
    (-1.483529864, 1.832595715),
    (-3.14, 3.14),
    (-1.745329252, 1.745329252),
    (-3.14, 3.14),
]

_AGIBOT_G1_ARM_CONTROL_LIMITS = [1.5] * 7
_AGIBOT_G1_ARM_KPS = [120.0, 120.0, 90.0, 90.0, 50.0, 40.0, 30.0]
_AGIBOT_G1_ARM_KDS = [3.0, 3.0, 2.0, 2.0, 1.5, 1.0, 0.8]


class AgiBotG1DualArmDynamic1Config(RobotConfig):
    visual_link_colors = AGIBOT_G1_VISUAL_LINK_COLORS
    agent_class_names = {
        "mujoco": "AgiBotG1DualArmAgent",
        "isaac": "AgiBotG1DualArmIsaacAgent",
    }
    isaac_articulation = DUAL_ARM_ISAAC_ARTICULATION
    simulator_dynamics = AGIBOT_G1_SIMULATOR_DYNAMICS
    backend_capability_overrides = {
        "isaac": agibot_g1_isaac_capability("AgiBotG1DualArmIsaacAgent")
    }
    dynamics_variant = "single_integrator"
    dynamics_order = 1
    dynamics_is_linear = True

    """AgiBot G1 fixed-base config exposing both arms with torso joints locked."""

    kinematics_class_name = "AgiBotG1DualArmKinematics"
    mujoco_model_path = "agibot_g1/agibot_g1_dual_arm.xml"
    joint_to_lock = list(_AGIBOT_G1_GRIPPER_JOINTS) + list(_AGIBOT_G1_TORSO_JOINTS)

    NumTotalMotors = 16

    class RealMotors(IntEnum):
        LeftJoint1 = 2
        LeftJoint2 = 3
        LeftJoint3 = 4
        LeftJoint4 = 5
        LeftJoint5 = 6
        LeftJoint6 = 7
        LeftJoint7 = 8

        RightJoint1 = 9
        RightJoint2 = 10
        RightJoint3 = 11
        RightJoint4 = 12
        RightJoint5 = 13
        RightJoint6 = 14
        RightJoint7 = 15

    RealMotorPosLimit = {
        RealMotors.LeftJoint1: _AGIBOT_G1_LEFT_LIMITS[0],
        RealMotors.LeftJoint2: _AGIBOT_G1_LEFT_LIMITS[1],
        RealMotors.LeftJoint3: _AGIBOT_G1_LEFT_LIMITS[2],
        RealMotors.LeftJoint4: _AGIBOT_G1_LEFT_LIMITS[3],
        RealMotors.LeftJoint5: _AGIBOT_G1_LEFT_LIMITS[4],
        RealMotors.LeftJoint6: _AGIBOT_G1_LEFT_LIMITS[5],
        RealMotors.LeftJoint7: _AGIBOT_G1_LEFT_LIMITS[6],
        RealMotors.RightJoint1: _AGIBOT_G1_RIGHT_LIMITS[0],
        RealMotors.RightJoint2: _AGIBOT_G1_RIGHT_LIMITS[1],
        RealMotors.RightJoint3: _AGIBOT_G1_RIGHT_LIMITS[2],
        RealMotors.RightJoint4: _AGIBOT_G1_RIGHT_LIMITS[3],
        RealMotors.RightJoint5: _AGIBOT_G1_RIGHT_LIMITS[4],
        RealMotors.RightJoint6: _AGIBOT_G1_RIGHT_LIMITS[5],
        RealMotors.RightJoint7: _AGIBOT_G1_RIGHT_LIMITS[6],
    }

    NormalMotor = [
        RealMotors.LeftJoint1,
        RealMotors.LeftJoint2,
        RealMotors.LeftJoint3,
        RealMotors.LeftJoint4,
        RealMotors.LeftJoint5,
        RealMotors.LeftJoint6,
        RealMotors.LeftJoint7,
        RealMotors.RightJoint1,
        RealMotors.RightJoint2,
        RealMotors.RightJoint3,
        RealMotors.RightJoint4,
        RealMotors.RightJoint5,
        RealMotors.RightJoint6,
        RealMotors.RightJoint7,
    ]

    WeakMotor = []
    DelicateMotor = []

    class DoFs(IntEnum):
        LeftJoint1 = 0
        LeftJoint2 = 1
        LeftJoint3 = 2
        LeftJoint4 = 3
        LeftJoint5 = 4
        LeftJoint6 = 5
        LeftJoint7 = 6

        RightJoint1 = 7
        RightJoint2 = 8
        RightJoint3 = 9
        RightJoint4 = 10
        RightJoint5 = 11
        RightJoint6 = 12
        RightJoint7 = 13

    DefaultDoFVal = {
        DoFs.LeftJoint1: _AGIBOT_G1_LEFT_DEFAULTS[0],
        DoFs.LeftJoint2: _AGIBOT_G1_LEFT_DEFAULTS[1],
        DoFs.LeftJoint3: _AGIBOT_G1_LEFT_DEFAULTS[2],
        DoFs.LeftJoint4: _AGIBOT_G1_LEFT_DEFAULTS[3],
        DoFs.LeftJoint5: _AGIBOT_G1_LEFT_DEFAULTS[4],
        DoFs.LeftJoint6: _AGIBOT_G1_LEFT_DEFAULTS[5],
        DoFs.LeftJoint7: _AGIBOT_G1_LEFT_DEFAULTS[6],
        DoFs.RightJoint1: _AGIBOT_G1_RIGHT_DEFAULTS[0],
        DoFs.RightJoint2: _AGIBOT_G1_RIGHT_DEFAULTS[1],
        DoFs.RightJoint3: _AGIBOT_G1_RIGHT_DEFAULTS[2],
        DoFs.RightJoint4: _AGIBOT_G1_RIGHT_DEFAULTS[3],
        DoFs.RightJoint5: _AGIBOT_G1_RIGHT_DEFAULTS[4],
        DoFs.RightJoint6: _AGIBOT_G1_RIGHT_DEFAULTS[5],
        DoFs.RightJoint7: _AGIBOT_G1_RIGHT_DEFAULTS[6],
    }

    class Control(IntEnum):
        vLeftJoint1 = 0
        vLeftJoint2 = 1
        vLeftJoint3 = 2
        vLeftJoint4 = 3
        vLeftJoint5 = 4
        vLeftJoint6 = 5
        vLeftJoint7 = 6

        vRightJoint1 = 7
        vRightJoint2 = 8
        vRightJoint3 = 9
        vRightJoint4 = 10
        vRightJoint5 = 11
        vRightJoint6 = 12
        vRightJoint7 = 13

    ControlLimit = {
        Control.vLeftJoint1: _AGIBOT_G1_ARM_CONTROL_LIMITS[0],
        Control.vLeftJoint2: _AGIBOT_G1_ARM_CONTROL_LIMITS[1],
        Control.vLeftJoint3: _AGIBOT_G1_ARM_CONTROL_LIMITS[2],
        Control.vLeftJoint4: _AGIBOT_G1_ARM_CONTROL_LIMITS[3],
        Control.vLeftJoint5: _AGIBOT_G1_ARM_CONTROL_LIMITS[4],
        Control.vLeftJoint6: _AGIBOT_G1_ARM_CONTROL_LIMITS[5],
        Control.vLeftJoint7: _AGIBOT_G1_ARM_CONTROL_LIMITS[6],
        Control.vRightJoint1: _AGIBOT_G1_ARM_CONTROL_LIMITS[0],
        Control.vRightJoint2: _AGIBOT_G1_ARM_CONTROL_LIMITS[1],
        Control.vRightJoint3: _AGIBOT_G1_ARM_CONTROL_LIMITS[2],
        Control.vRightJoint4: _AGIBOT_G1_ARM_CONTROL_LIMITS[3],
        Control.vRightJoint5: _AGIBOT_G1_ARM_CONTROL_LIMITS[4],
        Control.vRightJoint6: _AGIBOT_G1_ARM_CONTROL_LIMITS[5],
        Control.vRightJoint7: _AGIBOT_G1_ARM_CONTROL_LIMITS[6],
    }

    NormalControl = [
        Control.vLeftJoint1,
        Control.vLeftJoint2,
        Control.vLeftJoint3,
        Control.vLeftJoint4,
        Control.vLeftJoint5,
        Control.vLeftJoint6,
        Control.vLeftJoint7,
        Control.vRightJoint1,
        Control.vRightJoint2,
        Control.vRightJoint3,
        Control.vRightJoint4,
        Control.vRightJoint5,
        Control.vRightJoint6,
        Control.vRightJoint7,
    ]

    WeakControl = []
    DelicateControl = []

    @property
    def num_state(self):
        return int(len(self.DoFs))

    def compose_state_from_dof(self, dof_pos, dof_vel):
        return dof_pos.reshape(-1)

    def decompose_state_to_dof_pos(self, state):
        return state.reshape(-1)

    def decompose_state_to_dof_vel(self, state):
        return np.zeros(self.num_dof)

    def dynamics_f(self, state):
        return np.zeros((self.num_state, 1))

    def dynamics_g(self, state):
        return np.eye(self.num_state)

    class MujocoDoFs(IntEnum):
        LeftJoint1 = 2
        LeftJoint2 = 3
        LeftJoint3 = 4
        LeftJoint4 = 5
        LeftJoint5 = 6
        LeftJoint6 = 7
        LeftJoint7 = 8

        RightJoint1 = 17
        RightJoint2 = 18
        RightJoint3 = 19
        RightJoint4 = 20
        RightJoint5 = 21
        RightJoint6 = 22
        RightJoint7 = 23

    class MujocoMotors(IntEnum):
        LeftJoint1 = 2
        LeftJoint2 = 3
        LeftJoint3 = 4
        LeftJoint4 = 5
        LeftJoint5 = 6
        LeftJoint6 = 7
        LeftJoint7 = 8

        RightJoint1 = 9
        RightJoint2 = 10
        RightJoint3 = 11
        RightJoint4 = 12
        RightJoint5 = 13
        RightJoint6 = 14
        RightJoint7 = 15

    MujocoMotorKps = {
        MujocoMotors.LeftJoint1: _AGIBOT_G1_ARM_KPS[0],
        MujocoMotors.LeftJoint2: _AGIBOT_G1_ARM_KPS[1],
        MujocoMotors.LeftJoint3: _AGIBOT_G1_ARM_KPS[2],
        MujocoMotors.LeftJoint4: _AGIBOT_G1_ARM_KPS[3],
        MujocoMotors.LeftJoint5: _AGIBOT_G1_ARM_KPS[4],
        MujocoMotors.LeftJoint6: _AGIBOT_G1_ARM_KPS[5],
        MujocoMotors.LeftJoint7: _AGIBOT_G1_ARM_KPS[6],
        MujocoMotors.RightJoint1: _AGIBOT_G1_ARM_KPS[0],
        MujocoMotors.RightJoint2: _AGIBOT_G1_ARM_KPS[1],
        MujocoMotors.RightJoint3: _AGIBOT_G1_ARM_KPS[2],
        MujocoMotors.RightJoint4: _AGIBOT_G1_ARM_KPS[3],
        MujocoMotors.RightJoint5: _AGIBOT_G1_ARM_KPS[4],
        MujocoMotors.RightJoint6: _AGIBOT_G1_ARM_KPS[5],
        MujocoMotors.RightJoint7: _AGIBOT_G1_ARM_KPS[6],
    }

    MujocoMotorKds = {
        MujocoMotors.LeftJoint1: _AGIBOT_G1_ARM_KDS[0],
        MujocoMotors.LeftJoint2: _AGIBOT_G1_ARM_KDS[1],
        MujocoMotors.LeftJoint3: _AGIBOT_G1_ARM_KDS[2],
        MujocoMotors.LeftJoint4: _AGIBOT_G1_ARM_KDS[3],
        MujocoMotors.LeftJoint5: _AGIBOT_G1_ARM_KDS[4],
        MujocoMotors.LeftJoint6: _AGIBOT_G1_ARM_KDS[5],
        MujocoMotors.LeftJoint7: _AGIBOT_G1_ARM_KDS[6],
        MujocoMotors.RightJoint1: _AGIBOT_G1_ARM_KDS[0],
        MujocoMotors.RightJoint2: _AGIBOT_G1_ARM_KDS[1],
        MujocoMotors.RightJoint3: _AGIBOT_G1_ARM_KDS[2],
        MujocoMotors.RightJoint4: _AGIBOT_G1_ARM_KDS[3],
        MujocoMotors.RightJoint5: _AGIBOT_G1_ARM_KDS[4],
        MujocoMotors.RightJoint6: _AGIBOT_G1_ARM_KDS[5],
        MujocoMotors.RightJoint7: _AGIBOT_G1_ARM_KDS[6],
    }

    MujocoDoF_to_DoF = {
        MujocoDoFs.LeftJoint1: DoFs.LeftJoint1,
        MujocoDoFs.LeftJoint2: DoFs.LeftJoint2,
        MujocoDoFs.LeftJoint3: DoFs.LeftJoint3,
        MujocoDoFs.LeftJoint4: DoFs.LeftJoint4,
        MujocoDoFs.LeftJoint5: DoFs.LeftJoint5,
        MujocoDoFs.LeftJoint6: DoFs.LeftJoint6,
        MujocoDoFs.LeftJoint7: DoFs.LeftJoint7,
        MujocoDoFs.RightJoint1: DoFs.RightJoint1,
        MujocoDoFs.RightJoint2: DoFs.RightJoint2,
        MujocoDoFs.RightJoint3: DoFs.RightJoint3,
        MujocoDoFs.RightJoint4: DoFs.RightJoint4,
        MujocoDoFs.RightJoint5: DoFs.RightJoint5,
        MujocoDoFs.RightJoint6: DoFs.RightJoint6,
        MujocoDoFs.RightJoint7: DoFs.RightJoint7,
    }

    DoF_to_MujocoDoF = {
        DoFs.LeftJoint1: MujocoDoFs.LeftJoint1,
        DoFs.LeftJoint2: MujocoDoFs.LeftJoint2,
        DoFs.LeftJoint3: MujocoDoFs.LeftJoint3,
        DoFs.LeftJoint4: MujocoDoFs.LeftJoint4,
        DoFs.LeftJoint5: MujocoDoFs.LeftJoint5,
        DoFs.LeftJoint6: MujocoDoFs.LeftJoint6,
        DoFs.LeftJoint7: MujocoDoFs.LeftJoint7,
        DoFs.RightJoint1: MujocoDoFs.RightJoint1,
        DoFs.RightJoint2: MujocoDoFs.RightJoint2,
        DoFs.RightJoint3: MujocoDoFs.RightJoint3,
        DoFs.RightJoint4: MujocoDoFs.RightJoint4,
        DoFs.RightJoint5: MujocoDoFs.RightJoint5,
        DoFs.RightJoint6: MujocoDoFs.RightJoint6,
        DoFs.RightJoint7: MujocoDoFs.RightJoint7,
    }

    MujocoMotor_to_Control = {
        MujocoMotors.LeftJoint1: Control.vLeftJoint1,
        MujocoMotors.LeftJoint2: Control.vLeftJoint2,
        MujocoMotors.LeftJoint3: Control.vLeftJoint3,
        MujocoMotors.LeftJoint4: Control.vLeftJoint4,
        MujocoMotors.LeftJoint5: Control.vLeftJoint5,
        MujocoMotors.LeftJoint6: Control.vLeftJoint6,
        MujocoMotors.LeftJoint7: Control.vLeftJoint7,
        MujocoMotors.RightJoint1: Control.vRightJoint1,
        MujocoMotors.RightJoint2: Control.vRightJoint2,
        MujocoMotors.RightJoint3: Control.vRightJoint3,
        MujocoMotors.RightJoint4: Control.vRightJoint4,
        MujocoMotors.RightJoint5: Control.vRightJoint5,
        MujocoMotors.RightJoint6: Control.vRightJoint6,
        MujocoMotors.RightJoint7: Control.vRightJoint7,
    }

    class RealDoFs(IntEnum):
        LeftJoint1 = 2
        LeftJoint2 = 3
        LeftJoint3 = 4
        LeftJoint4 = 5
        LeftJoint5 = 6
        LeftJoint6 = 7
        LeftJoint7 = 8

        RightJoint1 = 9
        RightJoint2 = 10
        RightJoint3 = 11
        RightJoint4 = 12
        RightJoint5 = 13
        RightJoint6 = 14
        RightJoint7 = 15

    RealDoF_to_DoF = {
        RealDoFs.LeftJoint1: DoFs.LeftJoint1,
        RealDoFs.LeftJoint2: DoFs.LeftJoint2,
        RealDoFs.LeftJoint3: DoFs.LeftJoint3,
        RealDoFs.LeftJoint4: DoFs.LeftJoint4,
        RealDoFs.LeftJoint5: DoFs.LeftJoint5,
        RealDoFs.LeftJoint6: DoFs.LeftJoint6,
        RealDoFs.LeftJoint7: DoFs.LeftJoint7,
        RealDoFs.RightJoint1: DoFs.RightJoint1,
        RealDoFs.RightJoint2: DoFs.RightJoint2,
        RealDoFs.RightJoint3: DoFs.RightJoint3,
        RealDoFs.RightJoint4: DoFs.RightJoint4,
        RealDoFs.RightJoint5: DoFs.RightJoint5,
        RealDoFs.RightJoint6: DoFs.RightJoint6,
        RealDoFs.RightJoint7: DoFs.RightJoint7,
    }

    DoF_to_RealDoF = {
        DoFs.LeftJoint1: RealDoFs.LeftJoint1,
        DoFs.LeftJoint2: RealDoFs.LeftJoint2,
        DoFs.LeftJoint3: RealDoFs.LeftJoint3,
        DoFs.LeftJoint4: RealDoFs.LeftJoint4,
        DoFs.LeftJoint5: RealDoFs.LeftJoint5,
        DoFs.LeftJoint6: RealDoFs.LeftJoint6,
        DoFs.LeftJoint7: RealDoFs.LeftJoint7,
        DoFs.RightJoint1: RealDoFs.RightJoint1,
        DoFs.RightJoint2: RealDoFs.RightJoint2,
        DoFs.RightJoint3: RealDoFs.RightJoint3,
        DoFs.RightJoint4: RealDoFs.RightJoint4,
        DoFs.RightJoint5: RealDoFs.RightJoint5,
        DoFs.RightJoint6: RealDoFs.RightJoint6,
        DoFs.RightJoint7: RealDoFs.RightJoint7,
    }

    RealMotor_to_Control = {
        RealMotors.LeftJoint1: Control.vLeftJoint1,
        RealMotors.LeftJoint2: Control.vLeftJoint2,
        RealMotors.LeftJoint3: Control.vLeftJoint3,
        RealMotors.LeftJoint4: Control.vLeftJoint4,
        RealMotors.LeftJoint5: Control.vLeftJoint5,
        RealMotors.LeftJoint6: Control.vLeftJoint6,
        RealMotors.LeftJoint7: Control.vLeftJoint7,
        RealMotors.RightJoint1: Control.vRightJoint1,
        RealMotors.RightJoint2: Control.vRightJoint2,
        RealMotors.RightJoint3: Control.vRightJoint3,
        RealMotors.RightJoint4: Control.vRightJoint4,
        RealMotors.RightJoint5: Control.vRightJoint5,
        RealMotors.RightJoint6: Control.vRightJoint6,
        RealMotors.RightJoint7: Control.vRightJoint7,
    }

    Frames = AgiBotG1Frames
    CollisionVol = AGIBOT_G1_COLLISION_VOLUMES
    CollisionVolBodyNames = AGIBOT_G1_COLLISION_BODY_NAMES
    CollisionVolLocalOffsets = AGIBOT_G1_COLLISION_LOCAL_OFFSETS

    AdjacentCollisionVolPairs = []
    SelfCollisionVolIgnored = []
    EnvCollisionVolIgnored = []

    VisualizeSafeZone = [
        Frames.R_ee,
        Frames.L_ee,
    ]

    VisualizePhiTraj = [
        Frames.R_ee,
        Frames.L_ee,
    ]
