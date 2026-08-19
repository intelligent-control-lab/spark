from enum import IntEnum

import numpy as np

from spark_robot.agibot_g1.config.agibot_g1_dual_arm_dynamic_1_config import (
    _AGIBOT_G1_ARM_CONTROL_LIMITS,
    _AGIBOT_G1_ARM_KDS,
    _AGIBOT_G1_ARM_KPS,
    _AGIBOT_G1_GRIPPER_JOINTS,
    _AGIBOT_G1_LEFT_ARM_JOINTS,
    _AGIBOT_G1_RIGHT_DEFAULTS,
    _AGIBOT_G1_RIGHT_LIMITS,
    _AGIBOT_G1_TORSO_JOINTS,
)
from spark_robot.base.base_robot_config import RobotConfig
from .collision_geometry import (
    AgiBotG1Frames,
    AGIBOT_G1_COLLISION_BODY_NAMES,
    AGIBOT_G1_COLLISION_LOCAL_OFFSETS,
    AGIBOT_G1_COLLISION_VOLUMES,
)

from .isaac_articulation import (
    AGIBOT_G1_VISUAL_LINK_COLORS,
    agibot_g1_isaac_capability,
    RIGHT_ARM_ISAAC_ARTICULATION,
)
from .simulator_dynamics import AGIBOT_G1_SIMULATOR_DYNAMICS


class AgiBotG1RightArmDynamic1Config(RobotConfig):
    visual_link_colors = AGIBOT_G1_VISUAL_LINK_COLORS
    agent_class_names = {
        "mujoco": "AgiBotG1RightArmAgent",
        "isaac": "AgiBotG1RightArmIsaacAgent",
    }
    isaac_articulation = RIGHT_ARM_ISAAC_ARTICULATION
    simulator_dynamics = AGIBOT_G1_SIMULATOR_DYNAMICS
    backend_capability_overrides = {
        "isaac": agibot_g1_isaac_capability("AgiBotG1RightArmIsaacAgent")
    }
    dynamics_variant = "single_integrator"
    dynamics_order = 1
    dynamics_is_linear = True

    """AgiBot G1 fixed-base config exposing only the right arm."""

    kinematics_class_name = "AgiBotG1RightArmKinematics"
    mujoco_model_path = "agibot_g1/agibot_g1_right_arm.xml"
    joint_to_lock = (
        list(_AGIBOT_G1_GRIPPER_JOINTS)
        + list(_AGIBOT_G1_TORSO_JOINTS)
        + list(_AGIBOT_G1_LEFT_ARM_JOINTS)
    )

    NumTotalMotors = 16

    class RealMotors(IntEnum):
        RightJoint1 = 9
        RightJoint2 = 10
        RightJoint3 = 11
        RightJoint4 = 12
        RightJoint5 = 13
        RightJoint6 = 14
        RightJoint7 = 15

    RealMotorPosLimit = {
        RealMotors.RightJoint1: _AGIBOT_G1_RIGHT_LIMITS[0],
        RealMotors.RightJoint2: _AGIBOT_G1_RIGHT_LIMITS[1],
        RealMotors.RightJoint3: _AGIBOT_G1_RIGHT_LIMITS[2],
        RealMotors.RightJoint4: _AGIBOT_G1_RIGHT_LIMITS[3],
        RealMotors.RightJoint5: _AGIBOT_G1_RIGHT_LIMITS[4],
        RealMotors.RightJoint6: _AGIBOT_G1_RIGHT_LIMITS[5],
        RealMotors.RightJoint7: _AGIBOT_G1_RIGHT_LIMITS[6],
    }

    NormalMotor = [
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
        RightJoint1 = 0
        RightJoint2 = 1
        RightJoint3 = 2
        RightJoint4 = 3
        RightJoint5 = 4
        RightJoint6 = 5
        RightJoint7 = 6

    DefaultDoFVal = {
        DoFs.RightJoint1: _AGIBOT_G1_RIGHT_DEFAULTS[0],
        DoFs.RightJoint2: _AGIBOT_G1_RIGHT_DEFAULTS[1],
        DoFs.RightJoint3: _AGIBOT_G1_RIGHT_DEFAULTS[2],
        DoFs.RightJoint4: _AGIBOT_G1_RIGHT_DEFAULTS[3],
        DoFs.RightJoint5: _AGIBOT_G1_RIGHT_DEFAULTS[4],
        DoFs.RightJoint6: _AGIBOT_G1_RIGHT_DEFAULTS[5],
        DoFs.RightJoint7: _AGIBOT_G1_RIGHT_DEFAULTS[6],
    }

    class Control(IntEnum):
        vRightJoint1 = 0
        vRightJoint2 = 1
        vRightJoint3 = 2
        vRightJoint4 = 3
        vRightJoint5 = 4
        vRightJoint6 = 5
        vRightJoint7 = 6

    ControlLimit = {
        Control.vRightJoint1: _AGIBOT_G1_ARM_CONTROL_LIMITS[0],
        Control.vRightJoint2: _AGIBOT_G1_ARM_CONTROL_LIMITS[1],
        Control.vRightJoint3: _AGIBOT_G1_ARM_CONTROL_LIMITS[2],
        Control.vRightJoint4: _AGIBOT_G1_ARM_CONTROL_LIMITS[3],
        Control.vRightJoint5: _AGIBOT_G1_ARM_CONTROL_LIMITS[4],
        Control.vRightJoint6: _AGIBOT_G1_ARM_CONTROL_LIMITS[5],
        Control.vRightJoint7: _AGIBOT_G1_ARM_CONTROL_LIMITS[6],
    }

    NormalControl = [
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
        RightJoint1 = 17
        RightJoint2 = 18
        RightJoint3 = 19
        RightJoint4 = 20
        RightJoint5 = 21
        RightJoint6 = 22
        RightJoint7 = 23

    class MujocoMotors(IntEnum):
        RightJoint1 = 9
        RightJoint2 = 10
        RightJoint3 = 11
        RightJoint4 = 12
        RightJoint5 = 13
        RightJoint6 = 14
        RightJoint7 = 15

    MujocoMotorKps = {
        MujocoMotors.RightJoint1: _AGIBOT_G1_ARM_KPS[0],
        MujocoMotors.RightJoint2: _AGIBOT_G1_ARM_KPS[1],
        MujocoMotors.RightJoint3: _AGIBOT_G1_ARM_KPS[2],
        MujocoMotors.RightJoint4: _AGIBOT_G1_ARM_KPS[3],
        MujocoMotors.RightJoint5: _AGIBOT_G1_ARM_KPS[4],
        MujocoMotors.RightJoint6: _AGIBOT_G1_ARM_KPS[5],
        MujocoMotors.RightJoint7: _AGIBOT_G1_ARM_KPS[6],
    }

    MujocoMotorKds = {
        MujocoMotors.RightJoint1: _AGIBOT_G1_ARM_KDS[0],
        MujocoMotors.RightJoint2: _AGIBOT_G1_ARM_KDS[1],
        MujocoMotors.RightJoint3: _AGIBOT_G1_ARM_KDS[2],
        MujocoMotors.RightJoint4: _AGIBOT_G1_ARM_KDS[3],
        MujocoMotors.RightJoint5: _AGIBOT_G1_ARM_KDS[4],
        MujocoMotors.RightJoint6: _AGIBOT_G1_ARM_KDS[5],
        MujocoMotors.RightJoint7: _AGIBOT_G1_ARM_KDS[6],
    }

    MujocoDoF_to_DoF = {
        MujocoDoFs.RightJoint1: DoFs.RightJoint1,
        MujocoDoFs.RightJoint2: DoFs.RightJoint2,
        MujocoDoFs.RightJoint3: DoFs.RightJoint3,
        MujocoDoFs.RightJoint4: DoFs.RightJoint4,
        MujocoDoFs.RightJoint5: DoFs.RightJoint5,
        MujocoDoFs.RightJoint6: DoFs.RightJoint6,
        MujocoDoFs.RightJoint7: DoFs.RightJoint7,
    }

    DoF_to_MujocoDoF = {
        DoFs.RightJoint1: MujocoDoFs.RightJoint1,
        DoFs.RightJoint2: MujocoDoFs.RightJoint2,
        DoFs.RightJoint3: MujocoDoFs.RightJoint3,
        DoFs.RightJoint4: MujocoDoFs.RightJoint4,
        DoFs.RightJoint5: MujocoDoFs.RightJoint5,
        DoFs.RightJoint6: MujocoDoFs.RightJoint6,
        DoFs.RightJoint7: MujocoDoFs.RightJoint7,
    }

    MujocoMotor_to_Control = {
        MujocoMotors.RightJoint1: Control.vRightJoint1,
        MujocoMotors.RightJoint2: Control.vRightJoint2,
        MujocoMotors.RightJoint3: Control.vRightJoint3,
        MujocoMotors.RightJoint4: Control.vRightJoint4,
        MujocoMotors.RightJoint5: Control.vRightJoint5,
        MujocoMotors.RightJoint6: Control.vRightJoint6,
        MujocoMotors.RightJoint7: Control.vRightJoint7,
    }

    class RealDoFs(IntEnum):
        RightJoint1 = 9
        RightJoint2 = 10
        RightJoint3 = 11
        RightJoint4 = 12
        RightJoint5 = 13
        RightJoint6 = 14
        RightJoint7 = 15

    RealDoF_to_DoF = {
        RealDoFs.RightJoint1: DoFs.RightJoint1,
        RealDoFs.RightJoint2: DoFs.RightJoint2,
        RealDoFs.RightJoint3: DoFs.RightJoint3,
        RealDoFs.RightJoint4: DoFs.RightJoint4,
        RealDoFs.RightJoint5: DoFs.RightJoint5,
        RealDoFs.RightJoint6: DoFs.RightJoint6,
        RealDoFs.RightJoint7: DoFs.RightJoint7,
    }

    DoF_to_RealDoF = {
        DoFs.RightJoint1: RealDoFs.RightJoint1,
        DoFs.RightJoint2: RealDoFs.RightJoint2,
        DoFs.RightJoint3: RealDoFs.RightJoint3,
        DoFs.RightJoint4: RealDoFs.RightJoint4,
        DoFs.RightJoint5: RealDoFs.RightJoint5,
        DoFs.RightJoint6: RealDoFs.RightJoint6,
        DoFs.RightJoint7: RealDoFs.RightJoint7,
    }

    RealMotor_to_Control = {
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
    ]

    VisualizePhiTraj = [
        Frames.R_ee,
    ]
