from enum import IntEnum

import numpy as np

from spark_robot.agibot_g1.config.agibot_g1_dual_arm_dynamic_1_config import (
    _AGIBOT_G1_ARM_CONTROL_LIMITS,
    _AGIBOT_G1_ARM_KDS,
    _AGIBOT_G1_ARM_KPS,
    _AGIBOT_G1_GRIPPER_JOINTS,
    _AGIBOT_G1_LEFT_DEFAULTS,
    _AGIBOT_G1_LEFT_LIMITS,
    _AGIBOT_G1_RIGHT_DEFAULTS,
    _AGIBOT_G1_RIGHT_LIMITS,
)
from spark_robot.base.base_robot_config import RobotConfig
from .collision_geometry import (
    AGIBOT_G1_COLLISION_BODY_NAMES,
    AGIBOT_G1_COLLISION_LOCAL_OFFSETS,
    AGIBOT_G1_COLLISION_VOLUMES,
    AgiBotG1Frames,
)

from .isaac_articulation import (
    AGIBOT_G1_VISUAL_LINK_COLORS,
    FIXED_BASE_ISAAC_ARTICULATION,
    agibot_g1_isaac_capability,
)
from .simulator_dynamics import AGIBOT_G1_SIMULATOR_DYNAMICS


class AgiBotG1FixedBaseDynamic1Config(RobotConfig):
    """Fixed-base AgiBot G1 config exposing vertical torso lift and both arms."""

    agent_class_names = {
        "mujoco": "AgiBotG1FixedBaseAgent",
        "isaac": "AgiBotG1FixedBaseIsaacAgent",
    }
    isaac_articulation = FIXED_BASE_ISAAC_ARTICULATION
    simulator_dynamics = AGIBOT_G1_SIMULATOR_DYNAMICS
    backend_capability_overrides = {
        "isaac": agibot_g1_isaac_capability("AgiBotG1FixedBaseIsaacAgent")
    }
    dynamics_variant = "single_integrator"
    dynamics_order = 1
    dynamics_is_linear = True

    kinematics_class_name = "AgiBotG1FixedBaseKinematics"
    mujoco_model_path = "agibot_g1/agibot_g1_fixed_base.xml"
    joint_to_lock = list(_AGIBOT_G1_GRIPPER_JOINTS) + ["joint_body_pitch"]

    NumTotalMotors = 16

    class RealMotors(IntEnum):
        LiftBody = 0

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
        RealMotors.LiftBody: (0.0, 0.5),
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
        RealMotors.LiftBody,
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
        LiftBody = 0

        LeftJoint1 = 1
        LeftJoint2 = 2
        LeftJoint3 = 3
        LeftJoint4 = 4
        LeftJoint5 = 5
        LeftJoint6 = 6
        LeftJoint7 = 7

        RightJoint1 = 8
        RightJoint2 = 9
        RightJoint3 = 10
        RightJoint4 = 11
        RightJoint5 = 12
        RightJoint6 = 13
        RightJoint7 = 14

    DefaultDoFVal = {
        DoFs.LiftBody: 0.25,
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
        vLiftBody = 0

        vLeftJoint1 = 1
        vLeftJoint2 = 2
        vLeftJoint3 = 3
        vLeftJoint4 = 4
        vLeftJoint5 = 5
        vLeftJoint6 = 6
        vLeftJoint7 = 7

        vRightJoint1 = 8
        vRightJoint2 = 9
        vRightJoint3 = 10
        vRightJoint4 = 11
        vRightJoint5 = 12
        vRightJoint6 = 13
        vRightJoint7 = 14

    ControlLimit = {
        Control.vLiftBody: 0.4,
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
        Control.vLiftBody,
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
        LiftBody = 0

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
        LiftBody = 0

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
        MujocoMotors.LiftBody: 400.0,
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
        # Near-critical response with the agent's 2x implicit damping scale.
        MujocoMotors.LiftBody: 80.0,
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
        MujocoDoFs.LiftBody: DoFs.LiftBody,
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
        DoFs.LiftBody: MujocoDoFs.LiftBody,
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
        MujocoMotors.LiftBody: Control.vLiftBody,
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
        LiftBody = 0

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
        RealDoFs.LiftBody: DoFs.LiftBody,
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
        DoFs.LiftBody: RealDoFs.LiftBody,
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
        RealMotors.LiftBody: Control.vLiftBody,
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
    visual_link_colors = AGIBOT_G1_VISUAL_LINK_COLORS
