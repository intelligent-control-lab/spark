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
    agibot_g1_isaac_capability,
    MOBILE_BASE_ISAAC_ARTICULATION,
)
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


class AgiBotG1MobileBaseDynamic1Config(RobotConfig):
    """AgiBot G1 mobile-base config."""

    agent_class_names = {
        "mujoco": "AgiBotG1MobileBaseAgent",
        "isaac": "AgiBotG1MobileBaseIsaacAgent",
    }
    isaac_articulation = MOBILE_BASE_ISAAC_ARTICULATION
    simulator_dynamics = AGIBOT_G1_SIMULATOR_DYNAMICS
    backend_capability_overrides = {
        "isaac": agibot_g1_isaac_capability("AgiBotG1MobileBaseIsaacAgent")
    }
    dynamics_variant = "single_integrator"
    dynamics_order = 1
    dynamics_is_linear = True

    # ---------------------------------------------------------------------------- #
    #                                      Kinematics                              #
    # ---------------------------------------------------------------------------- #

    kinematics_class_name = "AgiBotG1MobileBaseKinematics"
    mujoco_model_path = "agibot_g1/agibot_g1_mobile_base.xml"
    joint_to_lock = list(_AGIBOT_G1_GRIPPER_JOINTS)

    # ---------------------------------------------------------------------------- #
    #                                   hardware                                   #
    # ---------------------------------------------------------------------------- #

    NumTotalMotors = 16

    class RealMotors(IntEnum):
        LiftBody = 0
        BodyPitch = 1

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
        RealMotors.BodyPitch: (0.0, 1.047197551),
        RealMotors.LeftJoint1: (-3.14, 3.14),
        RealMotors.LeftJoint2: (-2.09, 2.09),
        RealMotors.LeftJoint3: (-3.14, 3.14),
        RealMotors.LeftJoint4: (-1.832595715, 1.483529864),
        RealMotors.LeftJoint5: (-3.14, 3.14),
        RealMotors.LeftJoint6: (-1.745329252, 1.745329252),
        RealMotors.LeftJoint7: (-3.14, 3.14),
        RealMotors.RightJoint1: (-3.14, 3.14),
        RealMotors.RightJoint2: (-2.09, 2.09),
        RealMotors.RightJoint3: (-3.14, 3.14),
        RealMotors.RightJoint4: (-1.483529864, 1.832595715),
        RealMotors.RightJoint5: (-3.14, 3.14),
        RealMotors.RightJoint6: (-1.745329252, 1.745329252),
        RealMotors.RightJoint7: (-3.14, 3.14),
    }

    NormalMotor = [
        RealMotors.LiftBody,
        RealMotors.BodyPitch,
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

    # ---------------------------------------------------------------------------- #
    #                                      DoF                                     #
    # ---------------------------------------------------------------------------- #

    class DoFs(IntEnum):
        LiftBody = 0
        BodyPitch = 1

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

        LinearX = 16
        LinearY = 17
        RotYaw = 18

    DefaultDoFVal = {
        DoFs.LiftBody: 0.25,
        DoFs.BodyPitch: 0.00217768,
        DoFs.LeftJoint1: -0.01408703,
        DoFs.LeftJoint2: 1.49600114,
        DoFs.LeftJoint3: 0.13254883,
        DoFs.LeftJoint4: -0.12675357,
        DoFs.LeftJoint5: 0.02853095,
        DoFs.LeftJoint6: 0.07266831,
        DoFs.LeftJoint7: 0.0,
        DoFs.RightJoint1: 0.01410166,
        DoFs.RightJoint2: -1.49600689,
        DoFs.RightJoint3: -0.13251528,
        DoFs.RightJoint4: 0.12672038,
        DoFs.RightJoint5: -0.02851918,
        DoFs.RightJoint6: -0.07263766,
        DoFs.RightJoint7: 0.0,
        DoFs.LinearX: 0.0,
        DoFs.LinearY: 0.0,
        DoFs.RotYaw: 0.0,
    }

    # ---------------------------------------------------------------------------- #
    #                                   Dynamics                                   #
    # ---------------------------------------------------------------------------- #

    class Control(IntEnum):
        vLiftBody = 0
        vBodyPitch = 1

        vLeftJoint1 = 2
        vLeftJoint2 = 3
        vLeftJoint3 = 4
        vLeftJoint4 = 5
        vLeftJoint5 = 6
        vLeftJoint6 = 7
        vLeftJoint7 = 8

        vRightJoint1 = 9
        vRightJoint2 = 10
        vRightJoint3 = 11
        vRightJoint4 = 12
        vRightJoint5 = 13
        vRightJoint6 = 14
        vRightJoint7 = 15

        vLinearX = 16
        vLinearY = 17
        vRotYaw = 18

    ControlLimit = {
        Control.vLiftBody: 0.4,
        Control.vBodyPitch: 0.8,
        Control.vLeftJoint1: 1.5,
        Control.vLeftJoint2: 1.5,
        Control.vLeftJoint3: 1.5,
        Control.vLeftJoint4: 1.5,
        Control.vLeftJoint5: 1.5,
        Control.vLeftJoint6: 1.5,
        Control.vLeftJoint7: 1.5,
        Control.vRightJoint1: 1.5,
        Control.vRightJoint2: 1.5,
        Control.vRightJoint3: 1.5,
        Control.vRightJoint4: 1.5,
        Control.vRightJoint5: 1.5,
        Control.vRightJoint6: 1.5,
        Control.vRightJoint7: 1.5,
        Control.vLinearX: 0.3,
        Control.vLinearY: 0.3,
        Control.vRotYaw: 0.3,
    }

    NormalControl = [
        Control.vLiftBody,
        Control.vBodyPitch,
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
        Control.vLinearX,
        Control.vLinearY,
        Control.vRotYaw,
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
        state = np.asarray(state, dtype=np.float64).reshape(-1)
        g_x = np.zeros((self.num_state, len(self.Control)))

        g_x[self.DoFs.LiftBody, self.Control.vLiftBody] = 1.0
        g_x[self.DoFs.BodyPitch, self.Control.vBodyPitch] = 1.0
        g_x[self.DoFs.LeftJoint1, self.Control.vLeftJoint1] = 1.0
        g_x[self.DoFs.LeftJoint2, self.Control.vLeftJoint2] = 1.0
        g_x[self.DoFs.LeftJoint3, self.Control.vLeftJoint3] = 1.0
        g_x[self.DoFs.LeftJoint4, self.Control.vLeftJoint4] = 1.0
        g_x[self.DoFs.LeftJoint5, self.Control.vLeftJoint5] = 1.0
        g_x[self.DoFs.LeftJoint6, self.Control.vLeftJoint6] = 1.0
        g_x[self.DoFs.LeftJoint7, self.Control.vLeftJoint7] = 1.0
        g_x[self.DoFs.RightJoint1, self.Control.vRightJoint1] = 1.0
        g_x[self.DoFs.RightJoint2, self.Control.vRightJoint2] = 1.0
        g_x[self.DoFs.RightJoint3, self.Control.vRightJoint3] = 1.0
        g_x[self.DoFs.RightJoint4, self.Control.vRightJoint4] = 1.0
        g_x[self.DoFs.RightJoint5, self.Control.vRightJoint5] = 1.0
        g_x[self.DoFs.RightJoint6, self.Control.vRightJoint6] = 1.0
        g_x[self.DoFs.RightJoint7, self.Control.vRightJoint7] = 1.0

        rot_yaw = state[self.DoFs.RotYaw]
        g_x[self.DoFs.LinearX, self.Control.vLinearX] = np.cos(rot_yaw)
        g_x[self.DoFs.LinearY, self.Control.vLinearX] = np.sin(rot_yaw)
        g_x[self.DoFs.LinearX, self.Control.vLinearY] = -np.sin(rot_yaw)
        g_x[self.DoFs.LinearY, self.Control.vLinearY] = np.cos(rot_yaw)
        g_x[self.DoFs.RotYaw, self.Control.vRotYaw] = 1.0

        return g_x

    # ---------------------------------------------------------------------------- #
    #                                    MuJoCo                                    #
    # ---------------------------------------------------------------------------- #

    class MujocoDoFs(IntEnum):
        LinearX = 0
        LinearY = 1
        RotYaw = 2

        LiftBody = 3
        BodyPitch = 4

        LeftJoint1 = 5
        LeftJoint2 = 6
        LeftJoint3 = 7
        LeftJoint4 = 8
        LeftJoint5 = 9
        LeftJoint6 = 10
        LeftJoint7 = 11

        RightJoint1 = 20
        RightJoint2 = 21
        RightJoint3 = 22
        RightJoint4 = 23
        RightJoint5 = 24
        RightJoint6 = 25
        RightJoint7 = 26

    class MujocoMotors(IntEnum):
        LinearX = 0
        LinearY = 1
        RotYaw = 2

        LiftBody = 3
        BodyPitch = 4

        LeftJoint1 = 5
        LeftJoint2 = 6
        LeftJoint3 = 7
        LeftJoint4 = 8
        LeftJoint5 = 9
        LeftJoint6 = 10
        LeftJoint7 = 11

        RightJoint1 = 12
        RightJoint2 = 13
        RightJoint3 = 14
        RightJoint4 = 15
        RightJoint5 = 16
        RightJoint6 = 17
        RightJoint7 = 18

    MujocoMotorKps = {
        MujocoMotors.LinearX: 200.0,
        MujocoMotors.LinearY: 200.0,
        MujocoMotors.RotYaw: 200.0,
        MujocoMotors.LiftBody: 400.0,
        MujocoMotors.BodyPitch: 250.0,
        MujocoMotors.LeftJoint1: 120.0,
        MujocoMotors.LeftJoint2: 120.0,
        MujocoMotors.LeftJoint3: 90.0,
        MujocoMotors.LeftJoint4: 90.0,
        MujocoMotors.LeftJoint5: 50.0,
        MujocoMotors.LeftJoint6: 40.0,
        MujocoMotors.LeftJoint7: 30.0,
        MujocoMotors.RightJoint1: 120.0,
        MujocoMotors.RightJoint2: 120.0,
        MujocoMotors.RightJoint3: 90.0,
        MujocoMotors.RightJoint4: 90.0,
        MujocoMotors.RightJoint5: 50.0,
        MujocoMotors.RightJoint6: 40.0,
        MujocoMotors.RightJoint7: 30.0,
    }

    MujocoMotorKds = {
        MujocoMotors.LinearX: 100.0,
        MujocoMotors.LinearY: 100.0,
        MujocoMotors.RotYaw: 100.0,
        # Near-critical response with the agent's 2x implicit damping scale.
        MujocoMotors.LiftBody: 80.0,
        MujocoMotors.BodyPitch: 6.0,
        MujocoMotors.LeftJoint1: 3.0,
        MujocoMotors.LeftJoint2: 3.0,
        MujocoMotors.LeftJoint3: 2.0,
        MujocoMotors.LeftJoint4: 2.0,
        MujocoMotors.LeftJoint5: 1.5,
        MujocoMotors.LeftJoint6: 1.0,
        MujocoMotors.LeftJoint7: 0.8,
        MujocoMotors.RightJoint1: 3.0,
        MujocoMotors.RightJoint2: 3.0,
        MujocoMotors.RightJoint3: 2.0,
        MujocoMotors.RightJoint4: 2.0,
        MujocoMotors.RightJoint5: 1.5,
        MujocoMotors.RightJoint6: 1.0,
        MujocoMotors.RightJoint7: 0.8,
    }

    # ---------------------------------------------------------------------------- #
    #                                   Mujoco Mappings                            #
    # ---------------------------------------------------------------------------- #

    MujocoDoF_to_DoF = {
        MujocoDoFs.LinearX: DoFs.LinearX,
        MujocoDoFs.LinearY: DoFs.LinearY,
        MujocoDoFs.RotYaw: DoFs.RotYaw,
        MujocoDoFs.LiftBody: DoFs.LiftBody,
        MujocoDoFs.BodyPitch: DoFs.BodyPitch,
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
        DoFs.BodyPitch: MujocoDoFs.BodyPitch,
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
        DoFs.LinearX: MujocoDoFs.LinearX,
        DoFs.LinearY: MujocoDoFs.LinearY,
        DoFs.RotYaw: MujocoDoFs.RotYaw,
    }

    MujocoMotor_to_Control = {
        MujocoMotors.LinearX: Control.vLinearX,
        MujocoMotors.LinearY: Control.vLinearY,
        MujocoMotors.RotYaw: Control.vRotYaw,
        MujocoMotors.LiftBody: Control.vLiftBody,
        MujocoMotors.BodyPitch: Control.vBodyPitch,
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

    # ---------------------------------------------------------------------------- #
    #                                    Real                                      #
    # ---------------------------------------------------------------------------- #

    class RealDoFs(IntEnum):
        LiftBody = 0
        BodyPitch = 1

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

        LinearX = 16
        LinearY = 17
        RotYaw = 18

    RealDoF_to_DoF = {
        RealDoFs.LiftBody: DoFs.LiftBody,
        RealDoFs.BodyPitch: DoFs.BodyPitch,
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
        RealDoFs.LinearX: DoFs.LinearX,
        RealDoFs.LinearY: DoFs.LinearY,
        RealDoFs.RotYaw: DoFs.RotYaw,
    }

    DoF_to_RealDoF = {
        DoFs.LiftBody: RealDoFs.LiftBody,
        DoFs.BodyPitch: RealDoFs.BodyPitch,
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
        DoFs.LinearX: RealDoFs.LinearX,
        DoFs.LinearY: RealDoFs.LinearY,
        DoFs.RotYaw: RealDoFs.RotYaw,
    }

    RealMotor_to_Control = {
        RealMotors.LiftBody: Control.vLiftBody,
        RealMotors.BodyPitch: Control.vBodyPitch,
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

    # ---------------------------------------------------------------------------- #
    #                                   Cartesian                                  #
    # ---------------------------------------------------------------------------- #

    Frames = AgiBotG1Frames

    # ---------------------------------------------------------------------------- #
    #                                   Collision                                  #
    # ---------------------------------------------------------------------------- #

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
