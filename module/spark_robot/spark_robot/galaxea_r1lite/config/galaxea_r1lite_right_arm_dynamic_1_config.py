from enum import IntEnum

import numpy as np
from spark_robot.base.base_robot_config import RobotConfig
from spark_utils import Geometry

from .isaac_articulation import ISAAC_CAPABILITY, RIGHT_ARM_ISAAC_ARTICULATION
from .simulator_dynamics import R1_LITE_SIMULATOR_DYNAMICS


class GalaxeaR1LiteRightArmDynamic1Config(RobotConfig):
    agent_class_names = {
        "mujoco": "GalaxeaR1LiteRightArmAgent",
        "isaac": "GalaxeaR1LiteRightArmIsaacAgent",
    }
    isaac_articulation = RIGHT_ARM_ISAAC_ARTICULATION
    simulator_dynamics = R1_LITE_SIMULATOR_DYNAMICS
    backend_capability_overrides = {"isaac": ISAAC_CAPABILITY}
    dynamics_variant = "single_integrator"
    dynamics_order = 1
    dynamics_is_linear = True

    # ---------------------------------------------------------------------------- #
    #                                      Kinematics                              #
    # ---------------------------------------------------------------------------- #

    kinematics_class_name = "GalaxeaR1LiteRightArmKinematics"
    mujoco_model_path = "galaxea_r1lite/r1lite_dual_arm.xml"
    collision_spheres_json_path = "galaxea_r1lite/config/galaxea_r1lite_collision_spheres.json"
    joint_to_lock = [
        "left_arm_joint1",
        "left_arm_joint2",
        "left_arm_joint3",
        "left_arm_joint4",
        "left_arm_joint5",
        "left_arm_joint6",
        "left_gripper_finger_joint1",
        "left_gripper_finger_joint2",
        "right_gripper_finger_joint1",
        "right_gripper_finger_joint2",
    ]

    # ---------------------------------------------------------------------------- #
    #                                   hardware                                   #
    # ---------------------------------------------------------------------------- #

    NumTotalMotors = 25

    class RealMotors(IntEnum):
        RightJoint1 = 0
        RightJoint2 = 1
        RightJoint3 = 2
        RightJoint4 = 3
        RightJoint5 = 4
        RightJoint6 = 5

    RealMotorPosLimit = {}

    NormalMotor = [
        RealMotors.RightJoint1,
        RealMotors.RightJoint2,
        RealMotors.RightJoint3,
        RealMotors.RightJoint4,
        RealMotors.RightJoint5,
        RealMotors.RightJoint6,
    ]

    WeakMotor = []
    DelicateMotor = []

    # ---------------------------------------------------------------------------- #
    #                                      DoF                                     #
    # ---------------------------------------------------------------------------- #

    class DoFs(IntEnum):
        RightJoint1 = 0
        RightJoint2 = 1
        RightJoint3 = 2
        RightJoint4 = 3
        RightJoint5 = 4
        RightJoint6 = 5

    DefaultDoFVal = {
        DoFs.RightJoint1: 0.0,
        DoFs.RightJoint2: 0.0,
        DoFs.RightJoint3: 0.0,
        DoFs.RightJoint4: 0.0,
        DoFs.RightJoint5: 0.0,
        DoFs.RightJoint6: 0.0,
    }

    # ---------------------------------------------------------------------------- #
    #                                   Dynamics                                   #
    # ---------------------------------------------------------------------------- #

    class Control(IntEnum):
        vRightJoint1 = 0
        vRightJoint2 = 1
        vRightJoint3 = 2
        vRightJoint4 = 3
        vRightJoint5 = 4
        vRightJoint6 = 5

    ControlLimit = {
        Control.vRightJoint1: 1,
        Control.vRightJoint2: 1,
        Control.vRightJoint3: 1,
        Control.vRightJoint4: 1,
        Control.vRightJoint5: 1,
        Control.vRightJoint6: 1,
    }

    NormalControl = [
        Control.vRightJoint1,
        Control.vRightJoint2,
        Control.vRightJoint3,
        Control.vRightJoint4,
        Control.vRightJoint5,
        Control.vRightJoint6,
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

    # ---------------------------------------------------------------------------- #
    #                                    MuJoCo                                    #
    # ---------------------------------------------------------------------------- #

    class MujocoDoFs(IntEnum):
        RightJoint1 = 8
        RightJoint2 = 9
        RightJoint3 = 10
        RightJoint4 = 11
        RightJoint5 = 12
        RightJoint6 = 13

    class MujocoMotors(IntEnum):
        RightJoint1 = 8
        RightJoint2 = 9
        RightJoint3 = 10
        RightJoint4 = 11
        RightJoint5 = 12
        RightJoint6 = 13

    MujocoMotorKps = {
        MujocoMotors.RightJoint1: 3000.0,
        MujocoMotors.RightJoint2: 3000.0,
        MujocoMotors.RightJoint3: 3000.0,
        MujocoMotors.RightJoint4: 3000.0,
        MujocoMotors.RightJoint5: 3000.0,
        MujocoMotors.RightJoint6: 3000.0,
    }

    MujocoMotorKds = {
        MujocoMotors.RightJoint1: 0.1,
        MujocoMotors.RightJoint2: 0.1,
        MujocoMotors.RightJoint3: 0.1,
        MujocoMotors.RightJoint4: 0.1,
        MujocoMotors.RightJoint5: 0.1,
        MujocoMotors.RightJoint6: 0.1,
    }

    MujocoDoF_to_DoF = {
        MujocoDoFs.RightJoint1: DoFs.RightJoint1,
        MujocoDoFs.RightJoint2: DoFs.RightJoint2,
        MujocoDoFs.RightJoint3: DoFs.RightJoint3,
        MujocoDoFs.RightJoint4: DoFs.RightJoint4,
        MujocoDoFs.RightJoint5: DoFs.RightJoint5,
        MujocoDoFs.RightJoint6: DoFs.RightJoint6,
    }

    DoF_to_MujocoDoF = {
        DoFs.RightJoint1: MujocoDoFs.RightJoint1,
        DoFs.RightJoint2: MujocoDoFs.RightJoint2,
        DoFs.RightJoint3: MujocoDoFs.RightJoint3,
        DoFs.RightJoint4: MujocoDoFs.RightJoint4,
        DoFs.RightJoint5: MujocoDoFs.RightJoint5,
        DoFs.RightJoint6: MujocoDoFs.RightJoint6,
    }

    MujocoMotor_to_Control = {
        MujocoMotors.RightJoint1: Control.vRightJoint1,
        MujocoMotors.RightJoint2: Control.vRightJoint2,
        MujocoMotors.RightJoint3: Control.vRightJoint3,
        MujocoMotors.RightJoint4: Control.vRightJoint4,
        MujocoMotors.RightJoint5: Control.vRightJoint5,
        MujocoMotors.RightJoint6: Control.vRightJoint6,
    }

    # ---------------------------------------------------------------------------- #
    #                                      Real                                    #
    # ---------------------------------------------------------------------------- #

    class RealDoFs(IntEnum):
        RightJoint1 = 0
        RightJoint2 = 1
        RightJoint3 = 2
        RightJoint4 = 3
        RightJoint5 = 4
        RightJoint6 = 5

    RealDoF_to_DoF = {
        RealDoFs.RightJoint1: DoFs.RightJoint1,
        RealDoFs.RightJoint2: DoFs.RightJoint2,
        RealDoFs.RightJoint3: DoFs.RightJoint3,
        RealDoFs.RightJoint4: DoFs.RightJoint4,
        RealDoFs.RightJoint5: DoFs.RightJoint5,
        RealDoFs.RightJoint6: DoFs.RightJoint6,
    }

    DoF_to_RealDoF = {
        DoFs.RightJoint1: RealDoFs.RightJoint1,
        DoFs.RightJoint2: RealDoFs.RightJoint2,
        DoFs.RightJoint3: RealDoFs.RightJoint3,
        DoFs.RightJoint4: RealDoFs.RightJoint4,
        DoFs.RightJoint5: RealDoFs.RightJoint5,
        DoFs.RightJoint6: RealDoFs.RightJoint6,
    }

    RealMotor_to_Control = {
        RealMotors.RightJoint1: Control.vRightJoint1,
        RealMotors.RightJoint2: Control.vRightJoint2,
        RealMotors.RightJoint3: Control.vRightJoint3,
        RealMotors.RightJoint4: Control.vRightJoint4,
        RealMotors.RightJoint5: Control.vRightJoint5,
        RealMotors.RightJoint6: Control.vRightJoint6,
    }

    # ---------------------------------------------------------------------------- #
    #                                   Cartesian                                  #
    # ---------------------------------------------------------------------------- #

    class Frames(IntEnum):
        R_ee = 0

    CollisionVol = {
        Frames.R_ee: Geometry(type="sphere", radius=0.05),
    }
    CollisionVolBodyNames = {Frames.R_ee: "right_gripper_link"}
    # R_ee is 0.15 m beyond joint 6. The URDF gripper body origin is already
    # 0.08165 m beyond that joint, so Isaac only needs the remaining offset.
    CollisionVolLocalOffsets = {Frames.R_ee: (0.06835, 0.0, 0.0)}

    AdjacentCollisionVolPairs = []
    SelfCollisionVolIgnored = []
    EnvCollisionVolIgnored = []
    VisualizeSafeZone = [Frames.R_ee]
    VisualizePhiTraj = [Frames.R_ee]
