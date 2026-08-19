from enum import IntEnum
from spark_robot.base.base_robot_config import RobotConfig
from spark_robot.base.backend_capability import (
    BackendCapability,
    IsaacArticulationInstanceSpec,
    IsaacArticulationSpec,
    SimulatorDynamicsSpec,
)
from spark_utils import Geometry, VizColor
import numpy as np

from .isaac_articulation import KINOVA_DUAL_ARM_GRIPPERS


class KinovaGen3DualArmDynamic1Config(RobotConfig):
    simulator_dynamics = SimulatorDynamicsSpec(physics_dt=0.005, control_decimation=4)
    agent_class_names = {
        "mujoco": "KinovaGen3DualArmAgent",
        "isaac": "KinovaGen3DualArmIsaacAgent",
    }
    isaac_articulation = IsaacArticulationSpec(
        urdf_path="kinova_gen3/gen3.urdf",
        joint_names=tuple(
            f"{side}_joint_{index}" for side in ("right", "left") for index in range(1, 8)
        ),
        merge_fixed_joints=True,
        allow_self_collision=True,
        instances=(
            IsaacArticulationInstanceSpec("right", translation=(0.0, -0.35, 0.0)),
            IsaacArticulationInstanceSpec("left", translation=(0.0, 0.35, 0.0)),
        ),
        grippers=KINOVA_DUAL_ARM_GRIPPERS,
    )
    backend_capability_overrides = {
        "isaac": BackendCapability(
            "ConfiguredIsaacAgent",
            batched=True,
            rendering=True,
            tensor_io=True,
            validation="validated",
            notes="CUDA batched hold/tracking validated dual Gen3 articulation.",
        )
    }
    dynamics_variant = "single_integrator"
    dynamics_order = 1
    dynamics_is_linear = True

    # ---------------------------------------------------------------------------- #
    #                                      Kinematics                              #
    # ---------------------------------------------------------------------------- #

    kinematics_class_name = "KinovaGen3DualArmKinematics"
    mujoco_model_path = "kinova_gen3/gen3_2f85_dual.xml"
    joint_to_lock = [
        "right/right_driver_joint",
        "right/right_coupler_joint",
        "right/right_spring_link_joint",
        "right/right_follower_joint",
        "right/left_driver_joint",
        "right/left_coupler_joint",
        "right/left_spring_link_joint",
        "right/left_follower_joint",
        "left/right_driver_joint",
        "left/right_coupler_joint",
        "left/right_spring_link_joint",
        "left/right_follower_joint",
        "left/left_driver_joint",
        "left/left_coupler_joint",
        "left/left_spring_link_joint",
        "left/left_follower_joint",
    ]

    # ---------------------------------------------------------------------------- #
    #                                   hardware                                   #
    # ---------------------------------------------------------------------------- #

    NumTotalMotors = 14

    class RealMotors(IntEnum):
        RightShoulderPan = 0
        RightShoulderLift = 1
        RightElbow = 2
        RightWrist1 = 3
        RightWrist2 = 4
        RightWrist3 = 5
        RightWristRoll = 6

        LeftShoulderPan = 7
        LeftShoulderLift = 8
        LeftElbow = 9
        LeftWrist1 = 10
        LeftWrist2 = 11
        LeftWrist3 = 12
        LeftWristRoll = 13

    RealMotorPosLimit = {
        RealMotors.RightShoulderPan: (-2.96, 2.96),
        RealMotors.RightShoulderLift: (-2.05, 2.05),
        RealMotors.RightElbow: (-2.96, 2.96),
        RealMotors.RightWrist1: (-2.05, 2.05),
        RealMotors.RightWrist2: (-2.96, 2.96),
        RealMotors.RightWrist3: (-2.05, 2.05),
        RealMotors.RightWristRoll: (-2.96, 2.96),
        RealMotors.LeftShoulderPan: (-2.96, 2.96),
        RealMotors.LeftShoulderLift: (-2.05, 2.05),
        RealMotors.LeftElbow: (-2.96, 2.96),
        RealMotors.LeftWrist1: (-2.05, 2.05),
        RealMotors.LeftWrist2: (-2.96, 2.96),
        RealMotors.LeftWrist3: (-2.05, 2.05),
        RealMotors.LeftWristRoll: (-2.96, 2.96),
    }

    NormalMotor = [
        RealMotors.RightShoulderPan,
        RealMotors.RightShoulderLift,
        RealMotors.RightElbow,
        RealMotors.RightWrist1,
        RealMotors.RightWrist2,
        RealMotors.RightWrist3,
        RealMotors.RightWristRoll,
        RealMotors.LeftShoulderPan,
        RealMotors.LeftShoulderLift,
        RealMotors.LeftElbow,
        RealMotors.LeftWrist1,
        RealMotors.LeftWrist2,
        RealMotors.LeftWrist3,
        RealMotors.LeftWristRoll,
    ]

    WeakMotor = []

    DelicateMotor = []

    # ---------------------------------------------------------------------------- #
    #                                      DoF                                     #
    # ---------------------------------------------------------------------------- #

    class DoFs(IntEnum):
        RightShoulderPan = 0
        RightShoulderLift = 1
        RightElbow = 2
        RightWrist1 = 3
        RightWrist2 = 4
        RightWrist3 = 5
        RightWristRoll = 6

        LeftShoulderPan = 7
        LeftShoulderLift = 8
        LeftElbow = 9
        LeftWrist1 = 10
        LeftWrist2 = 11
        LeftWrist3 = 12
        LeftWristRoll = 13

    DefaultDoFVal = {
        DoFs.RightShoulderPan: -0.35,
        DoFs.RightShoulderLift: 0.5,
        DoFs.RightElbow: 0.0,
        DoFs.RightWrist1: -1.0,
        DoFs.RightWrist2: 0.0,
        DoFs.RightWrist3: 0.5,
        DoFs.RightWristRoll: 0.0,
        DoFs.LeftShoulderPan: 0.35,
        DoFs.LeftShoulderLift: 0.5,
        DoFs.LeftElbow: 0.0,
        DoFs.LeftWrist1: -1.0,
        DoFs.LeftWrist2: 0.0,
        DoFs.LeftWrist3: 0.5,
        DoFs.LeftWristRoll: 0.0,
    }

    # ---------------------------------------------------------------------------- #
    #                                   Dynamics                                   #
    # ---------------------------------------------------------------------------- #

    class Control(IntEnum):
        # Velocity control (rad/s)

        vRightShoulderPan = 0
        vRightShoulderLift = 1
        vRightElbow = 2
        vRightWrist1 = 3
        vRightWrist2 = 4
        vRightWrist3 = 5
        vRightWristRoll = 6

        vLeftShoulderPan = 7
        vLeftShoulderLift = 8
        vLeftElbow = 9
        vLeftWrist1 = 10
        vLeftWrist2 = 11
        vLeftWrist3 = 12
        vLeftWristRoll = 13

    ControlLimit = {
        Control.vRightShoulderPan: 2.17,
        Control.vRightShoulderLift: 1.74,
        Control.vRightElbow: 2.17,
        Control.vRightWrist1: 2.61,
        Control.vRightWrist2: 2.61,
        Control.vRightWrist3: 2.61,
        Control.vRightWristRoll: 2.88,
        Control.vLeftShoulderPan: 2.17,
        Control.vLeftShoulderLift: 1.74,
        Control.vLeftElbow: 2.17,
        Control.vLeftWrist1: 2.61,
        Control.vLeftWrist2: 2.61,
        Control.vLeftWrist3: 2.61,
        Control.vLeftWristRoll: 2.88,
    }

    NormalControl = [
        Control.vRightShoulderPan,
        Control.vRightShoulderLift,
        Control.vRightElbow,
        Control.vRightWrist1,
        Control.vRightWrist2,
        Control.vRightWrist3,
        Control.vRightWristRoll,
        Control.vLeftShoulderPan,
        Control.vLeftShoulderLift,
        Control.vLeftElbow,
        Control.vLeftWrist1,
        Control.vLeftWrist2,
        Control.vLeftWrist3,
        Control.vLeftWristRoll,
    ]

    WeakControl = []

    DelicateControl = []

    """
        x_dot = f(x) + g(x) * control

        For velocity control, f = 0, g = I
    """

    @property
    def num_state(self):
        return int(len(self.DoFs))

    def compose_state_from_dof(self, dof_pos, dof_vel):
        """
        dof_pos: [num_dof,]
        """
        state = dof_pos.reshape(-1)
        return state

    def decompose_state_to_dof_pos(self, state):
        """
        state: [num_state,]
        return: [num_dof,]
        """
        dof_pos = state.reshape(-1)
        return dof_pos

    def decompose_state_to_dof_vel(self, state):
        """
        state: [num_state,]
        return: [num_dof,]
        """
        return np.zeros(self.num_dof)

    def dynamics_f(self, state):
        """
        state: [num_state, 1]
        return: [num_state, 1]
        """
        f_x = np.zeros((self.num_state, 1))
        return f_x

    def dynamics_g(self, state):
        """
        state: [num_state, 1]
        return: [num_state, num_control]
        """
        g_x = np.eye(self.num_state)
        return g_x

    # ---------------------------------------------------------------------------- #
    #                                    MuJoCo                                    #
    # ---------------------------------------------------------------------------- #

    class MujocoDoFs(IntEnum):
        RightShoulderPan = 0
        RightShoulderLift = 1
        RightElbow = 2
        RightWrist1 = 3
        RightWrist2 = 4
        RightWrist3 = 5
        RightWristRoll = 6

        LeftShoulderPan = 15
        LeftShoulderLift = 16
        LeftElbow = 17
        LeftWrist1 = 18
        LeftWrist2 = 19
        LeftWrist3 = 20
        LeftWristRoll = 21

    class MujocoMotors(IntEnum):
        RightShoulderPan = 0
        RightShoulderLift = 1
        RightElbow = 2
        RightWrist1 = 3
        RightWrist2 = 4
        RightWrist3 = 5
        RightWristRoll = 6

        LeftShoulderPan = 8
        LeftShoulderLift = 9
        LeftElbow = 10
        LeftWrist1 = 11
        LeftWrist2 = 12
        LeftWrist3 = 13
        LeftWristRoll = 14

    # ---------------------------------------------------------------------------- #
    #                                   Mujoco PID Parameters                      #
    # ---------------------------------------------------------------------------- #

    MujocoMotorKps = {
        MujocoMotors.RightShoulderPan: 600.0,
        MujocoMotors.RightShoulderLift: 600.0,
        MujocoMotors.RightElbow: 600.0,
        MujocoMotors.RightWrist1: 400.0,
        MujocoMotors.RightWrist2: 400.0,
        MujocoMotors.RightWrist3: 400.0,
        MujocoMotors.RightWristRoll: 200.0,
        MujocoMotors.LeftShoulderPan: 600.0,
        MujocoMotors.LeftShoulderLift: 600.0,
        MujocoMotors.LeftElbow: 600.0,
        MujocoMotors.LeftWrist1: 400.0,
        MujocoMotors.LeftWrist2: 400.0,
        MujocoMotors.LeftWrist3: 400.0,
        MujocoMotors.LeftWristRoll: 200.0,
    }

    MujocoMotorKds = {
        MujocoMotors.RightShoulderPan: 10.0,
        MujocoMotors.RightShoulderLift: 10.0,
        MujocoMotors.RightElbow: 10.0,
        MujocoMotors.RightWrist1: 5.0,
        MujocoMotors.RightWrist2: 5.0,
        MujocoMotors.RightWrist3: 5.0,
        MujocoMotors.RightWristRoll: 2.0,
        MujocoMotors.LeftShoulderPan: 10.0,
        MujocoMotors.LeftShoulderLift: 10.0,
        MujocoMotors.LeftElbow: 10.0,
        MujocoMotors.LeftWrist1: 5.0,
        MujocoMotors.LeftWrist2: 5.0,
        MujocoMotors.LeftWrist3: 5.0,
        MujocoMotors.LeftWristRoll: 2.0,
    }

    # ---------------------------------------------------------------------------- #
    #                                   Mujoco Mappings                            #
    # ---------------------------------------------------------------------------- #

    MujocoDoF_to_DoF = {
        MujocoDoFs.RightShoulderPan: DoFs.RightShoulderPan,
        MujocoDoFs.RightShoulderLift: DoFs.RightShoulderLift,
        MujocoDoFs.RightElbow: DoFs.RightElbow,
        MujocoDoFs.RightWrist1: DoFs.RightWrist1,
        MujocoDoFs.RightWrist2: DoFs.RightWrist2,
        MujocoDoFs.RightWrist3: DoFs.RightWrist3,
        MujocoDoFs.RightWristRoll: DoFs.RightWristRoll,
        MujocoDoFs.LeftShoulderPan: DoFs.LeftShoulderPan,
        MujocoDoFs.LeftShoulderLift: DoFs.LeftShoulderLift,
        MujocoDoFs.LeftElbow: DoFs.LeftElbow,
        MujocoDoFs.LeftWrist1: DoFs.LeftWrist1,
        MujocoDoFs.LeftWrist2: DoFs.LeftWrist2,
        MujocoDoFs.LeftWrist3: DoFs.LeftWrist3,
        MujocoDoFs.LeftWristRoll: DoFs.LeftWristRoll,
    }

    DoF_to_MujocoDoF = {
        DoFs.RightShoulderPan: MujocoDoFs.RightShoulderPan,
        DoFs.RightShoulderLift: MujocoDoFs.RightShoulderLift,
        DoFs.RightElbow: MujocoDoFs.RightElbow,
        DoFs.RightWrist1: MujocoDoFs.RightWrist1,
        DoFs.RightWrist2: MujocoDoFs.RightWrist2,
        DoFs.RightWrist3: MujocoDoFs.RightWrist3,
        DoFs.RightWristRoll: MujocoDoFs.RightWristRoll,
        DoFs.LeftShoulderPan: MujocoDoFs.LeftShoulderPan,
        DoFs.LeftShoulderLift: MujocoDoFs.LeftShoulderLift,
        DoFs.LeftElbow: MujocoDoFs.LeftElbow,
        DoFs.LeftWrist1: MujocoDoFs.LeftWrist1,
        DoFs.LeftWrist2: MujocoDoFs.LeftWrist2,
        DoFs.LeftWrist3: MujocoDoFs.LeftWrist3,
        DoFs.LeftWristRoll: MujocoDoFs.LeftWristRoll,
    }

    MujocoMotor_to_Control = {
        MujocoMotors.RightShoulderPan: Control.vRightShoulderPan,
        MujocoMotors.RightShoulderLift: Control.vRightShoulderLift,
        MujocoMotors.RightElbow: Control.vRightElbow,
        MujocoMotors.RightWrist1: Control.vRightWrist1,
        MujocoMotors.RightWrist2: Control.vRightWrist2,
        MujocoMotors.RightWrist3: Control.vRightWrist3,
        MujocoMotors.RightWristRoll: Control.vRightWristRoll,
        MujocoMotors.LeftShoulderPan: Control.vLeftShoulderPan,
        MujocoMotors.LeftShoulderLift: Control.vLeftShoulderLift,
        MujocoMotors.LeftElbow: Control.vLeftElbow,
        MujocoMotors.LeftWrist1: Control.vLeftWrist1,
        MujocoMotors.LeftWrist2: Control.vLeftWrist2,
        MujocoMotors.LeftWrist3: Control.vLeftWrist3,
        MujocoMotors.LeftWristRoll: Control.vLeftWristRoll,
    }

    # ---------------------------------------------------------------------------- #
    #                                    Real                                      #
    # ---------------------------------------------------------------------------- #

    class RealDoFs(IntEnum):
        RightShoulderPan = 0
        RightShoulderLift = 1
        RightElbow = 2
        RightWrist1 = 3
        RightWrist2 = 4
        RightWrist3 = 5
        RightWristRoll = 6

        LeftShoulderPan = 7
        LeftShoulderLift = 8
        LeftElbow = 9
        LeftWrist1 = 10
        LeftWrist2 = 11
        LeftWrist3 = 12
        LeftWristRoll = 13

    # ---------------------------------------------------------------------------- #
    #                                   Real Mappings                              #
    # ---------------------------------------------------------------------------- #

    RealDoF_to_DoF = {
        RealDoFs.RightShoulderPan: DoFs.RightShoulderPan,
        RealDoFs.RightShoulderLift: DoFs.RightShoulderLift,
        RealDoFs.RightElbow: DoFs.RightElbow,
        RealDoFs.RightWrist1: DoFs.RightWrist1,
        RealDoFs.RightWrist2: DoFs.RightWrist2,
        RealDoFs.RightWrist3: DoFs.RightWrist3,
        RealDoFs.RightWristRoll: DoFs.RightWristRoll,
        RealDoFs.LeftShoulderPan: DoFs.LeftShoulderPan,
        RealDoFs.LeftShoulderLift: DoFs.LeftShoulderLift,
        RealDoFs.LeftElbow: DoFs.LeftElbow,
        RealDoFs.LeftWrist1: DoFs.LeftWrist1,
        RealDoFs.LeftWrist2: DoFs.LeftWrist2,
        RealDoFs.LeftWrist3: DoFs.LeftWrist3,
        RealDoFs.LeftWristRoll: DoFs.LeftWristRoll,
    }

    DoF_to_RealDoF = {
        DoFs.RightShoulderPan: RealDoFs.RightShoulderPan,
        DoFs.RightShoulderLift: RealDoFs.RightShoulderLift,
        DoFs.RightElbow: RealDoFs.RightElbow,
        DoFs.RightWrist1: RealDoFs.RightWrist1,
        DoFs.RightWrist2: RealDoFs.RightWrist2,
        DoFs.RightWrist3: RealDoFs.RightWrist3,
        DoFs.RightWristRoll: RealDoFs.RightWristRoll,
        DoFs.LeftShoulderPan: RealDoFs.LeftShoulderPan,
        DoFs.LeftShoulderLift: RealDoFs.LeftShoulderLift,
        DoFs.LeftElbow: RealDoFs.LeftElbow,
        DoFs.LeftWrist1: RealDoFs.LeftWrist1,
        DoFs.LeftWrist2: RealDoFs.LeftWrist2,
        DoFs.LeftWrist3: RealDoFs.LeftWrist3,
        DoFs.LeftWristRoll: RealDoFs.LeftWristRoll,
    }

    RealMotor_to_Control = {
        RealMotors.RightShoulderPan: Control.vRightShoulderPan,
        RealMotors.RightShoulderLift: Control.vRightShoulderLift,
        RealMotors.RightElbow: Control.vRightElbow,
        RealMotors.RightWrist1: Control.vRightWrist1,
        RealMotors.RightWrist2: Control.vRightWrist2,
        RealMotors.RightWrist3: Control.vRightWrist3,
        RealMotors.RightWristRoll: Control.vRightWristRoll,
        RealMotors.LeftShoulderPan: Control.vLeftShoulderPan,
        RealMotors.LeftShoulderLift: Control.vLeftShoulderLift,
        RealMotors.LeftElbow: Control.vLeftElbow,
        RealMotors.LeftWrist1: Control.vLeftWrist1,
        RealMotors.LeftWrist2: Control.vLeftWrist2,
        RealMotors.LeftWrist3: Control.vLeftWrist3,
        RealMotors.LeftWristRoll: Control.vLeftWristRoll,
    }

    # ---------------------------------------------------------------------------- #
    #                                   Cartesian                                  #
    # ---------------------------------------------------------------------------- #

    class Frames(IntEnum):
        R_ee = 0
        L_ee = 1

    # ---------------------------------------------------------------------------- #
    #                                   Collision                                  #
    # ---------------------------------------------------------------------------- #

    CollisionVol = {
        Frames.R_ee: Geometry(type="sphere", radius=0.05, color=VizColor.collision_volume),
        Frames.L_ee: Geometry(type="sphere", radius=0.05, color=VizColor.collision_volume),
    }

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
