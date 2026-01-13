from enum import IntEnum
from spark_robot.base.base_robot_config import RobotConfig
from spark_utils import Geometry, VizColor
import numpy as np

class Gen3SingleDynamic1Config(RobotConfig):
    
    # ---------------------------------------------------------------------------- #
    #                                      Kinematics                              #
    # ---------------------------------------------------------------------------- #
    
    kinematics_class_name = "Gen3SingleKinematics"
    mujoco_model_path = "gen3/gen3_single_scene.xml"
    collision_spheres_json_path = "gen3/config/gen3_single_collision_spheres.json"
    joint_to_lock = [
        # Lock the passive finger sliders to keep the kinematics 7-DoF.
        "right/right_driver_joint",
        "right/right_coupler_joint",
        "right/right_spring_link_joint",
        "right/right_follower_joint",
        "right/left_driver_joint",
        "right/left_coupler_joint",
        "right/left_spring_link_joint",
        "right/left_follower_joint",
    ]
    
    # ---------------------------------------------------------------------------- #
    #                                   hardware                                   #
    # ---------------------------------------------------------------------------- #
    
    NumTotalMotors = 7
    
    class RealMotors(IntEnum):

        ShoulderPan = 0
        ShoulderLift = 1
        Elbow = 2
        Wrist1 = 3
        Wrist2 = 4
        Wrist3 = 5
        WristRoll = 6

    # Joint limits taken from Kinova Gen3 public specification (rad).
    RealMotorPosLimit = {
        RealMotors.ShoulderPan: (-2.96, 2.96),
        RealMotors.ShoulderLift: (-2.05, 2.05),
        RealMotors.Elbow: (-2.96, 2.96),
        RealMotors.Wrist1: (-2.05, 2.05),
        RealMotors.Wrist2: (-2.96, 2.96),
        RealMotors.Wrist3: (-2.05, 2.05),
        RealMotors.WristRoll: (-2.96, 2.96),
    }
    
    NormalMotor = [
    RealMotors.ShoulderPan,
    RealMotors.ShoulderLift,
    RealMotors.Elbow,
    RealMotors.Wrist1,
    RealMotors.Wrist2,
    RealMotors.Wrist3,
    RealMotors.WristRoll,
    ]
    
    WeakMotor = []
    
    DelicateMotor = []
    
    # ---------------------------------------------------------------------------- #
    #                                      DoF                                     #
    # ---------------------------------------------------------------------------- #

    class DoFs(IntEnum):

        ShoulderPan = 0
        ShoulderLift = 1
        Elbow = 2
        Wrist1 = 3
        Wrist2 = 4
        Wrist3 = 5
        WristRoll = 6

    DefaultDoFVal = {
        DoFs.ShoulderPan: 0.0,
        DoFs.ShoulderLift: 0.0,
        DoFs.Elbow: 0.0,
        DoFs.Wrist1: 0.0,
        DoFs.Wrist2: 0.0,
        DoFs.Wrist3: 0.0,
        DoFs.WristRoll: 0.0,
    }

    # ---------------------------------------------------------------------------- #
    #                                   Dynamics                                   #
    # ---------------------------------------------------------------------------- #

    class Control(IntEnum):
        # Velocity control (rad/s)

        vShoulderPan = 0
        vShoulderLift = 1
        vElbow = 2
        vWrist1 = 3
        vWrist2 = 4
        vWrist3 = 5
        vWristRoll = 6

    ControlLimit = {
        Control.vShoulderPan: 2.17,
        Control.vShoulderLift: 1.74,
        Control.vElbow: 2.17,
        Control.vWrist1: 2.61,
        Control.vWrist2: 2.61,
        Control.vWrist3: 2.61,
        Control.vWristRoll: 2.88,
    }

    NormalControl = [
        Control.vShoulderPan,
        Control.vShoulderLift,
        Control.vElbow,
        Control.vWrist1,
        Control.vWrist2,
        Control.vWrist3,
        Control.vWristRoll,
    ]
    
    WeakControl = []
    
    DelicateControl = []

    '''
        x_dot = f(x) + g(x) * control

        For velocity control, f = 0, g = I
    '''

    @property
    def num_state(self):
        return int(len(self.DoFs))

    def compose_state_from_dof(self, dof_pos, dof_vel):
        '''
            dof_pos: [num_dof,]
        '''
        state = dof_pos.reshape(-1)
        return state

    def decompose_state_to_dof_pos(self, state):
        '''
            state: [num_state,]
            return: [num_dof,]
        '''
        dof_pos = state.reshape(-1)
        return dof_pos
    
    def decompose_state_to_dof_vel(self, state):
        '''
            state: [num_state,]
            return: [num_dof,]
        '''
        # First order dynamic state does not have velocity
        return np.zeros(self.num_dof)
    
    def dynamics_f(self, state):
        '''
            state: [num_state, 1]
            return: [num_state, 1]
        '''
        f_x = np.zeros((self.num_state, 1))
        return f_x

    def dynamics_g(self, state):
        '''
            state: [num_state, 1]
            return: [num_state, num_control]
        '''
        g_x = np.eye(self.num_state)
        return g_x

    # ---------------------------------------------------------------------------- #
    #                                    MuJoCo                                    #
    # ---------------------------------------------------------------------------- #
    class MujocoDoFs(IntEnum):

        ShoulderPan = 0
        ShoulderLift = 1
        Elbow = 2
        Wrist1 = 3
        Wrist2 = 4
        Wrist3 = 5
        WristRoll = 6

    class MujocoMotors(IntEnum):

        ShoulderPan = 0
        ShoulderLift = 1
        Elbow = 2
        Wrist1 = 3
        Wrist2 = 4
        Wrist3 = 5
        WristRoll = 6

    # ---------------------------------------------------------------------------- #
    #                                   Mujoco PID Parameters                      #
    # ---------------------------------------------------------------------------- #
    
    # Kp parameters for Mujoco Motors
    MujocoMotorKps = {
        MujocoMotors.ShoulderPan: 600.0,
        MujocoMotors.ShoulderLift: 600.0,
        MujocoMotors.Elbow: 600.0,
        MujocoMotors.Wrist1: 400.0,
        MujocoMotors.Wrist2: 400.0,
        MujocoMotors.Wrist3: 400.0,
        MujocoMotors.WristRoll: 200.0,
    }
    
    # Kd parameters for Mujoco Motors
    MujocoMotorKds = {
        MujocoMotors.ShoulderPan: 10.0,
        MujocoMotors.ShoulderLift: 10.0,
        MujocoMotors.Elbow: 10.0,
        MujocoMotors.Wrist1: 5.0,
        MujocoMotors.Wrist2: 5.0,
        MujocoMotors.Wrist3: 5.0,
        MujocoMotors.WristRoll: 2.0,
    }

    # ---------------------------------------------------------------------------- #
    #                                   Mujoco Mappings                            #
    # ---------------------------------------------------------------------------- #

    # Mapping from Mujoco DoFs to DoFs
    MujocoDoF_to_DoF = {
        MujocoDoFs.ShoulderPan: DoFs.ShoulderPan,
        MujocoDoFs.ShoulderLift: DoFs.ShoulderLift,
        MujocoDoFs.Elbow: DoFs.Elbow,
        MujocoDoFs.Wrist1: DoFs.Wrist1,
        MujocoDoFs.Wrist2: DoFs.Wrist2,
        MujocoDoFs.Wrist3: DoFs.Wrist3,
        MujocoDoFs.WristRoll: DoFs.WristRoll,
    }

    # Mapping from DoFs to Mujoco DoFs
    DoF_to_MujocoDoF = {
        DoFs.ShoulderPan: MujocoDoFs.ShoulderPan,
        DoFs.ShoulderLift: MujocoDoFs.ShoulderLift,
        DoFs.Elbow: MujocoDoFs.Elbow,
        DoFs.Wrist1: MujocoDoFs.Wrist1,
        DoFs.Wrist2: MujocoDoFs.Wrist2,
        DoFs.Wrist3: MujocoDoFs.Wrist3,
        DoFs.WristRoll: MujocoDoFs.WristRoll,
    }

    # Mapping from Mujoco Motors to Control
    MujocoMotor_to_Control = {
        MujocoMotors.ShoulderPan: Control.vShoulderPan,
        MujocoMotors.ShoulderLift: Control.vShoulderLift,
        MujocoMotors.Elbow: Control.vElbow,
        MujocoMotors.Wrist1: Control.vWrist1,
        MujocoMotors.Wrist2: Control.vWrist2,
        MujocoMotors.Wrist3: Control.vWrist3,
        MujocoMotors.WristRoll: Control.vWristRoll,
    }
    
    # ---------------------------------------------------------------------------- #
    #                                    Real                                    #
    # ---------------------------------------------------------------------------- #
    class RealDoFs(IntEnum):

        ShoulderPan = 0
        ShoulderLift = 1
        Elbow = 2
        Wrist1 = 3
        Wrist2 = 4
        Wrist3 = 5
        WristRoll = 6
    
    # ---------------------------------------------------------------------------- #
    #                                   Real Mappings                                   #
    # ---------------------------------------------------------------------------- #
    
    
    # Mapping from Real DoFs to DoFs
    RealDoF_to_DoF = {
        RealDoFs.ShoulderPan: DoFs.ShoulderPan,
        RealDoFs.ShoulderLift: DoFs.ShoulderLift,
        RealDoFs.Elbow: DoFs.Elbow,
        RealDoFs.Wrist1: DoFs.Wrist1,
        RealDoFs.Wrist2: DoFs.Wrist2,
        RealDoFs.Wrist3: DoFs.Wrist3,
        RealDoFs.WristRoll: DoFs.WristRoll,
    }

    # Mapping from DoFs to Real DoFs
    DoF_to_RealDoF = {
        DoFs.ShoulderPan: RealDoFs.ShoulderPan,
        DoFs.ShoulderLift: RealDoFs.ShoulderLift,
        DoFs.Elbow: RealDoFs.Elbow,
        DoFs.Wrist1: RealDoFs.Wrist1,
        DoFs.Wrist2: RealDoFs.Wrist2,
        DoFs.Wrist3: RealDoFs.Wrist3,
        DoFs.WristRoll: RealDoFs.WristRoll,
    }

    # Mapping from real motors to Control
    RealMotor_to_Control = {
        RealMotors.ShoulderPan: Control.vShoulderPan,
        RealMotors.ShoulderLift: Control.vShoulderLift,
        RealMotors.Elbow: Control.vElbow,
        RealMotors.Wrist1: Control.vWrist1,
        RealMotors.Wrist2: Control.vWrist2,
        RealMotors.Wrist3: Control.vWrist3,
        RealMotors.WristRoll: Control.vWristRoll,
    }

    # ---------------------------------------------------------------------------- #
    #                                   Cartesian                                  #
    # ---------------------------------------------------------------------------- #
    
    class Frames(IntEnum):

        R_ee = 0
    
    # ---------------------------------------------------------------------------- #
    #                                   Collision                                  #
    # ---------------------------------------------------------------------------- #

    CollisionVol = {
        Frames.R_ee: Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
    }

    # Pairs of adjacent joints to be ignored in collision checking
    AdjacentCollisionVolPairs = []

    SelfCollisionVolIgnored = []
    
    EnvCollisionVolIgnored = []

    VisualizeSafeZone = [
        Frames.R_ee,
    ]

    VisualizePhiTraj = [
        Frames.R_ee,
    ]