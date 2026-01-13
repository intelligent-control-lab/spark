from enum import IntEnum
from spark_robot.base.base_robot_config import RobotConfig
from spark_utils import Geometry, VizColor
import numpy as np

class Gen3SingleDynamic2Config(RobotConfig):
    
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
        # Acceleration control

        aShoulderPan = 0
        aShoulderLift = 1
        aElbow = 2
        aWrist1 = 3
        aWrist2 = 4
        aWrist3 = 5
        aWristRoll = 6

    ControlLimit = {
        Control.aShoulderPan: 21.7,
        Control.aShoulderLift: 17.4,
        Control.aElbow: 21.7,
        Control.aWrist1: 26.1,
        Control.aWrist2: 26.1,
        Control.aWrist3: 26.1,
        Control.aWristRoll: 28.8,
    }

    NormalControl = [
        Control.aShoulderPan,
        Control.aShoulderLift,
        Control.aElbow,
        Control.aWrist1,
        Control.aWrist2,
        Control.aWrist3,
        Control.aWristRoll,
    ]
    
    WeakControl = []
    
    DelicateControl = []

    '''
        x_dot = f(x) + g(x) * control

        For acceleration control, 
        
        state = [position; velocity]
    '''

    @property
    def num_state(self):
        return int(len(self.DoFs) * 2) # pos, vel for double integrator dynamics.

    def compose_state_from_dof(self, dof_pos, dof_vel):
        '''
            dof_pos: [num_dof,]
        '''
        state = np.concatenate((dof_pos.reshape(-1), dof_vel.reshape(-1)), axis=0)
        return state

    def decompose_state_to_dof_pos(self, state):
        '''
            state: [num_state,]
            return: [num_dof,]
        '''
        dof_pos = state.reshape(-1)[:self.num_dof] # Take only the first half entries.
        return dof_pos
    
    def decompose_state_to_dof_vel(self, state):
        '''
            state: [num_state,]
            return: [num_dof,]
        '''
        dof_vel = state.reshape(-1)[self.num_dof:] # Take only the second half entries.
        return dof_vel

    def dynamics_f(self, state):
        '''
            state: [num_state, 1]
            return: [num_state, 1]
        '''
        positions = state[:self.num_dof]
        velocities = state[self.num_dof:]
  
        f_x = np.zeros_like(state)
        f_x[:self.num_dof] = velocities
        return f_x

    def dynamics_g(self, state):
        '''
            state: [num_state, 1]
            return: [num_state, num_control]
        '''
        g_x = np.zeros((2 * self.num_dof, self.num_dof))
        g_x[self.num_dof:] = np.eye(self.num_dof)

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
        MujocoMotors.ShoulderPan: Control.aShoulderPan,
        MujocoMotors.ShoulderLift: Control.aShoulderLift,
        MujocoMotors.Elbow: Control.aElbow,
        MujocoMotors.Wrist1: Control.aWrist1,
        MujocoMotors.Wrist2: Control.aWrist2,
        MujocoMotors.Wrist3: Control.aWrist3,
        MujocoMotors.WristRoll: Control.aWristRoll,
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
        RealMotors.ShoulderPan: Control.aShoulderPan,
        RealMotors.ShoulderLift: Control.aShoulderLift,
        RealMotors.Elbow: Control.aElbow,
        RealMotors.Wrist1: Control.aWrist1,
        RealMotors.Wrist2: Control.aWrist2,
        RealMotors.Wrist3: Control.aWrist3,
        RealMotors.WristRoll: Control.aWristRoll,
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