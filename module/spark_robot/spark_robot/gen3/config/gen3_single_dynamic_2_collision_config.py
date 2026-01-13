# AUTO-GENERATED FILE
# Base config: Gen3SingleDynamic1Config
# Collision JSON: module/spark_robot/spark_robot/gen3/config/gen3_single_collision_spheres.json
# JSON hash: 095e8b1a63d4f110
# DO NOT EDIT MANUALLY

from enum import IntEnum
from spark_robot.base.base_robot_config import RobotConfig
from spark_utils import Geometry, VizColor
import numpy as np

class Gen3SingleDynamic2CollisionConfig(RobotConfig):
    
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

        base_link_sphere_0 = 1
        base_link_sphere_1 = 2
        shoulder_link_sphere_0 = 3
        shoulder_link_sphere_1 = 4
        shoulder_link_sphere_2 = 5
        shoulder_link_sphere_3 = 6
        half_arm_1_link_sphere_0 = 7
        half_arm_1_link_sphere_1 = 8
        half_arm_1_link_sphere_2 = 9
        half_arm_1_link_sphere_3 = 10
        half_arm_1_link_sphere_4 = 11
        half_arm_1_link_sphere_5 = 12
        half_arm_1_link_sphere_6 = 13
        half_arm_1_link_sphere_7 = 14
        half_arm_2_link_sphere_0 = 15
        half_arm_2_link_sphere_1 = 16
        half_arm_2_link_sphere_2 = 17
        half_arm_2_link_sphere_3 = 18
        half_arm_2_link_sphere_4 = 19
        half_arm_2_link_sphere_5 = 20
        half_arm_2_link_sphere_6 = 21
        half_arm_2_link_sphere_7 = 22
        forearm_link_sphere_0 = 23
        forearm_link_sphere_1 = 24
        forearm_link_sphere_2 = 25
        forearm_link_sphere_3 = 26
        forearm_link_sphere_4 = 27
        forearm_link_sphere_5 = 28
        forearm_link_sphere_6 = 29
        forearm_link_sphere_7 = 30
        spherical_wrist_1_link_sphere_0 = 31
        spherical_wrist_1_link_sphere_1 = 32
        spherical_wrist_1_link_sphere_2 = 33
        spherical_wrist_1_link_sphere_3 = 34
        spherical_wrist_1_link_sphere_4 = 35
        spherical_wrist_2_link_sphere_0 = 36
        spherical_wrist_2_link_sphere_1 = 37
        spherical_wrist_2_link_sphere_2 = 38
        spherical_wrist_2_link_sphere_3 = 39
        spherical_wrist_2_link_sphere_4 = 40
        spherical_wrist_2_link_sphere_5 = 41
        bracelet_link_sphere_0 = 42
        bracelet_link_sphere_1 = 43
        bracelet_link_sphere_2 = 44
        bracelet_link_sphere_3 = 45
        bracelet_link_sphere_4 = 46
    CollisionVol = {
        Frames.R_ee: Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        Frames.base_link_sphere_0: Geometry(type='sphere', radius=0.080037),
        Frames.base_link_sphere_1: Geometry(type='sphere', radius=0.071739),
        Frames.shoulder_link_sphere_0: Geometry(type='sphere', radius=0.061053),
        Frames.shoulder_link_sphere_1: Geometry(type='sphere', radius=0.060361),
        Frames.shoulder_link_sphere_2: Geometry(type='sphere', radius=0.055802),
        Frames.shoulder_link_sphere_3: Geometry(type='sphere', radius=0.058253),
        Frames.half_arm_1_link_sphere_0: Geometry(type='sphere', radius=0.054874),
        Frames.half_arm_1_link_sphere_1: Geometry(type='sphere', radius=0.055329),
        Frames.half_arm_1_link_sphere_2: Geometry(type='sphere', radius=0.050521),
        Frames.half_arm_1_link_sphere_3: Geometry(type='sphere', radius=0.051374),
        Frames.half_arm_1_link_sphere_4: Geometry(type='sphere', radius=0.050334),
        Frames.half_arm_1_link_sphere_5: Geometry(type='sphere', radius=0.0504),
        Frames.half_arm_1_link_sphere_6: Geometry(type='sphere', radius=0.044832),
        Frames.half_arm_1_link_sphere_7: Geometry(type='sphere', radius=0.046377),
        Frames.half_arm_2_link_sphere_0: Geometry(type='sphere', radius=0.061837),
        Frames.half_arm_2_link_sphere_1: Geometry(type='sphere', radius=0.049257),
        Frames.half_arm_2_link_sphere_2: Geometry(type='sphere', radius=0.048429),
        Frames.half_arm_2_link_sphere_3: Geometry(type='sphere', radius=0.024134),
        Frames.half_arm_2_link_sphere_4: Geometry(type='sphere', radius=0.048485),
        Frames.half_arm_2_link_sphere_5: Geometry(type='sphere', radius=0.049863),
        Frames.half_arm_2_link_sphere_6: Geometry(type='sphere', radius=0.047139),
        Frames.half_arm_2_link_sphere_7: Geometry(type='sphere', radius=0.054377),
        Frames.forearm_link_sphere_0: Geometry(type='sphere', radius=0.049546),
        Frames.forearm_link_sphere_1: Geometry(type='sphere', radius=0.052442),
        Frames.forearm_link_sphere_2: Geometry(type='sphere', radius=0.052202),
        Frames.forearm_link_sphere_3: Geometry(type='sphere', radius=0.044807),
        Frames.forearm_link_sphere_4: Geometry(type='sphere', radius=0.048266),
        Frames.forearm_link_sphere_5: Geometry(type='sphere', radius=0.039198),
        Frames.forearm_link_sphere_6: Geometry(type='sphere', radius=0.044241),
        Frames.forearm_link_sphere_7: Geometry(type='sphere', radius=0.043936),
        Frames.spherical_wrist_1_link_sphere_0: Geometry(type='sphere', radius=0.050826),
        Frames.spherical_wrist_1_link_sphere_1: Geometry(type='sphere', radius=0.046045),
        Frames.spherical_wrist_1_link_sphere_2: Geometry(type='sphere', radius=0.047032),
        Frames.spherical_wrist_1_link_sphere_3: Geometry(type='sphere', radius=0.043681),
        Frames.spherical_wrist_1_link_sphere_4: Geometry(type='sphere', radius=0.036668),
        Frames.spherical_wrist_2_link_sphere_0: Geometry(type='sphere', radius=0.04593),
        Frames.spherical_wrist_2_link_sphere_1: Geometry(type='sphere', radius=0.037153),
        Frames.spherical_wrist_2_link_sphere_2: Geometry(type='sphere', radius=0.036297),
        Frames.spherical_wrist_2_link_sphere_3: Geometry(type='sphere', radius=0.049708),
        Frames.spherical_wrist_2_link_sphere_4: Geometry(type='sphere', radius=0.04297),
        Frames.spherical_wrist_2_link_sphere_5: Geometry(type='sphere', radius=0.042011),
        Frames.bracelet_link_sphere_0: Geometry(type='sphere', radius=0.038217),
        Frames.bracelet_link_sphere_1: Geometry(type='sphere', radius=0.046985),
        Frames.bracelet_link_sphere_2: Geometry(type='sphere', radius=0.046999),
        Frames.bracelet_link_sphere_3: Geometry(type='sphere', radius=0.041845),
        Frames.bracelet_link_sphere_4: Geometry(type='sphere', radius=0.043868),
    }

    # Pairs of adjacent joints to be ignored in collision checking
    AdjacentCollisionVolPairs = [
        [Frames.R_ee, Frames.spherical_wrist_2_link_sphere_4],
        [Frames.R_ee, Frames.spherical_wrist_2_link_sphere_5],
        [Frames.R_ee, Frames.bracelet_link_sphere_0],
        [Frames.R_ee, Frames.bracelet_link_sphere_1],
        [Frames.R_ee, Frames.bracelet_link_sphere_2],
        [Frames.R_ee, Frames.bracelet_link_sphere_3],
        [Frames.R_ee, Frames.bracelet_link_sphere_4],
        [Frames.base_link_sphere_0, Frames.base_link_sphere_1],
        [Frames.base_link_sphere_0, Frames.shoulder_link_sphere_2],
        [Frames.base_link_sphere_1, Frames.shoulder_link_sphere_0],
        [Frames.base_link_sphere_1, Frames.shoulder_link_sphere_1],
        [Frames.base_link_sphere_1, Frames.shoulder_link_sphere_2],
        [Frames.shoulder_link_sphere_0, Frames.shoulder_link_sphere_1],
        [Frames.shoulder_link_sphere_0, Frames.shoulder_link_sphere_2],
        [Frames.shoulder_link_sphere_0, Frames.shoulder_link_sphere_3],
        [Frames.shoulder_link_sphere_0, Frames.half_arm_1_link_sphere_1],
        [Frames.shoulder_link_sphere_0, Frames.half_arm_1_link_sphere_2],
        [Frames.shoulder_link_sphere_1, Frames.shoulder_link_sphere_2],
        [Frames.shoulder_link_sphere_1, Frames.shoulder_link_sphere_3],
        [Frames.shoulder_link_sphere_1, Frames.half_arm_1_link_sphere_1],
        [Frames.shoulder_link_sphere_1, Frames.half_arm_1_link_sphere_2],
        [Frames.shoulder_link_sphere_2, Frames.shoulder_link_sphere_3],
        [Frames.shoulder_link_sphere_2, Frames.half_arm_1_link_sphere_1],
        [Frames.shoulder_link_sphere_3, Frames.half_arm_1_link_sphere_0],
        [Frames.shoulder_link_sphere_3, Frames.half_arm_1_link_sphere_1],
        [Frames.shoulder_link_sphere_3, Frames.half_arm_1_link_sphere_2],
        [Frames.shoulder_link_sphere_3, Frames.half_arm_1_link_sphere_3],
        [Frames.half_arm_1_link_sphere_0, Frames.half_arm_1_link_sphere_1],
        [Frames.half_arm_1_link_sphere_0, Frames.half_arm_1_link_sphere_2],
        [Frames.half_arm_1_link_sphere_0, Frames.half_arm_1_link_sphere_3],
        [Frames.half_arm_1_link_sphere_0, Frames.half_arm_1_link_sphere_4],
        [Frames.half_arm_1_link_sphere_0, Frames.half_arm_1_link_sphere_5],
        [Frames.half_arm_1_link_sphere_0, Frames.half_arm_1_link_sphere_6],
        [Frames.half_arm_1_link_sphere_0, Frames.half_arm_1_link_sphere_7],
        [Frames.half_arm_1_link_sphere_1, Frames.half_arm_1_link_sphere_2],
        [Frames.half_arm_1_link_sphere_1, Frames.half_arm_1_link_sphere_3],
        [Frames.half_arm_1_link_sphere_1, Frames.half_arm_1_link_sphere_4],
        [Frames.half_arm_1_link_sphere_1, Frames.half_arm_1_link_sphere_5],
        [Frames.half_arm_1_link_sphere_1, Frames.half_arm_1_link_sphere_6],
        [Frames.half_arm_1_link_sphere_1, Frames.half_arm_1_link_sphere_7],
        [Frames.half_arm_1_link_sphere_2, Frames.half_arm_1_link_sphere_3],
        [Frames.half_arm_1_link_sphere_2, Frames.half_arm_1_link_sphere_4],
        [Frames.half_arm_1_link_sphere_2, Frames.half_arm_1_link_sphere_5],
        [Frames.half_arm_1_link_sphere_2, Frames.half_arm_1_link_sphere_6],
        [Frames.half_arm_1_link_sphere_2, Frames.half_arm_1_link_sphere_7],
        [Frames.half_arm_1_link_sphere_3, Frames.half_arm_1_link_sphere_4],
        [Frames.half_arm_1_link_sphere_3, Frames.half_arm_1_link_sphere_5],
        [Frames.half_arm_1_link_sphere_3, Frames.half_arm_1_link_sphere_6],
        [Frames.half_arm_1_link_sphere_3, Frames.half_arm_1_link_sphere_7],
        [Frames.half_arm_1_link_sphere_4, Frames.half_arm_1_link_sphere_5],
        [Frames.half_arm_1_link_sphere_4, Frames.half_arm_1_link_sphere_6],
        [Frames.half_arm_1_link_sphere_4, Frames.half_arm_1_link_sphere_7],
        [Frames.half_arm_1_link_sphere_5, Frames.half_arm_1_link_sphere_6],
        [Frames.half_arm_1_link_sphere_5, Frames.half_arm_1_link_sphere_7],
        [Frames.half_arm_1_link_sphere_5, Frames.half_arm_2_link_sphere_1],
        [Frames.half_arm_1_link_sphere_5, Frames.half_arm_2_link_sphere_6],
        [Frames.half_arm_1_link_sphere_6, Frames.half_arm_1_link_sphere_7],
        [Frames.half_arm_1_link_sphere_6, Frames.half_arm_2_link_sphere_1],
        [Frames.half_arm_1_link_sphere_7, Frames.half_arm_2_link_sphere_1],
        [Frames.half_arm_1_link_sphere_7, Frames.half_arm_2_link_sphere_6],
        [Frames.half_arm_2_link_sphere_0, Frames.half_arm_2_link_sphere_1],
        [Frames.half_arm_2_link_sphere_0, Frames.half_arm_2_link_sphere_2],
        [Frames.half_arm_2_link_sphere_0, Frames.half_arm_2_link_sphere_3],
        [Frames.half_arm_2_link_sphere_0, Frames.half_arm_2_link_sphere_4],
        [Frames.half_arm_2_link_sphere_0, Frames.half_arm_2_link_sphere_5],
        [Frames.half_arm_2_link_sphere_0, Frames.half_arm_2_link_sphere_6],
        [Frames.half_arm_2_link_sphere_0, Frames.half_arm_2_link_sphere_7],
        [Frames.half_arm_2_link_sphere_0, Frames.forearm_link_sphere_0],
        [Frames.half_arm_2_link_sphere_0, Frames.forearm_link_sphere_1],
        [Frames.half_arm_2_link_sphere_0, Frames.forearm_link_sphere_2],
        [Frames.half_arm_2_link_sphere_0, Frames.forearm_link_sphere_3],
        [Frames.half_arm_2_link_sphere_1, Frames.half_arm_2_link_sphere_2],
        [Frames.half_arm_2_link_sphere_1, Frames.half_arm_2_link_sphere_3],
        [Frames.half_arm_2_link_sphere_1, Frames.half_arm_2_link_sphere_4],
        [Frames.half_arm_2_link_sphere_1, Frames.half_arm_2_link_sphere_5],
        [Frames.half_arm_2_link_sphere_1, Frames.half_arm_2_link_sphere_6],
        [Frames.half_arm_2_link_sphere_1, Frames.half_arm_2_link_sphere_7],
        [Frames.half_arm_2_link_sphere_2, Frames.half_arm_2_link_sphere_3],
        [Frames.half_arm_2_link_sphere_2, Frames.half_arm_2_link_sphere_4],
        [Frames.half_arm_2_link_sphere_2, Frames.half_arm_2_link_sphere_5],
        [Frames.half_arm_2_link_sphere_2, Frames.half_arm_2_link_sphere_6],
        [Frames.half_arm_2_link_sphere_2, Frames.half_arm_2_link_sphere_7],
        [Frames.half_arm_2_link_sphere_2, Frames.forearm_link_sphere_1],
        [Frames.half_arm_2_link_sphere_2, Frames.forearm_link_sphere_2],
        [Frames.half_arm_2_link_sphere_2, Frames.forearm_link_sphere_3],
        [Frames.half_arm_2_link_sphere_3, Frames.half_arm_2_link_sphere_4],
        [Frames.half_arm_2_link_sphere_3, Frames.half_arm_2_link_sphere_5],
        [Frames.half_arm_2_link_sphere_3, Frames.half_arm_2_link_sphere_6],
        [Frames.half_arm_2_link_sphere_3, Frames.half_arm_2_link_sphere_7],
        [Frames.half_arm_2_link_sphere_3, Frames.forearm_link_sphere_1],
        [Frames.half_arm_2_link_sphere_3, Frames.forearm_link_sphere_2],
        [Frames.half_arm_2_link_sphere_3, Frames.forearm_link_sphere_3],
        [Frames.half_arm_2_link_sphere_4, Frames.half_arm_2_link_sphere_5],
        [Frames.half_arm_2_link_sphere_4, Frames.half_arm_2_link_sphere_6],
        [Frames.half_arm_2_link_sphere_4, Frames.half_arm_2_link_sphere_7],
        [Frames.half_arm_2_link_sphere_4, Frames.forearm_link_sphere_1],
        [Frames.half_arm_2_link_sphere_4, Frames.forearm_link_sphere_2],
        [Frames.half_arm_2_link_sphere_4, Frames.forearm_link_sphere_3],
        [Frames.half_arm_2_link_sphere_5, Frames.half_arm_2_link_sphere_6],
        [Frames.half_arm_2_link_sphere_5, Frames.half_arm_2_link_sphere_7],
        [Frames.half_arm_2_link_sphere_5, Frames.forearm_link_sphere_0],
        [Frames.half_arm_2_link_sphere_5, Frames.forearm_link_sphere_1],
        [Frames.half_arm_2_link_sphere_5, Frames.forearm_link_sphere_2],
        [Frames.half_arm_2_link_sphere_5, Frames.forearm_link_sphere_3],
        [Frames.half_arm_2_link_sphere_5, Frames.forearm_link_sphere_4],
        [Frames.half_arm_2_link_sphere_6, Frames.half_arm_2_link_sphere_7],
        [Frames.forearm_link_sphere_0, Frames.forearm_link_sphere_1],
        [Frames.forearm_link_sphere_0, Frames.forearm_link_sphere_2],
        [Frames.forearm_link_sphere_0, Frames.forearm_link_sphere_3],
        [Frames.forearm_link_sphere_0, Frames.forearm_link_sphere_4],
        [Frames.forearm_link_sphere_0, Frames.forearm_link_sphere_5],
        [Frames.forearm_link_sphere_0, Frames.forearm_link_sphere_6],
        [Frames.forearm_link_sphere_0, Frames.forearm_link_sphere_7],
        [Frames.forearm_link_sphere_1, Frames.forearm_link_sphere_2],
        [Frames.forearm_link_sphere_1, Frames.forearm_link_sphere_3],
        [Frames.forearm_link_sphere_1, Frames.forearm_link_sphere_4],
        [Frames.forearm_link_sphere_1, Frames.forearm_link_sphere_5],
        [Frames.forearm_link_sphere_1, Frames.forearm_link_sphere_6],
        [Frames.forearm_link_sphere_1, Frames.forearm_link_sphere_7],
        [Frames.forearm_link_sphere_2, Frames.forearm_link_sphere_3],
        [Frames.forearm_link_sphere_2, Frames.forearm_link_sphere_4],
        [Frames.forearm_link_sphere_2, Frames.forearm_link_sphere_5],
        [Frames.forearm_link_sphere_2, Frames.forearm_link_sphere_6],
        [Frames.forearm_link_sphere_2, Frames.forearm_link_sphere_7],
        [Frames.forearm_link_sphere_3, Frames.forearm_link_sphere_4],
        [Frames.forearm_link_sphere_3, Frames.forearm_link_sphere_5],
        [Frames.forearm_link_sphere_3, Frames.forearm_link_sphere_6],
        [Frames.forearm_link_sphere_3, Frames.forearm_link_sphere_7],
        [Frames.forearm_link_sphere_4, Frames.forearm_link_sphere_5],
        [Frames.forearm_link_sphere_4, Frames.forearm_link_sphere_6],
        [Frames.forearm_link_sphere_4, Frames.forearm_link_sphere_7],
        [Frames.forearm_link_sphere_5, Frames.forearm_link_sphere_6],
        [Frames.forearm_link_sphere_5, Frames.forearm_link_sphere_7],
        [Frames.forearm_link_sphere_6, Frames.forearm_link_sphere_7],
        [Frames.forearm_link_sphere_6, Frames.spherical_wrist_1_link_sphere_0],
        [Frames.forearm_link_sphere_6, Frames.spherical_wrist_1_link_sphere_1],
        [Frames.forearm_link_sphere_7, Frames.spherical_wrist_1_link_sphere_0],
        [Frames.forearm_link_sphere_7, Frames.spherical_wrist_1_link_sphere_1],
        [Frames.spherical_wrist_1_link_sphere_0, Frames.spherical_wrist_1_link_sphere_1],
        [Frames.spherical_wrist_1_link_sphere_0, Frames.spherical_wrist_1_link_sphere_2],
        [Frames.spherical_wrist_1_link_sphere_0, Frames.spherical_wrist_1_link_sphere_3],
        [Frames.spherical_wrist_1_link_sphere_0, Frames.spherical_wrist_1_link_sphere_4],
        [Frames.spherical_wrist_1_link_sphere_0, Frames.spherical_wrist_2_link_sphere_0],
        [Frames.spherical_wrist_1_link_sphere_0, Frames.spherical_wrist_2_link_sphere_1],
        [Frames.spherical_wrist_1_link_sphere_0, Frames.spherical_wrist_2_link_sphere_2],
        [Frames.spherical_wrist_1_link_sphere_0, Frames.spherical_wrist_2_link_sphere_3],
        [Frames.spherical_wrist_1_link_sphere_0, Frames.spherical_wrist_2_link_sphere_4],
        [Frames.spherical_wrist_1_link_sphere_1, Frames.spherical_wrist_1_link_sphere_2],
        [Frames.spherical_wrist_1_link_sphere_1, Frames.spherical_wrist_1_link_sphere_3],
        [Frames.spherical_wrist_1_link_sphere_1, Frames.spherical_wrist_1_link_sphere_4],
        [Frames.spherical_wrist_1_link_sphere_1, Frames.spherical_wrist_2_link_sphere_0],
        [Frames.spherical_wrist_1_link_sphere_1, Frames.spherical_wrist_2_link_sphere_1],
        [Frames.spherical_wrist_1_link_sphere_2, Frames.spherical_wrist_1_link_sphere_3],
        [Frames.spherical_wrist_1_link_sphere_2, Frames.spherical_wrist_1_link_sphere_4],
        [Frames.spherical_wrist_1_link_sphere_2, Frames.spherical_wrist_2_link_sphere_0],
        [Frames.spherical_wrist_1_link_sphere_2, Frames.spherical_wrist_2_link_sphere_1],
        [Frames.spherical_wrist_1_link_sphere_2, Frames.spherical_wrist_2_link_sphere_2],
        [Frames.spherical_wrist_1_link_sphere_2, Frames.spherical_wrist_2_link_sphere_3],
        [Frames.spherical_wrist_1_link_sphere_3, Frames.spherical_wrist_1_link_sphere_4],
        [Frames.spherical_wrist_1_link_sphere_3, Frames.spherical_wrist_2_link_sphere_0],
        [Frames.spherical_wrist_1_link_sphere_3, Frames.spherical_wrist_2_link_sphere_1],
        [Frames.spherical_wrist_1_link_sphere_3, Frames.spherical_wrist_2_link_sphere_2],
        [Frames.spherical_wrist_1_link_sphere_3, Frames.spherical_wrist_2_link_sphere_3],
        [Frames.spherical_wrist_1_link_sphere_4, Frames.spherical_wrist_2_link_sphere_0],
        [Frames.spherical_wrist_1_link_sphere_4, Frames.spherical_wrist_2_link_sphere_1],
        [Frames.spherical_wrist_1_link_sphere_4, Frames.spherical_wrist_2_link_sphere_2],
        [Frames.spherical_wrist_1_link_sphere_4, Frames.spherical_wrist_2_link_sphere_3],
        [Frames.spherical_wrist_2_link_sphere_0, Frames.spherical_wrist_2_link_sphere_1],
        [Frames.spherical_wrist_2_link_sphere_0, Frames.spherical_wrist_2_link_sphere_2],
        [Frames.spherical_wrist_2_link_sphere_0, Frames.spherical_wrist_2_link_sphere_3],
        [Frames.spherical_wrist_2_link_sphere_0, Frames.spherical_wrist_2_link_sphere_4],
        [Frames.spherical_wrist_2_link_sphere_0, Frames.spherical_wrist_2_link_sphere_5],
        [Frames.spherical_wrist_2_link_sphere_1, Frames.spherical_wrist_2_link_sphere_2],
        [Frames.spherical_wrist_2_link_sphere_1, Frames.spherical_wrist_2_link_sphere_3],
        [Frames.spherical_wrist_2_link_sphere_1, Frames.spherical_wrist_2_link_sphere_4],
        [Frames.spherical_wrist_2_link_sphere_1, Frames.spherical_wrist_2_link_sphere_5],
        [Frames.spherical_wrist_2_link_sphere_2, Frames.spherical_wrist_2_link_sphere_3],
        [Frames.spherical_wrist_2_link_sphere_2, Frames.spherical_wrist_2_link_sphere_4],
        [Frames.spherical_wrist_2_link_sphere_2, Frames.spherical_wrist_2_link_sphere_5],
        [Frames.spherical_wrist_2_link_sphere_3, Frames.spherical_wrist_2_link_sphere_4],
        [Frames.spherical_wrist_2_link_sphere_3, Frames.spherical_wrist_2_link_sphere_5],
        [Frames.spherical_wrist_2_link_sphere_3, Frames.bracelet_link_sphere_2],
        [Frames.spherical_wrist_2_link_sphere_3, Frames.bracelet_link_sphere_3],
        [Frames.spherical_wrist_2_link_sphere_4, Frames.spherical_wrist_2_link_sphere_5],
        [Frames.spherical_wrist_2_link_sphere_4, Frames.bracelet_link_sphere_0],
        [Frames.spherical_wrist_2_link_sphere_4, Frames.bracelet_link_sphere_2],
        [Frames.spherical_wrist_2_link_sphere_4, Frames.bracelet_link_sphere_3],
        [Frames.spherical_wrist_2_link_sphere_5, Frames.bracelet_link_sphere_0],
        [Frames.spherical_wrist_2_link_sphere_5, Frames.bracelet_link_sphere_1],
        [Frames.spherical_wrist_2_link_sphere_5, Frames.bracelet_link_sphere_2],
        [Frames.spherical_wrist_2_link_sphere_5, Frames.bracelet_link_sphere_3],
        [Frames.spherical_wrist_2_link_sphere_5, Frames.bracelet_link_sphere_4],
        [Frames.bracelet_link_sphere_0, Frames.bracelet_link_sphere_1],
        [Frames.bracelet_link_sphere_0, Frames.bracelet_link_sphere_2],
        [Frames.bracelet_link_sphere_0, Frames.bracelet_link_sphere_3],
        [Frames.bracelet_link_sphere_0, Frames.bracelet_link_sphere_4],
        [Frames.bracelet_link_sphere_1, Frames.bracelet_link_sphere_2],
        [Frames.bracelet_link_sphere_1, Frames.bracelet_link_sphere_3],
        [Frames.bracelet_link_sphere_1, Frames.bracelet_link_sphere_4],
        [Frames.bracelet_link_sphere_2, Frames.bracelet_link_sphere_3],
        [Frames.bracelet_link_sphere_2, Frames.bracelet_link_sphere_4],
        [Frames.bracelet_link_sphere_3, Frames.bracelet_link_sphere_4],
        [Frames.half_arm_2_link_sphere_4, Frames.forearm_link_sphere_0],
        [Frames.half_arm_2_link_sphere_4, Frames.forearm_link_sphere_4],
        [Frames.spherical_wrist_1_link_sphere_4, Frames.spherical_wrist_2_link_sphere_4],
        [Frames.shoulder_link_sphere_0, Frames.half_arm_1_link_sphere_0],
        [Frames.shoulder_link_sphere_0, Frames.half_arm_1_link_sphere_3],
        [Frames.shoulder_link_sphere_1, Frames.half_arm_1_link_sphere_0],
        [Frames.half_arm_2_link_sphere_2, Frames.forearm_link_sphere_0],
        [Frames.half_arm_2_link_sphere_2, Frames.forearm_link_sphere_4],
        [Frames.half_arm_2_link_sphere_7, Frames.forearm_link_sphere_2],
        [Frames.spherical_wrist_1_link_sphere_4, Frames.spherical_wrist_2_link_sphere_5],
        [Frames.spherical_wrist_1_link_sphere_2, Frames.spherical_wrist_2_link_sphere_4],
        [Frames.shoulder_link_sphere_1, Frames.half_arm_1_link_sphere_3],
        [Frames.shoulder_link_sphere_2, Frames.half_arm_1_link_sphere_2],
    ]

    SelfCollisionVolIgnored = []
    
    EnvCollisionVolIgnored = []

    VisualizeSafeZone = [
        Frames.R_ee,
    ]

    VisualizePhiTraj = [
        Frames.R_ee,
    ]