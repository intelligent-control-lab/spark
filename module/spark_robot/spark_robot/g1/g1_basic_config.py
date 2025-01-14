from enum import IntEnum
from spark_robot.base.base_robot_config import RobotConfig
from spark_utils import Geometry, VizColor
import numpy as np

kNotUsedJoint = 29

class G1BasicConfig(RobotConfig):
    
    # ---------------------------------------------------------------------------- #
    #                                      Kinematics                              #
    # ---------------------------------------------------------------------------- #
    
    mjcf_path = 'g1/g1_29dof_upper_body.xml'
    joint_to_lock = [  
                        "left_hip_pitch_joint" ,
                        "left_hip_roll_joint" ,
                        "left_hip_yaw_joint" ,
                        "left_knee_joint" ,
                        "left_ankle_pitch_joint" ,
                        "left_ankle_roll_joint" ,
                        "right_hip_pitch_joint" ,
                        "right_hip_roll_joint" ,
                        "right_hip_yaw_joint" ,
                        "right_knee_joint" ,
                        "right_ankle_pitch_joint" ,
                        "right_ankle_roll_joint" ,        
                    ]
    
    # ---------------------------------------------------------------------------- #
    #                                   hardware                                   #
    # ---------------------------------------------------------------------------- #
    
    class RealMotors(IntEnum):

        # Waist
        WaistYaw   = 12
        WaistRoll  = 13
        WaistPitch = 14

        # Left arm
        LeftShoulderPitch = 15
        LeftShoulderRoll  = 16
        LeftShoulderYaw   = 17
        LeftElbow         = 18
        LeftWristRoll     = 19
        LeftWristPitch    = 20
        LeftWristYaw      = 21
        
        # Right arm
        RightShoulderPitch = 22
        RightShoulderRoll  = 23
        RightShoulderYaw   = 24
        RightElbow         = 25
        RightWristRoll     = 26
        RightWristPitch    = 27
        RightWristYaw      = 28
    
    # Based on https://support.unitree.com/home/en/G1_developer/about_G1
    RealMotorPosLimit = {
        RealMotors.WaistYaw: (-2.618, 2.618),
        RealMotors.WaistRoll: (-0.52, 0.52),
        RealMotors.WaistPitch: (-0.52, 0.52),
        RealMotors.LeftShoulderPitch: (-3.0892, 2.6704),
        RealMotors.LeftShoulderRoll: (-1.5882, 2.2515),
        RealMotors.LeftShoulderYaw: (-2.618, 2.618),
        RealMotors.LeftElbow: (-1.0472, 2.0944),
        RealMotors.LeftWristRoll: (-1.972222054, 1.972222054),
        RealMotors.LeftWristPitch: (-1.614429558, 1.614429558),
        RealMotors.LeftWristYaw: (-1.614429558, 1.614429558),
        RealMotors.RightShoulderPitch: (-3.0892, 2.6704),
        RealMotors.RightShoulderRoll: (-2.2515, 1.5882),
        RealMotors.RightShoulderYaw: (-2.618, 2.618),
        RealMotors.RightElbow: (-1.0472, 2.0944),
        RealMotors.RightWristRoll: (-1.972222054, 1.972222054),
        RealMotors.RightWristPitch: (-1.614429558, 1.614429558),
        RealMotors.RightWristYaw: (-1.614429558, 1.614429558)
    }
    
    # ---------------------------------------------------------------------------- #
    #                                      DoF                                     #
    # ---------------------------------------------------------------------------- #

    class DoFs (IntEnum):

        # Waist
        WaistYaw   = 0
        WaistRoll  = 1
        WaistPitch = 2

        # Left arm
        LeftShoulderPitch = 3
        LeftShoulderRoll  = 4
        LeftShoulderYaw   = 5
        LeftElbow         = 6
        LeftWristRoll     = 7
        LeftWristPitch    = 8
        LeftWristYaw      = 9
        
        # Right arm
        RightShoulderPitch = 10
        RightShoulderRoll  = 11
        RightShoulderYaw   = 12
        RightElbow         = 13
        RightWristRoll     = 14
        RightWristPitch    = 15
        RightWristYaw      = 16
        
        # linear
        LinearX = 17
        LinearY = 18
        RotYaw  = 19

    # ---------------------------------------------------------------------------- #
    #                                   Dynamics                                   #
    # ---------------------------------------------------------------------------- #

    class Control (IntEnum):

        # Waist
        vWaistYaw   = 0
        vWaistRoll  = 1
        vWaistPitch = 2

        # Left arm
        vLeftShoulderPitch = 3
        vLeftShoulderRoll  = 4
        vLeftShoulderYaw   = 5
        vLeftElbow         = 6
        vLeftWristRoll     = 7
        vLeftWristPitch    = 8
        vLeftWristYaw      = 9
        
        # Right arm
        vRightShoulderPitch = 10
        vRightShoulderRoll  = 11
        vRightShoulderYaw   = 12
        vRightElbow         = 13
        vRightWristRoll     = 14
        vRightWristPitch    = 15
        vRightWristYaw      = 16
        
        # linear
        vLinearX = 17
        vLinearY = 18
        vRotYaw  = 19
    
    ControlLimit = {
        Control.vWaistYaw          : 0.0,
        Control.vWaistRoll         : 1.0,
        Control.vWaistPitch        : 0.0,
        Control.vLeftShoulderPitch : 5.0,
        Control.vLeftShoulderRoll  : 5.0,
        Control.vLeftShoulderYaw   : 5.0,
        Control.vLeftElbow         : 5.0,
        Control.vLeftWristRoll     : 5.0,
        Control.vLeftWristPitch    : 5.0,
        Control.vLeftWristYaw      : 5.0,
        Control.vRightShoulderPitch: 5.0,
        Control.vRightShoulderRoll : 5.0,
        Control.vRightShoulderYaw  : 5.0,
        Control.vRightElbow        : 5.0,
        Control.vRightWristRoll    : 5.0,
        Control.vRightWristPitch   : 5.0,
        Control.vRightWristYaw     : 5.0,
        Control.vLinearX           : 1.0,
        Control.vLinearY           : 1.0,
        Control.vRotYaw            : 1.0
    }

    NormalControl = [
        Control.vWaistYaw,
        Control.vWaistRoll,
        Control.vWaistPitch,
        Control.vLinearX,
        Control.vLinearY,
        Control.vRotYaw,
    ]
    
    WeakControl = [
        Control.vLeftShoulderPitch,
        Control.vLeftShoulderRoll,
        Control.vLeftShoulderYaw,
        Control.vLeftElbow,
        Control.vRightShoulderPitch,
        Control.vRightShoulderRoll,
        Control.vRightShoulderYaw,
        Control.vRightElbow,
    ]
    
    DelicateControl = [
        Control.vLeftWristRoll,
        Control.vLeftWristPitch,
        Control.vLeftWristYaw,
        Control.vRightWristRoll,
        Control.vRightWristPitch,
        Control.vRightWristYaw,
    ]

    '''
        x_dot = f(x) + g(x) * control

        For velocity control, f = 0, g = I
    '''

    @property
    def num_state(self):
        return len(self.DoFs)

    def compose_state_from_dof(self, dof_pos):
        '''
            dof_pos: [num_dof,]
        '''
        state = dof_pos.reshape(-1)
        return state

    def decompose_state_to_dof(self, state):
        '''
            state: [num_state,]
            return: [num_dof,]
        '''
        dof_pos = state.reshape(-1)
        return dof_pos

    def dynamics_f(self, state):
        '''
            state: [num_state, 1]
            return: [num_state, 1]
        '''
        
        return np.zeros((self.num_state, 1))

    def dynamics_g(self, state):
        '''
            state: [num_state, 1]
            return: [num_state, num_control]
        '''

        return np.eye(self.num_state)

    # ---------------------------------------------------------------------------- #
    #                                    MuJoCo                                    #
    # ---------------------------------------------------------------------------- #
    class MujocoDoFs(IntEnum):

        WaistYaw   = 15
        WaistRoll  = 16
        WaistPitch = 17
        
        LeftShoulderPitch = 18
        LeftShoulderRoll  = 19
        LeftShoulderYaw   = 20
        LeftElbow         = 21
        LeftWristRoll     = 22
        LeftWristPitch    = 23
        LeftWristYaw      = 24
        
        RightShoulderPitch = 25
        RightShoulderRoll  = 26
        RightShoulderYaw   = 27
        RightElbow         = 28
        RightWristRoll     = 29
        RightWristPitch    = 30
        RightWristYaw      = 31
        
        LinearX=  0
        LinearY=  1
        RotYaw =  2

    class MujocoMotors(IntEnum):

        WaistYaw   = 0
        WaistRoll  = 1
        WaistPitch = 2
        
        LeftShoulderPitch = 3
        LeftShoulderRoll  = 4
        LeftShoulderYaw   = 5
        LeftElbow         = 6
        LeftWristRoll     = 7
        LeftWristPitch    = 8
        LeftWristYaw      = 9
        
        RightShoulderPitch = 10
        RightShoulderRoll  = 11
        RightShoulderYaw   = 12
        RightElbow         = 13
        RightWristRoll     = 14
        RightWristPitch    = 15
        RightWristYaw      = 16

        LinearX = 17
        LinearY = 18
        RotYaw  = 19

    # ---------------------------------------------------------------------------- #
    #                                   Mappings                                   #
    # ---------------------------------------------------------------------------- #

    # Mapping from Mujoco DoFs to DoFs
    MujocoDoF_to_DoF = {
        MujocoDoFs.WaistYaw   : DoFs.WaistYaw,
        MujocoDoFs.WaistRoll  : DoFs.WaistRoll,
        MujocoDoFs.WaistPitch : DoFs.WaistPitch,

        MujocoDoFs.LeftShoulderPitch : DoFs.LeftShoulderPitch,
        MujocoDoFs.LeftShoulderRoll  : DoFs.LeftShoulderRoll,
        MujocoDoFs.LeftShoulderYaw   : DoFs.LeftShoulderYaw,
        MujocoDoFs.LeftElbow         : DoFs.LeftElbow,
        MujocoDoFs.LeftWristRoll     : DoFs.LeftWristRoll,
        MujocoDoFs.LeftWristPitch    : DoFs.LeftWristPitch,
        MujocoDoFs.LeftWristYaw      : DoFs.LeftWristYaw,

        MujocoDoFs.RightShoulderPitch : DoFs.RightShoulderPitch,
        MujocoDoFs.RightShoulderRoll  : DoFs.RightShoulderRoll,
        MujocoDoFs.RightShoulderYaw   : DoFs.RightShoulderYaw,
        MujocoDoFs.RightElbow         : DoFs.RightElbow,
        MujocoDoFs.RightWristRoll     : DoFs.RightWristRoll,
        MujocoDoFs.RightWristPitch    : DoFs.RightWristPitch,
        MujocoDoFs.RightWristYaw      : DoFs.RightWristYaw,

        MujocoDoFs.LinearX: DoFs.LinearX,
        MujocoDoFs.LinearY: DoFs.LinearY,
        MujocoDoFs.RotYaw : DoFs.RotYaw
    }

    # Mapping from DoFs to Mujoco DoFs
    DoF_to_MujocoDoF = {
        DoFs.WaistYaw  : MujocoDoFs.WaistYaw,
        DoFs.WaistRoll : MujocoDoFs.WaistRoll,
        DoFs.WaistPitch: MujocoDoFs.WaistPitch,

        DoFs.LeftShoulderPitch: MujocoDoFs.LeftShoulderPitch,
        DoFs.LeftShoulderRoll : MujocoDoFs.LeftShoulderRoll,
        DoFs.LeftShoulderYaw  : MujocoDoFs.LeftShoulderYaw,
        DoFs.LeftElbow        : MujocoDoFs.LeftElbow,
        DoFs.LeftWristRoll    : MujocoDoFs.LeftWristRoll,
        DoFs.LeftWristPitch   : MujocoDoFs.LeftWristPitch,
        DoFs.LeftWristYaw     : MujocoDoFs.LeftWristYaw,

        DoFs.RightShoulderPitch: MujocoDoFs.RightShoulderPitch,
        DoFs.RightShoulderRoll : MujocoDoFs.RightShoulderRoll,
        DoFs.RightShoulderYaw  : MujocoDoFs.RightShoulderYaw,
        DoFs.RightElbow        : MujocoDoFs.RightElbow,
        DoFs.RightWristRoll    : MujocoDoFs.RightWristRoll,
        DoFs.RightWristPitch   : MujocoDoFs.RightWristPitch,
        DoFs.RightWristYaw     : MujocoDoFs.RightWristYaw,

        DoFs.LinearX: MujocoDoFs.LinearX,
        DoFs.LinearY: MujocoDoFs.LinearY,
        DoFs.RotYaw : MujocoDoFs.RotYaw
    }

    # Mapping from Mujoco Motors to Control
    MujocoMotor_to_Control = {
        MujocoMotors.WaistYaw   : Control.vWaistYaw,
        MujocoMotors.WaistRoll  : Control.vWaistRoll,
        MujocoMotors.WaistPitch : Control.vWaistPitch,

        MujocoMotors.LeftShoulderPitch : Control.vLeftShoulderPitch,
        MujocoMotors.LeftShoulderRoll  : Control.vLeftShoulderRoll,
        MujocoMotors.LeftShoulderYaw   : Control.vLeftShoulderYaw,
        MujocoMotors.LeftElbow         : Control.vLeftElbow,
        MujocoMotors.LeftWristRoll     : Control.vLeftWristRoll,
        MujocoMotors.LeftWristPitch    : Control.vLeftWristPitch,
        MujocoMotors.LeftWristYaw      : Control.vLeftWristYaw,

        MujocoMotors.RightShoulderPitch : Control.vRightShoulderPitch,
        MujocoMotors.RightShoulderRoll  : Control.vRightShoulderRoll,
        MujocoMotors.RightShoulderYaw   : Control.vRightShoulderYaw,
        MujocoMotors.RightElbow         : Control.vRightElbow,
        MujocoMotors.RightWristRoll     : Control.vRightWristRoll,
        MujocoMotors.RightWristPitch    : Control.vRightWristPitch,
        MujocoMotors.RightWristYaw      : Control.vRightWristYaw,

        MujocoMotors.LinearX: Control.vLinearX,
        MujocoMotors.LinearY: Control.vLinearY,
        MujocoMotors.RotYaw : Control.vRotYaw
    }

    # Mapping from real motors to Control
    RealMotor_to_Control = {
        RealMotors.WaistYaw  : Control.vWaistYaw,
        RealMotors.WaistRoll : Control.vWaistRoll,
        RealMotors.WaistPitch: Control.vWaistPitch,
        
        RealMotors.LeftShoulderPitch: Control.vLeftShoulderPitch,
        RealMotors.LeftShoulderRoll : Control.vLeftShoulderRoll,
        RealMotors.LeftShoulderYaw  : Control.vLeftShoulderYaw,
        RealMotors.LeftElbow        : Control.vLeftElbow,
        RealMotors.LeftWristRoll    : Control.vLeftWristRoll,
        RealMotors.LeftWristPitch   : Control.vLeftWristPitch,
        RealMotors.LeftWristYaw     : Control.vLeftWristYaw,
        
        RealMotors.RightShoulderPitch: Control.vRightShoulderPitch,
        RealMotors.RightShoulderRoll : Control.vRightShoulderRoll,
        RealMotors.RightShoulderYaw  : Control.vRightShoulderYaw,
        RealMotors.RightElbow        : Control.vRightElbow,
        RealMotors.RightWristRoll    : Control.vRightWristRoll,
        RealMotors.RightWristPitch   : Control.vRightWristPitch,
        RealMotors.RightWristYaw     : Control.vRightWristYaw,
        
        # Control.vLinearX : None,
        # Control.vLinearY : None,
        # Control.vRotYaw  : None,
    }

    # ---------------------------------------------------------------------------- #
    #                                   Cartesian                                  #
    # ---------------------------------------------------------------------------- #
    
    class Frames(IntEnum):
        
        waist_yaw_joint = 0
        waist_roll_joint = 1
        waist_pitch_joint = 2
        
        left_shoulder_pitch_joint = 3
        left_shoulder_roll_joint = 4
        left_shoulder_yaw_joint = 5
        left_elbow_joint = 6
        left_wrist_roll_joint = 7
        left_wrist_pitch_joint = 8
        left_wrist_yaw_joint = 9
        
        right_shoulder_pitch_joint = 10
        right_shoulder_roll_joint = 11
        right_shoulder_yaw_joint = 12
        right_elbow_joint = 13
        right_wrist_roll_joint = 14
        right_wrist_pitch_joint = 15
        right_wrist_yaw_joint = 16
        
        L_ee = 17
        R_ee = 18
        
        torso_link_1 = 19
        torso_link_2 = 20
        torso_link_3 = 21
        
        pelvis_link_1 = 22
        pelvis_link_2 = 23
        pelvis_link_3 = 24
    
    # ---------------------------------------------------------------------------- #
    #                                   Collision                                  #
    # ---------------------------------------------------------------------------- #
    
    CollisionVol = {
        Frames.waist_yaw_joint  : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        Frames.waist_roll_joint : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        Frames.waist_pitch_joint: Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        
        Frames.left_shoulder_pitch_joint: Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        Frames.left_shoulder_roll_joint : Geometry(type='sphere', radius=0.06, color=VizColor.collision_volume),
        Frames.left_shoulder_yaw_joint  : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        Frames.left_elbow_joint         : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        Frames.left_wrist_roll_joint    : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        Frames.left_wrist_pitch_joint   : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        Frames.left_wrist_yaw_joint     : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        
        Frames.right_shoulder_pitch_joint: Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        Frames.right_shoulder_roll_joint : Geometry(type='sphere', radius=0.06, color=VizColor.collision_volume),
        Frames.right_shoulder_yaw_joint  : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        Frames.right_elbow_joint         : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        Frames.right_wrist_roll_joint    : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        Frames.right_wrist_pitch_joint   : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        Frames.right_wrist_yaw_joint     : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        
        Frames.L_ee: Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        Frames.R_ee: Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),

        Frames.torso_link_1: Geometry(type='sphere', radius=0.10, color=VizColor.collision_volume),
        Frames.torso_link_2: Geometry(type='sphere', radius=0.10, color=VizColor.collision_volume),
        Frames.torso_link_3: Geometry(type='sphere', radius=0.08, color=VizColor.collision_volume),
        
        Frames.pelvis_link_1: Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        Frames.pelvis_link_2: Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        Frames.pelvis_link_3: Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume)
    }

    # Pairs of adjacent joints to be ignored in collision checking
    AdjacentCollisionVolPairs = [
        [Frames.waist_yaw_joint, Frames.waist_roll_joint],
        [Frames.waist_yaw_joint, Frames.waist_pitch_joint],
        [Frames.waist_yaw_joint, Frames.torso_link_1],
        [Frames.waist_roll_joint, Frames.waist_pitch_joint],
        [Frames.waist_roll_joint, Frames.torso_link_1],
        [Frames.waist_pitch_joint, Frames.torso_link_1],
        [Frames.torso_link_1, Frames.torso_link_2],
        [Frames.torso_link_1, Frames.torso_link_3],
        [Frames.torso_link_2, Frames.torso_link_3],

        [Frames.left_shoulder_pitch_joint, Frames.torso_link_1],
        [Frames.left_shoulder_pitch_joint, Frames.torso_link_2],
        [Frames.left_shoulder_roll_joint, Frames.torso_link_1],
        [Frames.left_shoulder_roll_joint, Frames.torso_link_2],
        [Frames.left_shoulder_pitch_joint, Frames.left_shoulder_roll_joint],
        [Frames.left_shoulder_pitch_joint, Frames.left_shoulder_yaw_joint],
        [Frames.left_shoulder_roll_joint, Frames.left_shoulder_yaw_joint],
        [Frames.left_shoulder_yaw_joint, Frames.left_elbow_joint],
        [Frames.left_elbow_joint, Frames.left_wrist_roll_joint],
        [Frames.left_wrist_roll_joint, Frames.left_wrist_pitch_joint],
        [Frames.left_wrist_roll_joint, Frames.left_wrist_yaw_joint],
        [Frames.left_wrist_roll_joint, Frames.L_ee],
        [Frames.left_wrist_pitch_joint, Frames.left_wrist_yaw_joint],
        [Frames.left_wrist_pitch_joint, Frames.L_ee],
        [Frames.left_wrist_yaw_joint, Frames.L_ee],

        [Frames.right_shoulder_pitch_joint, Frames.torso_link_1],
        [Frames.right_shoulder_pitch_joint, Frames.torso_link_2],
        [Frames.right_shoulder_roll_joint, Frames.torso_link_1],
        [Frames.right_shoulder_roll_joint, Frames.torso_link_2],
        [Frames.right_shoulder_pitch_joint, Frames.right_shoulder_roll_joint],
        [Frames.right_shoulder_pitch_joint, Frames.right_shoulder_yaw_joint],
        [Frames.right_shoulder_roll_joint, Frames.right_shoulder_yaw_joint],
        [Frames.right_shoulder_yaw_joint, Frames.right_elbow_joint],
        [Frames.right_elbow_joint, Frames.right_wrist_roll_joint],
        [Frames.right_wrist_roll_joint, Frames.right_wrist_pitch_joint],
        [Frames.right_wrist_roll_joint, Frames.right_wrist_yaw_joint],
        [Frames.right_wrist_roll_joint, Frames.R_ee],
        [Frames.right_wrist_pitch_joint, Frames.right_wrist_yaw_joint],
        [Frames.right_wrist_pitch_joint, Frames.R_ee],
        [Frames.right_wrist_yaw_joint, Frames.R_ee],
    ]

    SelfCollisionVolIgnored = [
        Frames.waist_yaw_joint,
        Frames.waist_roll_joint,
        Frames.waist_pitch_joint,
        
        Frames.left_shoulder_pitch_joint,
        Frames.left_shoulder_yaw_joint,
        Frames.left_wrist_roll_joint,
        Frames.left_wrist_pitch_joint,
        Frames.left_wrist_yaw_joint,
        
        Frames.right_shoulder_pitch_joint,
        Frames.right_shoulder_yaw_joint,
        Frames.right_wrist_roll_joint,
        Frames.right_wrist_pitch_joint,
        Frames.right_wrist_yaw_joint,
        
        Frames.pelvis_link_1,
        Frames.pelvis_link_2,
        Frames.pelvis_link_3,
    ]
