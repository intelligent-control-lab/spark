from enum import IntEnum
from spark_robot.base.base_robot_config import RobotConfig
from spark_utils import Geometry, VizColor
import numpy as np

kNotUsedJoint = 29

class G1DualArmDynamic1Config(RobotConfig):
    
    # ---------------------------------------------------------------------------- #
    #                                      Kinematics                              #
    # ---------------------------------------------------------------------------- #
    
    kinematics_class_name = "G1DualArmBaseKinematics"
    mujoco_model_path = "g1/g1_29dof_dual_arm.xml"
    joint_to_lock = []
    
    # ---------------------------------------------------------------------------- #
    #                                   hardware                                   #
    # ---------------------------------------------------------------------------- #
    
    NumTotalMotors = 35
    
    class RealMotors(IntEnum):

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
    
    NormalMotor = [
    ]
    
    WeakMotor = [
        RealMotors.LeftShoulderPitch,
        RealMotors.LeftShoulderRoll,
        RealMotors.LeftShoulderYaw,
        RealMotors.LeftElbow,
        RealMotors.RightShoulderPitch,
        RealMotors.RightShoulderRoll,
        RealMotors.RightShoulderYaw,
        RealMotors.RightElbow,
    ]
    
    DelicateMotor = [
        RealMotors.LeftWristRoll,
        RealMotors.LeftWristPitch,
        RealMotors.LeftWristYaw,
        RealMotors.RightWristRoll,
        RealMotors.RightWristPitch,
        RealMotors.RightWristYaw,
    ]
    
    # ---------------------------------------------------------------------------- #
    #                                      DoF                                     #
    # ---------------------------------------------------------------------------- #

    class DoFs (IntEnum):

        # Left arm
        LeftShoulderPitch = 0
        LeftShoulderRoll  = 1
        LeftShoulderYaw   = 2
        LeftElbow         = 3
        LeftWristRoll     = 4
        LeftWristPitch    = 5
        LeftWristYaw      = 6

        # Right arm
        RightShoulderPitch = 7
        RightShoulderRoll  = 8
        RightShoulderYaw   = 9
        RightElbow         = 10
        RightWristRoll     = 11
        RightWristPitch    = 12
        RightWristYaw      = 13
    
    DefaultDoFVal = {
        DoFs.LeftShoulderPitch : 0.0,
        DoFs.LeftShoulderRoll  : 0.5,
        DoFs.LeftShoulderYaw   : 0.0,
        DoFs.LeftElbow         : 1.57,
        DoFs.LeftWristRoll     : 0.0,
        DoFs.LeftWristPitch    : 0.0,
        DoFs.LeftWristYaw      : 0.0,

        DoFs.RightShoulderPitch: 0.0,
        DoFs.RightShoulderRoll : -0.5,
        DoFs.RightShoulderYaw  : 0.0,
        DoFs.RightElbow        : 1.57,
        DoFs.RightWristRoll    : 0.0,
        DoFs.RightWristPitch   : 0.0,
        DoFs.RightWristYaw     : 0.0
    }

    # ---------------------------------------------------------------------------- #
    #                                   Dynamics                                   #
    # ---------------------------------------------------------------------------- #

    class Control (IntEnum):
        # Left arm
        vLeftShoulderPitch = 0
        vLeftShoulderRoll  = 1
        vLeftShoulderYaw   = 2
        vLeftElbow         = 3
        vLeftWristRoll     = 4
        vLeftWristPitch    = 5
        vLeftWristYaw      = 6

        # Right arm
        vRightShoulderPitch = 7
        vRightShoulderRoll  = 8
        vRightShoulderYaw   = 9
        vRightElbow         = 10
        vRightWristRoll     = 11
        vRightWristPitch    = 12
        vRightWristYaw      = 13

    
    ControlLimit = {
        Control.vLeftShoulderPitch : 1.5,
        Control.vLeftShoulderRoll  : 1.5,
        Control.vLeftShoulderYaw   : 1.5,
        Control.vLeftElbow         : 1.5,
        Control.vLeftWristRoll     : 1.5,
        Control.vLeftWristPitch    : 1.5,
        Control.vLeftWristYaw      : 1.5,
        Control.vRightShoulderPitch: 1.5,
        Control.vRightShoulderRoll : 1.5,
        Control.vRightShoulderYaw  : 1.5,
        Control.vRightElbow        : 1.5,
        Control.vRightWristRoll    : 1.5,
        Control.vRightWristPitch   : 1.5,
        Control.vRightWristYaw     : 1.5,
    }

    NormalControl = [
    ]
    
    WeakControl = [
        Control.vLeftShoulderPitch,
        Control.vLeftShoulderRoll,
        Control.vLeftShoulderYaw,
        Control.vLeftElbow,
        Control.vRightShoulderPitch,
        Control.vRightShoulderRoll,
        Control.vRightShoulderYaw,
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

        # Left arm
        LeftShoulderPitch = 0
        LeftShoulderRoll  = 1
        LeftShoulderYaw   = 2
        LeftElbow         = 3
        LeftWristRoll     = 4
        LeftWristPitch    = 5
        LeftWristYaw      = 6

        # Right arm
        RightShoulderPitch = 7
        RightShoulderRoll  = 8
        RightShoulderYaw   = 9
        RightElbow         = 10
        RightWristRoll     = 11
        RightWristPitch    = 12
        RightWristYaw      = 13


    class MujocoMotors(IntEnum):
        # Left arm
        LeftShoulderPitch = 0
        LeftShoulderRoll  = 1
        LeftShoulderYaw   = 2
        LeftElbow         = 3
        LeftWristRoll     = 4
        LeftWristPitch    = 5
        LeftWristYaw      = 6

        # Right arm
        RightShoulderPitch = 7
        RightShoulderRoll  = 8
        RightShoulderYaw   = 9
        RightElbow         = 10
        RightWristRoll     = 11
        RightWristPitch    = 12
        RightWristYaw      = 13


    # ---------------------------------------------------------------------------- #
    #                                   Mujoco PID Parameters                      #
    # ---------------------------------------------------------------------------- #
    
    # Kp parameters for Mujoco Motors
    MujocoMotorKps = {

        MujocoMotors.LeftShoulderPitch : 100.0,
        MujocoMotors.LeftShoulderRoll  : 100.0,
        MujocoMotors.LeftShoulderYaw   : 50.0,
        MujocoMotors.LeftElbow         : 50.0,
        MujocoMotors.LeftWristRoll     : 20.0,
        MujocoMotors.LeftWristPitch    : 20.0,
        MujocoMotors.LeftWristYaw      : 20.0,

        MujocoMotors.RightShoulderPitch : 100.0,
        MujocoMotors.RightShoulderRoll  : 100.0,
        MujocoMotors.RightShoulderYaw   : 50.0,
        MujocoMotors.RightElbow         : 50.0,
        MujocoMotors.RightWristRoll     : 20.0,
        MujocoMotors.RightWristPitch    : 20.0,
        MujocoMotors.RightWristYaw      : 20.0,
    }
    
    # Kd parameters for Mujoco Motors
    MujocoMotorKds = {
        MujocoMotors.LeftShoulderPitch : 2.0,
        MujocoMotors.LeftShoulderRoll  : 2.0,
        MujocoMotors.LeftShoulderYaw   : 2.0,
        MujocoMotors.LeftElbow         : 2.0,
        MujocoMotors.LeftWristRoll     : 1.0,
        MujocoMotors.LeftWristPitch    : 1.0,
        MujocoMotors.LeftWristYaw      : 1.0,

        MujocoMotors.RightShoulderPitch : 2.0,
        MujocoMotors.RightShoulderRoll  : 2.0,
        MujocoMotors.RightShoulderYaw   : 2.0,
        MujocoMotors.RightElbow         : 2.0,
        MujocoMotors.RightWristRoll     : 1.0,
        MujocoMotors.RightWristPitch    : 1.0,
        MujocoMotors.RightWristYaw      : 1.0,
    }

    # ---------------------------------------------------------------------------- #
    #                                   Mujoco Mappings                            #
    # ---------------------------------------------------------------------------- #

    # Mapping from Mujoco DoFs to DoFs
    MujocoDoF_to_DoF = {
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
    }

    # Mapping from DoFs to Mujoco DoFs
    DoF_to_MujocoDoF = {

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
    }

    # Mapping from Mujoco Motors to Control
    MujocoMotor_to_Control = {
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
    }
    
    # ---------------------------------------------------------------------------- #
    #                                    Real                                    #
    # ---------------------------------------------------------------------------- #
    class RealDoFs(IntEnum):

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
    
    # ---------------------------------------------------------------------------- #
    #                                   Real Mappings                                   #
    # ---------------------------------------------------------------------------- #
    
    
    # Mapping from Real DoFs to DoFs
    RealDoF_to_DoF = {
        RealDoFs.LeftShoulderPitch : DoFs.LeftShoulderPitch,
        RealDoFs.LeftShoulderRoll  : DoFs.LeftShoulderRoll,
        RealDoFs.LeftShoulderYaw   : DoFs.LeftShoulderYaw,
        RealDoFs.LeftElbow         : DoFs.LeftElbow,
        RealDoFs.LeftWristRoll     : DoFs.LeftWristRoll,
        RealDoFs.LeftWristPitch    : DoFs.LeftWristPitch,
        RealDoFs.LeftWristYaw      : DoFs.LeftWristYaw,

        RealDoFs.RightShoulderPitch : DoFs.RightShoulderPitch,
        RealDoFs.RightShoulderRoll  : DoFs.RightShoulderRoll,
        RealDoFs.RightShoulderYaw   : DoFs.RightShoulderYaw,
        RealDoFs.RightElbow         : DoFs.RightElbow,
        RealDoFs.RightWristRoll     : DoFs.RightWristRoll,
        RealDoFs.RightWristPitch    : DoFs.RightWristPitch,
        RealDoFs.RightWristYaw      : DoFs.RightWristYaw,
    }

    # Mapping from DoFs to Real DoFs
    DoF_to_RealDoF = {
        DoFs.LeftShoulderPitch: RealDoFs.LeftShoulderPitch,
        DoFs.LeftShoulderRoll : RealDoFs.LeftShoulderRoll,
        DoFs.LeftShoulderYaw  : RealDoFs.LeftShoulderYaw,
        DoFs.LeftElbow        : RealDoFs.LeftElbow,
        DoFs.LeftWristRoll    : RealDoFs.LeftWristRoll,
        DoFs.LeftWristPitch   : RealDoFs.LeftWristPitch,
        DoFs.LeftWristYaw     : RealDoFs.LeftWristYaw,

        DoFs.RightShoulderPitch: RealDoFs.RightShoulderPitch,
        DoFs.RightShoulderRoll : RealDoFs.RightShoulderRoll,
        DoFs.RightShoulderYaw  : RealDoFs.RightShoulderYaw,
        DoFs.RightElbow        : RealDoFs.RightElbow,
        DoFs.RightWristRoll    : RealDoFs.RightWristRoll,
        DoFs.RightWristPitch   : RealDoFs.RightWristPitch,
        DoFs.RightWristYaw     : RealDoFs.RightWristYaw,
    }

    # Mapping from real motors to Control
    RealMotor_to_Control = {
        
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
    }

    # ---------------------------------------------------------------------------- #
    #                                   Cartesian                                  #
    # ---------------------------------------------------------------------------- #
    
    class Frames(IntEnum):
        
        # Left arm
        left_shoulder_pitch_joint = 0
        left_shoulder_roll_joint  = 1
        left_shoulder_yaw_joint   = 2
        left_elbow_joint          = 3
        left_wrist_roll_joint     = 4
        left_wrist_pitch_joint    = 5
        left_wrist_yaw_joint      = 6

        # Right arm
        right_shoulder_pitch_joint = 7
        right_shoulder_roll_joint  = 8
        right_shoulder_yaw_joint   = 9
        right_elbow_joint          = 10
        right_wrist_roll_joint     = 11
        right_wrist_pitch_joint    = 12
        right_wrist_yaw_joint      = 13

        # End-effectors
        L_ee = 14
        R_ee = 15


    
    # ---------------------------------------------------------------------------- #
    #                                   Collision                                  #
    # ---------------------------------------------------------------------------- #
    
    CollisionVol = {
      
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
    }

    # Pairs of adjacent joints to be ignored in collision checking
    AdjacentCollisionVolPairs = [
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
        
    ]
    
    EnvCollisionVolIgnored = [
    ]

    VisualizeSafeZone = [
        Frames.L_ee,
        Frames.R_ee,
    ]
    
    VisualizePhiTraj = [
        Frames.L_ee,
        Frames.R_ee,
    ]