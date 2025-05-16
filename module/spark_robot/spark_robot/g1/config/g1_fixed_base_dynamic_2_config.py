from enum import IntEnum
from spark_robot.base.base_robot_config import RobotConfig
from spark_utils import Geometry, VizColor
import numpy as np

kNotUsedJoint = 29

class G1FixedBaseDynamic2Config(RobotConfig):
    
    # ---------------------------------------------------------------------------- #
    #                                      Kinematics                              #
    # ---------------------------------------------------------------------------- #
    
    kinematics_class_name = "G1FixedBaseKinematics"
    mujoco_model_path = "g1/g1_29dof_fixed_base.xml"
    joint_to_lock = []
    
    # ---------------------------------------------------------------------------- #
    #                                   hardware                                   #
    # ---------------------------------------------------------------------------- #
    
    NumTotalMotors = 35
    
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
    
    NormalMotor = [
        RealMotors.WaistYaw,
        RealMotors.WaistRoll,
        RealMotors.WaistPitch,
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
    
    DefaultDoFVal = {
        DoFs.WaistYaw          : 0.0,
        DoFs.WaistRoll         : 0.0,
        DoFs.WaistPitch        : 0.0,

        DoFs.LeftShoulderPitch : 0.0,
        DoFs.LeftShoulderRoll  : 0.5,
        DoFs.LeftShoulderYaw   : 0.0,
        DoFs.LeftElbow         : 0.0,
        DoFs.LeftWristRoll     : 0.0,
        DoFs.LeftWristPitch    : 0.0,
        DoFs.LeftWristYaw      : 0.0,

        DoFs.RightShoulderPitch: 0.0,
        DoFs.RightShoulderRoll : -0.5,
        DoFs.RightShoulderYaw  : 0.0,
        DoFs.RightElbow        : 0.0,
        DoFs.RightWristRoll    : 0.0,
        DoFs.RightWristPitch   : 0.0,
        DoFs.RightWristYaw     : 0.0
    }

    # ---------------------------------------------------------------------------- #
    #                                   Dynamics                                   #
    # ---------------------------------------------------------------------------- #

    class Control (IntEnum):
        # Acceleration control
        
        # Waist
        aWaistYaw   = 0
        aWaistRoll  = 1
        aWaistPitch = 2

        # Left arm
        aLeftShoulderPitch = 3
        aLeftShoulderRoll  = 4
        aLeftShoulderYaw   = 5
        aLeftElbow         = 6
        aLeftWristRoll     = 7
        aLeftWristPitch    = 8
        aLeftWristYaw      = 9
        
        # Right arm
        aRightShoulderPitch = 10
        aRightShoulderRoll  = 11
        aRightShoulderYaw   = 12
        aRightElbow         = 13
        aRightWristRoll     = 14
        aRightWristPitch    = 15
        aRightWristYaw      = 16
    
    ControlLimit = {
        Control.aWaistYaw          : 10.0,
        Control.aWaistRoll         : 10.0,
        Control.aWaistPitch        : 10.0,
        Control.aLeftShoulderPitch : 150.0,
        Control.aLeftShoulderRoll  : 150.0,
        Control.aLeftShoulderYaw   : 150.0,
        Control.aLeftElbow         : 150.0,
        Control.aLeftWristRoll     : 150.0,
        Control.aLeftWristPitch    : 150.0,
        Control.aLeftWristYaw      : 150.0,
        Control.aRightShoulderPitch: 150.0,
        Control.aRightShoulderRoll : 150.0,
        Control.aRightShoulderYaw  : 150.0,
        Control.aRightElbow        : 150.0,
        Control.aRightWristRoll    : 150.0,
        Control.aRightWristPitch   : 150.0,
        Control.aRightWristYaw     : 150.0,
    }

    NormalControl = [
        Control.aWaistYaw,
        Control.aWaistRoll,
        Control.aWaistPitch,
    ]
    
    WeakControl = [
        Control.aLeftShoulderPitch,
        Control.aLeftShoulderRoll,
        Control.aLeftShoulderYaw,
        Control.aLeftElbow,
        Control.aRightShoulderPitch,
        Control.aRightShoulderRoll,
        Control.aRightShoulderYaw,
        Control.aRightElbow,
    ]
    
    DelicateControl = [
        Control.aLeftWristRoll,
        Control.aLeftWristPitch,
        Control.aLeftWristYaw,
        Control.aRightWristRoll,
        Control.aRightWristPitch,
        Control.aRightWristYaw,
    ]

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

    # ---------------------------------------------------------------------------- #
    #                                   Mujoco PID Parameters                      #
    # ---------------------------------------------------------------------------- #
    
    # Kp parameters for Mujoco Motors
    MujocoMotorKps = {
        MujocoMotors.WaistYaw   : 1000.0,
        MujocoMotors.WaistRoll  : 1000.0,
        MujocoMotors.WaistPitch : 1000.0,

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
        MujocoMotors.WaistYaw   : 3.0,
        MujocoMotors.WaistRoll  : 3.0,
        MujocoMotors.WaistPitch : 3.0,

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
    }

    # Mapping from Mujoco Motors to Control
    MujocoMotor_to_Control = {
        MujocoMotors.WaistYaw   : Control.aWaistYaw,
        MujocoMotors.WaistRoll  : Control.aWaistRoll,
        MujocoMotors.WaistPitch : Control.aWaistPitch,

        MujocoMotors.LeftShoulderPitch : Control.aLeftShoulderPitch,
        MujocoMotors.LeftShoulderRoll  : Control.aLeftShoulderRoll,
        MujocoMotors.LeftShoulderYaw   : Control.aLeftShoulderYaw,
        MujocoMotors.LeftElbow         : Control.aLeftElbow,
        MujocoMotors.LeftWristRoll     : Control.aLeftWristRoll,
        MujocoMotors.LeftWristPitch    : Control.aLeftWristPitch,
        MujocoMotors.LeftWristYaw      : Control.aLeftWristYaw,

        MujocoMotors.RightShoulderPitch : Control.aRightShoulderPitch,
        MujocoMotors.RightShoulderRoll  : Control.aRightShoulderRoll,
        MujocoMotors.RightShoulderYaw   : Control.aRightShoulderYaw,
        MujocoMotors.RightElbow         : Control.aRightElbow,
        MujocoMotors.RightWristRoll     : Control.aRightWristRoll,
        MujocoMotors.RightWristPitch    : Control.aRightWristPitch,
        MujocoMotors.RightWristYaw      : Control.aRightWristYaw,
    }
    
    # ---------------------------------------------------------------------------- #
    #                                    Real                                    #
    # ---------------------------------------------------------------------------- #
    class RealDoFs(IntEnum):
        
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
    
    # ---------------------------------------------------------------------------- #
    #                                   Real Mappings                                   #
    # ---------------------------------------------------------------------------- #
    
    
    # Mapping from Real DoFs to DoFs
    RealDoF_to_DoF = {
        RealDoFs.WaistYaw   : DoFs.WaistYaw,
        RealDoFs.WaistRoll  : DoFs.WaistRoll,
        RealDoFs.WaistPitch : DoFs.WaistPitch,

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
        DoFs.WaistYaw  : RealDoFs.WaistYaw,
        DoFs.WaistRoll : RealDoFs.WaistRoll,
        DoFs.WaistPitch: RealDoFs.WaistPitch,

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
        RealMotors.WaistYaw  : Control.aWaistYaw,
        RealMotors.WaistRoll : Control.aWaistRoll,
        RealMotors.WaistPitch: Control.aWaistPitch,
        
        RealMotors.LeftShoulderPitch: Control.aLeftShoulderPitch,
        RealMotors.LeftShoulderRoll : Control.aLeftShoulderRoll,
        RealMotors.LeftShoulderYaw  : Control.aLeftShoulderYaw,
        RealMotors.LeftElbow        : Control.aLeftElbow,
        RealMotors.LeftWristRoll    : Control.aLeftWristRoll,
        RealMotors.LeftWristPitch   : Control.aLeftWristPitch,
        RealMotors.LeftWristYaw     : Control.aLeftWristYaw,
        
        RealMotors.RightShoulderPitch: Control.aRightShoulderPitch,
        RealMotors.RightShoulderRoll : Control.aRightShoulderRoll,
        RealMotors.RightShoulderYaw  : Control.aRightShoulderYaw,
        RealMotors.RightElbow        : Control.aRightElbow,
        RealMotors.RightWristRoll    : Control.aRightWristRoll,
        RealMotors.RightWristPitch   : Control.aRightWristPitch,
        RealMotors.RightWristYaw     : Control.aRightWristYaw,
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
    
    EnvCollisionVolIgnored = [
        Frames.waist_yaw_joint,
        Frames.waist_roll_joint,
        Frames.waist_pitch_joint,
        
        Frames.pelvis_link_1,
        Frames.pelvis_link_2,
        Frames.pelvis_link_3,
    ]

    VisualizeSafeZone = [
        Frames.L_ee,
        Frames.R_ee,
    ]
    
    VisualizePhiTraj = [
        Frames.L_ee,
        Frames.R_ee,
    ]