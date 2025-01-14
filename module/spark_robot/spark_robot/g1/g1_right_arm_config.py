from enum import IntEnum
from spark_robot.base.base_robot_config import RobotConfig
from spark_utils import Geometry
import numpy as np

kNotUsedJoint = 29

class G1RightArmConfig(RobotConfig):
    
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
                        "waist_yaw_joint",
                        "waist_roll_joint",
                        "waist_pitch_joint",
                        "left_shoulder_pitch_joint",
                        "left_shoulder_roll_joint",
                        "left_shoulder_yaw_joint",
                        "left_elbow_joint",
                        "left_wrist_roll_joint",
                        "left_wrist_pitch_joint",
                        "left_wrist_yaw_joint",
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
        
        # Right arm
        RightShoulderPitch = 0
        RightShoulderRoll  = 1
        RightShoulderYaw   = 2
        RightElbow         = 3
        RightWristRoll     = 4
        RightWristPitch    = 5
        RightWristYaw      = 6

    # ---------------------------------------------------------------------------- #
    #                                   Dynamics                                   #
    # ---------------------------------------------------------------------------- #

    class State (IntEnum):
        
        # Right arm
        RightShoulderPitch = 0
        RightShoulderRoll  = 1
        RightShoulderYaw   = 2
        RightElbow         = 3
        RightWristRoll     = 4
        RightWristPitch    = 5
        RightWristYaw      = 6
        
    class Control (IntEnum):

        # Right arm
        vRightShoulderPitch = 0
        vRightShoulderRoll  = 1
        vRightShoulderYaw   = 2
        vRightElbow         = 3
        vRightWristRoll     = 4
        vRightWristPitch    = 5
        vRightWristYaw      = 6
    
    ControlLimit = {

        Control.vRightShoulderPitch: 5.0,
        Control.vRightShoulderRoll : 5.0,
        Control.vRightShoulderYaw  : 5.0,
        Control.vRightElbow        : 5.0,
        Control.vRightWristRoll    : 5.0,
        Control.vRightWristPitch   : 5.0,
        Control.vRightWristYaw     : 5.0,
    }

    NormalControl = []
    
    WeakControl = [
        Control.vRightShoulderPitch,
        Control.vRightShoulderRoll,
        Control.vRightShoulderYaw,
        Control.vRightElbow,
    ]
    
    DelicateControl = [
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

        RightShoulderPitch = 25
        RightShoulderRoll  = 26
        RightShoulderYaw   = 27
        RightElbow         = 28
        RightWristRoll     = 29
        RightWristPitch    = 30
        RightWristYaw      = 31
        

    class MujocoMotors(IntEnum):
        
        RightShoulderPitch = 10
        RightShoulderRoll  = 11
        RightShoulderYaw   = 12
        RightElbow         = 13
        RightWristRoll     = 14
        RightWristPitch    = 15
        RightWristYaw      = 16

    # ---------------------------------------------------------------------------- #
    #                                   Mappings                                   #
    # ---------------------------------------------------------------------------- #

    # todo change dof to state mapping func

    # Mapping from DoFs to State
    DoF_to_State = {
        
        DoFs.RightShoulderPitch : State.RightShoulderPitch,
        DoFs.RightShoulderRoll  : State.RightShoulderRoll,
        DoFs.RightShoulderYaw   : State.RightShoulderYaw,
        DoFs.RightElbow         : State.RightElbow,
        DoFs.RightWristRoll     : State.RightWristRoll,
        DoFs.RightWristPitch    : State.RightWristPitch,
        DoFs.RightWristYaw      : State.RightWristYaw,

    }

    # Mapping from State to DoFs
    State_to_DoF = {

        State.RightShoulderPitch: DoFs.RightShoulderPitch,
        State.RightShoulderRoll : DoFs.RightShoulderRoll,
        State.RightShoulderYaw  : DoFs.RightShoulderYaw,
        State.RightElbow        : DoFs.RightElbow,
        State.RightWristRoll    : DoFs.RightWristRoll,
        State.RightWristPitch   : DoFs.RightWristPitch,
        State.RightWristYaw     : DoFs.RightWristYaw,

    }

    # Mapping from Mujoco DoFs to DoFs
    MujocoDoF_to_DoF = {

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

        MujocoMotors.RightShoulderPitch : Control.vRightShoulderPitch,
        MujocoMotors.RightShoulderRoll  : Control.vRightShoulderRoll,
        MujocoMotors.RightShoulderYaw   : Control.vRightShoulderYaw,
        MujocoMotors.RightElbow         : Control.vRightElbow,
        MujocoMotors.RightWristRoll     : Control.vRightWristRoll,
        MujocoMotors.RightWristPitch    : Control.vRightWristPitch,
        MujocoMotors.RightWristYaw      : Control.vRightWristYaw,

    }

    # Mapping from real motors to Control
    RealMotor_to_Control = {
        
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
        R_ee = 0
        
    
    # ---------------------------------------------------------------------------- #
    #                                   Collision                                  #
    # ---------------------------------------------------------------------------- #
    
    CollisionVol = {  
        Frames.R_ee: Geometry(type='sphere', radius=0.05),
    }

    # Pairs of adjacent joints to be ignored in collision checking
    AdjacentCollisionVolPairs = []

    SelfCollisionVolIgnored = []
