from enum import IntEnum
from spark_robot.base.base_robot_config import RobotConfig
from spark_utils import Geometry, VizColor
import numpy as np

class G1WholeBodyDynamic1Config(RobotConfig):
    
    # ---------------------------------------------------------------------------- #
    #                                      Kinematics                              #
    # ---------------------------------------------------------------------------- #
    
    kinematics_class_name = "G1WholeBodyKinematics"
    mujoco_model_path = "g1/g1_29dof_whole_body.xml"
    joint_to_lock = []
    
    # ---------------------------------------------------------------------------- #
    #                                   hardware                                   #
    # ---------------------------------------------------------------------------- #
    
    NumTotalMotors = 35
    
    class RealMotors(IntEnum):
        # Left Leg
        LeftHipPitch   = 0
        LeftHipRoll    = 1
        LeftHipYaw     = 2
        LeftKnee       = 3
        LeftAnklePitch = 4
        LeftAnkleRoll  = 5
        
        # Right Leg
        RightHipPitch   = 6
        RightHipRoll    = 7
        RightHipYaw     = 8
        RightKnee       = 9
        RightAnklePitch = 10
        RightAnkleRoll  = 11

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
        RealMotors.LeftHipPitch: (-2.5307, 2.8798),
        RealMotors.LeftHipRoll: (-0.5236, 2.9671),
        RealMotors.LeftHipYaw: (-2.7576, 2.7576),
        RealMotors.LeftKnee: (-0.087267, 2.8798),
        RealMotors.LeftAnklePitch: (-0.87267, 0.5236),
        RealMotors.LeftAnkleRoll: (-0.2618,0.2618),
        RealMotors.RightHipPitch: (-2.5307, 2.8798),
        RealMotors.RightHipRoll: (-2.9671, 0.5236),
        RealMotors.RightHipYaw: (-2.7576, 2.7576),
        RealMotors.RightKnee: (-0.087267, 2.8798),
        RealMotors.RightAnklePitch: (-0.87267, 0.5236),
        RealMotors.RightAnkleRoll: (-0.2618, 0.2618),
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
        RealMotors.LeftHipPitch,
        RealMotors.LeftHipRoll,
        RealMotors.LeftHipYaw,
        RealMotors.LeftKnee,
        RealMotors.LeftAnklePitch,
        RealMotors.LeftAnkleRoll,
        RealMotors.RightHipPitch,
        RealMotors.RightHipRoll,
        RealMotors.RightHipYaw,
        RealMotors.RightKnee,
        RealMotors.RightAnklePitch,
        RealMotors.RightAnkleRoll,
        
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
        # Floating Base
        LinearX = 0
        LinearY = 1
        LinearZ = 2
        QuaternionW = 3
        QuaternionX = 4
        QuaternionY = 5
        QuaternionZ = 6
        
        # Left Leg
        LeftHipPitch   = 7
        LeftHipRoll    = 8
        LeftHipYaw     = 9
        LeftKnee       = 10
        LeftAnklePitch = 11
        LeftAnkleRoll  = 12
        
        # Right Leg
        RightHipPitch   = 13
        RightHipRoll    = 14
        RightHipYaw     = 15
        RightKnee       = 16
        RightAnklePitch = 17
        RightAnkleRoll  = 18

        # Waist
        WaistYaw   = 19
        WaistRoll  = 20
        WaistPitch = 21

        # Left arm
        LeftShoulderPitch = 22
        LeftShoulderRoll  = 23
        LeftShoulderYaw   = 24
        LeftElbow         = 25
        LeftWristRoll     = 26
        LeftWristPitch    = 27
        LeftWristYaw      = 28
        
        # Right arm
        RightShoulderPitch = 29
        RightShoulderRoll  = 30
        RightShoulderYaw   = 31
        RightElbow         = 32
        RightWristRoll     = 33
        RightWristPitch    = 34
        RightWristYaw      = 35
    
    DefaultDoFVal = {
        DoFs.LinearX       : 0.0,
        DoFs.LinearY       : 0.0,
        DoFs.LinearZ       : 0.793,
        DoFs.QuaternionW   : 1.0,
        DoFs.QuaternionX   : 0.0,
        DoFs.QuaternionY   : 0.0,
        DoFs.QuaternionZ   : 0.0,
        
        DoFs.LeftHipPitch   : -0.1,
        DoFs.LeftHipRoll    : 0.0,
        DoFs.LeftHipYaw     : 0.0,
        DoFs.LeftKnee       : 0.3,
        DoFs.LeftAnklePitch : -0.2,
        DoFs.LeftAnkleRoll  : 0.0,

        DoFs.RightHipPitch   : -0.1,
        DoFs.RightHipRoll    : 0.0,
        DoFs.RightHipYaw     : 0.0,
        DoFs.RightKnee       : 0.3,
        DoFs.RightAnklePitch : -0.2,
        DoFs.RightAnkleRoll  : 0.0,
        
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
        DoFs.RightWristYaw     : 0.0,
    }

    # ---------------------------------------------------------------------------- #
    #                                   Dynamics                                   #
    # ---------------------------------------------------------------------------- #

    class Control (IntEnum):
        # Left Leg
        vLeftHipPitch   = 0
        vLeftHipRoll    = 1
        vLeftHipYaw     = 2
        vLeftKnee       = 3
        vLeftAnklePitch = 4
        vLeftAnkleRoll  = 5
        
        # Right Leg
        vRightHipPitch   = 6
        vRightHipRoll    = 7
        vRightHipYaw     = 8
        vRightKnee       = 9
        vRightAnklePitch = 10
        vRightAnkleRoll  = 11

        # Waist
        vWaistYaw   = 12
        vWaistRoll  = 13
        vWaistPitch = 14

        # Left arm
        vLeftShoulderPitch = 15
        vLeftShoulderRoll  = 16
        vLeftShoulderYaw   = 17
        vLeftElbow         = 18
        vLeftWristRoll     = 19
        vLeftWristPitch    = 20
        vLeftWristYaw      = 21
        
        # Right arm
        vRightShoulderPitch = 22
        vRightShoulderRoll  = 23
        vRightShoulderYaw   = 24
        vRightElbow         = 25
        vRightWristRoll     = 26
        vRightWristPitch    = 27
        vRightWristYaw      = 28
    
    ControlLimit = {
        Control.vLeftHipPitch      : 0.1,
        Control.vLeftHipRoll       : 0.1,
        Control.vLeftHipYaw        : 0.1,
        Control.vLeftKnee          : 1.0,
        Control.vLeftAnklePitch    : 1.0,
        Control.vLeftAnkleRoll     : 1.0,
        Control.vRightHipPitch     : 1.0,
        Control.vRightHipRoll      : 1.0,
        Control.vRightHipYaw       : 1.0,
        Control.vRightKnee         : 1.0,
        Control.vRightAnklePitch   : 1.0,
        Control.vRightAnkleRoll    : 1.0,
        Control.vWaistYaw          : 1.0,
        Control.vWaistRoll         : 1.0,
        Control.vWaistPitch        : 1.0,
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
        # Left Leg
        Control.vLeftHipPitch, 
        Control.vLeftHipRoll, 
        Control.vLeftHipYaw,   
        Control.vLeftKnee,     
        Control.vLeftAnklePitch,
        Control.vLeftAnkleRoll,
        
        # Right Leg
        Control.vRightHipPitch, 
        Control.vRightHipRoll,  
        Control.vRightHipYaw,   
        Control.vRightKnee,     
        Control.vRightAnklePitch,
        Control.vRightAnkleRoll, 
        
        Control.vWaistYaw,
        Control.vWaistRoll,
        Control.vWaistPitch,
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
        return len(self.DoFs)  # exclude floating base

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
        return None

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
        # Floating Base
        LinearX       = 0
        LinearY       = 1
        LinearZ       = 2
        QuaternionW   = 3
        QuaternionX   = 4
        QuaternionY   = 5
        QuaternionZ   = 6
        
        # Left Leg
        LeftHipPitch   = 7
        LeftHipRoll    = 8
        LeftHipYaw     = 9
        LeftKnee       = 10
        LeftAnklePitch = 11
        LeftAnkleRoll  = 12
        
        # Right Leg
        RightHipPitch   = 13
        RightHipRoll    = 14
        RightHipYaw     = 15
        RightKnee       = 16
        RightAnklePitch = 17
        RightAnkleRoll  = 18

        WaistYaw   = 19
        WaistRoll  = 20
        WaistPitch = 21
        
        LeftShoulderPitch = 22
        LeftShoulderRoll  = 23
        LeftShoulderYaw   = 24
        LeftElbow         = 25
        LeftWristRoll     = 26
        LeftWristPitch    = 27
        LeftWristYaw      = 28
        
        RightShoulderPitch = 29
        RightShoulderRoll  = 30
        RightShoulderYaw   = 31
        RightElbow         = 32
        RightWristRoll     = 33
        RightWristPitch    = 34
        RightWristYaw      = 35

    class MujocoMotors(IntEnum):

        # Left Leg
        LeftHipPitch   = 0
        LeftHipRoll    = 1
        LeftHipYaw     = 2
        LeftKnee       = 3
        LeftAnklePitch = 4
        LeftAnkleRoll  = 5
        
        # Right Leg
        RightHipPitch   = 6
        RightHipRoll    = 7
        RightHipYaw     = 8
        RightKnee       = 9
        RightAnklePitch = 10
        RightAnkleRoll  = 11

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
    #                                   Mujoco PID Parameters                      #
    # ---------------------------------------------------------------------------- #
    
    # Kp parameters for Mujoco Motors
    MujocoMotorKps = {
        MujocoMotors.LeftHipPitch   : 100.0,
        MujocoMotors.LeftHipRoll    : 100.0,
        MujocoMotors.LeftHipYaw     : 100.0,
        MujocoMotors.LeftKnee       : 150.0,
        MujocoMotors.LeftAnklePitch : 40.0,
        MujocoMotors.LeftAnkleRoll  : 40.0,
        
        MujocoMotors.RightHipPitch   : 100.0,
        MujocoMotors.RightHipRoll    : 100.0,
        MujocoMotors.RightHipYaw     : 100.0,
        MujocoMotors.RightKnee       : 150.0,
        MujocoMotors.RightAnklePitch : 40.0,
        MujocoMotors.RightAnkleRoll  : 40.0,
        
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
        MujocoMotors.LeftHipPitch   : 2.0,
        MujocoMotors.LeftHipRoll    : 2.0,
        MujocoMotors.LeftHipYaw     : 2.0,
        MujocoMotors.LeftKnee       : 4.0,
        MujocoMotors.LeftAnklePitch : 2.0,
        MujocoMotors.LeftAnkleRoll  : 2.0,
        
        MujocoMotors.RightHipPitch   : 2.0,
        MujocoMotors.RightHipRoll    : 2.0,
        MujocoMotors.RightHipYaw     : 2.0,
        MujocoMotors.RightKnee       : 4.0,
        MujocoMotors.RightAnklePitch : 2.0,
        MujocoMotors.RightAnkleRoll  : 2.0,
        
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
        MujocoDoFs.LinearX       : DoFs.LinearX,
        MujocoDoFs.LinearY       : DoFs.LinearY,
        MujocoDoFs.LinearZ       : DoFs.LinearZ,
        MujocoDoFs.QuaternionW   : DoFs.QuaternionW,
        MujocoDoFs.QuaternionX   : DoFs.QuaternionX,
        MujocoDoFs.QuaternionY   : DoFs.QuaternionY,
        MujocoDoFs.QuaternionZ   : DoFs.QuaternionZ,
        
        MujocoDoFs.LeftHipPitch   : DoFs.LeftHipPitch,
        MujocoDoFs.LeftHipRoll    : DoFs.LeftHipRoll,
        MujocoDoFs.LeftHipYaw     : DoFs.LeftHipYaw,
        MujocoDoFs.LeftKnee       : DoFs.LeftKnee,
        MujocoDoFs.LeftAnklePitch : DoFs.LeftAnklePitch,
        MujocoDoFs.LeftAnkleRoll  : DoFs.LeftAnkleRoll,
        
        MujocoDoFs.RightHipPitch   : DoFs.RightHipPitch,
        MujocoDoFs.RightHipRoll    : DoFs.RightHipRoll,
        MujocoDoFs.RightHipYaw     : DoFs.RightHipYaw,
        MujocoDoFs.RightKnee       : DoFs.RightKnee,
        MujocoDoFs.RightAnklePitch : DoFs.RightAnklePitch,
        MujocoDoFs.RightAnkleRoll  : DoFs.RightAnkleRoll,
        
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
        DoFs.LinearX       : MujocoDoFs.LinearX,
        DoFs.LinearY       : MujocoDoFs.LinearY,
        DoFs.LinearZ       : MujocoDoFs.LinearZ,
        DoFs.QuaternionW   : MujocoDoFs.QuaternionW,
        DoFs.QuaternionX   : MujocoDoFs.QuaternionX,
        DoFs.QuaternionY   : MujocoDoFs.QuaternionY,
        DoFs.QuaternionZ   : MujocoDoFs.QuaternionZ,
        
        DoFs.LeftHipPitch   : MujocoDoFs.LeftHipPitch,
        DoFs.LeftHipRoll    : MujocoDoFs.LeftHipRoll,
        DoFs.LeftHipYaw     : MujocoDoFs.LeftHipYaw,
        DoFs.LeftKnee       : MujocoDoFs.LeftKnee,
        DoFs.LeftAnklePitch : MujocoDoFs.LeftAnklePitch,
        DoFs.LeftAnkleRoll  : MujocoDoFs.LeftAnkleRoll,
        
        DoFs.RightHipPitch   : MujocoDoFs.RightHipPitch,
        DoFs.RightHipRoll    : MujocoDoFs.RightHipRoll,
        DoFs.RightHipYaw     : MujocoDoFs.RightHipYaw,
        DoFs.RightKnee       : MujocoDoFs.RightKnee,
        DoFs.RightAnklePitch : MujocoDoFs.RightAnklePitch,
        DoFs.RightAnkleRoll  : MujocoDoFs.RightAnkleRoll,
        
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
        MujocoMotors.LeftHipPitch   : Control.vLeftHipPitch,
        MujocoMotors.LeftHipRoll    : Control.vLeftHipRoll,
        MujocoMotors.LeftHipYaw     : Control.vLeftHipYaw,
        MujocoMotors.LeftKnee       : Control.vLeftKnee,
        MujocoMotors.LeftAnklePitch : Control.vLeftAnklePitch,
        MujocoMotors.LeftAnkleRoll  : Control.vLeftAnkleRoll,
        
        MujocoMotors.RightHipPitch   : Control.vRightHipPitch,
        MujocoMotors.RightHipRoll    : Control.vRightHipRoll,
        MujocoMotors.RightHipYaw     : Control.vRightHipYaw,
        MujocoMotors.RightKnee       : Control.vRightKnee,
        MujocoMotors.RightAnklePitch : Control.vRightAnklePitch,
        MujocoMotors.RightAnkleRoll  : Control.vRightAnkleRoll,
        
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
    }

    # ---------------------------------------------------------------------------- #
    #                                    Real                                    #
    # ---------------------------------------------------------------------------- #
    class RealDoFs(IntEnum):
        
        # Left Leg
        LeftHipPitch   = 0
        LeftHipRoll    = 1
        LeftHipYaw     = 2
        LeftKnee       = 3
        LeftAnklePitch = 4
        LeftAnkleRoll  = 5
        
        # Right Leg
        RightHipPitch   = 6
        RightHipRoll    = 7
        RightHipYaw     = 8
        RightKnee       = 9
        RightAnklePitch = 10
        RightAnkleRoll  = 11

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
        RealDoFs.LeftHipPitch   : DoFs.LeftHipPitch,
        RealDoFs.LeftHipRoll    : DoFs.LeftHipRoll,
        RealDoFs.LeftHipYaw     : DoFs.LeftHipYaw,
        RealDoFs.LeftKnee       : DoFs.LeftKnee,
        RealDoFs.LeftAnklePitch : DoFs.LeftAnklePitch,
        RealDoFs.LeftAnkleRoll  : DoFs.LeftAnkleRoll,
        
        RealDoFs.RightHipPitch   : DoFs.RightHipPitch,
        RealDoFs.RightHipRoll    : DoFs.RightHipRoll,
        RealDoFs.RightHipYaw     : DoFs.RightHipYaw,
        RealDoFs.RightKnee       : DoFs.RightKnee,
        RealDoFs.RightAnklePitch : DoFs.RightAnklePitch,
        RealDoFs.RightAnkleRoll  : DoFs.RightAnkleRoll,
        
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
        DoFs.LeftHipPitch   : RealDoFs.LeftHipPitch,
        DoFs.LeftHipRoll    : RealDoFs.LeftHipRoll,
        DoFs.LeftHipYaw     : RealDoFs.LeftHipYaw,
        DoFs.LeftKnee       : RealDoFs.LeftKnee,
        DoFs.LeftAnklePitch : RealDoFs.LeftAnklePitch,
        DoFs.LeftAnkleRoll  : RealDoFs.LeftAnkleRoll,
        
        DoFs.RightHipPitch   : RealDoFs.RightHipPitch,
        DoFs.RightHipRoll    : RealDoFs.RightHipRoll,
        DoFs.RightHipYaw     : RealDoFs.RightHipYaw,
        DoFs.RightKnee       : RealDoFs.RightKnee,
        DoFs.RightAnklePitch : RealDoFs.RightAnklePitch,
        DoFs.RightAnkleRoll  : RealDoFs.RightAnkleRoll,
        
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
        RealMotors.LeftHipPitch   : Control.vLeftHipPitch,
        RealMotors.LeftHipRoll    : Control.vLeftHipRoll,
        RealMotors.LeftHipYaw     : Control.vLeftHipYaw,
        RealMotors.LeftKnee       : Control.vLeftKnee,
        RealMotors.LeftAnklePitch : Control.vLeftAnklePitch,
        RealMotors.LeftAnkleRoll  : Control.vLeftAnkleRoll,
        
        RealMotors.RightHipPitch   : Control.vRightHipPitch,
        RealMotors.RightHipRoll    : Control.vRightHipRoll,
        RealMotors.RightHipYaw     : Control.vRightHipYaw,
        RealMotors.RightKnee       : Control.vRightKnee,
        RealMotors.RightAnklePitch : Control.vRightAnklePitch,
        RealMotors.RightAnkleRoll  : Control.vRightAnkleRoll,
        
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
    }

    # ---------------------------------------------------------------------------- #
    #                                   Cartesian                                  #
    # ---------------------------------------------------------------------------- #
    
    class Frames(IntEnum):
        left_hip_pitch_joint = 0
        left_hip_roll_joint = 1
        left_hip_yaw_joint = 2
        left_knee_joint = 3
        left_ankle_pitch_joint = 4
        left_ankle_roll_joint = 5
        
        right_hip_pitch_joint = 6
        right_hip_roll_joint = 7
        right_hip_yaw_joint = 8
        right_knee_joint = 9
        right_ankle_pitch_joint = 10
        right_ankle_roll_joint = 11
        
        waist_yaw_joint = 12
        waist_roll_joint = 13
        waist_pitch_joint = 14
        
        left_shoulder_pitch_joint = 15
        left_shoulder_roll_joint = 16
        left_shoulder_yaw_joint = 17
        left_elbow_joint = 18
        left_wrist_roll_joint = 19
        left_wrist_pitch_joint = 20
        left_wrist_yaw_joint = 21
        
        right_shoulder_pitch_joint = 22
        right_shoulder_roll_joint = 23
        right_shoulder_yaw_joint = 24
        right_elbow_joint = 25
        right_wrist_roll_joint = 26
        right_wrist_pitch_joint = 27
        right_wrist_yaw_joint = 28
        
        L_ee = 29
        R_ee = 30
        
        torso_link_1 = 31
        torso_link_2 = 32
        torso_link_3 = 33
        
        pelvis_link_1 = 34
        pelvis_link_2 = 35
        pelvis_link_3 = 36
    
    # ---------------------------------------------------------------------------- #
    #                                   Collision                                  #
    # ---------------------------------------------------------------------------- #
    
    CollisionVol = {
        Frames.left_hip_pitch_joint: Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        Frames.left_hip_roll_joint : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        Frames.left_hip_yaw_joint  : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        Frames.left_knee_joint     : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        Frames.left_ankle_pitch_joint: Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        Frames.left_ankle_roll_joint : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        
        Frames.right_hip_pitch_joint: Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        Frames.right_hip_roll_joint : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        Frames.right_hip_yaw_joint  : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        Frames.right_knee_joint     : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        Frames.right_ankle_pitch_joint: Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        Frames.right_ankle_roll_joint : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        
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
        Frames.left_hip_pitch_joint,
        Frames.left_hip_roll_joint,
        Frames.left_hip_yaw_joint,
        Frames.left_knee_joint,
        Frames.left_ankle_pitch_joint,
        Frames.left_ankle_roll_joint,
        
        Frames.right_hip_pitch_joint,
        Frames.right_hip_roll_joint,
        Frames.right_hip_yaw_joint,
        Frames.right_knee_joint,
        Frames.right_ankle_pitch_joint,
        Frames.right_ankle_roll_joint,
        
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
        Frames.left_hip_pitch_joint,
        Frames.left_hip_roll_joint,
        Frames.left_hip_yaw_joint,
        Frames.left_knee_joint,
        Frames.left_ankle_pitch_joint,
        Frames.left_ankle_roll_joint,
        
        Frames.right_hip_pitch_joint,
        Frames.right_hip_roll_joint,
        Frames.right_hip_yaw_joint,
        Frames.right_knee_joint,
        Frames.right_ankle_pitch_joint,
        Frames.right_ankle_roll_joint,
        
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