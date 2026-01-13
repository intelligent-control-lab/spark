from enum import IntEnum
from spark_robot.base.base_robot_config import RobotConfig
from spark_utils import Geometry, VizColor
import numpy as np

kNotUsedJoint = 29

class G1WholeBodyDynamic2Config(RobotConfig):
    
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
        aLeftHipPitch   = 0
        aLeftHipRoll    = 1
        aLeftHipYaw     = 2
        aLeftKnee       = 3
        aLeftAnklePitch = 4
        aLeftAnkleRoll  = 5
        
        # Right Leg
        aRightHipPitch   = 6
        aRightHipRoll    = 7
        aRightHipYaw     = 8
        aRightKnee       = 9
        aRightAnklePitch = 10
        aRightAnkleRoll  = 11

        # Waist
        aWaistYaw   = 12
        aWaistRoll  = 13
        aWaistPitch = 14

        # Left arm
        aLeftShoulderPitch = 15
        aLeftShoulderRoll  = 16
        aLeftShoulderYaw   = 17
        aLeftElbow         = 18
        aLeftWristRoll     = 19
        aLeftWristPitch    = 20
        aLeftWristYaw      = 21
        
        # Right arm
        aRightShoulderPitch = 22
        aRightShoulderRoll  = 23
        aRightShoulderYaw   = 24
        aRightElbow         = 25
        aRightWristRoll     = 26
        aRightWristPitch    = 27
        aRightWristYaw      = 28
    
    ControlLimit = {
        Control.aLeftHipPitch      : 10.0,
        Control.aLeftHipRoll       : 10.0,
        Control.aLeftHipYaw        : 10.0,
        Control.aLeftKnee          : 100.0,
        Control.aLeftAnklePitch    : 100.0,
        Control.aLeftAnkleRoll     : 100.0,
        Control.aRightHipPitch     : 100.0,
        Control.aRightHipRoll      : 100.0,
        Control.aRightHipYaw       : 100.0,
        Control.aRightKnee         : 100.0,
        Control.aRightAnklePitch   : 100.0,
        Control.aRightAnkleRoll    : 100.0,
        Control.aWaistYaw          : 100.0,
        Control.aWaistRoll         : 100.0,
        Control.aWaistPitch        : 100.0,
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
        # Left Leg
        Control.aLeftHipPitch, 
        Control.aLeftHipRoll, 
        Control.aLeftHipYaw,   
        Control.aLeftKnee,     
        Control.aLeftAnklePitch,
        Control.aLeftAnkleRoll,
        
        # Right Leg
        Control.aRightHipPitch, 
        Control.aRightHipRoll,  
        Control.aRightHipYaw,   
        Control.aRightKnee,     
        Control.aRightAnklePitch,
        Control.aRightAnkleRoll, 
        
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
        q_w, q_x, q_y, q_z = positions[3:7]
        omega_x, omega_y, omega_z = velocities[3:6]

        f_x = np.zeros_like(state)
        f_x[:3] = velocities[:3]
        
        f_x[self.DoFs.QuaternionW] = -0.5 * (q_x * omega_x + q_y * omega_y + q_z * omega_z)
        f_x[self.DoFs.QuaternionX] = 0.5 * (q_w * omega_x + q_y * omega_z - q_z * omega_y)
        f_x[self.DoFs.QuaternionY] = 0.5 * (q_w * omega_y - q_x * omega_z + q_z * omega_x)
        f_x[self.DoFs.QuaternionZ] = 0.5 * (q_w * omega_z + q_x * omega_y - q_y * omega_x)
        
        f_x[7:self.num_dof] = velocities[6:]
        
        return f_x

    def dynamics_g(self, state):
        '''
            state: [num_state, 1]
            return: [num_state, num_control]
        '''
        g_x = np.zeros((2 * self.num_dof - 1, self.num_dof - 1))
        g_x[self.num_dof:] = np.eye(self.num_dof - 1)

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
        MujocoMotors.LeftHipPitch   : Control.aLeftHipPitch,
        MujocoMotors.LeftHipRoll    : Control.aLeftHipRoll,
        MujocoMotors.LeftHipYaw     : Control.aLeftHipYaw,
        MujocoMotors.LeftKnee       : Control.aLeftKnee,
        MujocoMotors.LeftAnklePitch : Control.aLeftAnklePitch,
        MujocoMotors.LeftAnkleRoll  : Control.aLeftAnkleRoll,
        
        MujocoMotors.RightHipPitch   : Control.aRightHipPitch,
        MujocoMotors.RightHipRoll    : Control.aRightHipRoll,
        MujocoMotors.RightHipYaw     : Control.aRightHipYaw,
        MujocoMotors.RightKnee       : Control.aRightKnee,
        MujocoMotors.RightAnklePitch : Control.aRightAnklePitch,
        MujocoMotors.RightAnkleRoll  : Control.aRightAnkleRoll,
        
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
        RealMotors.LeftHipPitch   : Control.aLeftHipPitch,
        RealMotors.LeftHipRoll    : Control.aLeftHipRoll,
        RealMotors.LeftHipYaw     : Control.aLeftHipYaw,
        RealMotors.LeftKnee       : Control.aLeftKnee,
        RealMotors.LeftAnklePitch : Control.aLeftAnklePitch,
        RealMotors.LeftAnkleRoll  : Control.aLeftAnkleRoll,
        
        RealMotors.RightHipPitch   : Control.aRightHipPitch,
        RealMotors.RightHipRoll    : Control.aRightHipRoll,
        RealMotors.RightHipYaw     : Control.aRightHipYaw,
        RealMotors.RightKnee       : Control.aRightKnee,
        RealMotors.RightAnklePitch : Control.aRightAnklePitch,
        RealMotors.RightAnkleRoll  : Control.aRightAnkleRoll,
        
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
    
    # class Frames(IntEnum):
    #     left_hip_pitch_joint = 0
    #     left_hip_roll_joint = 1
    #     left_hip_yaw_joint = 2
    #     left_knee_joint = 3
    #     left_ankle_pitch_joint = 4
    #     left_ankle_roll_joint = 5
        
    #     right_hip_pitch_joint = 6
    #     right_hip_roll_joint = 7
    #     right_hip_yaw_joint = 8
    #     right_knee_joint = 9
    #     right_ankle_pitch_joint = 10
    #     right_ankle_roll_joint = 11
        
    #     waist_yaw_joint = 12
    #     waist_roll_joint = 13
    #     waist_pitch_joint = 14
        
    #     left_shoulder_pitch_joint = 15
    #     left_shoulder_roll_joint = 16
    #     left_shoulder_yaw_joint = 17
    #     left_elbow_joint = 18
    #     left_wrist_roll_joint = 19
    #     left_wrist_pitch_joint = 20
    #     left_wrist_yaw_joint = 21
        
    #     right_shoulder_pitch_joint = 22
    #     right_shoulder_roll_joint = 23
    #     right_shoulder_yaw_joint = 24
    #     right_elbow_joint = 25
    #     right_wrist_roll_joint = 26
    #     right_wrist_pitch_joint = 27
    #     right_wrist_yaw_joint = 28
        
    #     L_ee = 29
    #     R_ee = 30
        
    #     torso_link_1 = 31
    #     torso_link_2 = 32
    #     torso_link_3 = 33
        
    #     pelvis_link_1 = 34
    #     pelvis_link_2 = 35
    #     pelvis_link_3 = 36
    
    class Frames(IntEnum):
        L_ee = 0
        R_ee = 1
        torso_link_3 = 2
        
        # waist_yaw_joint = 0
        # waist_roll_joint = 1
        # waist_pitch_joint = 2
        
        # left_shoulder_pitch_joint = 3
        # left_shoulder_roll_joint = 4
        # left_shoulder_yaw_joint = 5
        # left_elbow_joint = 6
        # left_wrist_roll_joint = 7
        # left_wrist_pitch_joint = 8
        # left_wrist_yaw_joint = 9
        
        # right_shoulder_pitch_joint = 10
        # right_shoulder_roll_joint = 11
        # right_shoulder_yaw_joint = 12
        # right_elbow_joint = 13
        # right_wrist_roll_joint = 14
        # right_wrist_pitch_joint = 15
        # right_wrist_yaw_joint = 16
        
        # L_ee = 17
        # R_ee = 18
        
        # torso_link_1 = 19
        # torso_link_2 = 20
        # torso_link_3 = 21
        
        # pelvis_link_1 = 22
        # pelvis_link_2 = 23
        # pelvis_link_3 = 24
    
    # ---------------------------------------------------------------------------- #
    #                                   Collision                                  #
    # ---------------------------------------------------------------------------- #
    
    CollisionVol = {
        # Frames.left_hip_pitch_joint: Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        # Frames.left_hip_roll_joint : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        # Frames.left_hip_yaw_joint  : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        # Frames.left_knee_joint     : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        # Frames.left_ankle_pitch_joint: Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        # Frames.left_ankle_roll_joint : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        
        # Frames.right_hip_pitch_joint: Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        # Frames.right_hip_roll_joint : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        # Frames.right_hip_yaw_joint  : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        # Frames.right_knee_joint     : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        # Frames.right_ankle_pitch_joint: Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        # Frames.right_ankle_roll_joint : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        
        # Frames.waist_yaw_joint  : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        # Frames.waist_roll_joint : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        # Frames.waist_pitch_joint: Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        
        # Frames.left_shoulder_pitch_joint: Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        # Frames.left_shoulder_roll_joint : Geometry(type='sphere', radius=0.06, color=VizColor.collision_volume),
        # Frames.left_shoulder_yaw_joint  : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        # Frames.left_elbow_joint         : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        # Frames.left_wrist_roll_joint    : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        # Frames.left_wrist_pitch_joint   : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        # Frames.left_wrist_yaw_joint     : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        
        # Frames.right_shoulder_pitch_joint: Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        # Frames.right_shoulder_roll_joint : Geometry(type='sphere', radius=0.06, color=VizColor.collision_volume),
        # Frames.right_shoulder_yaw_joint  : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        # Frames.right_elbow_joint         : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        # Frames.right_wrist_roll_joint    : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        # Frames.right_wrist_pitch_joint   : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        # Frames.right_wrist_yaw_joint     : Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        
        Frames.L_ee: Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        Frames.R_ee: Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),

        # Frames.torso_link_1: Geometry(type='sphere', radius=0.10, color=VizColor.collision_volume),
        # Frames.torso_link_2: Geometry(type='sphere', radius=0.10, color=VizColor.collision_volume),
        Frames.torso_link_3: Geometry(type='sphere', radius=0.08, color=VizColor.collision_volume),
        
        # Frames.pelvis_link_1: Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        # Frames.pelvis_link_2: Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume),
        # Frames.pelvis_link_3: Geometry(type='sphere', radius=0.05, color=VizColor.collision_volume)
    }

    # Pairs of adjacent joints to be ignored in collision checking
    AdjacentCollisionVolPairs = [
        # [Frames.waist_yaw_joint, Frames.waist_roll_joint],
        # [Frames.waist_yaw_joint, Frames.waist_pitch_joint],
        # [Frames.waist_yaw_joint, Frames.torso_link_1],
        # [Frames.waist_roll_joint, Frames.waist_pitch_joint],
        # [Frames.waist_roll_joint, Frames.torso_link_1],
        # [Frames.waist_pitch_joint, Frames.torso_link_1],
        # [Frames.torso_link_1, Frames.torso_link_2],
        # [Frames.torso_link_1, Frames.torso_link_3],
        # [Frames.torso_link_2, Frames.torso_link_3],

        # [Frames.left_shoulder_pitch_joint, Frames.torso_link_1],
        # [Frames.left_shoulder_pitch_joint, Frames.torso_link_2],
        # [Frames.left_shoulder_roll_joint, Frames.torso_link_1],
        # [Frames.left_shoulder_roll_joint, Frames.torso_link_2],
        # [Frames.left_shoulder_pitch_joint, Frames.left_shoulder_roll_joint],
        # [Frames.left_shoulder_pitch_joint, Frames.left_shoulder_yaw_joint],
        # [Frames.left_shoulder_roll_joint, Frames.left_shoulder_yaw_joint],
        # [Frames.left_shoulder_yaw_joint, Frames.left_elbow_joint],
        # [Frames.left_elbow_joint, Frames.left_wrist_roll_joint],
        # [Frames.left_wrist_roll_joint, Frames.left_wrist_pitch_joint],
        # [Frames.left_wrist_roll_joint, Frames.left_wrist_yaw_joint],
        # [Frames.left_wrist_roll_joint, Frames.L_ee],
        # [Frames.left_wrist_pitch_joint, Frames.left_wrist_yaw_joint],
        # [Frames.left_wrist_pitch_joint, Frames.L_ee],
        # [Frames.left_wrist_yaw_joint, Frames.L_ee],

        # [Frames.right_shoulder_pitch_joint, Frames.torso_link_1],
        # [Frames.right_shoulder_pitch_joint, Frames.torso_link_2],
        # [Frames.right_shoulder_roll_joint, Frames.torso_link_1],
        # [Frames.right_shoulder_roll_joint, Frames.torso_link_2],
        # [Frames.right_shoulder_pitch_joint, Frames.right_shoulder_roll_joint],
        # [Frames.right_shoulder_pitch_joint, Frames.right_shoulder_yaw_joint],
        # [Frames.right_shoulder_roll_joint, Frames.right_shoulder_yaw_joint],
        # [Frames.right_shoulder_yaw_joint, Frames.right_elbow_joint],
        # [Frames.right_elbow_joint, Frames.right_wrist_roll_joint],
        # [Frames.right_wrist_roll_joint, Frames.right_wrist_pitch_joint],
        # [Frames.right_wrist_roll_joint, Frames.right_wrist_yaw_joint],
        # [Frames.right_wrist_roll_joint, Frames.R_ee],
        # [Frames.right_wrist_pitch_joint, Frames.right_wrist_yaw_joint],
        # [Frames.right_wrist_pitch_joint, Frames.R_ee],
        # [Frames.right_wrist_yaw_joint, Frames.R_ee],
    ]

    SelfCollisionVolIgnored = [
        # Frames.left_hip_pitch_joint,
        # Frames.left_hip_roll_joint,
        # Frames.left_hip_yaw_joint,
        # Frames.left_knee_joint,
        # Frames.left_ankle_pitch_joint,
        # Frames.left_ankle_roll_joint,
        
        # Frames.right_hip_pitch_joint,
        # Frames.right_hip_roll_joint,
        # Frames.right_hip_yaw_joint,
        # Frames.right_knee_joint,
        # Frames.right_ankle_pitch_joint,
        # Frames.right_ankle_roll_joint,
        
        # Frames.waist_yaw_joint,
        # Frames.waist_roll_joint,
        # Frames.waist_pitch_joint,
        
        # Frames.left_shoulder_pitch_joint,
        # Frames.left_shoulder_yaw_joint,
        # Frames.left_wrist_roll_joint,
        # Frames.left_wrist_pitch_joint,
        # Frames.left_wrist_yaw_joint,
        
        # Frames.right_shoulder_pitch_joint,
        # Frames.right_shoulder_yaw_joint,
        # Frames.right_wrist_roll_joint,
        # Frames.right_wrist_pitch_joint,
        # Frames.right_wrist_yaw_joint,
        
        # Frames.pelvis_link_1,
        # Frames.pelvis_link_2,
        # Frames.pelvis_link_3,
    ]
    
    EnvCollisionVolIgnored = [
        # Frames.left_hip_pitch_joint,
        # Frames.left_hip_roll_joint,
        # Frames.left_hip_yaw_joint,
        # Frames.left_knee_joint,
        # Frames.left_ankle_pitch_joint,
        # Frames.left_ankle_roll_joint,
        
        # Frames.right_hip_pitch_joint,
        # Frames.right_hip_roll_joint,
        # Frames.right_hip_yaw_joint,
        # Frames.right_knee_joint,
        # Frames.right_ankle_pitch_joint,
        # Frames.right_ankle_roll_joint,
        
        # Frames.waist_yaw_joint,
        # Frames.waist_roll_joint,
        # Frames.waist_pitch_joint,
        
        # Frames.pelvis_link_1,
        # Frames.pelvis_link_2,
        # Frames.pelvis_link_3,
    ]

    VisualizeSafeZone = [
        Frames.L_ee,
        Frames.R_ee,
    ]
    
    VisualizePhiTraj = [
        Frames.L_ee,
        Frames.R_ee,
    ]