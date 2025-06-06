from enum import IntEnum
from spark_robot.base.base_robot_config import RobotConfig
from spark_utils import Geometry
import numpy as np

kNotUsedJoint = 29

class G1RightArmDynamic2Config(RobotConfig):
    
    # ---------------------------------------------------------------------------- #
    #                                      Kinematics                              #
    # ---------------------------------------------------------------------------- #
    
    kinematics_class_name = "G1RightArmKinematics"
    mujoco_model_path = "g1/g1_29dof_right_arm.xml"
    joint_to_lock = [
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
    
    NumTotalMotors = 35
    
    class RealMotors(IntEnum):
        
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
        RealMotors.RightShoulderPitch: (-3.0892, 2.6704),
        RealMotors.RightShoulderRoll: (-2.2515, 1.5882),
        RealMotors.RightShoulderYaw: (-2.618, 2.618),
        RealMotors.RightElbow: (-1.0472, 2.0944),
        RealMotors.RightWristRoll: (-1.972222054, 1.972222054),
        RealMotors.RightWristPitch: (-1.614429558, 1.614429558),
        RealMotors.RightWristYaw: (-1.614429558, 1.614429558)
    }
    
    NormalMotor = []
    
    WeakMotor = [
        RealMotors.RightShoulderPitch,
        RealMotors.RightShoulderRoll,
        RealMotors.RightShoulderYaw,
        RealMotors.RightElbow,
    ]
    
    DelicateMotor = [
        RealMotors.RightWristRoll,
        RealMotors.RightWristPitch,
        RealMotors.RightWristYaw,
    ]
    
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
    
    DefaultDoFVal = {
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
        aRightShoulderPitch = 0
        aRightShoulderRoll  = 1
        aRightShoulderYaw   = 2
        aRightElbow         = 3
        aRightWristRoll     = 4
        aRightWristPitch    = 5
        aRightWristYaw      = 6
    
    ControlLimit = {
        Control.aRightShoulderPitch: 150.0,
        Control.aRightShoulderRoll : 150.0,
        Control.aRightShoulderYaw  : 150.0,
        Control.aRightElbow        : 150.0,
        Control.aRightWristRoll    : 150.0,
        Control.aRightWristPitch   : 150.0,
        Control.aRightWristYaw     : 150.0,
    }

    NormalControl = []
    
    WeakControl = [
        Control.aRightShoulderPitch,
        Control.aRightShoulderRoll,
        Control.aRightShoulderYaw,
        Control.aRightElbow,
    ]
    
    DelicateControl = [
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

        RightShoulderPitch = 0
        RightShoulderRoll  = 1
        RightShoulderYaw   = 2
        RightElbow         = 3
        RightWristRoll     = 4
        RightWristPitch    = 5
        RightWristYaw      = 6
        

    class MujocoMotors(IntEnum):
        
        RightShoulderPitch = 0
        RightShoulderRoll  = 1
        RightShoulderYaw   = 2
        RightElbow         = 3
        RightWristRoll     = 4
        RightWristPitch    = 5
        RightWristYaw      = 6
        
    # Kp parameters for Mujoco Motors
    MujocoMotorKps = {
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

        MujocoMotors.RightShoulderPitch : 2.0,
        MujocoMotors.RightShoulderRoll  : 2.0,
        MujocoMotors.RightShoulderYaw   : 2.0,
        MujocoMotors.RightElbow         : 2.0,
        MujocoMotors.RightWristRoll     : 1.0,
        MujocoMotors.RightWristPitch    : 1.0,
        MujocoMotors.RightWristYaw      : 1.0,
    }

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

        MujocoMotors.RightShoulderPitch : Control.aRightShoulderPitch,
        MujocoMotors.RightShoulderRoll  : Control.aRightShoulderRoll,
        MujocoMotors.RightShoulderYaw   : Control.aRightShoulderYaw,
        MujocoMotors.RightElbow         : Control.aRightElbow,
        MujocoMotors.RightWristRoll     : Control.aRightWristRoll,
        MujocoMotors.RightWristPitch    : Control.aRightWristPitch,
        MujocoMotors.RightWristYaw      : Control.aRightWristYaw,

    }

    # Mapping from real motors to Control
    RealMotor_to_Control = {
        
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
    
    EnvCollisionVolIgnored = []
    
    VisualizeSafeZone = [
        Frames.R_ee,
    ]

    VisualizePhiTraj = [
        Frames.R_ee,
    ]