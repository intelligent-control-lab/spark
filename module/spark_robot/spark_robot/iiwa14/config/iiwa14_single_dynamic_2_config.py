from enum import IntEnum
from spark_robot.base.base_robot_config import RobotConfig
from spark_utils import Geometry, VizColor
import numpy as np

class IIWA14SingleDynamic2Config(RobotConfig):
    
    # ---------------------------------------------------------------------------- #
    #                                      Kinematics                              #
    # ---------------------------------------------------------------------------- #
    
    kinematics_class_name = "IIWA14SingleKinematics"
    mujoco_model_path = "iiwa14/iiwa14_single_scene.xml"
    collision_spheres_json_path = "iiwa14/config/iiwa14_single_collision_spheres.json"
    joint_to_lock = [
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
    
    NumTotalMotors = 35
    
    class RealMotors(IntEnum):

        Joint1 = 0
        Joint2 = 1
        Joint3 = 2
        Joint4 = 3
        Joint5 = 4
        Joint6 = 5
        Joint7 = 6
    
    # Based on https://support.unitree.com/home/en/G1_developer/about_G1
    RealMotorPosLimit = {
        }
    
    NormalMotor = [
        RealMotors.Joint1,
        RealMotors.Joint2,
        RealMotors.Joint3,
        RealMotors.Joint4,
        RealMotors.Joint5,
        RealMotors.Joint6,
        RealMotors.Joint7,
    ]
    
    WeakMotor = []
    
    DelicateMotor = []
    
    # ---------------------------------------------------------------------------- #
    #                                      DoF                                     #
    # ---------------------------------------------------------------------------- #

    class DoFs (IntEnum):

        Joint1 = 0
        Joint2 = 1
        Joint3 = 2
        Joint4 = 3
        Joint5 = 4
        Joint6 = 5
        Joint7 = 6
    
    DefaultDoFVal = {
        DoFs.Joint1          : -np.pi / 2,
        DoFs.Joint2          : 0.0,
        DoFs.Joint3          : 0.0,
        DoFs.Joint4          : np.pi/2,
        DoFs.Joint5          : 0.0,
        DoFs.Joint6          : -np.pi/2,
        DoFs.Joint7          : 0.0,
    }

    # ---------------------------------------------------------------------------- #
    #                                   Dynamics                                   #
    # ---------------------------------------------------------------------------- #

    class Control (IntEnum):
        # Acceleration control

        aJoint1   = 0
        aJoint2   = 1
        aJoint3   = 2
        aJoint4   = 3
        aJoint5   = 4
        aJoint6   = 5
        aJoint7   = 6

    ControlLimit = {
        Control.aJoint1          : 10,
        Control.aJoint2          : 10,
        Control.aJoint3          : 10,
        Control.aJoint4          : 10,
        Control.aJoint5          : 10,
        Control.aJoint6          : 10,
        Control.aJoint7          : 10,
    }

    NormalControl = [
        Control.aJoint1,
        Control.aJoint2,
        Control.aJoint3,
        Control.aJoint4,
        Control.aJoint5,
        Control.aJoint6,
        Control.aJoint7,
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

        Joint1   = 0
        Joint2   = 1
        Joint3   = 2
        Joint4   = 3
        Joint5   = 4
        Joint6   = 5
        Joint7   = 6

    class MujocoMotors(IntEnum):

        Joint1   = 0
        Joint2   = 1
        Joint3   = 2
        Joint4   = 3
        Joint5   = 4
        Joint6   = 5
        Joint7   = 6

    # ---------------------------------------------------------------------------- #
    #                                   Mujoco PID Parameters                      #
    # ---------------------------------------------------------------------------- #
    
    # Kp parameters for Mujoco Motors
    MujocoMotorKps = {
        MujocoMotors.Joint1   : 1000.0,
        MujocoMotors.Joint2   : 1000.0,
        MujocoMotors.Joint3   : 1000.0,
        MujocoMotors.Joint4   : 1000.0,
        MujocoMotors.Joint5   : 1000.0,
        MujocoMotors.Joint6   : 1000.0,
    }
    
    # Kd parameters for Mujoco Motors
    MujocoMotorKds = {
        MujocoMotors.Joint1   : 1.0,
        MujocoMotors.Joint2   : 1.0,
        MujocoMotors.Joint3   : 1.0,
        MujocoMotors.Joint4   : 1.0,
        MujocoMotors.Joint5   : 1.0,
        MujocoMotors.Joint6   : 1.0,
    }

    # ---------------------------------------------------------------------------- #
    #                                   Mujoco Mappings                            #
    # ---------------------------------------------------------------------------- #

    # Mapping from Mujoco DoFs to DoFs
    MujocoDoF_to_DoF = {
        MujocoDoFs.Joint1   : DoFs.Joint1,
        MujocoDoFs.Joint2   : DoFs.Joint2,
        MujocoDoFs.Joint3   : DoFs.Joint3,
        MujocoDoFs.Joint4   : DoFs.Joint4,
        MujocoDoFs.Joint5   : DoFs.Joint5,
        MujocoDoFs.Joint6   : DoFs.Joint6,
        MujocoDoFs.Joint7   : DoFs.Joint7,
    }

    # Mapping from DoFs to Mujoco DoFs
    DoF_to_MujocoDoF = {
        DoFs.Joint1  : MujocoDoFs.Joint1,
        DoFs.Joint2  : MujocoDoFs.Joint2,
        DoFs.Joint3  : MujocoDoFs.Joint3,
        DoFs.Joint4  : MujocoDoFs.Joint4,
        DoFs.Joint5  : MujocoDoFs.Joint5,
        DoFs.Joint6  : MujocoDoFs.Joint6,
        DoFs.Joint7  : MujocoDoFs.Joint7,
    }

    # Mapping from Mujoco Motors to Control
    MujocoMotor_to_Control = {
        MujocoMotors.Joint1   : Control.aJoint1,
        MujocoMotors.Joint2   : Control.aJoint2,
        MujocoMotors.Joint3   : Control.aJoint3,
        MujocoMotors.Joint4   : Control.aJoint4,
        MujocoMotors.Joint5   : Control.aJoint5,
        MujocoMotors.Joint6   : Control.aJoint6,
        MujocoMotors.Joint7   : Control.aJoint7,
    }
    
    # ---------------------------------------------------------------------------- #
    #                                    Real                                    #
    # ---------------------------------------------------------------------------- #
    class RealDoFs(IntEnum):
        
        Joint1   = 0
        Joint2   = 1
        Joint3   = 2
        Joint4   = 3
        Joint5   = 4
        Joint6   = 5
        Joint7   = 6
    
    # ---------------------------------------------------------------------------- #
    #                                   Real Mappings                                   #
    # ---------------------------------------------------------------------------- #
    
    
    # Mapping from Real DoFs to DoFs
    RealDoF_to_DoF = {
        RealDoFs.Joint1   : DoFs.Joint1,
        RealDoFs.Joint2   : DoFs.Joint2,
        RealDoFs.Joint3   : DoFs.Joint3,
        RealDoFs.Joint4   : DoFs.Joint4,
        RealDoFs.Joint5   : DoFs.Joint5,
        RealDoFs.Joint6   : DoFs.Joint6,
        RealDoFs.Joint7   : DoFs.Joint7,
    }

    # Mapping from DoFs to Real DoFs
    DoF_to_RealDoF = {
        DoFs.Joint1  : RealDoFs.Joint1,
        DoFs.Joint2  : RealDoFs.Joint2,
        DoFs.Joint3  : RealDoFs.Joint3,
        DoFs.Joint4  : RealDoFs.Joint4,
        DoFs.Joint5  : RealDoFs.Joint5,
        DoFs.Joint6  : RealDoFs.Joint6,
        DoFs.Joint7  : RealDoFs.Joint7,
    }

    # Mapping from real motors to Control
    RealMotor_to_Control = {
        RealMotors.Joint1  : Control.aJoint1,
        RealMotors.Joint2  : Control.aJoint2,
        RealMotors.Joint3  : Control.aJoint3,
        RealMotors.Joint4  : Control.aJoint4,
        RealMotors.Joint5  : Control.aJoint5,
        RealMotors.Joint6  : Control.aJoint6,
        RealMotors.Joint7  : Control.aJoint7,
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