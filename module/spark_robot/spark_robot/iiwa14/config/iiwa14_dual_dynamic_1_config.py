from enum import IntEnum
from spark_robot.base.base_robot_config import RobotConfig
from spark_utils import Geometry, VizColor
import numpy as np

class IIWA14DualDynamic1Config(RobotConfig):
    
    # ---------------------------------------------------------------------------- #
    #                                      Kinematics                              #
    # ---------------------------------------------------------------------------- #
    
    kinematics_class_name = "IIWA14DualKinematics"
    mujoco_model_path = "iiwa14/iiwa14_2f85_dual.xml"
    # mujoco_model_path = "iiwa14/iiwa14_dual_scene.xml"
    joint_to_lock = [
        "right/right_driver_joint",
        "right/right_coupler_joint",
        "right/right_spring_link_joint",
        "right/right_follower_joint",
        "right/left_driver_joint",
        "right/left_coupler_joint",
        "right/left_spring_link_joint",
        "right/left_follower_joint",

        "left/right_driver_joint",
        "left/right_coupler_joint",
        "left/right_spring_link_joint",
        "left/right_follower_joint",
        "left/left_driver_joint",
        "left/left_coupler_joint",
        "left/left_spring_link_joint",
        "left/left_follower_joint"
    ]
    
    # ---------------------------------------------------------------------------- #
    #                                   hardware                                   #
    # ---------------------------------------------------------------------------- #
    
    NumTotalMotors = 35
    
    class RealMotors(IntEnum):

        RightJoint1 = 0
        RightJoint2 = 1
        RightJoint3 = 2
        RightJoint4 = 3
        RightJoint5 = 4
        RightJoint6 = 5
        RightJoint7 = 6

        LeftJoint1 = 7
        LeftJoint2 = 8
        LeftJoint3 = 9
        LeftJoint4 = 10
        LeftJoint5 = 11
        LeftJoint6 = 12
        LeftJoint7 = 13
    
    # Based on https://support.unitree.com/home/en/G1_developer/about_G1
    RealMotorPosLimit = {
        }
    
    NormalMotor = [
        RealMotors.RightJoint1,
        RealMotors.RightJoint2,
        RealMotors.RightJoint3,
        RealMotors.RightJoint4,
        RealMotors.RightJoint5,
        RealMotors.RightJoint6,
        RealMotors.RightJoint7,

        RealMotors.LeftJoint1,
        RealMotors.LeftJoint2,
        RealMotors.LeftJoint3,
        RealMotors.LeftJoint4,
        RealMotors.LeftJoint5,
        RealMotors.LeftJoint6,
        RealMotors.LeftJoint7,
    ]
    
    WeakMotor = []
    
    DelicateMotor = []
    
    # ---------------------------------------------------------------------------- #
    #                                      DoF                                     #
    # ---------------------------------------------------------------------------- #

    class DoFs (IntEnum):

        RightJoint1 = 0
        RightJoint2 = 1
        RightJoint3 = 2
        RightJoint4 = 3
        RightJoint5 = 4
        RightJoint6 = 5
        RightJoint7 = 6

        LeftJoint1 = 7
        LeftJoint2 = 8
        LeftJoint3 = 9
        LeftJoint4 = 10
        LeftJoint5 = 11
        LeftJoint6 = 12
        LeftJoint7 = 13
    
    DefaultDoFVal = {
        DoFs.RightJoint1          : -np.pi / 2,
        DoFs.RightJoint2          : 0.0,
        DoFs.RightJoint3          : 0.0,
        DoFs.RightJoint4          : np.pi/2,
        DoFs.RightJoint5          : 0.0,
        DoFs.RightJoint6          : -np.pi/2,
        DoFs.RightJoint7          : 0.0,

        DoFs.LeftJoint1          : np.pi / 2,
        DoFs.LeftJoint2          : 0.0,
        DoFs.LeftJoint3          : 0.0,
        DoFs.LeftJoint4          : np.pi/2,
        DoFs.LeftJoint5          : 0.0,
        DoFs.LeftJoint6          : -np.pi/2,
        DoFs.LeftJoint7          : 0.0,
    }

    # ---------------------------------------------------------------------------- #
    #                                   Dynamics                                   #
    # ---------------------------------------------------------------------------- #

    class Control (IntEnum):
        # Velocity control

        vRightJoint1   = 0
        vRightJoint2   = 1
        vRightJoint3   = 2
        vRightJoint4   = 3
        vRightJoint5   = 4
        vRightJoint6   = 5
        vRightJoint7   = 6

        vLeftJoint1   = 7
        vLeftJoint2   = 8
        vLeftJoint3   = 9
        vLeftJoint4   = 10
        vLeftJoint5   = 11
        vLeftJoint6   = 12
        vLeftJoint7   = 13


    ControlLimit = {
        Control.vRightJoint1          : 1,
        Control.vRightJoint2          : 1,
        Control.vRightJoint3          : 1,
        Control.vRightJoint4          : 1,
        Control.vRightJoint5          : 1,
        Control.vRightJoint6          : 1,
        Control.vRightJoint7          : 1,

        Control.vLeftJoint1          : 1,
        Control.vLeftJoint2          : 1,
        Control.vLeftJoint3          : 1,
        Control.vLeftJoint4          : 1,
        Control.vLeftJoint5          : 1,
        Control.vLeftJoint6          : 1,
        Control.vLeftJoint7          : 1,
    }

    NormalControl = [
        Control.vRightJoint1,
        Control.vRightJoint2,
        Control.vRightJoint3,
        Control.vRightJoint4,
        Control.vRightJoint5,
        Control.vRightJoint6,
        Control.vRightJoint7,

        Control.vLeftJoint1,
        Control.vLeftJoint2,
        Control.vLeftJoint3,
        Control.vLeftJoint4,
        Control.vLeftJoint5,
        Control.vLeftJoint6,
        Control.vLeftJoint7,
    ]
    
    WeakControl = []
    
    DelicateControl = []

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

        RightJoint1   = 0
        RightJoint2   = 1
        RightJoint3   = 2
        RightJoint4   = 3
        RightJoint5   = 4
        RightJoint6   = 5
        RightJoint7   = 6

        LeftJoint1   = 15
        LeftJoint2   = 16
        LeftJoint3   = 17
        LeftJoint4   = 18
        LeftJoint5   = 19
        LeftJoint6   = 20
        LeftJoint7   = 21

    class MujocoMotors(IntEnum):

        RightJoint1   = 0
        RightJoint2   = 1
        RightJoint3   = 2
        RightJoint4   = 3
        RightJoint5   = 4
        RightJoint6   = 5
        RightJoint7   = 6

        LeftJoint1   = 8
        LeftJoint2   = 9
        LeftJoint3   = 10
        LeftJoint4   = 11
        LeftJoint5   = 12
        LeftJoint6   = 13
        LeftJoint7   = 14

    # ---------------------------------------------------------------------------- #
    #                                   Mujoco PID Parameters                      #
    # ---------------------------------------------------------------------------- #
    
    # Kp parameters for Mujoco Motors
    MujocoMotorKps = {
        MujocoMotors.RightJoint1   : 1000.0,
        MujocoMotors.RightJoint2   : 1000.0,
        MujocoMotors.RightJoint3   : 1000.0,
        MujocoMotors.RightJoint4   : 1000.0,
        MujocoMotors.RightJoint5   : 1000.0,
        MujocoMotors.RightJoint6   : 1000.0,

        MujocoMotors.LeftJoint1   : 1000.0,
        MujocoMotors.LeftJoint2   : 1000.0,
        MujocoMotors.LeftJoint3   : 1000.0,
        MujocoMotors.LeftJoint4   : 1000.0,
        MujocoMotors.LeftJoint5   : 1000.0,
        MujocoMotors.LeftJoint6   : 1000.0,
    }
    
    # Kd parameters for Mujoco Motors
    MujocoMotorKds = {
        MujocoMotors.RightJoint1   : 0.1,
        MujocoMotors.RightJoint2   : 0.1,
        MujocoMotors.RightJoint3   : 0.1,
        MujocoMotors.RightJoint4   : 0.1,
        MujocoMotors.RightJoint5   : 0.1,
        MujocoMotors.RightJoint6   : 0.1,

        MujocoMotors.LeftJoint1   : 0.1,
        MujocoMotors.LeftJoint2   : 0.1,
        MujocoMotors.LeftJoint3   : 0.1,
        MujocoMotors.LeftJoint4   : 0.1,
        MujocoMotors.LeftJoint5   : 0.1,
        MujocoMotors.LeftJoint6   : 0.1,
    }

    # ---------------------------------------------------------------------------- #
    #                                   Mujoco Mappings                            #
    # ---------------------------------------------------------------------------- #

    # Mapping from Mujoco DoFs to DoFs
    MujocoDoF_to_DoF = {
        MujocoDoFs.RightJoint1   : DoFs.RightJoint1,
        MujocoDoFs.RightJoint2   : DoFs.RightJoint2,
        MujocoDoFs.RightJoint3   : DoFs.RightJoint3,
        MujocoDoFs.RightJoint4   : DoFs.RightJoint4,
        MujocoDoFs.RightJoint5   : DoFs.RightJoint5,
        MujocoDoFs.RightJoint6   : DoFs.RightJoint6,
        MujocoDoFs.RightJoint7   : DoFs.RightJoint7,

        MujocoDoFs.LeftJoint1   : DoFs.LeftJoint1,
        MujocoDoFs.LeftJoint2   : DoFs.LeftJoint2,
        MujocoDoFs.LeftJoint3   : DoFs.LeftJoint3,
        MujocoDoFs.LeftJoint4   : DoFs.LeftJoint4,
        MujocoDoFs.LeftJoint5   : DoFs.LeftJoint5,
        MujocoDoFs.LeftJoint6   : DoFs.LeftJoint6,
        MujocoDoFs.LeftJoint7   : DoFs.LeftJoint7,
    }

    # Mapping from DoFs to Mujoco DoFs
    DoF_to_MujocoDoF = {
        DoFs.RightJoint1  : MujocoDoFs.RightJoint1,
        DoFs.RightJoint2  : MujocoDoFs.RightJoint2,
        DoFs.RightJoint3  : MujocoDoFs.RightJoint3,
        DoFs.RightJoint4  : MujocoDoFs.RightJoint4,
        DoFs.RightJoint5  : MujocoDoFs.RightJoint5,
        DoFs.RightJoint6  : MujocoDoFs.RightJoint6,
        DoFs.RightJoint7  : MujocoDoFs.RightJoint7,

        DoFs.LeftJoint1  : MujocoDoFs.LeftJoint1,
        DoFs.LeftJoint2  : MujocoDoFs.LeftJoint2,
        DoFs.LeftJoint3  : MujocoDoFs.LeftJoint3,
        DoFs.LeftJoint4  : MujocoDoFs.LeftJoint4,
        DoFs.LeftJoint5  : MujocoDoFs.LeftJoint5,
        DoFs.LeftJoint6  : MujocoDoFs.LeftJoint6,
        DoFs.LeftJoint7  : MujocoDoFs.LeftJoint7,
    }

    # Mapping from Mujoco Motors to Control
    MujocoMotor_to_Control = {
        MujocoMotors.RightJoint1   : Control.vRightJoint1,
        MujocoMotors.RightJoint2   : Control.vRightJoint2,
        MujocoMotors.RightJoint3   : Control.vRightJoint3,
        MujocoMotors.RightJoint4   : Control.vRightJoint4,
        MujocoMotors.RightJoint5   : Control.vRightJoint5,
        MujocoMotors.RightJoint6   : Control.vRightJoint6,
        MujocoMotors.RightJoint7   : Control.vRightJoint7,

        MujocoMotors.LeftJoint1   : Control.vLeftJoint1,
        MujocoMotors.LeftJoint2   : Control.vLeftJoint2,
        MujocoMotors.LeftJoint3   : Control.vLeftJoint3,
        MujocoMotors.LeftJoint4   : Control.vLeftJoint4,
        MujocoMotors.LeftJoint5   : Control.vLeftJoint5,
        MujocoMotors.LeftJoint6   : Control.vLeftJoint6,
        MujocoMotors.LeftJoint7   : Control.vLeftJoint7,
    }
    
    # ---------------------------------------------------------------------------- #
    #                                    Real                                    #
    # ---------------------------------------------------------------------------- #
    class RealDoFs(IntEnum):
        
        RightJoint1   = 0
        RightJoint2   = 1
        RightJoint3   = 2
        RightJoint4   = 3
        RightJoint5   = 4
        RightJoint6   = 5
        RightJoint7   = 6

        LeftJoint1   = 7
        LeftJoint2   = 8
        LeftJoint3   = 9
        LeftJoint4   = 10
        LeftJoint5   = 11
        LeftJoint6   = 12
        LeftJoint7   = 13
    
    # ---------------------------------------------------------------------------- #
    #                                   Real Mappings                                   #
    # ---------------------------------------------------------------------------- #
    
    
    # Mapping from Real DoFs to DoFs
    RealDoF_to_DoF = {
        RealDoFs.RightJoint1   : DoFs.RightJoint1,
        RealDoFs.RightJoint2   : DoFs.RightJoint2,
        RealDoFs.RightJoint3   : DoFs.RightJoint3,
        RealDoFs.RightJoint4   : DoFs.RightJoint4,
        RealDoFs.RightJoint5   : DoFs.RightJoint5,
        RealDoFs.RightJoint6   : DoFs.RightJoint6,
        RealDoFs.RightJoint7   : DoFs.RightJoint7,

        RealDoFs.LeftJoint1   : DoFs.LeftJoint1,
        RealDoFs.LeftJoint2   : DoFs.LeftJoint2,
        RealDoFs.LeftJoint3   : DoFs.LeftJoint3,
        RealDoFs.LeftJoint4   : DoFs.LeftJoint4,
        RealDoFs.LeftJoint5   : DoFs.LeftJoint5,
        RealDoFs.LeftJoint6   : DoFs.LeftJoint6,
        RealDoFs.LeftJoint7   : DoFs.LeftJoint7,
    }

    # Mapping from DoFs to Real DoFs
    DoF_to_RealDoF = {
        DoFs.RightJoint1  : RealDoFs.RightJoint1,
        DoFs.RightJoint2  : RealDoFs.RightJoint2,
        DoFs.RightJoint3  : RealDoFs.RightJoint3,
        DoFs.RightJoint4  : RealDoFs.RightJoint4,
        DoFs.RightJoint5  : RealDoFs.RightJoint5,
        DoFs.RightJoint6  : RealDoFs.RightJoint6,
        DoFs.RightJoint7  : RealDoFs.RightJoint7,

        DoFs.LeftJoint1  : RealDoFs.LeftJoint1,
        DoFs.LeftJoint2  : RealDoFs.LeftJoint2,
        DoFs.LeftJoint3  : RealDoFs.LeftJoint3,
        DoFs.LeftJoint4  : RealDoFs.LeftJoint4,
        DoFs.LeftJoint5  : RealDoFs.LeftJoint5,
        DoFs.LeftJoint6  : RealDoFs.LeftJoint6,
        DoFs.LeftJoint7  : RealDoFs.LeftJoint7,
    }

    # Mapping from real motors to Control
    RealMotor_to_Control = {
        RealMotors.RightJoint1  : Control.vRightJoint1,
        RealMotors.RightJoint2  : Control.vRightJoint2,
        RealMotors.RightJoint3  : Control.vRightJoint3,
        RealMotors.RightJoint4  : Control.vRightJoint4,
        RealMotors.RightJoint5  : Control.vRightJoint5,
        RealMotors.RightJoint6  : Control.vRightJoint6,
        RealMotors.RightJoint7  : Control.vRightJoint7,

        RealMotors.LeftJoint1  : Control.vLeftJoint1,
        RealMotors.LeftJoint2  : Control.vLeftJoint2,
        RealMotors.LeftJoint3  : Control.vLeftJoint3,
        RealMotors.LeftJoint4  : Control.vLeftJoint4,
        RealMotors.LeftJoint5  : Control.vLeftJoint5,
        RealMotors.LeftJoint6  : Control.vLeftJoint6,
        RealMotors.LeftJoint7  : Control.vLeftJoint7,
    }

    # ---------------------------------------------------------------------------- #
    #                                   Cartesian                                  #
    # ---------------------------------------------------------------------------- #
    
    class Frames(IntEnum):
        
        R_ee = 0
        L_ee = 1
    
    # ---------------------------------------------------------------------------- #
    #                                   Collision                                  #
    # ---------------------------------------------------------------------------- #
    
    CollisionVol = {  
        Frames.R_ee: Geometry(type='sphere', radius=0.05),
        Frames.L_ee: Geometry(type='sphere', radius=0.05),
    }

    # Pairs of adjacent joints to be ignored in collision checking
    AdjacentCollisionVolPairs = []

    SelfCollisionVolIgnored = []
    
    EnvCollisionVolIgnored = []

    VisualizeSafeZone = [
        Frames.R_ee,
        Frames.L_ee,
    ]

    VisualizePhiTraj = [
        Frames.R_ee,
        Frames.L_ee,
    ]