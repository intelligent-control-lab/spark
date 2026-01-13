from enum import IntEnum
from spark_robot.base.base_robot_config import RobotConfig
from spark_utils import Geometry, VizColor
import numpy as np

class R1LiteDualArmDynamic1Config(RobotConfig):
    
    # ---------------------------------------------------------------------------- #
    #                                      Kinematics                              #
    # ---------------------------------------------------------------------------- #
    
    kinematics_class_name = "R1LiteDualArmKinematics"
    mujoco_model_path = "r1lite/r1lite_dual_arm.xml"
    collision_spheres_json_path = "r1lite/config/r1lite_collision_spheres.json"
    joint_to_lock = [

    ]
    
    # ---------------------------------------------------------------------------- #
    #                                   hardware                                   #
    # ---------------------------------------------------------------------------- #
    
    NumTotalMotors = 25
    
    class RealMotors(IntEnum):

        RightJoint1 = 0
        RightJoint2 = 1
        RightJoint3 = 2
        RightJoint4 = 3
        RightJoint5 = 4
        RightJoint6 = 5

        LeftJoint1  = 6
        LeftJoint2  = 7
        LeftJoint3  = 8
        LeftJoint4  = 9
        LeftJoint5  = 10
        LeftJoint6  = 11

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

        RealMotors.LeftJoint1,
        RealMotors.LeftJoint2,
        RealMotors.LeftJoint3,
        RealMotors.LeftJoint4,
        RealMotors.LeftJoint5,
        RealMotors.LeftJoint6,
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

        LeftJoint1  = 6
        LeftJoint2  = 7
        LeftJoint3  = 8
        LeftJoint4  = 9
        LeftJoint5  = 10
        LeftJoint6  = 11

    DefaultDoFVal = {
        DoFs.RightJoint1          : 0.0,
        DoFs.RightJoint2          : 0.0,
        DoFs.RightJoint3          : 0.0,
        DoFs.RightJoint4          : 0.0,
        DoFs.RightJoint5          : 0.0,
        DoFs.RightJoint6          : 0.0,

        DoFs.LeftJoint1          : 0.0,
        DoFs.LeftJoint2          : 0.0,
        DoFs.LeftJoint3          : 0.0,
        DoFs.LeftJoint4          : 0.0,
        DoFs.LeftJoint5          : 0.0,
        DoFs.LeftJoint6          : 0.0,
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

        vLeftJoint1   = 6
        vLeftJoint2   = 7
        vLeftJoint3   = 8
        vLeftJoint4   = 9
        vLeftJoint5   = 10
        vLeftJoint6   = 11


    ControlLimit = {
        Control.vRightJoint1          : 1,
        Control.vRightJoint2          : 1,
        Control.vRightJoint3          : 1,
        Control.vRightJoint4          : 1,
        Control.vRightJoint5          : 1,
        Control.vRightJoint6          : 1,

        Control.vLeftJoint1          : 1,
        Control.vLeftJoint2          : 1,
        Control.vLeftJoint3          : 1,
        Control.vLeftJoint4          : 1,
        Control.vLeftJoint5          : 1,
        Control.vLeftJoint6          : 1,
    }

    NormalControl = [
        Control.vRightJoint1,
        Control.vRightJoint2,
        Control.vRightJoint3,
        Control.vRightJoint4,
        Control.vRightJoint5,
        Control.vRightJoint6,

        Control.vLeftJoint1,
        Control.vLeftJoint2,
        Control.vLeftJoint3,
        Control.vLeftJoint4,
        Control.vLeftJoint5,
        Control.vLeftJoint6,
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

        LeftJoint1   = 6
        LeftJoint2   = 7
        LeftJoint3   = 8
        LeftJoint4   = 9
        LeftJoint5   = 10
        LeftJoint6   = 11

    class MujocoMotors(IntEnum):

        RightJoint1   = 0
        RightJoint2   = 1
        RightJoint3   = 2
        RightJoint4   = 3
        RightJoint5   = 4
        RightJoint6   = 5

        LeftJoint1   = 6
        LeftJoint2   = 7
        LeftJoint3   = 8
        LeftJoint4   = 9
        LeftJoint5   = 10
        LeftJoint6   = 11

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

        MujocoDoFs.LeftJoint1   : DoFs.LeftJoint1,
        MujocoDoFs.LeftJoint2   : DoFs.LeftJoint2,
        MujocoDoFs.LeftJoint3   : DoFs.LeftJoint3,
        MujocoDoFs.LeftJoint4   : DoFs.LeftJoint4,
        MujocoDoFs.LeftJoint5   : DoFs.LeftJoint5,
        MujocoDoFs.LeftJoint6   : DoFs.LeftJoint6,
    }

    # Mapping from DoFs to Mujoco DoFs
    DoF_to_MujocoDoF = {
        DoFs.RightJoint1  : MujocoDoFs.RightJoint1,
        DoFs.RightJoint2  : MujocoDoFs.RightJoint2,
        DoFs.RightJoint3  : MujocoDoFs.RightJoint3,
        DoFs.RightJoint4  : MujocoDoFs.RightJoint4,
        DoFs.RightJoint5  : MujocoDoFs.RightJoint5,
        DoFs.RightJoint6  : MujocoDoFs.RightJoint6,

        DoFs.LeftJoint1  : MujocoDoFs.LeftJoint1,
        DoFs.LeftJoint2  : MujocoDoFs.LeftJoint2,
        DoFs.LeftJoint3  : MujocoDoFs.LeftJoint3,
        DoFs.LeftJoint4  : MujocoDoFs.LeftJoint4,
        DoFs.LeftJoint5  : MujocoDoFs.LeftJoint5,
        DoFs.LeftJoint6  : MujocoDoFs.LeftJoint6,
    }

    # Mapping from Mujoco Motors to Control
    MujocoMotor_to_Control = {
        MujocoMotors.RightJoint1   : Control.vRightJoint1,
        MujocoMotors.RightJoint2   : Control.vRightJoint2,
        MujocoMotors.RightJoint3   : Control.vRightJoint3,
        MujocoMotors.RightJoint4   : Control.vRightJoint4,
        MujocoMotors.RightJoint5   : Control.vRightJoint5,
        MujocoMotors.RightJoint6   : Control.vRightJoint6,

        MujocoMotors.LeftJoint1   : Control.vLeftJoint1,
        MujocoMotors.LeftJoint2   : Control.vLeftJoint2,
        MujocoMotors.LeftJoint3   : Control.vLeftJoint3,
        MujocoMotors.LeftJoint4   : Control.vLeftJoint4,
        MujocoMotors.LeftJoint5   : Control.vLeftJoint5,
        MujocoMotors.LeftJoint6   : Control.vLeftJoint6,
    }
    
    # ---------------------------------------------------------------------------- #
    #                                    Real                                    #
    # ---------------------------------------------------------------------------- #
    class RealDoFs(IntEnum):
        
        RightJoint1 = 0
        RightJoint2 = 1
        RightJoint3 = 2
        RightJoint4 = 3
        RightJoint5 = 4
        RightJoint6 = 5

        LeftJoint1  = 6
        LeftJoint2  = 7
        LeftJoint3  = 8
        LeftJoint4  = 9
        LeftJoint5  = 10
        LeftJoint6  = 11
    
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

        RealDoFs.LeftJoint1   : DoFs.LeftJoint1,
        RealDoFs.LeftJoint2   : DoFs.LeftJoint2,
        RealDoFs.LeftJoint3   : DoFs.LeftJoint3,
        RealDoFs.LeftJoint4   : DoFs.LeftJoint4,
        RealDoFs.LeftJoint5   : DoFs.LeftJoint5,
        RealDoFs.LeftJoint6   : DoFs.LeftJoint6,
    }

    # Mapping from DoFs to Real DoFs
    DoF_to_RealDoF = {
        DoFs.RightJoint1  : RealDoFs.RightJoint1,
        DoFs.RightJoint2  : RealDoFs.RightJoint2,
        DoFs.RightJoint3  : RealDoFs.RightJoint3,
        DoFs.RightJoint4  : RealDoFs.RightJoint4,
        DoFs.RightJoint5  : RealDoFs.RightJoint5,
        DoFs.RightJoint6  : RealDoFs.RightJoint6,

        DoFs.LeftJoint1  : RealDoFs.LeftJoint1,
        DoFs.LeftJoint2  : RealDoFs.LeftJoint2,
        DoFs.LeftJoint3  : RealDoFs.LeftJoint3,
        DoFs.LeftJoint4  : RealDoFs.LeftJoint4,
        DoFs.LeftJoint5  : RealDoFs.LeftJoint5,
        DoFs.LeftJoint6  : RealDoFs.LeftJoint6,
    }

    # Mapping from real motors to Control
    RealMotor_to_Control = {
        RealMotors.RightJoint1  : Control.vRightJoint1,
        RealMotors.RightJoint2  : Control.vRightJoint2,
        RealMotors.RightJoint3  : Control.vRightJoint3,
        RealMotors.RightJoint4  : Control.vRightJoint4,
        RealMotors.RightJoint5  : Control.vRightJoint5,
        RealMotors.RightJoint6  : Control.vRightJoint6,

        RealMotors.LeftJoint1  : Control.vLeftJoint1,
        RealMotors.LeftJoint2  : Control.vLeftJoint2,
        RealMotors.LeftJoint3  : Control.vLeftJoint3,
        RealMotors.LeftJoint4  : Control.vLeftJoint4,
        RealMotors.LeftJoint5  : Control.vLeftJoint5,
        RealMotors.LeftJoint6  : Control.vLeftJoint6,
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