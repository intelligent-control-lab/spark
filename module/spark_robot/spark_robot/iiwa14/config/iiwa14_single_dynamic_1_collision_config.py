from enum import IntEnum
from spark_robot.base.base_robot_config import RobotConfig
from spark_utils import Geometry, VizColor
import numpy as np

class IIWA14SingleDynamic1CollisionConfig(RobotConfig):
    
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
        # Velocity control

        vJoint1   = 0
        vJoint2   = 1
        vJoint3   = 2
        vJoint4   = 3
        vJoint5   = 4
        vJoint6   = 5
        vJoint7   = 6

    ControlLimit = {
        Control.vJoint1          : 1,
        Control.vJoint2          : 1,
        Control.vJoint3          : 1,
        Control.vJoint4          : 1,
        Control.vJoint5          : 1,
        Control.vJoint6          : 1,
        Control.vJoint7          : 1,
    }

    NormalControl = [
        Control.vJoint1,
        Control.vJoint2,
        Control.vJoint3,
        Control.vJoint4,
        Control.vJoint5,
        Control.vJoint6,
        Control.vJoint7,
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
        MujocoMotors.Joint1   : Control.vJoint1,
        MujocoMotors.Joint2   : Control.vJoint2,
        MujocoMotors.Joint3   : Control.vJoint3,
        MujocoMotors.Joint4   : Control.vJoint4,
        MujocoMotors.Joint5   : Control.vJoint5,
        MujocoMotors.Joint6   : Control.vJoint6,
        MujocoMotors.Joint7   : Control.vJoint7,
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
        RealMotors.Joint1  : Control.vJoint1,
        RealMotors.Joint2  : Control.vJoint2,
        RealMotors.Joint3  : Control.vJoint3,
        RealMotors.Joint4  : Control.vJoint4,
        RealMotors.Joint5  : Control.vJoint5,
        RealMotors.Joint6  : Control.vJoint6,
        RealMotors.Joint7  : Control.vJoint7,
    }

    # ---------------------------------------------------------------------------- #
    #                                   Cartesian                                  #
    # ---------------------------------------------------------------------------- #
    
    class Frames(IntEnum):
        R_ee = 0

        iiwa_link_1_sphere_0 = 1
        iiwa_link_1_sphere_1 = 2
        iiwa_link_2_sphere_0 = 3
        iiwa_link_2_sphere_1 = 4
        iiwa_link_2_sphere_2 = 5
        iiwa_link_3_sphere_0 = 6
        iiwa_link_3_sphere_1 = 7
        iiwa_link_3_sphere_2 = 8
        iiwa_link_4_sphere_0 = 9
        iiwa_link_4_sphere_1 = 10
        iiwa_link_5_sphere_0 = 11
        iiwa_link_5_sphere_1 = 12
        iiwa_link_5_sphere_2 = 13
        iiwa_link_5_sphere_3 = 14
        iiwa_link_5_sphere_4 = 15
        iiwa_link_5_sphere_5 = 16
        iiwa_link_6_sphere_0 = 17
        iiwa_link_7_sphere_0 = 18
    CollisionVol = {
        Frames.R_ee: Geometry(type='sphere', radius=0.05),
        Frames.iiwa_link_1_sphere_0: Geometry(type='sphere', radius=0.111126),
        Frames.iiwa_link_1_sphere_1: Geometry(type='sphere', radius=0.107078),
        Frames.iiwa_link_2_sphere_0: Geometry(type='sphere', radius=0.109519),
        Frames.iiwa_link_2_sphere_1: Geometry(type='sphere', radius=0.092455),
        Frames.iiwa_link_2_sphere_2: Geometry(type='sphere', radius=0.080673),
        Frames.iiwa_link_3_sphere_0: Geometry(type='sphere', radius=0.100601),
        Frames.iiwa_link_3_sphere_1: Geometry(type='sphere', radius=0.094024),
        Frames.iiwa_link_3_sphere_2: Geometry(type='sphere', radius=0.092457),
        Frames.iiwa_link_4_sphere_0: Geometry(type='sphere', radius=0.109693),
        Frames.iiwa_link_4_sphere_1: Geometry(type='sphere', radius=0.094005),
        Frames.iiwa_link_5_sphere_0: Geometry(type='sphere', radius=0.084118),
        Frames.iiwa_link_5_sphere_1: Geometry(type='sphere', radius=0.084618),
        Frames.iiwa_link_5_sphere_2: Geometry(type='sphere', radius=0.054473),
        Frames.iiwa_link_5_sphere_3: Geometry(type='sphere', radius=0.053816),
        Frames.iiwa_link_5_sphere_4: Geometry(type='sphere', radius=0.048896),
        Frames.iiwa_link_5_sphere_5: Geometry(type='sphere', radius=0.048153),
        Frames.iiwa_link_6_sphere_0: Geometry(type='sphere', radius=0.10125),
        Frames.iiwa_link_7_sphere_0: Geometry(type='sphere', radius=0.058188),
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