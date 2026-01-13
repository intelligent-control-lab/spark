# AUTO-GENERATED FILE
# Base config: LRMate200iD3fSingleDynamic1Config
# Collision JSON: module/spark_robot/spark_robot/lrmate200id/config/lrmate200id_3f_single_collision_spheres.json
# JSON hash: 6cfa953b26ca5961
# DO NOT EDIT MANUALLY

from enum import IntEnum
from spark_robot.base.base_robot_config import RobotConfig
from spark_utils import Geometry, VizColor
import numpy as np

class LRMate200iD3fSingleDynamic1CollisionConfig(RobotConfig):
    
    # ---------------------------------------------------------------------------- #
    #                                      Kinematics                              #
    # ---------------------------------------------------------------------------- #
    
    kinematics_class_name = "LRMate200iD3fSingleKinematics"
    mujoco_model_path = "lrmate200id/lrmate200id_single_scene.xml"
    collision_spheres_json_path = "lrmate200id/config/lrmate200id_3f_single_collision_spheres.json"
    joint_to_lock = [
        "finger_A_joint_0",
        "finger_A_joint_1",
        "finger_A_joint_2",
        "finger_A_joint_3",
        "finger_B_joint_0",
        "finger_B_joint_1",
        "finger_B_joint_2",
        "finger_B_joint_3",
        "finger_C_joint_0",
        "finger_C_joint_1",
        "finger_C_joint_2",
        "finger_C_joint_3"
    ]
    
    # ---------------------------------------------------------------------------- #
    #                                   hardware                                   #
    # ---------------------------------------------------------------------------- #
    
    NumTotalMotors = 6
    
    class RealMotors(IntEnum):

        joint_1 = 0
        joint_2 = 1
        joint_3 = 2
        joint_4 = 3
        joint_5 = 4
        joint_6 = 5
    

    RealMotorPosLimit = {

        #RealMotors.joint_1: (-3.141592654,  3.141592654),
        #RealMotors.joint_2: (-2.138028334,  2.138028334),
        #RealMotors.joint_3: (-3.752457892,  3.752457892),
        #RealMotors.joint_4: (-3.316125579,  3.316125579),
        #RealMotors.joint_5: (-2.181661565,  2.181661565),
        #RealMotors.joint_6: (-6.283185307,  6.283185307),
        
    }
    
    NormalMotor = [
        RealMotors.joint_1,
        RealMotors.joint_2,
        RealMotors.joint_3,
        RealMotors.joint_4,
        RealMotors.joint_5,
        RealMotors.joint_6,
    ]
    
    WeakMotor = []
    
    DelicateMotor = []

    # ---------------------------------------------------------------------------- #
    #                                      DoF                                     #
    # ---------------------------------------------------------------------------- #

    class DoFs(IntEnum):

        joint_1 = 0
        joint_2 = 1
        joint_3 = 2
        joint_4 = 3
        joint_5 = 4
        joint_6 = 5
    
    DefaultDoFVal = {

        DoFs.joint_1 : 0.0,        
        DoFs.joint_2 : -0.3,        
        DoFs.joint_3 : 0.0,        
        DoFs.joint_4 : 0.0,        
        DoFs.joint_5 : -1.8,        
        DoFs.joint_6 : 0.0,        
    }

    # ---------------------------------------------------------------------------- #
    #                                   Dynamics                                   #
    # ---------------------------------------------------------------------------- #

    #class State(IntEnum):
    #    
    #    # lrmate200id arm joints
    #    joint_1 = 0
    #    joint_2 = 1
    #    joint_3 = 2
    #    joint_4 = 3
    #    joint_5 = 4
    #    joint_6 = 5
    #    
        
    class Control(IntEnum):

        vjoint_1 = 0
        vjoint_2 = 1
        vjoint_3 = 2
        vjoint_4 = 3
        vjoint_5 = 4
        vjoint_6 = 5
    
    ControlLimit = {

        Control.vjoint_1 : 1,
        Control.vjoint_2 : 1,
        Control.vjoint_3 : 1,
        Control.vjoint_4 : 1,
        Control.vjoint_5 : 1,
        Control.vjoint_6 : 1,
    }

    NormalControl = [
        Control.vjoint_1,
        Control.vjoint_2,
        Control.vjoint_3,
        Control.vjoint_4,
        Control.vjoint_5,
        Control.vjoint_6,
    ]
    
    WeakControl = []
    
    DelicateControl = []

    '''
        x_dot = f(x) + g(x) * control

        For velocity control, f = 0, g = I
    '''

    @property
    def num_state(self):
        return len(self.DoFs)

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

        joint_1 = 0
        joint_2 = 1
        joint_3 = 2
        joint_4 = 3
        joint_5 = 4
        joint_6 = 5

    class MujocoMotors(IntEnum):
        
        joint_1 = 0
        joint_2 = 1
        joint_3 = 2
        joint_4 = 3
        joint_5 = 4
        joint_6 = 5
        
    MujocoMotorKps = {

        MujocoMotors.joint_1 : 2000.0,  
        MujocoMotors.joint_2 : 3000.0,  
        MujocoMotors.joint_3 : 2500.0,  
        MujocoMotors.joint_4 : 1500.0,  
        MujocoMotors.joint_5 : 1200.0,  
        MujocoMotors.joint_6 : 800.0,   
        
    }
    
    MujocoMotorKds = {

        MujocoMotors.joint_1 : 10.0,    
        MujocoMotors.joint_2 : 30.0,    
        MujocoMotors.joint_3 : 12.0,    
        MujocoMotors.joint_4 : 8.0,     
        MujocoMotors.joint_5 : 6.0,     
        MujocoMotors.joint_6 : 4.0,     
        
    }

    # ---------------------------------------------------------------------------- #
    #                                   Mappings                                   #
    # ---------------------------------------------------------------------------- #

    # todo change dof to state mapping func

    # Mapping from DoFs to State
#    DoF_to_State = {
#        
#        # lrmate200id arm joints
#        DoFs.joint_1: State.joint_1,
#        DoFs.joint_2: State.joint_2,
#        DoFs.joint_3: State.joint_3,
#        DoFs.joint_4: State.joint_4,
#        DoFs.joint_5: State.joint_5,
#        DoFs.joint_6: State.joint_6,
#        
#
#    }

    # Mapping from State to DoFs
#    State_to_DoF = {
#
#        # lrmate200id arm joints
#        State.joint_1: DoFs.joint_1,
#        State.joint_2: DoFs.joint_2,
#        State.joint_3: DoFs.joint_3,
#        State.joint_4: DoFs.joint_4,
#        State.joint_5: DoFs.joint_5,
#        State.joint_6: DoFs.joint_6,
#        
#
#    }

    # Mapping from Mujoco DoFs to DoFs
    MujocoDoF_to_DoF = {

        MujocoDoFs.joint_1 : DoFs.joint_1,
        MujocoDoFs.joint_2 : DoFs.joint_2,
        MujocoDoFs.joint_3 : DoFs.joint_3,
        MujocoDoFs.joint_4 : DoFs.joint_4,
        MujocoDoFs.joint_5 : DoFs.joint_5,
        MujocoDoFs.joint_6 : DoFs.joint_6,
        

    }

    # Mapping from DoFs to Mujoco DoFs
    DoF_to_MujocoDoF = {

        DoFs.joint_1 : MujocoDoFs.joint_1,
        DoFs.joint_2 : MujocoDoFs.joint_2,
        DoFs.joint_3 : MujocoDoFs.joint_3,
        DoFs.joint_4 : MujocoDoFs.joint_4,
        DoFs.joint_5 : MujocoDoFs.joint_5,
        DoFs.joint_6 : MujocoDoFs.joint_6,
        

    }

    # Mapping from Mujoco Motors to Control
    MujocoMotor_to_Control = {

        MujocoMotors.joint_1 : Control.vjoint_1,
        MujocoMotors.joint_2 : Control.vjoint_2,
        MujocoMotors.joint_3 : Control.vjoint_3,
        MujocoMotors.joint_4 : Control.vjoint_4,
        MujocoMotors.joint_5 : Control.vjoint_5,
        MujocoMotors.joint_6 : Control.vjoint_6,
        

    }

    # Mapping from real motors to Control
    RealMotor_to_Control = {
        
        RealMotors.joint_1 : Control.vjoint_1,
        RealMotors.joint_2 : Control.vjoint_2,
        RealMotors.joint_3 : Control.vjoint_3,
        RealMotors.joint_4 : Control.vjoint_4,
        RealMotors.joint_5 : Control.vjoint_5,
        RealMotors.joint_6 : Control.vjoint_6,
        
        
    }

    # ---------------------------------------------------------------------------- #
    #                                   Cartesian                                  #
    # ---------------------------------------------------------------------------- #
    
    class Frames(IntEnum):
        R_ee = 0

        base_link_sphere_0 = 1
        link_1_sphere_0 = 2
        link_2_sphere_0 = 3
        link_2_sphere_1 = 4
        link_2_sphere_2 = 5
        link_2_sphere_3 = 6
        link_2_sphere_4 = 7
        link_3_sphere_0 = 8
        link_4_sphere_0 = 9
        link_4_sphere_1 = 10
        link_4_sphere_2 = 11
        link_4_sphere_3 = 12
        link_4_sphere_4 = 13
        link_5_sphere_0 = 14
        link_6_sphere_0 = 15
        link_6_sphere_1 = 16
        link_6_sphere_2 = 17
    CollisionVol = {
        Frames.R_ee: Geometry(type='sphere', radius=0.05),
        Frames.base_link_sphere_0: Geometry(type='sphere', radius=0.162246),
        Frames.link_1_sphere_0: Geometry(type='sphere', radius=0.155945),
        Frames.link_2_sphere_0: Geometry(type='sphere', radius=0.111092),
        Frames.link_2_sphere_1: Geometry(type='sphere', radius=0.114988),
        Frames.link_2_sphere_2: Geometry(type='sphere', radius=0.125693),
        Frames.link_2_sphere_3: Geometry(type='sphere', radius=0.131615),
        Frames.link_2_sphere_4: Geometry(type='sphere', radius=0.129192),
        Frames.link_3_sphere_0: Geometry(type='sphere', radius=0.115381),
        Frames.link_4_sphere_0: Geometry(type='sphere', radius=0.082276),
        Frames.link_4_sphere_1: Geometry(type='sphere', radius=0.076014),
        Frames.link_4_sphere_2: Geometry(type='sphere', radius=0.084356),
        Frames.link_4_sphere_3: Geometry(type='sphere', radius=0.063496),
        Frames.link_4_sphere_4: Geometry(type='sphere', radius=0.082623),
        Frames.link_5_sphere_0: Geometry(type='sphere', radius=0.070897),
        Frames.link_6_sphere_0: Geometry(type='sphere', radius=0.017394),
        Frames.link_6_sphere_1: Geometry(type='sphere', radius=0.017591),
        Frames.link_6_sphere_2: Geometry(type='sphere', radius=0.017218),
    }

    # Pairs of adjacent joints to be ignored in collision checking
    AdjacentCollisionVolPairs = [
        [Frames.base_link_sphere_0, Frames.link_1_sphere_0],
        [Frames.link_1_sphere_0, Frames.link_2_sphere_3],
        [Frames.link_1_sphere_0, Frames.link_2_sphere_4],
        [Frames.link_2_sphere_0, Frames.link_2_sphere_1],
        [Frames.link_2_sphere_0, Frames.link_2_sphere_2],
        [Frames.link_2_sphere_0, Frames.link_2_sphere_3],
        [Frames.link_2_sphere_0, Frames.link_2_sphere_4],
        [Frames.link_2_sphere_1, Frames.link_2_sphere_2],
        [Frames.link_2_sphere_1, Frames.link_2_sphere_3],
        [Frames.link_2_sphere_1, Frames.link_2_sphere_4],
        [Frames.link_2_sphere_1, Frames.link_3_sphere_0],
        [Frames.link_2_sphere_2, Frames.link_2_sphere_3],
        [Frames.link_2_sphere_2, Frames.link_2_sphere_4],
        [Frames.link_2_sphere_2, Frames.link_3_sphere_0],
        [Frames.link_2_sphere_3, Frames.link_2_sphere_4],
        [Frames.link_3_sphere_0, Frames.link_4_sphere_1],
        [Frames.link_4_sphere_0, Frames.link_4_sphere_1],
        [Frames.link_4_sphere_0, Frames.link_4_sphere_2],
        [Frames.link_4_sphere_0, Frames.link_4_sphere_3],
        [Frames.link_4_sphere_0, Frames.link_4_sphere_4],
        [Frames.link_4_sphere_1, Frames.link_4_sphere_2],
        [Frames.link_4_sphere_1, Frames.link_4_sphere_3],
        [Frames.link_4_sphere_1, Frames.link_4_sphere_4],
        [Frames.link_4_sphere_2, Frames.link_4_sphere_3],
        [Frames.link_4_sphere_2, Frames.link_4_sphere_4],
        [Frames.link_4_sphere_3, Frames.link_4_sphere_4],
        [Frames.link_4_sphere_4, Frames.link_5_sphere_0],
        [Frames.link_4_sphere_4, Frames.link_6_sphere_0],
        [Frames.link_4_sphere_4, Frames.link_6_sphere_1],
        [Frames.link_4_sphere_4, Frames.link_6_sphere_2],
        [Frames.link_5_sphere_0, Frames.link_6_sphere_0],
        [Frames.link_5_sphere_0, Frames.link_6_sphere_1],
        [Frames.link_5_sphere_0, Frames.link_6_sphere_2],
        [Frames.link_6_sphere_0, Frames.link_6_sphere_1],
        [Frames.link_6_sphere_0, Frames.link_6_sphere_2],
        [Frames.link_6_sphere_1, Frames.link_6_sphere_2],
    ]

    SelfCollisionVolIgnored = []
    
    EnvCollisionVolIgnored = []
    
    VisualizeSafeZone = [
        Frames.R_ee,
    ]

    VisualizePhiTraj = [
        Frames.R_ee,
    ]