from enum import IntEnum
from spark_robot.base.base_robot_config import RobotConfig
from spark_utils import Geometry, VizColor
import numpy as np

class R1LiteUpperDynamic1CollisionConfig(RobotConfig):
    
    # ---------------------------------------------------------------------------- #
    #                                      Kinematics                              #
    # ---------------------------------------------------------------------------- #
    
    kinematics_class_name = "R1LiteFlatKinematics"
    mujoco_model_path = "r1lite/r1lite_upper_body_scene.xml"
    collision_spheres_json_path = "r1lite/config/r1lite_collision_spheres.json"
    joint_to_lock = [
        "left_gripper_finger_joint1",
        "left_gripper_finger_joint2",
        "right_gripper_finger_joint1",
        "right_gripper_finger_joint2"
    ]
    
    # ---------------------------------------------------------------------------- #
    #                                   hardware                                   #
    # ---------------------------------------------------------------------------- #
    
    NumTotalMotors = 25
    
    class RealMotors(IntEnum):

        TorsoJoint1 = 0
        TorsoJoint2 = 1
        TorsoJoint3 = 2

        RightJoint1 = 3
        RightJoint2 = 4
        RightJoint3 = 5
        RightJoint4 = 6
        RightJoint5 = 7
        RightJoint6 = 8

        LeftJoint1  = 9
        LeftJoint2  = 10
        LeftJoint3  = 11
        LeftJoint4  = 12
        LeftJoint5  = 13
        LeftJoint6  = 14

    # Based on https://support.unitree.com/home/en/G1_developer/about_G1
    RealMotorPosLimit = {
        }
    
    NormalMotor = [
        RealMotors.TorsoJoint1,
        RealMotors.TorsoJoint2,
        RealMotors.TorsoJoint3,

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

        TorsoJoint1 = 0
        TorsoJoint2 = 1
        TorsoJoint3 = 2

        RightJoint1 = 3
        RightJoint2 = 4
        RightJoint3 = 5
        RightJoint4 = 6
        RightJoint5 = 7
        RightJoint6 = 8

        LeftJoint1  = 9
        LeftJoint2  = 10
        LeftJoint3  = 11
        LeftJoint4  = 12
        LeftJoint5  = 13
        LeftJoint6  = 14


    DefaultDoFVal = {
        DoFs.TorsoJoint1 : 0.0,
        DoFs.TorsoJoint2 : 0.0,
        DoFs.TorsoJoint3 : 0.0,

        DoFs.RightJoint1 : 0.0,
        DoFs.RightJoint2 : 0.0,
        DoFs.RightJoint3 : 0.0,
        DoFs.RightJoint4 : 0.0,
        DoFs.RightJoint5 : 0.0,
        DoFs.RightJoint6 : 0.0,

        DoFs.LeftJoint1  : 0.0,
        DoFs.LeftJoint2  : 0.0,
        DoFs.LeftJoint3  : 0.0,
        DoFs.LeftJoint4  : 0.0,
        DoFs.LeftJoint5  : 0.0,
        DoFs.LeftJoint6  : 0.0,

    }

    # ---------------------------------------------------------------------------- #
    #                                   Dynamics                                   #
    # ---------------------------------------------------------------------------- #

    class Control (IntEnum):
        # Velocity control

        vTorsoJoint1  = 0
        vTorsoJoint2  = 1
        vTorsoJoint3  = 2

        vRightJoint1  = 3
        vRightJoint2  = 4
        vRightJoint3  = 5
        vRightJoint4  = 6
        vRightJoint5  = 7
        vRightJoint6  = 8

        vLeftJoint1   = 9
        vLeftJoint2   = 10
        vLeftJoint3   = 11
        vLeftJoint4   = 12
        vLeftJoint5   = 13
        vLeftJoint6   = 14



    ControlLimit = {
        Control.vTorsoJoint1 : 1,
        Control.vTorsoJoint2 : 1,
        Control.vTorsoJoint3 : 1,

        Control.vRightJoint1 : 1,
        Control.vRightJoint2 : 1,
        Control.vRightJoint3 : 1,
        Control.vRightJoint4 : 1,
        Control.vRightJoint5 : 1,
        Control.vRightJoint6 : 1,

        Control.vLeftJoint1  : 1,
        Control.vLeftJoint2  : 1,
        Control.vLeftJoint3  : 1,
        Control.vLeftJoint4  : 1,
        Control.vLeftJoint5  : 1,
        Control.vLeftJoint6  : 1,

    }

    NormalControl = [
        Control.vTorsoJoint1,
        Control.vTorsoJoint2,
        Control.vTorsoJoint3,

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

        TorsoJoint1 = 0
        TorsoJoint2 = 1
        TorsoJoint3 = 2

        RightJoint1 = 3
        RightJoint2 = 4
        RightJoint3 = 5
        RightJoint4 = 6
        RightJoint5 = 7
        RightJoint6 = 8

        LeftJoint1  = 11
        LeftJoint2  = 12
        LeftJoint3  = 13
        LeftJoint4  = 14
        LeftJoint5  = 15
        LeftJoint6  = 16


    class MujocoMotors(IntEnum):

        TorsoJoint1 = 0
        TorsoJoint2 = 1
        TorsoJoint3 = 2

        RightJoint1 = 3
        RightJoint2 = 4
        RightJoint3 = 5
        RightJoint4 = 6
        RightJoint5 = 7
        RightJoint6 = 8

        LeftJoint1  = 11
        LeftJoint2  = 12
        LeftJoint3  = 13
        LeftJoint4  = 14
        LeftJoint5  = 15
        LeftJoint6  = 16


    # ---------------------------------------------------------------------------- #
    #                                   Mujoco PID Parameters                      #
    # ---------------------------------------------------------------------------- #
    
    # Kp parameters for Mujoco Motors
    MujocoMotorKps = {
        MujocoMotors.TorsoJoint1 : 3000.0,
        MujocoMotors.TorsoJoint2 : 3000.0,
        MujocoMotors.TorsoJoint3 : 3000.0,

        MujocoMotors.RightJoint1 : 1000.0,
        MujocoMotors.RightJoint2 : 1000.0,
        MujocoMotors.RightJoint3 : 1000.0,
        MujocoMotors.RightJoint4 : 1000.0,
        MujocoMotors.RightJoint5 : 1000.0,
        MujocoMotors.RightJoint6 : 1000.0,

        MujocoMotors.LeftJoint1  : 1000.0,
        MujocoMotors.LeftJoint2  : 1000.0,
        MujocoMotors.LeftJoint3  : 1000.0,
        MujocoMotors.LeftJoint4  : 1000.0,
        MujocoMotors.LeftJoint5  : 1000.0,
        MujocoMotors.LeftJoint6  : 1000.0,

    }
    
    # Kd parameters for Mujoco Motors
    MujocoMotorKds = {
        MujocoMotors.TorsoJoint1 : 0.1,
        MujocoMotors.TorsoJoint2 : 0.1,
        MujocoMotors.TorsoJoint3 : 0.1,

        MujocoMotors.RightJoint1 : 0.1,
        MujocoMotors.RightJoint2 : 0.1,
        MujocoMotors.RightJoint3 : 0.1,
        MujocoMotors.RightJoint4 : 0.1,
        MujocoMotors.RightJoint5 : 0.1,
        MujocoMotors.RightJoint6 : 0.1,

        MujocoMotors.LeftJoint1  : 0.1,
        MujocoMotors.LeftJoint2  : 0.1,
        MujocoMotors.LeftJoint3  : 0.1,
        MujocoMotors.LeftJoint4  : 0.1,
        MujocoMotors.LeftJoint5  : 0.1,
        MujocoMotors.LeftJoint6  : 0.1,

    }

    # ---------------------------------------------------------------------------- #
    #                                   Mujoco Mappings                            #
    # ---------------------------------------------------------------------------- #

    # Mapping from Mujoco DoFs to DoFs
    MujocoDoF_to_DoF = {
        MujocoDoFs.TorsoJoint1 : DoFs.TorsoJoint1,
        MujocoDoFs.TorsoJoint2 : DoFs.TorsoJoint2,
        MujocoDoFs.TorsoJoint3 : DoFs.TorsoJoint3,

        MujocoDoFs.RightJoint1 : DoFs.RightJoint1,
        MujocoDoFs.RightJoint2 : DoFs.RightJoint2,
        MujocoDoFs.RightJoint3 : DoFs.RightJoint3,
        MujocoDoFs.RightJoint4 : DoFs.RightJoint4,
        MujocoDoFs.RightJoint5 : DoFs.RightJoint5,
        MujocoDoFs.RightJoint6 : DoFs.RightJoint6,

        MujocoDoFs.LeftJoint1  : DoFs.LeftJoint1,
        MujocoDoFs.LeftJoint2  : DoFs.LeftJoint2,
        MujocoDoFs.LeftJoint3  : DoFs.LeftJoint3,
        MujocoDoFs.LeftJoint4  : DoFs.LeftJoint4,
        MujocoDoFs.LeftJoint5  : DoFs.LeftJoint5,
        MujocoDoFs.LeftJoint6  : DoFs.LeftJoint6,

    }

    # Mapping from DoFs to Mujoco DoFs
    DoF_to_MujocoDoF = {
        DoFs.TorsoJoint1 : MujocoDoFs.TorsoJoint1,
        DoFs.TorsoJoint2 : MujocoDoFs.TorsoJoint2,
        DoFs.TorsoJoint3 : MujocoDoFs.TorsoJoint3,

        DoFs.RightJoint1 : MujocoDoFs.RightJoint1,
        DoFs.RightJoint2 : MujocoDoFs.RightJoint2,
        DoFs.RightJoint3 : MujocoDoFs.RightJoint3,
        DoFs.RightJoint4 : MujocoDoFs.RightJoint4,
        DoFs.RightJoint5 : MujocoDoFs.RightJoint5,
        DoFs.RightJoint6 : MujocoDoFs.RightJoint6,

        DoFs.LeftJoint1  : MujocoDoFs.LeftJoint1,
        DoFs.LeftJoint2  : MujocoDoFs.LeftJoint2,
        DoFs.LeftJoint3  : MujocoDoFs.LeftJoint3,
        DoFs.LeftJoint4  : MujocoDoFs.LeftJoint4,
        DoFs.LeftJoint5  : MujocoDoFs.LeftJoint5,
        DoFs.LeftJoint6  : MujocoDoFs.LeftJoint6,

    }

    # Mapping from Mujoco Motors to Control
    MujocoMotor_to_Control = {
        MujocoMotors.TorsoJoint1 : Control.vTorsoJoint1,
        MujocoMotors.TorsoJoint2 : Control.vTorsoJoint2,
        MujocoMotors.TorsoJoint3 : Control.vTorsoJoint3,

        MujocoMotors.RightJoint1 : Control.vRightJoint1,
        MujocoMotors.RightJoint2 : Control.vRightJoint2,
        MujocoMotors.RightJoint3 : Control.vRightJoint3,
        MujocoMotors.RightJoint4 : Control.vRightJoint4,
        MujocoMotors.RightJoint5 : Control.vRightJoint5,
        MujocoMotors.RightJoint6 : Control.vRightJoint6,

        MujocoMotors.LeftJoint1  : Control.vLeftJoint1,
        MujocoMotors.LeftJoint2  : Control.vLeftJoint2,
        MujocoMotors.LeftJoint3  : Control.vLeftJoint3,
        MujocoMotors.LeftJoint4  : Control.vLeftJoint4,
        MujocoMotors.LeftJoint5  : Control.vLeftJoint5,
        MujocoMotors.LeftJoint6  : Control.vLeftJoint6,

    }
    
    # ---------------------------------------------------------------------------- #
    #                                    Real                                    #
    # ---------------------------------------------------------------------------- #
    class RealDoFs(IntEnum):
        

        TorsoJoint1 = 0
        TorsoJoint2 = 1
        TorsoJoint3 = 2

        RightJoint1 = 3
        RightJoint2 = 4
        RightJoint3 = 5
        RightJoint4 = 6
        RightJoint5 = 7
        RightJoint6 = 8

        LeftJoint1  = 9
        LeftJoint2  = 10
        LeftJoint3  = 11
        LeftJoint4  = 12
        LeftJoint5  = 13
        LeftJoint6  = 14
    
    # ---------------------------------------------------------------------------- #
    #                                   Real Mappings                                   #
    # ---------------------------------------------------------------------------- #
    
    
    # Mapping from Real DoFs to DoFs
    RealDoF_to_DoF = {
        RealDoFs.TorsoJoint1 : DoFs.TorsoJoint1,
        RealDoFs.TorsoJoint2 : DoFs.TorsoJoint2,
        RealDoFs.TorsoJoint3 : DoFs.TorsoJoint3,

        RealDoFs.RightJoint1 : DoFs.RightJoint1,
        RealDoFs.RightJoint2 : DoFs.RightJoint2,
        RealDoFs.RightJoint3 : DoFs.RightJoint3,
        RealDoFs.RightJoint4 : DoFs.RightJoint4,
        RealDoFs.RightJoint5 : DoFs.RightJoint5,
        RealDoFs.RightJoint6 : DoFs.RightJoint6,

        RealDoFs.LeftJoint1  : DoFs.LeftJoint1,
        RealDoFs.LeftJoint2  : DoFs.LeftJoint2,
        RealDoFs.LeftJoint3  : DoFs.LeftJoint3,
        RealDoFs.LeftJoint4  : DoFs.LeftJoint4,
        RealDoFs.LeftJoint5  : DoFs.LeftJoint5,
        RealDoFs.LeftJoint6  : DoFs.LeftJoint6,

    }

    # Mapping from DoFs to Real DoFs
    DoF_to_RealDoF = {
        DoFs.TorsoJoint1 : RealDoFs.TorsoJoint1,
        DoFs.TorsoJoint2 : RealDoFs.TorsoJoint2,
        DoFs.TorsoJoint3 : RealDoFs.TorsoJoint3,

        DoFs.RightJoint1 : RealDoFs.RightJoint1,
        DoFs.RightJoint2 : RealDoFs.RightJoint2,
        DoFs.RightJoint3 : RealDoFs.RightJoint3,
        DoFs.RightJoint4 : RealDoFs.RightJoint4,
        DoFs.RightJoint5 : RealDoFs.RightJoint5,
        DoFs.RightJoint6 : RealDoFs.RightJoint6,

        DoFs.LeftJoint1  : RealDoFs.LeftJoint1,
        DoFs.LeftJoint2  : RealDoFs.LeftJoint2,
        DoFs.LeftJoint3  : RealDoFs.LeftJoint3,
        DoFs.LeftJoint4  : RealDoFs.LeftJoint4,
        DoFs.LeftJoint5  : RealDoFs.LeftJoint5,
        DoFs.LeftJoint6  : RealDoFs.LeftJoint6,

    }

    # Mapping from real motors to Control
    RealMotor_to_Control = {
        RealMotors.TorsoJoint1 : Control.vTorsoJoint1,
        RealMotors.TorsoJoint2 : Control.vTorsoJoint2,
        RealMotors.TorsoJoint3 : Control.vTorsoJoint3,

        RealMotors.RightJoint1 : Control.vRightJoint1,
        RealMotors.RightJoint2 : Control.vRightJoint2,
        RealMotors.RightJoint3 : Control.vRightJoint3,
        RealMotors.RightJoint4 : Control.vRightJoint4,
        RealMotors.RightJoint5 : Control.vRightJoint5,
        RealMotors.RightJoint6 : Control.vRightJoint6,

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

        base_link_sphere_0 = 2
        base_link_sphere_1 = 3
        base_link_sphere_2 = 4
        steer_motor_link1_sphere_0 = 5
        steer_motor_link1_sphere_1 = 6
        wheel_motor_link1_sphere_0 = 7
        wheel_motor_link1_sphere_1 = 8
        wheel_motor_link1_sphere_2 = 9
        steer_motor_link2_sphere_0 = 10
        steer_motor_link2_sphere_1 = 11
        wheel_motor_link2_sphere_0 = 12
        wheel_motor_link2_sphere_1 = 13
        wheel_motor_link2_sphere_2 = 14
        steer_motor_link3_sphere_0 = 15
        steer_motor_link3_sphere_1 = 16
        wheel_motor_link3_sphere_0 = 17
        wheel_motor_link3_sphere_1 = 18
        wheel_motor_link3_sphere_2 = 19
        torso_link1_sphere_0 = 20
        torso_link1_sphere_1 = 21
        torso_link1_sphere_2 = 22
        torso_link1_sphere_3 = 23
        torso_link2_sphere_0 = 24
        torso_link2_sphere_1 = 25
        torso_link2_sphere_2 = 26
        torso_link2_sphere_3 = 27
        torso_link3_sphere_0 = 28
        torso_link3_sphere_1 = 29
        torso_link3_sphere_2 = 30
        torso_link3_sphere_3 = 31
        torso_link3_sphere_4 = 32
        camera_head_left_link_sphere_0 = 33
        camera_head_left_link_sphere_1 = 34
        camera_head_left_link_sphere_2 = 35
        left_arm_base_link_sphere_0 = 36
        left_arm_link1_sphere_0 = 37
        left_arm_link2_sphere_0 = 38
        left_arm_link2_sphere_1 = 39
        left_arm_link2_sphere_2 = 40
        left_arm_link2_sphere_3 = 41
        left_arm_link2_sphere_4 = 42
        left_arm_link2_sphere_5 = 43
        left_arm_link3_sphere_0 = 44
        left_arm_link3_sphere_1 = 45
        left_arm_link3_sphere_2 = 46
        left_arm_link4_sphere_0 = 47
        left_arm_link4_sphere_1 = 48
        left_arm_link5_sphere_0 = 49
        left_arm_link6_sphere_0 = 50
        left_arm_link6_sphere_1 = 51
        left_arm_link6_sphere_2 = 52
        left_arm_link6_sphere_3 = 53
        left_gripper_link_sphere_0 = 54
        left_gripper_link_sphere_1 = 55
        left_D405_link_sphere_0 = 56
        left_D405_link_sphere_1 = 57
        right_arm_base_link_sphere_0 = 58
        right_arm_link1_sphere_0 = 59
        right_arm_link2_sphere_0 = 60
        right_arm_link2_sphere_1 = 61
        right_arm_link2_sphere_2 = 62
        right_arm_link2_sphere_3 = 63
        right_arm_link2_sphere_4 = 64
        right_arm_link2_sphere_5 = 65
        right_arm_link3_sphere_0 = 66
        right_arm_link3_sphere_1 = 67
        right_arm_link3_sphere_2 = 68
        right_arm_link4_sphere_0 = 69
        right_arm_link4_sphere_1 = 70
        right_arm_link5_sphere_0 = 71
        right_arm_link6_sphere_0 = 72
        right_arm_link6_sphere_1 = 73
        right_arm_link6_sphere_2 = 74
        right_arm_link6_sphere_3 = 75
        right_gripper_link_sphere_0 = 76
        right_gripper_link_sphere_1 = 77
        right_D405_link_sphere_0 = 78
        right_D405_link_sphere_1 = 79
    CollisionVol = {
        Frames.R_ee: Geometry(type='sphere', radius=0.05),
        Frames.L_ee: Geometry(type='sphere', radius=0.05),
        Frames.base_link_sphere_0: Geometry(type='sphere', radius=0.165927),
        Frames.base_link_sphere_1: Geometry(type='sphere', radius=0.140596),
        Frames.base_link_sphere_2: Geometry(type='sphere', radius=0.143662),
        Frames.steer_motor_link1_sphere_0: Geometry(type='sphere', radius=0.036919),
        Frames.steer_motor_link1_sphere_1: Geometry(type='sphere', radius=0.036918),
        Frames.wheel_motor_link1_sphere_0: Geometry(type='sphere', radius=0.029803),
        Frames.wheel_motor_link1_sphere_1: Geometry(type='sphere', radius=0.038577),
        Frames.wheel_motor_link1_sphere_2: Geometry(type='sphere', radius=0.033424),
        Frames.steer_motor_link2_sphere_0: Geometry(type='sphere', radius=0.036917),
        Frames.steer_motor_link2_sphere_1: Geometry(type='sphere', radius=0.036918),
        Frames.wheel_motor_link2_sphere_0: Geometry(type='sphere', radius=0.029803),
        Frames.wheel_motor_link2_sphere_1: Geometry(type='sphere', radius=0.038577),
        Frames.wheel_motor_link2_sphere_2: Geometry(type='sphere', radius=0.033424),
        Frames.steer_motor_link3_sphere_0: Geometry(type='sphere', radius=0.036917),
        Frames.steer_motor_link3_sphere_1: Geometry(type='sphere', radius=0.036918),
        Frames.wheel_motor_link3_sphere_0: Geometry(type='sphere', radius=0.029803),
        Frames.wheel_motor_link3_sphere_1: Geometry(type='sphere', radius=0.038577),
        Frames.wheel_motor_link3_sphere_2: Geometry(type='sphere', radius=0.033424),
        Frames.torso_link1_sphere_0: Geometry(type='sphere', radius=0.062539),
        Frames.torso_link1_sphere_1: Geometry(type='sphere', radius=0.06254),
        Frames.torso_link1_sphere_2: Geometry(type='sphere', radius=0.061148),
        Frames.torso_link1_sphere_3: Geometry(type='sphere', radius=0.06099),
        Frames.torso_link2_sphere_0: Geometry(type='sphere', radius=0.063622),
        Frames.torso_link2_sphere_1: Geometry(type='sphere', radius=0.063623),
        Frames.torso_link2_sphere_2: Geometry(type='sphere', radius=0.063284),
        Frames.torso_link2_sphere_3: Geometry(type='sphere', radius=0.061918),
        Frames.torso_link3_sphere_0: Geometry(type='sphere', radius=0.108322),
        Frames.torso_link3_sphere_1: Geometry(type='sphere', radius=0.108121),
        Frames.torso_link3_sphere_2: Geometry(type='sphere', radius=0.104974),
        Frames.torso_link3_sphere_3: Geometry(type='sphere', radius=0.093936),
        Frames.torso_link3_sphere_4: Geometry(type='sphere', radius=0.093232),
        Frames.camera_head_left_link_sphere_0: Geometry(type='sphere', radius=0.020184),
        Frames.camera_head_left_link_sphere_1: Geometry(type='sphere', radius=0.019048),
        Frames.camera_head_left_link_sphere_2: Geometry(type='sphere', radius=0.019083),
        Frames.left_arm_base_link_sphere_0: Geometry(type='sphere', radius=0.041182),
        Frames.left_arm_link1_sphere_0: Geometry(type='sphere', radius=0.033595),
        Frames.left_arm_link2_sphere_0: Geometry(type='sphere', radius=0.035927),
        Frames.left_arm_link2_sphere_1: Geometry(type='sphere', radius=0.035672),
        Frames.left_arm_link2_sphere_2: Geometry(type='sphere', radius=0.033873),
        Frames.left_arm_link2_sphere_3: Geometry(type='sphere', radius=0.033052),
        Frames.left_arm_link2_sphere_4: Geometry(type='sphere', radius=0.031251),
        Frames.left_arm_link2_sphere_5: Geometry(type='sphere', radius=0.028499),
        Frames.left_arm_link3_sphere_0: Geometry(type='sphere', radius=0.040502),
        Frames.left_arm_link3_sphere_1: Geometry(type='sphere', radius=0.040189),
        Frames.left_arm_link3_sphere_2: Geometry(type='sphere', radius=0.041513),
        Frames.left_arm_link4_sphere_0: Geometry(type='sphere', radius=0.032286),
        Frames.left_arm_link4_sphere_1: Geometry(type='sphere', radius=0.032286),
        Frames.left_arm_link5_sphere_0: Geometry(type='sphere', radius=0.025302),
        Frames.left_arm_link6_sphere_0: Geometry(type='sphere', radius=0.008862),
        Frames.left_arm_link6_sphere_1: Geometry(type='sphere', radius=0.00889),
        Frames.left_arm_link6_sphere_2: Geometry(type='sphere', radius=0.008484),
        Frames.left_arm_link6_sphere_3: Geometry(type='sphere', radius=0.008861),
        Frames.left_gripper_link_sphere_0: Geometry(type='sphere', radius=0.028587),
        Frames.left_gripper_link_sphere_1: Geometry(type='sphere', radius=0.028587),
        Frames.left_D405_link_sphere_0: Geometry(type='sphere', radius=0.020762),
        Frames.left_D405_link_sphere_1: Geometry(type='sphere', radius=0.020761),
        Frames.right_arm_base_link_sphere_0: Geometry(type='sphere', radius=0.041182),
        Frames.right_arm_link1_sphere_0: Geometry(type='sphere', radius=0.033595),
        Frames.right_arm_link2_sphere_0: Geometry(type='sphere', radius=0.035927),
        Frames.right_arm_link2_sphere_1: Geometry(type='sphere', radius=0.035672),
        Frames.right_arm_link2_sphere_2: Geometry(type='sphere', radius=0.033873),
        Frames.right_arm_link2_sphere_3: Geometry(type='sphere', radius=0.033052),
        Frames.right_arm_link2_sphere_4: Geometry(type='sphere', radius=0.031251),
        Frames.right_arm_link2_sphere_5: Geometry(type='sphere', radius=0.028499),
        Frames.right_arm_link3_sphere_0: Geometry(type='sphere', radius=0.040502),
        Frames.right_arm_link3_sphere_1: Geometry(type='sphere', radius=0.040189),
        Frames.right_arm_link3_sphere_2: Geometry(type='sphere', radius=0.041513),
        Frames.right_arm_link4_sphere_0: Geometry(type='sphere', radius=0.032286),
        Frames.right_arm_link4_sphere_1: Geometry(type='sphere', radius=0.032286),
        Frames.right_arm_link5_sphere_0: Geometry(type='sphere', radius=0.025302),
        Frames.right_arm_link6_sphere_0: Geometry(type='sphere', radius=0.008862),
        Frames.right_arm_link6_sphere_1: Geometry(type='sphere', radius=0.00889),
        Frames.right_arm_link6_sphere_2: Geometry(type='sphere', radius=0.008484),
        Frames.right_arm_link6_sphere_3: Geometry(type='sphere', radius=0.008861),
        Frames.right_gripper_link_sphere_0: Geometry(type='sphere', radius=0.028587),
        Frames.right_gripper_link_sphere_1: Geometry(type='sphere', radius=0.028587),
        Frames.right_D405_link_sphere_0: Geometry(type='sphere', radius=0.020762),
        Frames.right_D405_link_sphere_1: Geometry(type='sphere', radius=0.020761),
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