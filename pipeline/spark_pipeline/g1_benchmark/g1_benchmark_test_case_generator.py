from spark_pipeline import G1BenchmarkPipeline, G1BenchmarkPipelineConfig
from spark_utils import update_class_attributes
import numpy as np

def generate_g1_benchmark_test_case(
    cfg,
    test_case = "G1MobileBase_D1_WG_SO_v0"
):
    '''
    Generate G1 benchmark test case
    
    Robot Types:
        G1Fixedbody: Using G1 robot with 17 upperbody Dofs only.
        G1MobileBase: Using G1 robot with 17 upperbody Dofs plus 3 base Dofs (floating base).
        G1SportMode: Using G1 robot with 17 upperbody Dofs plus 3 base Dofs (locomotion).
        
    Task Types:
        SG: Static Goal
        DG: Dynamic Goal
        SO: Static Obstacle
        DO: Dynamic Obstacle    
    '''


    # --------------------------------- test case -------------------------------- #


    match test_case:

        # ---------------------------------------------------------------------------- #
        #                                   G1MobileBase                                #
        # ---------------------------------------------------------------------------- #
        
        case "G1MobileBase_D1_WG_SO_v0":
            
            # robot config
            cfg.robot.cfg.class_name = "G1MobileBaseDynamic1Config"
            cfg.env.agent.mujoco_model = "g1/g1_29dof_mobile_base.xml"
            
            # task config
            cfg.env.task.class_name = "G1BenchmarkTask"
            cfg.env.task.task_name = test_case
            # Set the number of obstacles for the task
            cfg.env.task.num_obstacle_task = 10

            # Obstacle parameters
            cfg.env.task.obstacle_range = [(-1, 1), (-1, 1), (0.5, 1.3)]  # Range for obstacles in x, y, and z axes
            cfg.env.task.obstacle_size = 0.05        # Size of obstacles
            cfg.env.task.obstacle_keepout = 0.05      # Keepout distance for obstacles
            cfg.env.task.obstacle_velocity = 0.0     # Velocity of obstacles
            cfg.env.task.mode = "Brownian"  # Obstacle movement mode

            # Arm goal parameters
            cfg.env.task.arm_goal_velocity = 0.0     # Velocity for arm goal movement
            cfg.env.task.arm_goal_reach_done = True  # End episode when the arm goal is reached

            # Base goal parameters
            cfg.env.task.base_goal_range = [(-0.8, 0.8), (-0.8, 0.8), (0.793, 0.793)]  # Range for base goal in x, y, and z axes
            cfg.env.task.base_goal_rot_range = (-np.pi, np.pi)  # Rotation range for the base goal
            cfg.env.task.base_goal_velocity = 0.0    # Velocity for base goal movement
            cfg.env.task.base_goal_reach_done = True  # End episode when the base goal is reached

            # Seed configuration
            cfg.env.task.seed = 0  # Random seed for reproducibility

        
        case "G1MobileBase_D1_WG_DO_v0":
            
            # robot config
            cfg.robot.cfg.class_name = "G1MobileBaseDynamic1Config"
            cfg.env.agent.mujoco_model = "g1/g1_29dof_mobile_base.xml"
            
            # task config
            cfg.env.task.class_name = "G1BenchmarkTask"
            cfg.env.task.task_name = test_case
            # Set the number of obstacles for the task
            cfg.env.task.num_obstacle_task = 10

            # Obstacle parameters
            cfg.env.task.obstacle_range = [(-1, 1), (-1, 1), (0.5, 1.3)]  # Range for obstacles in x, y, and z axes
            cfg.env.task.obstacle_size = 0.05        # Size of obstacles
            cfg.env.task.obstacle_keepout = 0.05      # Minimum keepout distance for obstacles
            cfg.env.task.obstacle_velocity = 0.005   # Velocity of obstacles
            cfg.env.task.mode = "Brownian"  # Obstacle movement mode

            # Arm goal parameters
            cfg.env.task.arm_goal_velocity = 0.0     # Velocity for arm goal movement
            cfg.env.task.arm_goal_reach_done = True  # End episode when the arm goal is reached

            # Base goal parameters
            cfg.env.task.base_goal_range = [(-0.8, 0.8), (-0.8, 0.8), (0.793, 0.793)]  # Range for base goal in x, y, and z axes
            cfg.env.task.base_goal_rot_range = (-np.pi, np.pi)  # Rotation range for the base goal
            cfg.env.task.base_goal_velocity = 0.0    # Velocity for base goal movement
            cfg.env.task.base_goal_reach_done = True  # End episode when the base goal is reached

            # Seed configuration
            cfg.env.task.seed = 0  # Random seed for reproducibility


        case "G1MobileBase_D1_WG_SO_v1":
            
            # robot config
            cfg.robot.cfg.class_name = "G1MobileBaseDynamic1Config"
            cfg.env.agent.mujoco_model = "g1/g1_29dof_mobile_base.xml"
            
            # task config
            cfg.env.task.class_name = "G1BenchmarkTask"
            cfg.env.task.task_name = test_case
            # Set the number of obstacles for the task
            cfg.env.task.num_obstacle_task = 50

            # Obstacle parameters
            cfg.env.task.obstacle_range = [(-1, 1), (-1, 1), (0.5, 1.3)]  # Range for obstacles in x, y, and z axes
            cfg.env.task.obstacle_size = 0.05        # Size of obstacles
            cfg.env.task.obstacle_keepout = 0.1      # Keepout distance for obstacles
            cfg.env.task.obstacle_velocity = 0.0     # Velocity of obstacles
            cfg.env.task.mode = "Brownian"  # Obstacle movement mode

            # Arm goal parameters
            cfg.env.task.arm_goal_velocity = 0.0     # Velocity for arm goal movement
            cfg.env.task.arm_goal_reach_done = True  # End episode when arm goal is reached

            # Base goal parameters
            cfg.env.task.base_goal_range = [(-0.8, 0.8), (-0.8, 0.8), (0.793, 0.793)]  # Range for base goal in x, y, and z axes
            cfg.env.task.base_goal_rot_range = (-np.pi, np.pi)  # Rotation range for the base goal
            cfg.env.task.base_goal_velocity = 0.0    # Velocity for base goal movement
            cfg.env.task.base_goal_reach_done = True  # End episode when base goal is reached

            # Seed configuration
            cfg.env.task.seed = 0  # Random seed for reproducibility

        
        case "G1MobileBase_D1_WG_DO_v1":
            
            # robot config
            cfg.robot.cfg.class_name = "G1MobileBaseDynamic1Config"
            cfg.env.agent.mujoco_model = "g1/g1_29dof_mobile_base.xml"

            # task config
            cfg.env.task.class_name = "G1BenchmarkTask"
            cfg.env.task.task_name = test_case
            # Set the number of obstacles for the task
            cfg.env.task.num_obstacle_task = 50

            # Obstacle parameters
            cfg.env.task.obstacle_range = [(-1, 1), (-1, 1), (0.5, 1.3)]  # Range for obstacles as (min, max) tuples for x, y, and z axes
            cfg.env.task.obstacle_size = 0.05       # Size of each obstacle
            cfg.env.task.obstacle_keepout = 0.1     # Minimum keepout distance from obstacles
            cfg.env.task.obstacle_velocity = 0.005  # Velocity of obstacles
            cfg.env.task.mode = "Brownian" # Obstacle movement mode

            # Arm goal parameters
            cfg.env.task.arm_goal_velocity = 0.0    # Velocity for arm goal movement
            cfg.env.task.arm_goal_reach_done = True # Flag to finish the episode when the arm goal is reached

            # Base goal parameters
            cfg.env.task.base_goal_range = [(-0.8, 0.8), (-0.8, 0.8), (0.793, 0.793)]  # Range for base goal as (min, max) tuples for x, y, and z axes
            cfg.env.task.base_goal_rot_range = (-np.pi, np.pi)  # Rotation range for base goal in radians
            cfg.env.task.base_goal_velocity = 0.0               # Velocity of the base goal
            cfg.env.task.base_goal_reach_done = True            # Flag to finish the episode when the base goal is reached

            # Set the random seed for reproducibility
            cfg.env.task.seed = 0


        case "G1MobileBase_D2_WG_SO_v0":
            
            # robot config
            cfg.robot.cfg.class_name = "G1MobileBaseDynamic2Config"
            cfg.env.agent.mujoco_model = "g1/g1_29dof_mobile_base.xml"
            
            # task config
            cfg.env.task.class_name = "G1BenchmarkTask"
            cfg.env.task.task_name = test_case
            # Set the number of obstacles for the task
            cfg.env.task.num_obstacle_task = 10

            # Obstacle parameters
            cfg.env.task.obstacle_range = [(-1, 1), (-1, 1), (0.5, 1.3)]  # Range for obstacles in x, y, and z axes
            cfg.env.task.obstacle_size = 0.05        # Size of obstacles
            cfg.env.task.obstacle_keepout = 0.05      # Keepout distance for obstacles
            cfg.env.task.obstacle_velocity = 0.0     # Velocity of obstacles
            cfg.env.task.mode = "Brownian"  # Obstacle movement mode

            # Arm goal parameters
            cfg.env.task.arm_goal_velocity = 0.0     # Velocity for arm goal movement
            cfg.env.task.arm_goal_reach_done = True  # End episode when the arm goal is reached

            # Base goal parameters
            cfg.env.task.base_goal_range = [(-0.8, 0.8), (-0.8, 0.8), (0.793, 0.793)]  # Range for base goal in x, y, and z axes
            cfg.env.task.base_goal_rot_range = (-np.pi, np.pi)  # Rotation range for the base goal
            cfg.env.task.base_goal_velocity = 0.0    # Velocity for base goal movement
            cfg.env.task.base_goal_reach_done = True  # End episode when the base goal is reached

            # Seed configuration
            cfg.env.task.seed = 0  # Random seed for reproducibility

        
        case "G1MobileBase_D2_WG_DO_v0":
            
            # robot config
            cfg.robot.cfg.class_name = "G1MobileBaseDynamic2Config"
            cfg.env.agent.mujoco_model = "g1/g1_29dof_mobile_base.xml"
            
            # task config
            cfg.env.task.class_name = "G1BenchmarkTask"
            cfg.env.task.task_name = test_case
            # Set the number of obstacles for the task
            cfg.env.task.num_obstacle_task = 10

            # Obstacle parameters
            cfg.env.task.obstacle_range = [(-1, 1), (-1, 1), (0.5, 1.3)]  # Range for obstacles in x, y, and z axes
            cfg.env.task.obstacle_size = 0.05        # Size of obstacles
            cfg.env.task.obstacle_keepout = 0.05      # Minimum keepout distance for obstacles
            cfg.env.task.obstacle_velocity = 0.005   # Velocity of obstacles
            cfg.env.task.mode = "Brownian"  # Obstacle movement mode

            # Arm goal parameters
            cfg.env.task.arm_goal_velocity = 0.0     # Velocity for arm goal movement
            cfg.env.task.arm_goal_reach_done = True  # End episode when the arm goal is reached

            # Base goal parameters
            cfg.env.task.base_goal_range = [(-0.8, 0.8), (-0.8, 0.8), (0.793, 0.793)]  # Range for base goal in x, y, and z axes
            cfg.env.task.base_goal_rot_range = (-np.pi, np.pi)  # Rotation range for the base goal
            cfg.env.task.base_goal_velocity = 0.0    # Velocity for base goal movement
            cfg.env.task.base_goal_reach_done = True  # End episode when the base goal is reached

            # Seed configuration
            cfg.env.task.seed = 0  # Random seed for reproducibility


        case "G1MobileBase_D2_WG_SO_v1":
            
            # robot config
            cfg.robot.cfg.class_name = "G1MobileBaseDynamic2Config"
            cfg.env.agent.mujoco_model = "g1/g1_29dof_mobile_base.xml"
            
            # task config
            cfg.env.task.class_name = "G1BenchmarkTask"
            cfg.env.task.task_name = test_case
            # Set the number of obstacles for the task
            cfg.env.task.num_obstacle_task = 50

            # Obstacle parameters
            cfg.env.task.obstacle_range = [(-1, 1), (-1, 1), (0.5, 1.3)]  # Range for obstacles in x, y, and z axes
            cfg.env.task.obstacle_size = 0.05        # Size of obstacles
            cfg.env.task.obstacle_keepout = 0.1      # Keepout distance for obstacles
            cfg.env.task.obstacle_velocity = 0.0     # Velocity of obstacles
            cfg.env.task.mode = "Brownian"  # Obstacle movement mode

            # Arm goal parameters
            cfg.env.task.arm_goal_velocity = 0.0     # Velocity for arm goal movement
            cfg.env.task.arm_goal_reach_done = True  # End episode when arm goal is reached

            # Base goal parameters
            cfg.env.task.base_goal_range = [(-0.8, 0.8), (-0.8, 0.8), (0.793, 0.793)]  # Range for base goal in x, y, and z axes
            cfg.env.task.base_goal_rot_range = (-np.pi, np.pi)  # Rotation range for the base goal
            cfg.env.task.base_goal_velocity = 0.0    # Velocity for base goal movement
            cfg.env.task.base_goal_reach_done = True  # End episode when base goal is reached

            # Seed configuration
            cfg.env.task.seed = 0  # Random seed for reproducibility

        
        case "G1MobileBase_D2_WG_DO_v1":
            
            # robot config
            cfg.robot.cfg.class_name = "G1MobileBaseDynamic2Config"
            cfg.env.agent.mujoco_model = "g1/g1_29dof_mobile_base.xml"

            # task config
            cfg.env.task.class_name = "G1BenchmarkTask"
            cfg.env.task.task_name = test_case
            # Set the number of obstacles for the task
            cfg.env.task.num_obstacle_task = 50

            # Obstacle parameters
            cfg.env.task.obstacle_range = [(-1, 1), (-1, 1), (0.5, 1.3)]  # Range for obstacles as (min, max) tuples for x, y, and z axes
            cfg.env.task.obstacle_size = 0.05       # Size of each obstacle
            cfg.env.task.obstacle_keepout = 0.1     # Minimum keepout distance from obstacles
            cfg.env.task.obstacle_velocity = 0.005  # Velocity of obstacles
            cfg.env.task.mode = "Brownian" # Obstacle movement mode

            # Arm goal parameters
            cfg.env.task.arm_goal_velocity = 0.0    # Velocity for arm goal movement
            cfg.env.task.arm_goal_reach_done = True # Flag to finish the episode when the arm goal is reached

            # Base goal parameters
            cfg.env.task.base_goal_range = [(-0.8, 0.8), (-0.8, 0.8), (0.793, 0.793)]  # Range for base goal as (min, max) tuples for x, y, and z axes
            cfg.env.task.base_goal_rot_range = (-np.pi, np.pi)  # Rotation range for base goal in radians
            cfg.env.task.base_goal_velocity = 0.0               # Velocity of the base goal
            cfg.env.task.base_goal_reach_done = True            # Flag to finish the episode when the base goal is reached

            # Set the random seed for reproducibility
            cfg.env.task.seed = 0

        # ---------------------------------------------------------------------------- #
        #                                   G1FixedBase                                #
        # ---------------------------------------------------------------------------- #

        case "G1FixedBase_D1_AG_SO_v0":
            
            # robot config
            cfg.robot.cfg.class_name = "G1FixedBaseDynamic1Config"
            cfg.env.agent.mujoco_model = "g1/g1_29dof_fixed_base.xml"

            # task config
            cfg.env.task.class_name = "G1BenchmarkTask"
            cfg.env.task.task_name = test_case
            cfg.env.task.num_obstacle_task = 5  # Number of obstacles in the task

            cfg.env.task.obstacle_range = [(-0.3, 0.5), (-0.4, 0.4), (0.5, 1.3)]
            cfg.env.task.obstacle_size = 0.05       # Size of obstacles
            cfg.env.task.obstacle_keepout = 0.01    # Keepout distance for obstacles
            cfg.env.task.obstacle_velocity = 0.0    # Velocity of obstacles
            cfg.env.task.mode = "Brownian" # Obstacle movement mode

            cfg.env.task.arm_goal_velocity = 0.0    # Velocity for arm goal movement
            cfg.env.task.arm_goal_size = 0.05       # Size of the arm goal
            cfg.env.task.arm_goal_reach_done = True # Flag to finish the episode when the arm goal is reached

            cfg.env.task.base_goal_enable = False   # Disable base goal functionality

            cfg.env.task.seed = 0                  # Random seed for reproducibility

            
        case "G1FixedBase_D1_AG_DO_v0":
            
            # robot config
            cfg.robot.cfg.class_name = "G1FixedBaseDynamic1Config"
            cfg.env.agent.mujoco_model = "g1/g1_29dof_fixed_base.xml"

            # task config
            cfg.env.task.class_name = "G1BenchmarkTask"
            cfg.env.task.task_name = test_case
            cfg.env.task.num_obstacle_task = 5  # Number of obstacles in the task

            cfg.env.task.obstacle_range = [(-0.3, 0.5), (-0.4, 0.4), (0.5, 1.3)]
            cfg.env.task.obstacle_size = 0.05       # Size of obstacles
            cfg.env.task.obstacle_keepout = 0.01    # Keepout distance for obstacles
            cfg.env.task.obstacle_velocity = 0.005  # Velocity of obstacles
            cfg.env.task.mode = "Brownian"

            cfg.env.task.arm_goal_velocity = 0.0    # Velocity for arm goal movement
            cfg.env.task.arm_goal_size = 0.05       # Size of the arm goal
            cfg.env.task.arm_goal_reach_done = True # Flag to finish the episode when the arm goal is reached

            cfg.env.task.base_goal_enable = False   # Disable base goal functionality

            cfg.env.task.seed = 0                  # Random seed for reproducibility


        case "G1FixedBase_D1_AG_SO_v1":
            
            # robot config
            cfg.robot.cfg.class_name = "G1FixedBaseDynamic1Config"
            cfg.env.agent.mujoco_model = "g1/g1_29dof_fixed_base.xml"

            # task config
            cfg.env.task.class_name = "G1BenchmarkTask"
            cfg.env.task.task_name = test_case
            cfg.env.task.num_obstacle_task = 10  # Number of obstacles in the task

            cfg.env.task.obstacle_range = [(-0.3, 0.5), (-0.4, 0.4), (0.5, 1.3)]
            cfg.env.task.obstacle_size = 0.05       # Size of obstacles
            cfg.env.task.obstacle_keepout = 0.1     # Keepout distance for obstacles
            cfg.env.task.obstacle_velocity = 0.0    # Velocity of obstacles
            cfg.env.task.mode = "Brownian"

            cfg.env.task.arm_goal_velocity = 0.0    # Velocity for arm goal movement
            cfg.env.task.arm_goal_size = 0.05       # Size of the arm goal
            cfg.env.task.arm_goal_reach_done = True # Flag to finish episode when arm goal is reached

            cfg.env.task.base_goal_enable = False   # Disable base goal functionality

            cfg.env.task.seed = 0                  # Random seed

            
        case "G1FixedBase_D1_AG_DO_v1":
            
            # robot config
            cfg.robot.cfg.class_name = "G1FixedBaseDynamic1Config"
            cfg.env.agent.mujoco_model = "g1/g1_29dof_fixed_base.xml"

            # task config
            cfg.env.task.class_name = "G1BenchmarkTask"
            cfg.env.task.task_name = test_case
            cfg.env.task.num_obstacle_task = 10  # Number of obstacles in the task

            cfg.env.task.obstacle_range = [(-0.3, 0.5), (-0.4, 0.4), (0.5, 1.3)]
            cfg.env.task.obstacle_size = 0.05        # Size of obstacles
            cfg.env.task.obstacle_keepout = 0.1      # Keepout distance for obstacles
            cfg.env.task.obstacle_velocity = 0.005   # Velocity of obstacles
            cfg.env.task.mode = "Brownian"

            cfg.env.task.arm_goal_velocity = 0.0     # Velocity of the arm goal
            cfg.env.task.arm_goal_size = 0.05        # Size of the arm goal
            cfg.env.task.arm_goal_reach_done = True  # Flag to finish episode upon reaching arm goal

            cfg.env.task.base_goal_enable = False    # Disable base goal

            cfg.env.task.seed = 0                   # Random seed
            
            
        case "G1FixedBase_D2_AG_SO_v0":
            
            # robot config
            cfg.robot.cfg.class_name = "G1FixedBaseDynamic2Config"
            cfg.env.agent.mujoco_model = "g1/g1_29dof_fixed_base.xml"

            # task config
            cfg.env.task.class_name = "G1BenchmarkTask"
            cfg.env.task.task_name = test_case
            cfg.env.task.num_obstacle_task = 5  # Number of obstacles in the task

            cfg.env.task.obstacle_range = [(-0.3, 0.5), (-0.4, 0.4), (0.5, 1.3)]
            cfg.env.task.obstacle_size = 0.05       # Size of obstacles
            cfg.env.task.obstacle_keepout = 0.01    # Keepout distance for obstacles
            cfg.env.task.obstacle_velocity = 0.0    # Velocity of obstacles
            cfg.env.task.mode = "Brownian" # Obstacle movement mode

            cfg.env.task.arm_goal_velocity = 0.0    # Velocity for arm goal movement
            cfg.env.task.arm_goal_size = 0.05       # Size of the arm goal
            cfg.env.task.arm_goal_reach_done = True # Flag to finish the episode when the arm goal is reached

            cfg.env.task.base_goal_enable = False   # Disable base goal functionality

            cfg.env.task.seed = 0                  # Random seed for reproducibility

            
        case "G1FixedBase_D2_AG_DO_v0":
            
            # robot config
            cfg.robot.cfg.class_name = "G1FixedBaseDynamic2Config"
            cfg.env.agent.mujoco_model = "g1/g1_29dof_fixed_base.xml"

            # task config
            cfg.env.task.class_name = "G1BenchmarkTask"
            cfg.env.task.task_name = test_case
            cfg.env.task.num_obstacle_task = 5  # Number of obstacles in the task

            cfg.env.task.obstacle_range = [(-0.3, 0.5), (-0.4, 0.4), (0.5, 1.3)]
            cfg.env.task.obstacle_size = 0.05       # Size of obstacles
            cfg.env.task.obstacle_keepout = 0.01    # Keepout distance for obstacles
            cfg.env.task.obstacle_velocity = 0.005  # Velocity of obstacles
            cfg.env.task.mode = "Brownian"

            cfg.env.task.arm_goal_velocity = 0.0    # Velocity for arm goal movement
            cfg.env.task.arm_goal_size = 0.05       # Size of the arm goal
            cfg.env.task.arm_goal_reach_done = True # Flag to finish the episode when the arm goal is reached

            cfg.env.task.base_goal_enable = False   # Disable base goal functionality

            cfg.env.task.seed = 0                  # Random seed for reproducibility


        case "G1FixedBase_D2_AG_SO_v1":
            
            # robot config
            cfg.robot.cfg.class_name = "G1FixedBaseDynamic2Config"
            cfg.env.agent.mujoco_model = "g1/g1_29dof_fixed_base.xml"

            # task config
            cfg.env.task.class_name = "G1BenchmarkTask"
            cfg.env.task.task_name = test_case
            cfg.env.task.num_obstacle_task = 10  # Number of obstacles in the task

            cfg.env.task.obstacle_range = [(-0.3, 0.5), (-0.4, 0.4), (0.5, 1.3)]
            cfg.env.task.obstacle_size = 0.05       # Size of obstacles
            cfg.env.task.obstacle_keepout = 0.1     # Keepout distance for obstacles
            cfg.env.task.obstacle_velocity = 0.0    # Velocity of obstacles
            cfg.env.task.mode = "Brownian"

            cfg.env.task.arm_goal_velocity = 0.0    # Velocity for arm goal movement
            cfg.env.task.arm_goal_size = 0.05       # Size of the arm goal
            cfg.env.task.arm_goal_reach_done = True # Flag to finish episode when arm goal is reached

            cfg.env.task.base_goal_enable = False   # Disable base goal functionality

            cfg.env.task.seed = 0                  # Random seed

            
        case "G1FixedBase_D2_AG_DO_v1":
            
            # robot config
            cfg.robot.cfg.class_name = "G1FixedBaseDynamic2Config"
            cfg.env.agent.mujoco_model = "g1/g1_29dof_fixed_base.xml"

            # task config
            cfg.env.task.class_name = "G1BenchmarkTask"
            cfg.env.task.task_name = test_case
            cfg.env.task.num_obstacle_task = 10  # Number of obstacles in the task

            cfg.env.task.obstacle_range = [(-0.3, 0.5), (-0.4, 0.4), (0.5, 1.3)]
            cfg.env.task.obstacle_size = 0.05        # Size of obstacles
            cfg.env.task.obstacle_keepout = 0.1      # Keepout distance for obstacles
            cfg.env.task.obstacle_velocity = 0.005   # Velocity of obstacles
            cfg.env.task.mode = "Brownian"

            cfg.env.task.arm_goal_velocity = 0.0     # Velocity of the arm goal
            cfg.env.task.arm_goal_size = 0.05        # Size of the arm goal
            cfg.env.task.arm_goal_reach_done = True  # Flag to finish episode upon reaching arm goal

            cfg.env.task.base_goal_enable = False    # Disable base goal

            cfg.env.task.seed = 0                   # Random seed

        # ---------------------------------------------------------------------------- #
        #                                  G1SportMode                                 #
        # ---------------------------------------------------------------------------- #

        case "G1SportMode_D1_WG_SO_v1":
            
            # robot config
            cfg.robot.cfg.class_name = "G1SportModeDynamic1Config"
            cfg.env.agent.mujoco_model = "g1/g1_29dof_whole_body.xml" 
            
            # task config
            cfg.env.task.class_name = "G1BenchmarkTask"
            cfg.env.task.task_name = test_case
            # Obstacle configuration
            cfg.env.task.num_obstacle_task = 10           # Number of obstacles in the task
            cfg.env.task.robot_keepout = 0.1               # Keepout distance for the robot

            cfg.env.task.obstacle_range = [(-1, 1), (-1, 1), (0.5, 1.3)]  
            cfg.env.task.obstacle_size = 0.05              # Size of obstacles
            cfg.env.task.obstacle_keepout = 0.0            # Keepout distance for obstacles
            cfg.env.task.obstacle_velocity = 0.0           # Velocity of obstacles
            cfg.env.task.mode = "Brownian"

            # Arm goal configuration
            cfg.env.task.arm_goal_velocity = 0.0           # Velocity of the arm goal
            cfg.env.task.arm_goal_reach_done = True        # Flag to finish episode when arm goal is reached

            # Base goal configuration
            cfg.env.task.base_goal_range = [(-0.8, 0.8), (-0.8, 0.8), (0.793, 0.793)]  
            cfg.env.task.base_goal_rot_range = (-np.pi, np.pi)  # Rotation range for base goal as a tuple
            cfg.env.task.base_goal_velocity = 0.0               # Velocity of the base goal
            cfg.env.task.base_goal_reach_done = True            # Flag to finish episode when base goal is reached

            # Seed configuration
            cfg.env.task.seed = 0  # Random seed
        
        case _:
            raise ValueError(f"Unknown test case: {test_case}")
            
    return cfg

