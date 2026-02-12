from spark_task.autonomy.benchmark_task import BenchmarkTask, TaskObject3D
from spark_utils import Geometry, VizColor
import numpy as np

class CorridorTask(BenchmarkTask):
    def __init__(self, robot_cfg, robot_kinematics, agent, **kwargs):
        super().__init__(robot_cfg, robot_kinematics, agent, **kwargs)
        
        # Initialize task configuration
        self.task_name = kwargs.get("task_name", "CorridorTask")  # Name of the task
        self.max_episode_length = 2000  # Maximum number of steps per episode
        self.mode = "Velocity" # Mode for obstacle movement

        # Obstacle configuration
        self.num_obstacle_task = 1  # Number of obstacles in the task
        self.robot_keepout = 0.1  # Keepout distance for the robot

        self.obstacle_range = [(-11.0, 11.0), (-3.0, 3.0), (0.5, 1.5)]  # Range for obstacle placement [xmin, xmax, ymin, ymax, zmin, zmax]
        self.obstacle_size = 0.1  # Size of each obstacle
        self.obstacle_keepout = 0.1  # Minimum keepout distance for obstacles
        self.obstacle_init = np.array([1.5, 0.2, 0.8])
        self.obstacle_velocity = 0.1
        self.obstacle_direction = np.array([-1.0, 0.0, 0.0])
        
        # Arm goal configuration
        self.arm_goal_velocity = 0.0  # Velocity of arm goals
        self.arm_goal_reach_done = True  # Flag to finish episode when arm goal is reached

        self.goal_left_init = np.array([0.1,0.3,0.0]) # Initial position of left arm goal relative to the robot base
        self.goal_left_velocity = 0.0  
        self.goal_right_init = np.array([0.1,-0.3,0.0]) # Initial position of right arm goal relative to the robot base
        self.goal_right_velocity = 0.0

        # Base goal configuration
        self.base_goal_range = [(2.0, 2.0), (0.0, 0.0), (0.793, 0.793)]  # Range for base goal [xmin, xmax, ymin, ymax, zmin, zmax]
        self.base_goal_rot_range = (0, 0)  # Rotation range for base goal
        self.base_goal_velocity = 0.0   # Velocity of base goal
        self.base_goal_reach_done = True  # Flag to finish episode when base goal is reached
        # Seed configuration for random state
        self._seed = 10  # Default random seed
        self._init_seed()
        
        # ------------------------------------ ROS ----------------------------------- #

    def _init_obstacle(self):
        super()._init_obstacle()
        wall_1 = TaskObject3D(velocity=0.0, 
                                    keep_direction_step = self.obstacle_keep_direction_step,
                                    bound=self.obstacle_range,
                                    direction=self.obstacle_direction,
                                    smooth_weight = self.obstacle_smooth_weight,
                                    _seed = self._seed + len(self.obstacle_task),
                                    dt = self.dt)
        wall_1.frame[:3,3] = np.array([0.0, -1.0, 0.8])
        obstacle_geom = Geometry(type="box", length=10.0, width=0.1, height=1.6, color=VizColor.obstacle_task)
        self.obstacle_task.append(wall_1)
        self.obstacle_task_geom.append(obstacle_geom)
        
        wall_2 = TaskObject3D(velocity=0.0, 
                                    keep_direction_step = self.obstacle_keep_direction_step,
                                    bound=self.obstacle_range,
                                    direction=self.obstacle_direction,
                                    smooth_weight = self.obstacle_smooth_weight,
                                    _seed = self._seed + len(self.obstacle_task),
                                    dt = self.dt)
        wall_2.frame[:3,3] = np.array([0.0, 1.0, 0.8])
        obstacle_geom = Geometry(type="box", length=10.0, width=0.1, height=1.6, color=VizColor.obstacle_task)
        self.obstacle_task.append(wall_2)
        self.obstacle_task_geom.append(obstacle_geom)
    