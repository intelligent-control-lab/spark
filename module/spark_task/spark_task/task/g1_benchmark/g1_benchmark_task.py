# from spark_task.task.g1_teleop_sim.g1_teleop_sim_task_config import G1TeleopSimTaskConfig
from spark_task.task.base.base_task import BaseTask
from spark_utils import Geometry, VizColor
from spark_utils import compute_masked_distance_matrix
import numpy as np
from scipy.spatial.transform import Rotation as R

class TaskObject3D():
    def __init__(self, **kwargs):
        self.frame = kwargs.get("frame", np.eye(4))
        self.velocity = kwargs.get("velocity", 0.01)
        self.bound = kwargs.get("bound", np.zeros((3,2)))
        self.smooth_weight = kwargs.get("smooth_weight", 1.0)
        self.direction = kwargs.get("direction", np.array([0.0,0.0,0.0]))
        self.last_direction = self.direction
        self.step_counter = 0
        self.keep_direction_step = kwargs.get("keep_direction_step", 1)
        self.dt = kwargs.get("dt", 0.01)
        self._seed = kwargs.get("_seed", 0)
        self.rs = np.random.RandomState(self._seed)
    
    def move(self, mode):
        if mode == "Brownian":
            if self.step_counter % self.keep_direction_step == 0:
                direction = self.rs.normal(loc=0.0, size=3)
                direction = self.velocity * direction / np.linalg.norm(direction)
            else:
                direction = self.last_direction
            self.last_frame = self.frame.copy()
            update_step = (1 - self.smooth_weight) * self.last_direction + self.smooth_weight * direction
            self.frame[:3, 3] += update_step
            self.last_direction = self.frame[:3, 3] - self.last_frame[:3, 3]
        elif mode == "Velocity":
            update_step = self.velocity * self.last_direction * self.dt
            self.frame[:3, 3] += update_step
        # Enforce bounds
        for dim in range(3):
            if self.frame[dim, 3] < self.bound[dim][0]:
                self.frame[dim, 3] = self.bound[dim][0]
            elif self.frame[dim, 3] > self.bound[dim][1]:
                self.frame[dim, 3] = self.bound[dim][1]
            
        self.step_counter += 1

class ResamplingError(AssertionError):
    ''' Raised when we fail to sample a valid distribution of objects or goals '''
    pass

class G1BenchmarkTask(BaseTask):
    def __init__(self, robot_cfg, robot_kinematics, agent, **kwargs):
        super().__init__(robot_cfg, robot_kinematics, agent)
        
        # Initialize task configuration
        self.task_name = kwargs.get("task_name", "G1BenchmarkTask")  # Name of the task
        self.max_episode_length = kwargs.get("max_episode_length", 200)  # Maximum number of steps per episode
        self.mode = kwargs.get("mode", "Brownian") # Mode for obstacle movement
        self.dt = kwargs.get("dt", 0.01)

        # Obstacle configuration
        self.num_obstacle_task = kwargs.get("num_obstacle_task", 3)  # Number of obstacles in the task
        self.robot_keepout = kwargs.get("robot_keepout", 0.0)  # Keepout distance for the robot

        self.obstacle_range = kwargs.get("obstacle_range", [(-2, 2), (-2, 2), (0.8, 1.1)])  # Range for obstacle placement [xmin, xmax, ymin, ymax, zmin, zmax]
        self.obstacle_size = kwargs.get("obstacle_size", 0.05)  # Size of each obstacle
        self.obstacle_keepout = kwargs.get("obstacle_keepout", 0.05)  # Minimum keepout distance for obstacles
        self.obstacle_init = np.array(kwargs.get("obstacle_init", [0.2,0.0,1.0]))
        self.obstacle_velocity = kwargs.get("obstacle_velocity", 0.001)
        self.obstacle_direction = np.array(kwargs.get("obstacle_direction", [0.0,0.0,0.0]))
        self.obstacle_keep_direction_step = kwargs.get("obstacle_keep_direction_step", 500)  # Steps to maintain obstacle direction
        self.obstacle_smooth_weight = kwargs.get("obstacle_smooth_weight", 0.8)  # Smoothing weight for obstacle motion
        
        # Arm goal configuration
        self.arm_goal_enable = kwargs.get("arm_goal_enable", True)  # Enable arm goal functionality
        self.arm_goal_size = kwargs.get("arm_goal_size", 0.05)  # Size of arm goal markers
        self.arm_goal_keepout = kwargs.get("arm_goal_keepout", 0.1)  # Keepout distance for arm goals
        self.arm_goal_velocity = kwargs.get("arm_goal_velocity", 0.0)  # Velocity of arm goals
        self.arm_goal_keep_direction_step = kwargs.get("arm_goal_keep_direction_step", 500)  # Steps to maintain arm goal direction
        self.arm_goal_smooth_weight = kwargs.get("arm_goal_smooth_weight", 0.8)  # Smoothing weight for arm goal movement
        self.arm_goal_reach_done = kwargs.get("arm_goal_reach_done", False)  # Flag to finish episode when arm goal is reached

        self.left_arm_goal_range = kwargs.get("left_arm_goal_range", [(0.1, 0.4), (0.1, 0.4), (0.0, 0.3)])  # Range for left arm goal [xmin, xmax, ymin, ymax, zmin, zmax] relative to the robot base
        self.goal_left_init = np.array(kwargs.get("goal_left_init", [0.25, 0.25, 0.1])) # Initial position of left arm goal relative to the robot base
        self.goal_left_velocity = kwargs.get("goal_left_velocity", 0.0)
        self.goal_left_direction = np.array(kwargs.get("goal_left_direction", [0.0,0.0,0.0]))
        self.right_arm_goal_range = kwargs.get("right_arm_goal_range", [(0.1, 0.4), (-0.4, -0.1), (0.0, 0.3)])  # Range for right arm goal [xmin, xmax, ymin, ymax, zmin, zmax] relative to the robot base
        self.goal_right_init = np.array(kwargs.get("goal_right_init", [0.25, -0.25, 0.1])) # Initial position of right arm goal relative to the robot base
        self.goal_right_velocity = kwargs.get("goal_right_velocity", 0.0)
        self.goal_right_direction = np.array(kwargs.get("goal_right_direction", [0.0, 0.0, 0.0]))

        # Base goal configuration
        self.base_goal_enable = kwargs.get("base_goal_enable", True)  # Enable base goal functionality
        self.base_goal_range = kwargs.get("base_goal_range", [(0, 0), (0, 0), (0.793, 0.793)])  # Range for base goal [xmin, xmax, ymin, ymax, zmin, zmax]
        self.base_goal_rot_range = kwargs.get("base_goal_rot_range", [0, 0])  # Rotation range for base goal
        self.base_goal_size = kwargs.get("base_goal_size", 0.1)  # Size of the base goal marker
        self.base_goal_init = np.array(kwargs.get("base_goal_init", [0.0, 0.0, 0.0]))
        self.base_goal_keepout = kwargs.get("base_goal_keepout", 0.1)  # Keepout distance for base goal
        self.base_goal_velocity = kwargs.get("base_goal_velocity", 0.0)  # Velocity of base goal
        self.base_goal_direction = np.array(kwargs.get("base_goal_direction", [1.0, 0.0, 0.0]))
        self.base_goal_keep_direction_step = kwargs.get("base_goal_keep_direction_step", 500)  # Steps to maintain base goal direction
        self.base_goal_smooth_weight = kwargs.get("base_goal_smooth_weight", 0.8)  # Smoothing weight for base goal movement
        self.base_goal_reach_done = kwargs.get("base_goal_reach_done", False)  # Flag to finish episode when base goal is reached
        # Seed configuration for random state
        self._seed = kwargs.get("seed", 0)  # Default random seed
        self.seed_list = kwargs.get("seed_list", [])  # List of seeds for resampling
        self._init_seed()
        
        # ------------------------------------ ROS ----------------------------------- #
        self.enable_ros = kwargs.get("enable_ros", False) # Enable ROS integration
       
    def _init_seed(self):
        '''
        Initialize seed to be one before the first seed to use
        reset() will increment the seed to the first seed to use
        '''
        # self._seed = np.random.randint(2**32) if seed is None else seed
        self._seed_list_idx = -1
        self._seed -= 1

    def _init_obstacle(self):
         # ------------------------------- init obstacle ------------------------------ #
        self.obstacle_task = []
        self.obstacle_task_geom = []

        cnt = 0
        while len(self.obstacle_task) < self.num_obstacle_task:
            
            # Randomly sample a point
            obstacle = TaskObject3D(velocity=self.obstacle_velocity, 
                                    keep_direction_step = self.obstacle_keep_direction_step,
                                    bound=self.obstacle_range,
                                    direction=self.obstacle_direction,
                                    smooth_weight = self.obstacle_smooth_weight,
                                    _seed = self._seed + len(self.obstacle_task),
                                    dt = self.dt)
            if self.mode == 'Velocity':
                obstacle.frame[:3,3] = self.obstacle_init
                obstacle_geom = Geometry(type="sphere", radius=self.obstacle_size, color=VizColor.obstacle_task)
                self.obstacle_task.append(obstacle)
                self.obstacle_task_geom.append(obstacle_geom)
            else:
                obstacle.direction =self.rs.uniform(low=-1.0, high=1.0, size=3)
                obstacle.frame[:3,3] = np.array([self.rs.uniform(low, high) for low, high in obstacle.bound])
                obstacle_geom = Geometry(type="sphere", radius=self.obstacle_size, color=VizColor.obstacle_task)
                valid_flag = True
                
                # check if the new obstacle is too close to existing obstacles or robot
                dist_robot_to_env, _ = compute_masked_distance_matrix(
                    frame_list_1=self.robot_frames_world,
                    geom_list_1=self.robot_cfg.CollisionVol.values(),
                    frame_list_2=[obstacle.frame,],
                    geom_list_2=[obstacle_geom,]
                )
                if dist_robot_to_env is not None and dist_robot_to_env.min() < self.robot_keepout:
                    valid_flag = False
                
                dist_obstacle_to_obstacle, _ = compute_masked_distance_matrix(
                    frame_list_1=[obs.frame for obs in self.obstacle_task],
                    geom_list_1=[obs_geom for obs_geom in self.obstacle_task_geom],
                    frame_list_2=[obstacle.frame,],
                    geom_list_2=[obstacle_geom,]
                )
                if dist_obstacle_to_obstacle is not None and dist_obstacle_to_obstacle.min() < self.obstacle_keepout:
                    valid_flag = False

                # add to task if valid
                if valid_flag:
                    self.obstacle_task.append(obstacle)
                    self.obstacle_task_geom.append(obstacle_geom)

                cnt += 1
                if cnt > 10000:
                    raise ResamplingError('Failed to sample layout of obstacles')
                
        return

    def _init_goal(self):
        # --------------------------------- init goal -------------------------------- #
        
        if self.arm_goal_enable:
            
            self.robot_goal_left = TaskObject3D(velocity=self.goal_left_velocity, 
                                                direction=self.goal_left_direction,
                                                keep_direction_step = self.arm_goal_keep_direction_step,
                                                bound=self.left_arm_goal_range,
                                                smooth_weight = self.arm_goal_smooth_weight,
                                                _seed = self._seed,
                                                dt = self.dt)
            
            self.robot_goal_right = TaskObject3D(velocity=self.goal_right_velocity, 
                                                direction=self.goal_right_direction,
                                                keep_direction_step = self.arm_goal_keep_direction_step,
                                                bound=self.right_arm_goal_range,
                                                smooth_weight = self.arm_goal_smooth_weight,
                                                _seed = self._seed,
                                                dt = self.dt)
            if self.mode == 'Velocity':
                self.robot_goal_left.frame[:3,3] = self.goal_left_init
                self.robot_goal_right.frame[:3,3] = self.goal_right_init
            else:
                self.robot_goal_left.frame[:3,3] = np.array([self.rs.uniform(low, high) for low, high in self.robot_goal_left.bound])
                self.robot_goal_right.frame[:3,3] = np.array([self.rs.uniform(low, high) for low, high in self.robot_goal_right.bound])
            
                # ------------------------------ Distance Check ------------------------------ #
                """
                Distance check between each arm goal and obstacles to prevent infeasible setting.
                """
                for _ in range(10_000_000):
                    leftGoal = np.eye(4)
                    leftGoal[:3,3] = np.array([self.rs.uniform(low, high) for low, high in self.robot_goal_left.bound])                
                    leftGoal_world = self.robot_base_frame @ leftGoal
                    valid_flag = True
                    for other_obstacle in self.obstacle_task:
                        if np.linalg.norm(leftGoal_world[:3,3] - other_obstacle.frame[:3,3]) < self.arm_goal_keepout:                        
                            valid_flag = False
                            print(f"Dis to L: {np.linalg.norm(leftGoal_world[:3,3] - other_obstacle.frame[:3,3])}")
                            break
                        
                    if valid_flag:
                        self.robot_goal_left.frame[:3,3] = leftGoal[:3,3]           
                        break
                else:
                    raise ResamplingError('Failed to sample layout of arm goal')
            
                for _ in range(10_000_000):
                    rightGoal = np.eye(4)
                    rightGoal[:3,3] = np.array([self.rs.uniform(low, high) for low, high in self.robot_goal_right.bound])
                    rightGoal_world = self.robot_base_frame @ rightGoal
                    
                    valid_flag = True
                    for other_obstacle in self.obstacle_task:
                        if np.linalg.norm(rightGoal_world[:3,3] - other_obstacle.frame[:3,3]) < self.arm_goal_keepout:
                            valid_flag = False
                            print(f"Dis to R: {np.linalg.norm(rightGoal_world[:3,3] - other_obstacle.frame[:3,3])}")
                            print(f"goal pos: {rightGoal_world[:3,3]}")
                            break
                        
                    if valid_flag:
                        self.robot_goal_right.frame[:3,3] = rightGoal[:3,3]
                        break
                else:
                    raise ResamplingError('Failed to sample layout of right arm goal')
            # ------------------------------ Distance Check ------------------------------ #
        
        if self.base_goal_enable:
            self.robot_goal_base = TaskObject3D(velocity=self.base_goal_velocity, 
                                                direction=self.base_goal_direction,
                                                keep_direction_step = self.base_goal_keep_direction_step,
                                                bound=self.base_goal_range,
                                                smooth_weight = self.base_goal_smooth_weight,
                                                _seed = self._seed,
                                                dt = self.dt)
            
            if self.mode == 'Velocity':
                self.robot_goal_base.frame[:3,3] = self.base_goal_init
            else:
                self.robot_goal_base.frame[:3,3] = np.array([self.rs.uniform(low, high) for low, high in self.robot_goal_left.bound])
            
                for _ in range(10000):
                    goal_frame = np.array([self.rs.uniform(low, high) for low, high in self.robot_goal_base.bound])
                    
                    valid_flag = True
                    for other_obstacle in self.obstacle_task:
                        if np.linalg.norm(goal_frame[:2] - other_obstacle.frame[:2,3]) < self.base_goal_keepout:
                            valid_flag = False
                            break
                
                    if valid_flag:
                        self.robot_goal_base.frame[:3,3] = goal_frame
                        break
                else:
                    raise ResamplingError('Failed to sample layout of base goal')
                
                self.robot_goal_base.frame[:3,:3] = R.from_euler('xyz', [0, 0, self.rs.uniform(self.base_goal_rot_range[0], self.base_goal_rot_range[1])]).as_matrix()
        
        return

    def _update_robot_goal(self):
        if self.arm_goal_enable:
            self.robot_goal_left.move(self.mode)
            self.robot_goal_right.move(self.mode)
        if self.base_goal_enable:
            self.robot_goal_base.move(self.mode)
         
    def _update_obstacle_task(self):
        for obstacle in self.obstacle_task:
            obstacle.move(self.mode)
    
    def _update_done(self):
        self._done = False
        
        # check goal reaching conditions
        arm_goal_reach_flag = False
        if self.arm_goal_enable:
            if hasattr(self.robot_cfg.Frames, "L_ee"):
                dist_goal_left = np.linalg.norm(self.robot_frames_world[self.robot_cfg.Frames.L_ee,:3,3] - (self.robot_base_frame @ self.robot_goal_left.frame)[:3,3])
            else:
                dist_goal_left = 0
            if hasattr(self.robot_cfg.Frames, "R_ee"):
                dist_goal_right = np.linalg.norm(self.robot_frames_world[self.robot_cfg.Frames.R_ee,:3,3] - (self.robot_base_frame @ self.robot_goal_right.frame)[:3,3])
            else:
                dist_goal_right = 0
            if dist_goal_left < self.arm_goal_size and dist_goal_right < self.arm_goal_size:
                arm_goal_reach_flag = True
        
        base_goal_reach_flag = False
        if self.base_goal_enable:
            dist_goal_base = np.linalg.norm(self.robot_base_frame[:2,3] - self.robot_goal_base.frame[:2,3])
            if dist_goal_base < self.base_goal_size:
                base_goal_reach_flag = True

        if self.arm_goal_enable and self.base_goal_enable:
            if self.arm_goal_reach_done and self.base_goal_reach_done:
                self._done |= (arm_goal_reach_flag and base_goal_reach_flag)
            elif self.arm_goal_reach_done:
                self._done |= arm_goal_reach_flag
            elif self.base_goal_reach_done:
                self._done |= base_goal_reach_flag
        elif self.arm_goal_enable:
            if self.arm_goal_reach_done:
                self._done |= arm_goal_reach_flag
        elif self.base_goal_enable:
            if self.base_goal_reach_done:
                self._done |= base_goal_reach_flag

        # check episode length limit
        if self.max_episode_length >= 0 and self.episode_length >= self.max_episode_length:
            self._done = True
    
    def _update_robot_state(self, feedback):
        # ---------------------------- compute robot state --------------------------- #
        self.robot_base_frame = feedback["robot_base_frame"]
        # compute transformations of robot collision volumes
        x = feedback["state"]
        dof_pos = self.robot_cfg.decompose_state_to_dof_pos(x)
        robot_frames = self.robot_kinematics.forward_kinematics(dof_pos)
        self.robot_frames_world = np.zeros_like(robot_frames)
        for i in range(len(robot_frames)):
            self.robot_frames_world[i, :, :] = self.robot_base_frame @ robot_frames[i, :, :]
            
        return
    
    def reset(self, feedback):
        
        # update seed
        if len(self.seed_list) > 0:
            self._seed_list_idx = (self._seed_list_idx + 1) % len(self.seed_list)  # Increment seed list index
            self._seed = self.seed_list[self._seed_list_idx]
        else:
            self._seed += 1  # Increment seed
            
        self.rs = np.random.RandomState(self._seed)
        
        # ----------------------------------- init ----------------------------------- #
        self.episode_length = 0

        self._update_robot_state(feedback)
        self._init_obstacle()
        self._init_goal()
        # --------------------------------- init info -------------------------------- #
        self.info = {}
        self.info["goal_teleop"] = {}
        self.info["obstacle_task"] = {}
        self.info["obstacle_debug"] = {}
        self.info["obstacle"] = {}
        self.info["robot_frames"] = None
        self.info["robot_state"] = {}
        self.info["arm_goal_enable"] = self.arm_goal_enable
        self.info["base_goal_enable"] = self.base_goal_enable
        self._done = False
    
    def step(self, feedback):
        
        self.episode_length += 1
        
        # update task objects for next iteration
        self._update_robot_state(feedback)
        self._update_done()
        self._update_robot_goal()
        self._update_obstacle_task()
    
    def get_info(self, feedback) -> dict:

        self.info["done"] = self._done
        
        self.info["seed"] = self._seed
        
        self.info["episode_length"] = self.episode_length

        self.info["robot_base_frame"]               = feedback["robot_base_frame"]
        
        if self.arm_goal_enable:
            self.info["goal_teleop"]["left"]            = self.robot_base_frame @ self.robot_goal_left.frame
            self.info["goal_teleop"]["right"]           = self.robot_base_frame @ self.robot_goal_right.frame
            self.info["arm_goal_size"] = self.arm_goal_size
        
        if self.base_goal_enable:
            self.info["base_goal_size"] = self.base_goal_size
            self.info["goal_teleop"]["base"]            = self.robot_goal_base.frame
        
        self.info["obstacle_task"]["frames_world"]  = [obstacle.frame for obstacle in self.obstacle_task] if len(self.obstacle_task) > 0 else np.empty((0, 4, 4))
        self.info["obstacle_task"]["geom"]          = self.obstacle_task_geom
        self.info["obstacle_task"]["velocity"]      = [obstacle.velocity * np.concatenate((obstacle.direction, np.zeros(3))) for obstacle in self.obstacle_task] if len(self.obstacle_task) > 0 else np.empty((0, 6))
        self.info["obstacle_debug"]["frames_world"] = feedback.get("obstacle_debug_frame", np.empty((0, 4, 4)))
        self.info["obstacle_debug"]["geom"]         = feedback.get("obstacle_debug_geom", [])
        self.info["obstacle_debug"]["velocity"]     = feedback.get("obstacle_debug_velocity", np.empty((0, 6)))
        self.info["obstacle"]["frames_world"]       = np.concatenate([self.info["obstacle_task"]["frames_world"], self.info["obstacle_debug"]["frames_world"]], axis=0)
        self.info["obstacle"]["velocity"]           = np.concatenate([self.info["obstacle_task"]["velocity"], self.info["obstacle_debug"]["velocity"]], axis=0) 
        self.info["obstacle"]["geom"]               = np.concatenate([self.info["obstacle_task"]["geom"], self.info["obstacle_debug"]["geom"]], axis=0)
        self.info["obstacle"]["num"]                = len(self.info["obstacle"]["frames_world"])
        # robot state
        self.info["robot_state"]["dof_pos_cmd"]     = feedback["dof_pos_cmd"]
        self.info["robot_state"]["dof_pos_fbk"]     = feedback["dof_pos_fbk"]
        self.info["robot_state"]["dof_vel_cmd"]     = feedback["dof_vel_cmd"]
        
        return self.info
            
            
if __name__ == "__main__":
    pass