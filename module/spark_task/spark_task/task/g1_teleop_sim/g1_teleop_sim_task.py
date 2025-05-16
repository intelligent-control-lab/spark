# from spark_task.task.g1_teleop_sim.g1_teleop_sim_task_config import G1TeleopSimTaskConfig
from spark_task.task.base.base_task import BaseTask
from spark_utils import Geometry, VizColor
from spark_utils import pos_quat_to_transformation, rpy2quat
from spark_pipeline.plotter import read_tensorboard_scalars, read_tensorboard_tags
import numpy as np
import os

class TaskObject3D():
    def __init__(self, **kwargs):
        self.frame = kwargs.get("frame", np.eye(4))
        self.last_frame = self.frame.copy()
        self.frame_callback = self.frame.copy()
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

class G1TeleopSimTask(BaseTask):
    def __init__(self, robot_cfg, robot_kinematics, agent, **kwargs):
        super().__init__(robot_cfg, robot_kinematics, agent)
        
        self.task_name = kwargs.get("task_name", "G1TeleopSimTask")
        self.max_episode_length = kwargs.get("max_episode_length", -1)
        self.mode = kwargs.get("mode", "Brownian") # Mode for obstacle movement
        self.dt = kwargs.get("dt", 0.01)
        # Obstacle configuration
        self.num_obstacle_task = kwargs.get("num_obstacle_task", 3)  # Number of obstacles in the task
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

        self.left_arm_goal_range = kwargs.get("left_arm_goal_range", [(0.1, 0.4), (0.1, 0.4), (0.0, 0.3)])  # Range for left arm goal [xmin, xmax, ymin, ymax, zmin, zmax] 
        self.goal_left_init = np.array(kwargs.get("goal_left_init", [0.25, 0.25, 0.1]))
        self.goal_left_velocity = kwargs.get("goal_left_velocity", 0.0)
        self.goal_left_direction = np.array(kwargs.get("goal_left_direction", [0.0,0.0,0.0]))
        self.right_arm_goal_range = kwargs.get("right_arm_goal_range", [(0.1, 0.4), (-0.4, -0.1), (0.0, 0.3)])  # Range for right arm goal [xmin, xmax, ymin, ymax, zmin, zmax]
        self.goal_right_init = np.array(kwargs.get("goal_right_init", [0.25, -0.25, 0.1]))
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
        
        # ------------------------------------ ROS ----------------------------------- #
        self.enable_ros = kwargs.get("enable_ros", False) # Enable ROS integration
        self.ros_args = kwargs["ros_params"]     
        self._init_ros()
        
    def _init_ros(self):
        if self.enable_ros:

            import rospy
            from std_msgs.msg import Float64MultiArray

            # data buf
            self.robot_command = Float64MultiArray()
            self.robot_state = Float64MultiArray()

            
            self.rospy = rospy

            # publisher
            self.robot_command_pub =  self.rospy.Publisher(self.ros_args["robot_command_topic"], Float64MultiArray, queue_size=17)
            self.robot_state_pub =  self.rospy.Publisher(self.ros_args["robot_state_topic"], Float64MultiArray, queue_size=17)
            
            # subscriber
            self.robot_teleop_sub = self.rospy.Subscriber(self.ros_args["robot_teleop_topic"], Float64MultiArray, self._teleop_goal_callback)
            self.obstacle_sub = self.rospy.Subscriber(self.ros_args["obstacle_topic"], Float64MultiArray, self._obstacle_state_callback)

            # launch ros node
            self.rospy.init_node(self.task_name, anonymous=True)
        return
    
    def _init_obstacle(self):
        # ------------------------------- init obstacle ------------------------------ #
        self.obstacle_task = []
        self.obstacle_task_geom = []
        for _ in range(self.num_obstacle_task):
            obstacle = TaskObject3D(velocity=self.obstacle_velocity, 
                                    keep_direction_step = self.obstacle_keep_direction_step,
                                    bound=self.obstacle_range,
                                    direction=self.obstacle_direction,
                                    smooth_weight = self.obstacle_smooth_weight,
                                    _seed = self._seed + len(self.obstacle_task),
                                    dt = self.dt)
            if self.mode == 'Velocity':
                obstacle.frame[:3,3] = self.obstacle_init
            else:
                obstacle.frame[:3,3] = np.array([np.random.uniform(low, high) for low, high in obstacle.bound])
            obstacle.frame_callback = obstacle.frame.copy()

            self.obstacle_task.append(obstacle)
            self.obstacle_task_geom.append(Geometry(type="sphere", radius = 0.05, color=VizColor.obstacle_task))
            
        return

    def _init_goal(self):
        # --------------------------------- init goal -------------------------------- #
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
            self.robot_goal_left.frame[:3,3] = np.array([np.random.uniform(low, high) for low, high in self.robot_goal_left.bound])
            self.robot_goal_right.frame[:3,3] = np.array([np.random.uniform(low, high) for low, high in self.robot_goal_right.bound])
    
        self.robot_goal_left.frame_callback = self.robot_goal_left.frame.copy()
        self.robot_goal_right.frame_callback = self.robot_goal_right.frame.copy()
        
        return

    def _update_robot_goal(self, feedback):
        if self.enable_ros == False:
            robot_goal_left = np.eye(4)
            robot_goal_left[:3, 3] = feedback["robot_goal_left_offset"][:3, 3] + self.goal_left_init
            self.robot_goal_left.frame = robot_goal_left
            
            robot_goal_right = np.eye(4)
            robot_goal_right[:3, 3] = feedback["robot_goal_right_offset"][:3, 3] + self.goal_right_init
            self.robot_goal_right.frame = robot_goal_right
            # self.robot_goal_left.move(self.mode)
            # self.robot_goal_right.move(self.mode)
        else:
            self.robot_goal_left.frame = self.robot_goal_left.frame_callback
            self.robot_goal_right.frame = self.robot_goal_right.frame_callback
        
        return
            
    def _update_obstacle_task(self, feedback):
        if self.enable_ros == False:
            for obstacle in self.obstacle_task:
                obstacle.move(self.mode)
        else:
            for i in range(self.num_obstacle_task):
                
                # update the obstacle position based on the ros callback
                self.obstacle_task[i].frame[:3,3] = self.obstacle_task[i].frame_callback[:3,3]
            
    def reset(self, feedback):
        self.episode_length = 0
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
        
        return   
    
    def step(self, feedback):        
        self.episode_length += 1
        
        # update task objects for next iteration
        self._update_robot_goal(feedback)
        self._update_obstacle_task(feedback)
    
    def get_info(self, feedback) -> dict:
        self.info["done"] = False
        
        self.info["seed"] = self._seed

        if self.enable_ros and self.rospy.is_shutdown():
            self.info["done"] = True
        
        if self.max_episode_length >= 0 and self.episode_length >= self.max_episode_length:
            self.info["done"] = True

        # robot base is available in feedback in sim
        self.info["robot_base_frame"]               = feedback["robot_base_frame"]
        
        self.info["goal_teleop"]["left"]            = feedback["robot_base_frame"] @ self.robot_goal_left.frame
        self.info["goal_teleop"]["right"]           = feedback["robot_base_frame"] @ self.robot_goal_right.frame
        
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
    
    # ---------------------------------- ROS helpers --------------------------------- #
    
    def _teleop_goal_callback(self, data):
        self.robot_goal_left.frame_callback = np.array(data.data[:16]).reshape(4,4)
        self.robot_goal_right.frame_callback = np.array(data.data[16:]).reshape(4,4)
        pass
        
    def _obstacle_state_callback(self, data):
        for i in range(self.num_obstacle_task):
            self.obstacle_task[i].frame_callback = self.info["robot_base_frame"] @ pos_quat_to_transformation(np.array(data.data[i*7+1: i*7 + 4]), 
                                                                        np.array([0.0, 0.0, 0.0, 1.0])) 
    
    def pub_robot_command(self, command):
        if self.enable_ros:
            self.robot_command.data = command
            self.robot_command_pub.publish(self.robot_command)
        
    def pub_robot_state(self, action_info = None):
        if self.enable_ros:
            self.robot_state.data = self.agent.get_feedback()["dof_pos_fbk"]
            self.robot_state_pub.publish(self.robot_state)
            
            
if __name__ == "__main__":
    pass