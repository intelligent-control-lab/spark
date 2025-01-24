# from spark_task.task.g1_teleop_sim.g1_teleop_sim_task_config import G1TeleopSimTaskConfig
from spark_task.task.base.base_task import BaseTask
from spark_utils import Geometry, VizColor
import numpy as np

class TaskObject3D():
    def __init__(self, **kwargs):
        self.frame = kwargs.get("frame", np.eye(4))
        self.velocity = kwargs.get("velocity", 1.0)
        self.bound = kwargs.get("bound", np.zeros((3,2)))
        self.smooth_weight = kwargs.get("smooth_weight", 1.0)
        self.last_direction = kwargs.get("direction", np.array([-1.0,0.0,0.0]))
        self.step_counter = 0
        self.keep_direction_step = kwargs.get("keep_direction_step", 1)
    
    def move(self, mode):
        if mode == "Brownian":
            if self.step_counter % self.keep_direction_step == 0:
                direction = np.random.normal(loc=0.0, size=3)
                direction = self.velocity * direction / np.linalg.norm(direction)
            else:
                direction = self.last_direction
            self.last_frame = self.frame.copy()
            update_step = (1 - self.smooth_weight) * self.last_direction + self.smooth_weight * direction
            self.frame[:3, 3] += update_step

            self.last_direction = self.frame[:3, 3] - self.last_frame[:3, 3]

        elif mode == "Velocity":
            update_step = self.velocity * self.last_direction
            self.frame[:3, 3] += update_step

        # Enforce bounds
        for dim in range(3):
            if self.frame[dim, 3] < self.bound[dim][0]:
                self.frame[dim, 3] = self.bound[dim][0]
            elif self.frame[dim, 3] > self.bound[dim][1]:
                self.frame[dim, 3] = self.bound[dim][1]
       
        self.step_counter += 1


class G1BenchmarkTask(BaseTask):
    def __init__(self, robot_cfg, robot_kinematics, agent, **kwargs):
        super().__init__(robot_cfg, robot_kinematics, agent)
        
        self.task_name = kwargs.get("task_name", "G1BenchmarkTask")
        self.max_episode_length = kwargs.get("max_episode_length", -1)
        self.num_obstacle_task = kwargs.get("num_obstacle", 3) # todo online update
        self.mode = kwargs.get("mode", "Brownian") #kwargs.get("mode", "Brownian")
        self.obstacle_init = kwargs.get("obstacle_init", np.array([0.2,0.0,1.0]))
        self.obstacle_velocity = kwargs.get("obstacle_velocity", 0.001)
        self.obstacle_direction = kwargs.get("obstacle_direction", np.array([-1.0,0.0,0.0]))
        self.goal_left_init = kwargs.get("goal_left_init", np.array([0.25, 0.25, 0.1]))
        self.goal_left_velocity = kwargs.get("goal_left_velocity", 0.001)
        self.goal_left_direction = kwargs.get("goal_left_direction", np.array([1.0,0.0,0.0]))
        self.goal_right_init = kwargs.get("goal_right_init", np.array([0.25, -0.25, 0.1]))
        self.goal_right_velocity = kwargs.get("goal_right_velocity", 0.001)
        self.goal_right_direction = kwargs.get("goal_right_direction", np.array([1.0, 0.0, 0.0]))
        
        self.reset()

    def reset(self):
        
        # ----------------------------------- init ----------------------------------- #
        self.episode_length = 0

        # ------------------------------- init obstacle ------------------------------ #
        self.obstacle_task = []
        self.obstacle_task_geom = []
            
        for _ in range(self.num_obstacle_task):
            obstacle = TaskObject3D(velocity=self.obstacle_velocity, 
                                    direction=self.obstacle_direction,
                                    keep_direction_step = 500,
                                    bound=np.array([[-0.3, 0.5], 
                                                    [-0.3, 0.5], 
                                                    [0.8, 1.0]]))
            if self.mode == 'Velocity':
                obstacle.frame[:3,3] = self.obstacle_init # need to deal with multiple init later
            else:
                obstacle.frame[:3,3] = np.array([np.random.uniform(low, high) for low, high in obstacle.bound])

            self.obstacle_task.append(obstacle)
            self.obstacle_task_geom.append(Geometry(type="sphere", radius=0.05, color=VizColor.obstacle_task))

        # --------------------------------- init goal -------------------------------- #
        
        self.robot_goal_left = TaskObject3D(velocity=self.goal_left_velocity, 
                                            direction=self.goal_left_direction,
                                            keep_direction_step = 10,
                                            bound=np.array([[0.1, 0.4], 
                                                            [0.1, 0.4], 
                                                            [0.0, 0.2]]),
                                            smooth_weight = 0.8)
        
        
        self.robot_goal_right = TaskObject3D(velocity=self.goal_right_velocity, 
                                             direction=self.goal_right_direction,
                                             keep_direction_step = 10,
                                             bound=np.array([[0.1, 0.4], 
                                                             [-0.4, -0.1], 
                                                             [0.0, 0.2]]),
                                             smooth_weight = 0.8)
        if self.mode == 'Velocity':
            self.robot_goal_left.frame[:3,3] = self.goal_left_init
            self.robot_goal_right.frame[:3,3] = self.goal_right_init
        else:
            self.robot_goal_left.frame[:3,3] = np.array([np.random.uniform(low, high) for low, high in self.robot_goal_left.bound])
            self.robot_goal_right.frame[:3,3] = np.array([np.random.uniform(low, high) for low, high in self.robot_goal_right.bound])

        self.goal_teleop = {}
        self.goal_teleop["left"] = self.robot_goal_left.frame
        self.goal_teleop["right"] = self.robot_goal_right.frame
        
        
        # --------------------------------- init info -------------------------------- #
        self.info = {}
        self.info["goal_teleop"] = {}
        self.info["obstacle_task"] = {}
        self.info["obstacle_debug"] = {}
        self.info["obstacle"] = {}
        self.info["robot_frames"] = None
        self.info["robot_state"] = {}

    def update_robot_goal(self):
        self.robot_goal_left.move(self.mode)
        self.robot_goal_right.move(self.mode)
    
    def update_obstacle_task(self):
        for obstacle in self.obstacle_task:
            obstacle.move(self.mode)
    
    def step(self, feedback):
        
        self.episode_length += 1
        
        # update task objects for next iteration
        self.update_robot_goal()
        self.update_obstacle_task()
    
    def get_info(self, feedback) -> dict:

        self.info["done"] = False
        
        if self.max_episode_length >= 0 and self.episode_length >= self.max_episode_length:
            self.info["done"] = True
        
        self.info["episode_length"] = self.episode_length

        self.info["robot_base_frame"]               = feedback["robot_base_frame"]
        
        self.info["goal_teleop"]["left"]            = self.robot_goal_left.frame
        self.info["goal_teleop"]["right"]           = self.robot_goal_right.frame
        
        self.info["obstacle_task"]["frames_world"]  = [obstacle.frame for obstacle in self.obstacle_task] if len(self.obstacle_task) > 0 else np.empty((0, 4, 4))
        self.info["obstacle_task"]["geom"]          = self.obstacle_task_geom
        self.info["obstacle_debug"]["frames_world"] = feedback.get("obstacle_debug_frame", np.empty((0, 4, 4)))
        self.info["obstacle_debug"]["geom"]         = feedback.get("obstacle_debug_geom", [])
        self.info["obstacle"]["frames_world"]       = np.concatenate([self.info["obstacle_task"]["frames_world"], self.info["obstacle_debug"]["frames_world"]], axis=0)
        self.info["obstacle"]["geom"]               = np.concatenate([self.info["obstacle_task"]["geom"], self.info["obstacle_debug"]["geom"]], axis=0)
        self.info["obstacle"]["num"]                = len(self.info["obstacle"]["frames_world"])
        
        self.info["robot_state"]["dof_pos_cmd"]     = feedback["dof_pos_cmd"]
        self.info["robot_state"]["dof_pos_fbk"]     = feedback["dof_pos_fbk"]
        self.info["robot_state"]["dof_vel_cmd"]     = feedback["dof_vel_cmd"]
        
        return self.info
            
            
if __name__ == "__main__":
    pass