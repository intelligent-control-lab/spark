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
        self.last_direction = np.zeros(3)
        self.step_counter = 0
        self.keep_direction_step = kwargs.get("keep_direction_step", 1)
    
    def move(self, mode="Brownian"):
        
        if mode == "Brownian":
            if self.step_counter % self.keep_direction_step == 0:
                direction = np.random.normal(loc=0.0, size=3)
                direction = self.velocity * direction / np.linalg.norm(direction)
            else:
                direction = self.last_direction
            self.last_frame = self.frame.copy()
            update_step = (1 - self.smooth_weight) * self.last_direction +  self.smooth_weight * direction
            self.frame[:3,3] += update_step
            
            for dim in range(3):
                if self.frame[dim,3] < self.bound[dim][0]:
                    self.frame[dim,3] = self.last_frame[dim,3] - update_step[dim]
                elif self.frame[dim,3] > self.bound[dim][1]:
                    self.frame[dim,3] = self.last_frame[dim,3] - update_step[dim]
            
            self.last_direction = self.frame[:3,3] - self.last_frame[:3,3]

        self.step_counter += 1
    
    

class G1BenchmarkTask(BaseTask):
    def __init__(self, robot_cfg, robot_kinematics, agent, **kwargs):
        super().__init__(robot_cfg, robot_kinematics, agent)
        
        self.task_name = kwargs.get("task_name", "G1BenchmarkTask")
        self.max_episode_length = kwargs.get("max_episode_length", -1)
        self.num_obstacle_task = kwargs.get("num_obstacle", 3) # todo online update
        
        self.reset()

    def reset(self):
        
        # ----------------------------------- init ----------------------------------- #
        self.episode_length = 0

        # ------------------------------- init obstacle ------------------------------ #
        self.obstacle_task = []
        self.obstacle_task_geom = []
            
        for _ in range(self.num_obstacle_task):
            obstacle = TaskObject3D(velocity=0.01, 
                                    keep_direction_step = 500,
                                    bound=np.array([[-0.3, 0.5], 
                                                    [-0.3, 0.5], 
                                                    [0.8, 1.0]]))
            obstacle.frame[:3,3] = np.array([np.random.uniform(low, high) for low, high in obstacle.bound])
            self.obstacle_task.append(obstacle)
            self.obstacle_task_geom.append(Geometry(type="sphere", radius=0.05, color=VizColor.obstacle_task))

        # --------------------------------- init goal -------------------------------- #
        self.goal_teleop = {}
        self.goal_teleop["left"] = np.array([[1., 0., 0., 0.25],
                                            [0., 1., 0., 0.25],
                                            [0., 0., 1., 0.1],
                                            [0., 0., 0., 1.]])
        self.goal_teleop["right"] = np.array([[1., 0., 0., 0.25],
                                            [0., 1., 0., -0.25],
                                            [0., 0., 1., 0.1],
                                            [0., 0., 0., 1.]])
        
        self.robot_goal_left = TaskObject3D(velocity=0.001, 
                                            keep_direction_step = 10,
                                            bound=np.array([[0.1, 0.4], 
                                                            [0.1, 0.4], 
                                                            [0.0, 0.2]]),
                                            smooth_weight = 0.8)
        self.robot_goal_left.frame[:3,3] = np.array([np.random.uniform(low, high) for low, high in self.robot_goal_left.bound])
        
        self.robot_goal_right = TaskObject3D(velocity=0.001, 
                                             keep_direction_step = 10,
                                             bound=np.array([[0.1, 0.4], 
                                                             [-0.4, -0.1], 
                                                             [0.0, 0.2]]),
                                             smooth_weight = 0.8)
        self.robot_goal_right.frame[:3,3] = np.array([np.random.uniform(low, high) for low, high in self.robot_goal_right.bound])
        
        
        
        # --------------------------------- init info -------------------------------- #
        self.info = {}
        self.info["goal_teleop"] = {}
        self.info["obstacle_task"] = {}
        self.info["obstacle_debug"] = {}
        self.info["obstacle"] = {}
        self.info["robot_frames"] = None
        self.info["robot_state"] = {}

    def update_robot_goal(self):
        self.robot_goal_left.move()
        self.robot_goal_right.move()
    
    def update_obstacle_task(self):
        for obstacle in self.obstacle_task:
            obstacle.move()
    
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