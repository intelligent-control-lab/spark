from spark_utils import initialize_class
from abc import ABC, abstractmethod
from .base_pipeline_config import BasePipelineConfig
from spark_robot import RobotConfig, RobotKinematics
from spark_algo import SparkAlgoWrapper
from spark_env import SparkEnvWrapper
import numpy as np
import time

class BasePipeline(ABC):
    
    def __init__(self, cfg : BasePipelineConfig):

        self.cfg = cfg
        self.max_num_steps = self.cfg.max_num_steps
        
        self.robot_cfg : RobotConfig = initialize_class(self.cfg.robot.cfg)
        self.robot_kinematics : RobotKinematics = initialize_class(self.cfg.robot.kinematics, robot_cfg=self.robot_cfg)
        self.env : SparkEnvWrapper = SparkEnvWrapper(self.cfg.env, robot_cfg=self.robot_cfg, robot_kinematics=self.robot_kinematics)
        self.algo : SparkAlgoWrapper = SparkAlgoWrapper(self.cfg.algo, robot_cfg=self.robot_cfg, robot_kinematics=self.robot_kinematics)
        
        self.pipeline_step = 0  

    @abstractmethod
    def post_physics_step(self):
        pass

    def run(self):
        
        # reset environment
        agent_feedback, task_info = self.env.reset()
        
        # initial action
        u_safe, action_info = self.algo.act(agent_feedback, task_info)
        
        while self.pipeline_step < self.max_num_steps or self.max_num_steps < 0:
            
            # environment step
            # s_next = env(s, a)
            agent_feedback, task_info = self.env.step(u_safe)
            
            # next action
            # a_next = algo(s_next)
            u_safe, action_info = self.algo.act(agent_feedback, task_info)
            
            # post physics step (e.g., rendering, status publishing)
            self.post_physics_step(agent_feedback, task_info, action_info)
            
            self.pipeline_step += 1
            
            if task_info["done"]:
                break
        
        # end of pipeline
        print("Pipeline done")
        


            