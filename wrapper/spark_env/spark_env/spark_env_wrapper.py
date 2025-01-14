from .spark_env_config import SparkEnvConfig
from spark_robot import RobotConfig, RobotKinematics
from spark_agent import BaseAgent
from spark_task import BaseTask
from spark_utils import initialize_class
        
class SparkEnvWrapper:
    
    def __init__(self, cfg: SparkEnvConfig, robot_cfg: RobotConfig, robot_kinematics: RobotKinematics):
        self.cfg = cfg
        
        self.agent : BaseAgent = initialize_class(self.cfg.agent, robot_cfg=robot_cfg)
        
        self.task : BaseTask = initialize_class(self.cfg.task,
                                                robot_cfg=robot_cfg,
                                                robot_kinematics=robot_kinematics,
                                                agent=self.agent)
    
    def reset(self):
        
        self.agent.reset()
        self.task.reset()
        
        # get agent feedback
        agent_feedback = self.agent.get_feedback()
        
        # process agent feedback, receive additional perception, generate goal
        task_info = self.task.get_info(agent_feedback)
        
        return agent_feedback, task_info
    
    def step(self, u_safe):
        
        # execute action
        self.agent.step(u_safe)

        # get agent feedback
        agent_feedback = self.agent.get_feedback()
        
        # execute task step (process agent feedback, receive additional perception, generate goal)
        self.task.step(agent_feedback)
        
        # get task info
        task_info = self.task.get_info(agent_feedback)
        
        return agent_feedback, task_info
