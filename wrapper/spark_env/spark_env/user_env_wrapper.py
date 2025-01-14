from .spark_env_config import SparkEnvConfig
from .spark_env_wrapper import SparkEnvWrapper
from spark_robot import RobotConfig, RobotKinematics
from spark_agent import BaseAgent
from spark_task import BaseTask
from spark_utils import initialize_class
        
class UserEnvWrapper(SparkEnvWrapper):
    
    def __init__(self, cfg: SparkEnvConfig, robot_cfg: RobotConfig, robot_kinematics: RobotKinematics):
        self.cfg = cfg
        
        self.agent : BaseAgent = initialize_class(self.cfg.agent, robot_cfg=robot_cfg)
        
        self.task : BaseTask = initialize_class(self.cfg.task,
                                                robot_cfg=robot_cfg,
                                                robot_kinematics=robot_kinematics,
                                                agent=self.agent)
    
    def reset(self):
        '''
        User need to implement this function to reset the environment
        And return the initial agent feedback and task info
        '''
        # get agent feedback
        agent_feedback = None
        
        # process agent feedback, receive additional perception, generate goal
        task_info = None
        
        return agent_feedback, task_info
    
    def step(self, u_safe):
        '''
        User need to implement this function to step the environment
        And return the agent feedback and task info
        '''

        # get agent feedback
        agent_feedback = None
        
        # get task info
        task_info = None
        
        return agent_feedback, task_info
