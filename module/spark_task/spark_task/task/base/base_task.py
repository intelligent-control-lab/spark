from .base_task_config import BaseTaskConfig
from spark_utils import class_to_dict
from spark_agent import BaseAgent
from spark_robot import RobotConfig, RobotKinematics
from abc import ABC, abstractmethod

def initialize_class(class_cfg, **kwargs):
    '''
    Initialize a class from a configuration object.
    Also pass any additional keyword arguments to the class constructor
    '''
    
    class_cfg_dict = class_to_dict(class_cfg)
    class_name = class_cfg_dict.pop('class_name', None)
    
    if class_name is None:
        raise ValueError(f'class_name not found in {class_cfg}')
    
    import spark_agent
    for module in [spark_agent]:
        
        if hasattr(module, class_name):
            class_name = getattr(module, class_name)

            return class_name(**{ **class_cfg_dict, **kwargs })

class BaseTask(ABC):
    
    def __init__(self, robot_cfg: RobotKinematics, robot_kinematics: RobotKinematics, agent : BaseAgent):
                
        self.robot_cfg : RobotConfig = robot_cfg
        self.robot_kinematics : RobotKinematics = robot_kinematics
        self.agent : BaseAgent = agent
        self.num_obstacle_agent = self.agent.num_obstacle_agent
        
    @abstractmethod
    def reset(self):
        '''
            Reset the task environment.
        '''
        pass
    
    @abstractmethod
    def step(self, feedback):
        '''
            Execute a step in the task environment.
        '''
        pass
        
    @abstractmethod
    def get_info(self, feedback) -> dict:
        """
        Get the state from the agent and the environment.
        Output the task information to the policy in the form of a dictionary.
        """
        pass