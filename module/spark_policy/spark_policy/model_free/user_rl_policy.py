from spark_policy.base.base_policy import BasePolicy
from spark_agent import BaseAgent
from spark_robot import RobotKinematics, RobotConfig
import numpy as np

class UserRLPolicy(BasePolicy):
    def __init__(self, **kwargs) -> None:
        super().__init__(**kwargs)

    def act(self, agent_feedback: dict, task_info: dict):
        '''
        User should implement their own policy here to get the control signal.
        '''
        dof_control = None
        info = None

        return dof_control, info