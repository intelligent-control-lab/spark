from spark_utils import initialize_class

from .spark_algo_config import SparkAlgoConfig
from spark_algo import SparkAlgoWrapper
from spark_robot import RobotConfig, RobotKinematics
class WBCAlgoWrapper(SparkAlgoWrapper):
    
    def __init__(self, cfg : SparkAlgoConfig, robot_cfg : RobotConfig, robot_kinematics : RobotKinematics):
               
        self.cfg = cfg
        self.robot_cfg = robot_cfg
        
        self.goal_tracking_policy = initialize_class(self.cfg.goal_tracking_policy, 
                                        robot_cfg=robot_cfg,
                                        robot_kinematics=robot_kinematics)
        
        self.wbc_policy = initialize_class(self.cfg.wbc_policy, 
                                        robot_cfg=robot_cfg,
                                        robot_kinematics=robot_kinematics)

        self.safe_controller = initialize_class(
            self.cfg.safe_controller,
            robot_cfg = robot_cfg,
            robot_kinematics = robot_kinematics
        )
        
        
        
    def act(self, agent_feedback, task_info):
        u_ref, action_info = self.goal_tracking_policy.act(
            agent_feedback = agent_feedback,
            task_info = task_info
        )
        
        task_info["ref_ddq"] = u_ref
        u_ref, action_info = self.wbc_policy.act(
            agent_feedback = agent_feedback,
            task_info = {**action_info, **task_info}
        )
        u_safe = u_ref
        safe_control_info = {}
        
        action_info['u_ref'] = u_ref
        action_info['u_safe'] = u_safe
        return u_safe, {**action_info, **safe_control_info}
    
