# from spark_task.task.g1_teleop_sim.g1_teleop_sim_task_config import G1TeleopSimTaskConfig
from spark_task.task.base.base_task import BaseTask
from spark_utils import Geometry
from spark_utils import pos_quat_to_transformation
import numpy as np

class G1TeleopSimTask(BaseTask):
    def __init__(self, robot_cfg, robot_kinematics, agent, **kwargs):
        super().__init__(robot_cfg, robot_kinematics, agent)
        
        self.task_name = kwargs.get("task_name", "G1TeleopSimTask")
        self.max_episode_length = kwargs.get("max_episode_length", -1)
        self.num_obstacle_task = 3 # todo online update
        
        # ------------------------------------ ROS ----------------------------------- #
        self.enable_ros = kwargs.get("enable_ros", False)
        
        if self.enable_ros:

            import rospy
            from std_msgs.msg import Float64MultiArray

            # data buf
            self.robot_command = Float64MultiArray()
            self.robot_state = Float64MultiArray()

            self.ros_args = kwargs["ros_params"]
            self.rospy = rospy

            # publisher
            self.robot_command_pub =  self.rospy.Publisher(self.ros_args["robot_command_topic"], Float64MultiArray, queue_size=17)
            self.robot_state_pub =  self.rospy.Publisher(self.ros_args["robot_state_topic"], Float64MultiArray, queue_size=17)
            
            # subscriber
            self.robot_teleop_sub = self.rospy.Subscriber(self.ros_args["robot_teleop_topic"], Float64MultiArray, self._teleop_goal_callback)
            self.obstacle_sub = self.rospy.Subscriber(self.ros_args["obstacle_topic"], Float64MultiArray, self._obstacle_state_callback)

            # launch ros node
            self.rospy.init_node(self.task_name, anonymous=True)
            
        self.reset()
    
    def reset(self):
        
        self.episode_length = 0
        
        # ------------------------------- init obstacle ------------------------------ #
        self.obstacle_task_geom = [Geometry(type="sphere", radius=0.05) for _ in range(self.num_obstacle_task)]
        self.obstacle_task_frames_world = [np.eye(4) for _ in range(self.num_obstacle_task)]
        for obstacle in self.obstacle_task_frames_world:
            obstacle[:3,3] = np.random.uniform(1.0, 1.5, 3)

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
        
        # --------------------------------- init info -------------------------------- #
        self.info = {}
        self.info["goal_teleop"] = {}
        self.info["obstacle_task"] = {}
        self.info["obstacle_debug"] = {}
        self.info["obstacle"] = {}
        self.info["robot_frames"] = None
        self.info["robot_state"] = {}
    
    def step(self, feedback):
        
        self.episode_length += 1
    
    def get_info(self, feedback) -> dict:

        self.info["done"] = False

        if self.enable_ros and self.rospy.is_shutdown():
            self.info["done"] = True
        
        if self.max_episode_length >= 0 and self.episode_length >= self.max_episode_length:
            self.info["done"] = True

        # robot base is available in feedback in sim
        self.info["robot_base_frame"]               = feedback["robot_base_frame"]
        
        self.info["goal_teleop"]["left"]            = self.goal_teleop["left"]
        self.info["goal_teleop"]["right"]           = self.goal_teleop["right"]
        
        # get obstacles from task and debug
        self.info["obstacle_task"]["frames_world"]  = self.obstacle_task_frames_world
        self.info["obstacle_task"]["geom"]          = self.obstacle_task_geom
        self.info["obstacle_debug"]["frames_world"] = feedback.get("obstacle_debug_frame", [])
        self.info["obstacle_debug"]["geom"]         = feedback.get("obstacle_debug_geom", [])
        
        # merge obstacles from task and debug
        self.info["obstacle"]["frames_world"]       = np.concatenate([self.info["obstacle_task"]["frames_world"], self.info["obstacle_debug"]["frames_world"]], axis=0)
        self.info["obstacle"]["geom"]               = np.concatenate([self.info["obstacle_task"]["geom"], self.info["obstacle_debug"]["geom"]], axis=0)
        self.info["obstacle"]["num"]                = len(self.info["obstacle"]["frames_world"])
        
        # robot state
        self.info["robot_state"]["dof_pos_cmd"]     = feedback["dof_pos_cmd"]
        self.info["robot_state"]["dof_pos_fbk"]     = feedback["dof_pos_fbk"]
        self.info["robot_state"]["dof_vel_cmd"]     = feedback["dof_vel_cmd"]
        
        return self.info
    
    # ---------------------------------- helpers --------------------------------- #
    
    def _teleop_goal_callback(self, data):
        self.goal_teleop["left"] = np.array(data.data[:16]).reshape(4,4)
        self.goal_teleop["right"] = np.array(data.data[16:]).reshape(4,4)
        
    def _obstacle_state_callback(self, data):
        for i in range(self.num_obstacle_task):
            self.obstacle_task_frames_world[i] = pos_quat_to_transformation(np.array(data.data[i*7+1: i*7 + 4]), 
                                                                        np.array([0.0, 0.0, 0.0, 1.0]))
            # todo: update geom as well
    
    def pub_robot_command(self, command):
        if self.enable_ros:
            self.robot_command.data = command
            self.robot_command_pub.publish(self.robot_command)
        
    def pub_robot_state(self):
        if self.enable_ros:
            self.robot_state.data = self.agent.get_feedback()["dof_pos_cmd"]
            self.robot_state_pub.publish(self.robot_state)
            
            
if __name__ == "__main__":
    pass