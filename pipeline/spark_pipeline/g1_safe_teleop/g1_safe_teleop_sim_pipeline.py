from spark_pipeline.base.base_pipeline import BasePipeline
from spark_utils import VizColor
import numpy as np

class G1SafeTeleopSimPipeline(BasePipeline):

    def __init__(self, cfg):
        super().__init__(cfg)

    def post_physics_step(self, agent_feedback, task_info, action_info):
        
        # retrieve data
        self.task_info = task_info
        self.agent_feedback = agent_feedback
        self.action_info = action_info

        # process data
        self.process_data()

        # render sim
        self.render()
        
        # publish robot state
        self.env.task.pub_robot_state()

    def viz_critical_env_pairs(self, mat, thres, mask, line_width, line_color):
        
        if mat is None:
            return []
        
        masked_mat = mat[mask]
        masked_indices = np.argwhere(mask)  # Get the row and column indices of masked positions
        indices_of_interest = masked_indices[np.argwhere(masked_mat >= thres).reshape(-1)]
        for i, j in indices_of_interest:
            # line connecting frame i and obstacle j
            self.env.agent.render_line_segment(
                pos1=self.robot_frames_world[i][:3,3], 
                pos2=self.task_info["obstacle"]["frames_world"][j][:3,3], 
                radius=line_width, 
                color=line_color)
        
        return indices_of_interest
    
    def viz_critical_self_pairs(self, mat, thres, mask, line_width, line_color):
        
        if mat is None:
            return []
        
        masked_mat = mat[mask]
        masked_indices = np.argwhere(mask)
        indices_of_interest = masked_indices[np.argwhere(masked_mat >= thres).reshape(-1)]
        for i, j in indices_of_interest:
            # line connecting frame i and frame j
            self.env.agent.render_line_segment(
                pos1=self.robot_frames_world[i][:3,3], 
                pos2=self.robot_frames_world[j][:3,3], 
                radius=line_width, 
                color=line_color)
        
        return indices_of_interest
    
    def process_data(self):
        
        self.goal_teleop = self.task_info["goal_teleop"]
        self.robot_base_frame = self.agent_feedback["robot_base_frame"]
        
        # compute transformations of robot collision volumes
        x = self.agent_feedback["state"]
        dof_pos = self.robot_cfg.decompose_state_to_dof(x)
        robot_frames = self.robot_kinematics.forward_kinematics(dof_pos)
        self.robot_frames_world = np.zeros_like(robot_frames)
        for i in range(len(robot_frames)):
            self.robot_frames_world[i, :, :] = self.robot_base_frame @ robot_frames[i, :, :]
    
    def render(self):

        # display goal in mujoco frame. transformation fixed for g1 geometry
        self.env.agent.render_sphere((self.robot_base_frame @ self.goal_teleop["left"])[:3,3], 0.05*np.ones(3), VizColor.goal)
        self.env.agent.render_sphere((self.robot_base_frame @ self.goal_teleop["right"])[:3,3], 0.05*np.ones(3), VizColor.goal)

        env_collision_mask = self.algo.safe_controller.safety_index.env_collision_mask
        self_collision_mask = self.algo.safe_controller.safety_index.self_collision_mask
        
        # display critical vol pairs (phi_hold)
        phi_hold_mat_env = self.action_info.get("phi_hold_mat_env", None)
        active_pairs_hold_env = self.viz_critical_env_pairs(phi_hold_mat_env, 0.0, env_collision_mask, 0.002, VizColor.hold)    
        
        phi_hold_mat_self = self.action_info.get("phi_hold_mat_self", None)
        active_pairs_hold_self = self.viz_critical_self_pairs(phi_hold_mat_self, 0.0, self_collision_mask, 0.002, VizColor.hold)
        
        # display critical vol pairs (phi_safe)
        phi_safe_mat_env = self.action_info.get("phi_safe_mat_env", None)
        active_pairs_unsafe_env = self.viz_critical_env_pairs(phi_safe_mat_env, 0.0, env_collision_mask, 0.02, VizColor.unsafe)
        
        phi_safe_mat_self = self.action_info.get("phi_safe_mat_self", None)
        active_pairs_unsafe_self = self.viz_critical_self_pairs(phi_safe_mat_self, 0.0, self_collision_mask, 0.02, VizColor.unsafe)
        
        # display slack vars
        slack_vars_mat_env = self.action_info.get("slack_vars_mat_env", None)
        active_pairs_slack_env = self.viz_critical_env_pairs(slack_vars_mat_env, 1e-8, env_collision_mask, 0.05, VizColor.slack_positive)
        
        slack_vars_mat_self = self.action_info.get("slack_vars_mat_self", None)
        active_pairs_slack_self = self.viz_critical_self_pairs(slack_vars_mat_self, 1e-8, self_collision_mask, 0.05, VizColor.slack_positive)
    
        # render collision volumes
        for frame_id, frame_world in enumerate(self.robot_frames_world):
            
            geom = self.robot_cfg.CollisionVol[self.robot_cfg.Frames(frame_id)]
            
            # highlight if triggering any phi
            if any(frame_id in pair for pair in active_pairs_unsafe_self) or any(frame_id == _frame_id for _frame_id, _ in active_pairs_unsafe_env):
                geom.color = VizColor.unsafe
            elif any(frame_id in pair for pair in active_pairs_hold_self) or any(frame_id == _frame_id for _frame_id, _ in active_pairs_hold_env):
                geom.color = VizColor.hold
            else:
                if frame_id in self.algo.safe_controller.safety_index.env_collision_vol_ignore:
                    geom.color = VizColor.collision_volume_ignored
                else:
                    geom.color = VizColor.collision_volume
            
            # todo generalize this to handle different geometry types
            if geom.type == "sphere":
                self.env.agent.render_sphere(frame_world[:3,3], geom.attributes["radius"]*np.ones(3), geom.color)
            else:
                raise ValueError(f'Unknown geometry type: {geom.type}')
        
        # render obstacles
        for obstacle_id_task, (frame_world, geom) in enumerate(zip(self.task_info["obstacle_task"]["frames_world"], self.task_info["obstacle_task"]["geom"])):
            
            # obs id is obs id in task
            obstacle_id = obstacle_id_task
        
            # highlight if triggering any phi
            # if any(obstacle_id == _obstacle_id for _, _obstacle_id in active_pairs_unsafe_env):
            #     geom.color = VizColor.unsafe
            # elif any(obstacle_id == _obstacle_id for _, _obstacle_id in active_pairs_hold_env):
            #     geom.color = VizColor.hold
            # else:
            #     geom.color = VizColor.obstacle_task
        
            # todo generalize this to handle different geometry types
            if geom.type == "sphere":
                self.env.agent.render_sphere(frame_world[:3,3], geom.attributes["radius"]*np.ones(3), geom.color)
            else:
                raise ValueError(f'Unknown geometry type: {geom.type}')
        
        # change debug obstacle color if in collision (agent will render them)
        for obstacle_id_debug, geom in enumerate(self.env.agent.obstacle_debug_geom):
            
            # obs id is obs id in debug + num_obstacle_task
            obstacle_id = obstacle_id_debug + self.env.task.num_obstacle_task
            
            # if any(obstacle_id == _obstacle_id for _, _obstacle_id in active_pairs_hold_env):
            #     geom.color = VizColor.hold
            # elif any(obstacle_id == _obstacle_id for _, _obstacle_id in active_pairs_unsafe_env):
            #     geom.color = VizColor.unsafe
            # else:
            #     geom.color = VizColor.obstacle_debug
        
        # call agent render
        self.env.agent.render()
        
        
        