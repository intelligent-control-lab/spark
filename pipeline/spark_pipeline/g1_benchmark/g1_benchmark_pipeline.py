from spark_pipeline.base.base_pipeline import BasePipeline
from spark_utils import compute_pairwise_dist, Logger, VizColor
import numpy as np
import os
from collections import OrderedDict

class G1BenchmarkPipeline(BasePipeline):

    def __init__(self, cfg):
        super().__init__(cfg)
        
        # ------------------------------- setup logger ------------------------------- #
        # todo from config
        exp_name = "g1_benchmark"
        logdir_prefix = 'debug'
        data_path = os.path.join(os.path.dirname(os.path.realpath(__file__)), '../log')
        if not (os.path.exists(data_path)):
            os.makedirs(data_path)
        logdir = logdir_prefix + '_' + exp_name
        # + '_'  + time.strftime("%d-%m-%Y_%H-%M-%S")
        logdir = os.path.join(data_path, logdir)
        if os.path.exists(logdir):
            import shutil
            shutil.rmtree(logdir)
        os.makedirs(logdir)
        print("\n\n\nLOGGING TO: ", logdir, "\n\n\n")
        self.logger = Logger(logdir)
        
        self.initialize_buffer()

    def post_physics_step(self, agent_feedback, task_info, action_info):
        
        # retrieve data
        self.task_info = task_info
        self.agent_feedback = agent_feedback
        self.action_info = action_info
        
        # common data processing
        self.process_data()
        
        # benchmark evaluation
        self.evaluate()
        
        # logger
        self.log()
        
        # render sim
        self.render()
    
    def run(self):
        
        # reset environment
        agent_feedback, task_info = self.env.reset()
        
        # initial action
        u_safe, action_info = self.algo.act(agent_feedback, task_info)
        
        for self.pipeline_step in range(self.max_num_steps):
            
            # reset if necessary
            if task_info["done"]:
                agent_feedback, task_info = self.env.reset()
                u_safe, action_info = self.algo.act(agent_feedback, task_info)
            
            # environment step
            # s_next = env(s, a)
            agent_feedback, task_info = self.env.step(u_safe)
            
            # next action
            # a_next = algo(s_next)
            u_safe, action_info = self.algo.act(agent_feedback, task_info)
            
            # post physics step (e.g., rendering, status publishing)
            self.post_physics_step(agent_feedback, task_info, action_info)
        
        # end of benchmark
        
        print("Simulation ended")
        
        self.min_dist_robot_to_env = np.array(self.min_dist_robot_to_env)
        self.mean_dist_goal = np.array(self.mean_dist_goal)
        
        print("average distance to obstacle: ", np.mean(self.min_dist_robot_to_env))
        print("minimum distance to obstacle: ", np.min(self.min_dist_robot_to_env))
        print("average distance to goal: ", np.mean(self.mean_dist_goal))
        print("maximum distance to goal: ", np.max(self.mean_dist_goal))
    
    # ---------------------------------- helpers --------------------------------- #
    
    def initialize_buffer(self):
        
        self.min_dist_robot_to_env = []
        self.mean_dist_goal = []
    
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
    
    def process_data(self):
        
        self.goal_teleop = self.task_info["goal_teleop"]
        self.robot_base_frame = self.agent_feedback["robot_base_frame"]
        
        # compute transformations of robot collision volumes
        x = self.agent_feedback["state"]
        dof_pos = self.robot_cfg.decompose_state_to_dof(x)
        robot_frame = self.robot_kinematics.forward_kinematics(dof_pos)
        self.robot_frames_world = np.zeros_like(robot_frame)
        for i in range(len(robot_frame)):
            self.robot_frames_world[i, :, :] = self.robot_base_frame @ robot_frame[i, :, :]
        
        # ------------------------- self collision processing ------------------------ #
        
        # compute pairwise distances for self collision
        self.dist_self = compute_pairwise_dist(
            frame_list_1 = self.robot_frames_world,
            geom_list_1  = self.robot_cfg.CollisionVol.values(),
            frame_list_2 = self.robot_frames_world,
            geom_list_2  = self.robot_cfg.CollisionVol.values()
        )
        mask_dist_self = self.algo.safe_controller.safety_index.self_collision_mask
        
        # Row and column indices of the minimum value in masked dist_self
        dist_self_masked = self.dist_self[mask_dist_self]
        indices_self_masked = np.argwhere(mask_dist_self)  # Get the row and column indices of unmasked positions
        self.index_min_dist_self = indices_self_masked[np.argmin(dist_self_masked).reshape(-1)]
          
        # ------------------------- env collision processing ------------------------ #
        
        # compute pairwise distances between collision volumes and obstacles
        self.dist_robot_to_env = compute_pairwise_dist(
            frame_list_1 = self.robot_frames_world,
            geom_list_1  = self.robot_cfg.CollisionVol.values(),
            frame_list_2 = self.task_info["obstacle"]["frames_world"],
            geom_list_2  = self.task_info["obstacle"]["geom"]
        )
        mask_dist_env = self.algo.safe_controller.safety_index.env_collision_mask
        
        # Row and column indices of the minimum value in masked dist_env
        dist_env_masked = self.dist_robot_to_env[mask_dist_env]
        masked_indices_env = np.argwhere(mask_dist_env)  # Get the row and column indices of unmasked positions
        self.index_min_dist_env = masked_indices_env[np.argmin(dist_env_masked).reshape(-1)]
        
        # ------------------------------ goal processing ----------------------------- #
        
        self.dist_goal_left = np.linalg.norm(self.robot_frames_world[self.robot_cfg.Frames.L_ee,:3,3] - (self.robot_base_frame @ self.goal_teleop["left"])[:3,3])
        self.dist_goal_right = np.linalg.norm(self.robot_frames_world[self.robot_cfg.Frames.R_ee,:3,3] - (self.robot_base_frame @ self.goal_teleop["right"])[:3,3])
    
    def evaluate(self):
            
        self.min_dist_robot_to_env.append(np.min(self.dist_robot_to_env))
        self.mean_dist_goal.append((self.dist_goal_left + self.dist_goal_right) / 2)
        
    def log(self):
        
        logs = OrderedDict()
        
        logs['SSA/phi_safe'] = self.action_info["trigger_safe"]
            
        # perform the logging
        for key, value in logs.items():
            if isinstance(value, dict):
                self.logger.log_scalars(value, key)
            else:
                self.logger.log_scalar(value, key)
        self.logger.flush() # internal counter +1, force buffered data in
        
    