from spark_pipeline.base.base_pipeline import BasePipeline
from spark_utils import Logger, VizColor, DataBuffer, compute_masked_distance_matrix
import numpy as np
import os
from spark_pipeline import SPARK_PIPELINE_ROOT
from collections import OrderedDict
from spark_pipeline.visualization import render_value_based_debug_info
from spark_pipeline import BenchmarkPipelineConfig
import time
import matplotlib.pyplot as plt
import subprocess

class BenchmarkPipeline(BasePipeline[BenchmarkPipelineConfig]):

    def __init__(self, cfg: BenchmarkPipelineConfig):
        
        super().__init__(cfg)
        self.max_num_reset = self.cfg.max_num_reset
        self.logger = None
        self.counter = 0
        self.initialize_log_buffer()

    def post_physics_step(self, agent_feedback, task_info, action_info):
        
        # retrieve data
        self.task_info = task_info
        self.agent_feedback = agent_feedback
        self.action_info = action_info

        # common data processing and logging
        self.process_and_save_data()
            
        # logger  
        self.log(agent_feedback, task_info, action_info)
        # render simulation
        self.render()
    
    def run(self, save_path=None):
        
        self.setup_logging(save_path)
       
        # reset environment
        agent_feedback, task_info = self.env.reset()
        self.num_reset = 1
        
        # initial action
        for _ in range(10): # ! temporary fix of failure to set seed
            u_safe, action_info = self.algo.act(agent_feedback, task_info)
        
        time_start = time.time()
        for self.pipeline_step in range(self.max_num_steps):
            start_t = time.time()
            # reset if necessary
            if task_info["done"]:
                
                if self.max_num_reset != -1 and self.num_reset >= self.max_num_reset:
                    break
                
                agent_feedback, task_info = self.env.reset()
                self.num_reset += 1
                
                u_safe, action_info = self.algo.act(agent_feedback, task_info)

            # environment step
            # s_next = env(s, a)
            agent_feedback, task_info = self.env.step(u_safe, action_info)

            # next action
            # a_next = algo(s_next)
            u_safe, action_info = self.algo.act(agent_feedback, task_info)
            
            # render s_next, a_next
            end_time = time.time()
            self.post_physics_step(agent_feedback, task_info, action_info)
    
            if time.time() - start_t < self.cfg.env.agent.dt:
                time.sleep(self.cfg.env.agent.dt - (end_time - start_t))
            
            self.loop_time = time.time() - start_t
            
            # print(f"Loop time: {loop_time}")
            # print(f"Frequencies: {1.0 / self.loop_time: .3f} Hz")
        
    
        # end of benchmark
        time_finish = time.time()
        print("Benchmark finished in ", time_finish - time_start, " seconds.")
        self.save_results()
        self.env.agent.close_viewer()
        time.sleep(1)
    
    # ---------------------------------- helpers --------------------------------- #
    
    def setup_logging(self, save_path=None):
        '''
            Save logger to {save_path}/log (clear if exists)
            Save data to {save_path}/{data}
        '''

        self.save_path = save_path
        
        if self.save_path is None:
            return

        if not (os.path.exists(self.save_path)):
            os.makedirs(self.save_path)
            
        print("\nSAVING TO: ", self.save_path, "\n")
        self.logdir = os.path.join(self.save_path, 'log')
        # logger
        if self.cfg.enable_logger:
            
            if os.path.exists(self.logdir):
                import shutil
                shutil.rmtree(self.logdir)
            os.makedirs(self.logdir)
            self.logger = Logger(self.logdir)
            print("\nLOGGING TO: ", self.logdir, "\n")
            if self.cfg.enable_plotter:
                tags_to_plot = []
                for frame in self.robot_cfg.VisualizePhiTraj:
                    tags_to_plot.append(f"phi_env_{frame.name}")
                    # tags_to_plot.append(f"phi0_env_{frame.name}")
                    # tags_to_plot.append(f"phi0dot_env_{frame.name}")
                    tags_to_plot.append(f"traj_env_{frame.name}")
                plotter_path = os.path.join(SPARK_PIPELINE_ROOT, "spark_pipeline/plotter.py")
                self.plotter_process = subprocess.Popen(["python3", plotter_path, self.logdir] + tags_to_plot) 
      
            
        else:
            self.logger = None
            print("\nLOGGING DISABLED\n")
    
    def initialize_log_buffer(self):
        
        self.data_buffer = DataBuffer()
        
        for metric_name, flag in self.cfg.metric_selection.__dict__.items():
            if flag:
                self.data_buffer.add(metric_name)
    
    def render(self):
        if self.cfg.env.agent.enable_viewer:
            render_value_based_debug_info(
                render_sphere_fn    = self.env.agent.render_sphere,
                render_line_fn      = self.env.agent.render_line_segment,
                render_box_fn       = self.env.agent.render_box,
                render_surface_fn   = self.env.agent.render_surface,
                render_coordinate_frame_fn = self.env.agent.render_coordinate_frame,
                agent_feedback      = self.agent_feedback,
                task_info           = self.task_info,
                action_info         = self.action_info,
                safety_index        = self.algo.safe_controller.safety_index,
                robot_frames        = self.robot_frames_world,
                dist_robot_to_env   = self.dist_robot_to_env,
                robot_cfg           = self.robot_cfg,
                enable_safe_zone_render    = self.cfg.enable_safe_zone_render,
            )
            
            # change debug obstacle color if in collision (agent will render them)
            for obstacle_id_debug, geom in enumerate(self.env.agent.obstacle_debug_geom):
                
                # obs id is obs id in debug + num_obstacle_task
                obstacle_id = obstacle_id_debug + self.env.task.num_obstacle_task
                
                # highlight if have collision
                if np.min(self.dist_robot_to_env[:, obstacle_id]) <= 0:
                    geom.color = VizColor.collision
                else:
                    geom.color = VizColor.obstacle_debug
            
            # call agent render
            self.env.agent.render()
    
    def process_and_save_data(self):
        
        self.goal_teleop = self.task_info.get("goal_teleop", None)
        self.goal_left_frame_world = self.goal_teleop.get("left", None) if self.goal_teleop is not None else None
        self.goal_right_frame_world = self.goal_teleop.get("right", None) if self.goal_teleop is not None else None
        self.goal_base_frame_world = self.goal_teleop.get("base", None) if self.goal_teleop is not None else None
        
        self.robot_base_frame = self.agent_feedback["robot_base_frame"]
        
        # compute transformations of robot collision volumes
        x = self.agent_feedback["state"]
        dof_pos = self.robot_cfg.decompose_state_to_dof_pos(x)
        robot_frames = self.robot_kinematics.forward_kinematics(dof_pos)
        self.robot_frames_world = np.zeros_like(robot_frames)
        for i in range(len(robot_frames)):
            self.robot_frames_world[i, :, :] = self.robot_base_frame @ robot_frames[i, :, :]
        
        # ------------------------------ self collision ------------------------------ #
        
        self.dist_self, self.index_min_dist_self = compute_masked_distance_matrix(
            frame_list_1=self.robot_frames_world,
            geom_list_1=self.robot_cfg.CollisionVol.values(),
            frame_list_2=self.robot_frames_world,
            geom_list_2=self.robot_cfg.CollisionVol.values(),
            mask=self.algo.safe_controller.safety_index.self_collision_mask
        )
        if self.cfg.metric_selection.dist_self:
            self.data_buffer.add("dist_self", self.dist_self)
          
        # ------------------------------- env collision ------------------------------ #
        
        self.dist_robot_to_env, self.index_min_dist_env = compute_masked_distance_matrix(
            frame_list_1=self.robot_frames_world,
            geom_list_1=self.robot_cfg.CollisionVol.values(),
            frame_list_2=self.task_info["obstacle"]["frames_world"],
            geom_list_2=self.task_info["obstacle"]["geom"],
            mask=self.algo.safe_controller.safety_index.env_collision_mask
        )
        if self.cfg.metric_selection.dist_robot_to_env:
            self.data_buffer.add("dist_robot_to_env", self.dist_robot_to_env)
        
        # --------------------------------- arm goal --------------------------------- #
        if self.goal_left_frame_world is not None:
            self.dist_goal_left = np.linalg.norm(self.robot_frames_world[self.robot_cfg.Frames.L_ee,:3,3] - self.goal_left_frame_world[0, :3,3])
        else:
            self.dist_goal_left = None
            
        if self.goal_right_frame_world is not None:
            self.dist_goal_right = np.linalg.norm(self.robot_frames_world[self.robot_cfg.Frames.R_ee,:3,3] - self.goal_right_frame_world[0, :3,3])
        else:
            self.dist_goal_right = None
        
        if self.cfg.metric_selection.dist_goal_arm:
            dist_arm_goal_list = []
            if self.dist_goal_left is not None:
                dist_arm_goal_list.append(self.dist_goal_left)
            if self.dist_goal_right is not None:
                dist_arm_goal_list.append(self.dist_goal_right)
            if len(dist_arm_goal_list) > 0:
                self.data_buffer.add("dist_goal_arm", np.array([self.dist_goal_left, self.dist_goal_right]))
            else:
                raise ValueError("Arm goal not set while dist_goal_arm is required.")
        
        # --------------------------------- base goal -------------------------------- #
        
        if self.goal_base_frame_world is not None:
            self.dist_goal_base = np.linalg.norm(self.robot_base_frame[:3,3] - self.goal_base_frame_world[:3,3])
        else:
            self.dist_goal_base = None
            
        if self.cfg.metric_selection.dist_goal_base:
            if self.dist_goal_base is not None:
                self.data_buffer.add("dist_goal_base", self.dist_goal_base)
            else:
                raise ValueError("Base goal not set while dist_goal_base is required.")
        
        # ----------------------------- safety violations ---------------------------- #

        if self.cfg.metric_selection.violation:
            
            if "violation_mat_env" in self.action_info:
                mat = self.action_info["violation_mat_env"]
                mat[~self.algo.safe_controller.safety_index.env_collision_mask] = 0.0
                self.data_buffer.add("violation_env", mat)
                
            if "violation_mat_self" in self.action_info:
                mat = self.action_info["violation_mat_self"]
                mat[~self.algo.safe_controller.safety_index.self_collision_mask] = 0.0
                self.data_buffer.add("violation_self", mat)

        # ------------------------------- other metrics ------------------------------ #
        if self.cfg.metric_selection.seed:
            self.data_buffer.add("seed", self.task_info["seed"])
            
        if self.cfg.metric_selection.done:
            self.data_buffer.add("done", self.task_info["done"])
        
    def log(self, agent_feedback, task_info, action_info):
        
        if self.logger is None:
            return
        
        logs = OrderedDict()
        
        logs['SSA/phi_safe'] = self.action_info["trigger_safe"]
  
        phi_safe_mat_self = action_info.get("phi_safe_mat_self", None)
        phi_safe_mat_env = action_info.get("phi_safe_mat_env", None)
        phi0_mat_env = action_info.get("phi0_mat_env", None)
        phi0dot_mat_env = action_info.get("phi0dot_mat_env", None)
        
        for frame in self.robot_cfg.VisualizePhiTraj:
            largest_phi_idx = np.argmax(phi_safe_mat_env[frame])
            logs[f"phi_env_{frame.name}"] = phi_safe_mat_env[frame, largest_phi_idx]
            logs[f"phi0_env_{frame.name}"] = phi0_mat_env[frame, largest_phi_idx]
            logs[f"phi0dot_env_{frame.name}"] = phi0dot_mat_env[frame, largest_phi_idx]
            logs[f"phi_k_env_{frame.name}"] = self.algo.safe_controller.safety_index.k
        
  
        # perform the logging
        for key, value in logs.items():
            if isinstance(value, dict):
                self.logger.log_scalars(value, key)
            else:
                self.logger.log_scalar(value, key)
        self.logger.flush() # internal counter +1, force buffered data in
        
    def save_results(self):
        
        if self.save_path is None:
            print("No save path specified. Not saving data.")
            return

        # add final data
        self.data_buffer.add("env_collision_mask", self.algo.safe_controller.safety_index.env_collision_mask)
        self.data_buffer.add("self_collision_mask", self.algo.safe_controller.safety_index.self_collision_mask)

        # save data
        self.data_buffer.save_npz(os.path.join(self.save_path, 'data.npz'))
