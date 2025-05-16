from spark_pipeline.base.base_pipeline import BasePipeline
from spark_utils import VizColor, compute_phi_and_extract_best
import numpy as np
from spark_safe.safe_algo import BaseSafetyIndex
from spark_robot import RobotConfig, RobotKinematics
import matplotlib.pyplot as plt
from collections import deque
from skimage import measure

def render_critical_pairs(line_render_fn, frame_list_1, frame_list_2, mat, thres, mask, line_width, line_color):
    '''
    line_render_fn: function to render line segment
    frame_list_1: list of frames
    frame_list_2: list of frames
    mat: pairwise distance matrix
    
    connect frame_list_1[i] to frame_list_2[j] if mat[i,j] >= thres (or isnan if thres is None) and mask[i,j] == True
    '''
    if mat is None:
        return []
    
    masked_mat = mat[mask]
    masked_indices = np.argwhere(mask)  # Get the row and column indices of masked positions
    if thres is not None:
        indices_of_interest = masked_indices[np.argwhere(masked_mat >= thres).reshape(-1)]
    else:
        indices_of_interest = masked_indices[np.argwhere(np.isnan(masked_mat)).reshape(-1)]
    for i, j in indices_of_interest:
        # line connecting frame i and obstacle j
        line_render_fn(
            pos1=frame_list_1[i][:3,3], 
            pos2=frame_list_2[j][:3,3], 
            radius=line_width, 
            color=line_color)
    
    return indices_of_interest

def render_value_based_debug_info(
    render_sphere_fn, render_line_fn, render_box_fn, render_surface_fn,
    agent_feedback, task_info, action_info,
    safety_index: BaseSafetyIndex,
    robot_frames, 
    dist_robot_to_env,
    robot_cfg: RobotConfig,
    enable_safe_zone_render=False
    ):

    robot_base_frame = agent_feedback["robot_base_frame"]

    # ----------------------------------- goal ----------------------------------- #
    goal_list = []
    goal_sizes = []

    goal_teleop = task_info.get("goal_teleop", None)
    goal_left_frame_base = goal_teleop.get("left", None) if goal_teleop is not None else None
    goal_right_frame_base = goal_teleop.get("right", None) if goal_teleop is not None else None
    goal_base_frame_world = goal_teleop.get("base", None) if goal_teleop is not None else None

    if goal_base_frame_world is not None:
        goal_list.append(goal_base_frame_world)
        goal_sizes.append(task_info.get("base_goal_size", 0.05))
    
    if goal_left_frame_base is not None and goal_right_frame_base is not None:
        goal_left_frame_world = goal_left_frame_base
        goal_right_frame_world = goal_right_frame_base
        goal_list += [goal_left_frame_world, goal_right_frame_world]
        goal_sizes += [task_info.get("arm_goal_size", 0.05), task_info.get("arm_goal_size", 0.05)]

    for goal, goal_size in zip(goal_list, goal_sizes):
        if goal is not None and goal_size is not None:
            render_sphere_fn(goal[:3,3], goal[:3,:3], goal_size*np.ones(3), VizColor.goal)
            
    if "zmp" in action_info.keys():
        zmp = action_info["zmp"]
        if zmp is not None:
            render_sphere_fn(zmp, np.eye(3), 0.02*np.ones(3), VizColor.unsafe)
                

    # ---------------------------- safety constraints ---------------------------- #

    obstacle_frames = task_info["obstacle"]["frames_world"]
    obstacle_geoms = task_info["obstacle"]["geom"]

    # get collision constraint masks
    env_collision_mask = safety_index.env_collision_mask
    self_collision_mask = safety_index.self_collision_mask

    # display critical vol pairs (phi_hold)
    phi_hold_mat_env = action_info.get("phi_hold_mat_env", None)
    active_pairs_hold_env = render_critical_pairs(render_line_fn, robot_frames, obstacle_frames, phi_hold_mat_env, 0.0, env_collision_mask, 0.002, VizColor.hold)    
    
    phi_hold_mat_self = action_info.get("phi_hold_mat_self", None)
    active_pairs_hold_self = render_critical_pairs(render_line_fn, robot_frames, robot_frames, phi_hold_mat_self, 0.0, self_collision_mask, 0.002, VizColor.hold)
    
    # display critical vol pairs (phi_safe)
    phi_safe_mat_env = action_info.get("phi_safe_mat_env", None)
    active_pairs_unsafe_env = render_critical_pairs(render_line_fn, robot_frames, obstacle_frames, phi_safe_mat_env, 0.0, env_collision_mask, 0.01, VizColor.unsafe)
    
    phi_safe_mat_self = action_info.get("phi_safe_mat_self", None)
    active_pairs_unsafe_self = render_critical_pairs(render_line_fn, robot_frames, robot_frames, phi_safe_mat_self, 0.0, self_collision_mask, 0.01, VizColor.unsafe)
    
    # display slack vars
    
    violation_mat_env = action_info.get("violation_mat_env", None)
    active_pairs_slack_env = render_critical_pairs(render_line_fn, robot_frames, obstacle_frames, violation_mat_env, 1e-5, env_collision_mask, 0.02, VizColor.violation)
    
    violation_mat_self = action_info.get("violation_mat_self", None)
    active_pairs_slack_self = render_critical_pairs(render_line_fn, robot_frames, robot_frames, violation_mat_self, 1e-5, self_collision_mask, 0.02, VizColor.violation)

    # display infeasible constraints
    if phi_safe_mat_env is not None and phi_safe_mat_self is not None:
        active_env_collision_mask = np.logical_and(env_collision_mask, phi_safe_mat_env >= 0.0)
        infeasible_pairs_env = render_critical_pairs(render_line_fn, robot_frames, obstacle_frames, violation_mat_env, None, active_env_collision_mask, 0.05, VizColor.not_a_number)
        active_self_collision_mask = np.logical_and(self_collision_mask, phi_safe_mat_self >= 0.0)
        infeasible_pairs_self = render_critical_pairs(render_line_fn, robot_frames, robot_frames, violation_mat_self, None, active_self_collision_mask, 0.05, VizColor.not_a_number)

    # ----------------------------- collision volumes ---------------------------- #

    # render robot collision volumes
    for frame_id, frame_world in enumerate(robot_frames):
        
        geom = robot_cfg.CollisionVol[robot_cfg.Frames(frame_id)]
        
        # highlight if triggering any phi
        if any(frame_id in pair for pair in active_pairs_unsafe_self) or any(frame_id == _frame_id for _frame_id, _ in active_pairs_unsafe_env):
            geom.color = VizColor.unsafe
        elif any(frame_id in pair for pair in active_pairs_hold_self) or any(frame_id == _frame_id for _frame_id, _ in active_pairs_hold_env):
            geom.color = VizColor.hold
        else:
            if frame_id in safety_index.env_collision_vol_ignore:
                geom.color = VizColor.collision_volume_ignored
            else:
                geom.color = VizColor.collision_volume
        
        # todo generalize this to handle different geometry types
        if geom.type == "sphere":
            render_sphere_fn(frame_world[:3,3], frame_world[:3,:3], geom.attributes["radius"]*np.ones(3), geom.color)
        elif geom.type == "box":
            render_box_fn(frame_world[:3,3],
                          frame_world[:3,:3], 
                          np.array([geom.attributes["length"], 
                                    geom.attributes["width"], 
                                    geom.attributes["height"]]), 
                          geom.color)
        else:
            raise ValueError(f'Unknown geometry type: {geom.type}')
    
    # render obstacles
    for obstacle_id_task, (frame_world, geom) in enumerate(zip(obstacle_frames, obstacle_geoms)):
        
        # obs id is obs id in task
        obstacle_id = obstacle_id_task
    
        # highlight if have collision
        if np.min(dist_robot_to_env[:, obstacle_id]) <= 0:
            geom.color = VizColor.collision
        else:
            geom.color = VizColor.obstacle_task
    
        # todo generalize this to handle different geometry types
        if geom.type == "sphere":
            render_sphere_fn(frame_world[:3,3], frame_world[:3,:3], geom.attributes["radius"]*np.ones(3), geom.color)
        elif geom.type == "box":
            render_box_fn(frame_world[:3,3],
                          frame_world[:3,:3], 
                          np.array([geom.attributes["length"], 
                                    geom.attributes["width"], 
                                    geom.attributes["height"]]), 
                          geom.color)
        else:
            raise ValueError(f'Unknown geometry type: {geom.type}')
    
    
    # render safe zone
    if enable_safe_zone_render:
            # Define the parameters for the zero-level set
        robot_collision_vol, obstacle_collision_vol = safety_index.get_vol_info(agent_feedback["state"], task_info)
        d_env, v_env, normal_env, curv_env = safety_index.compute_pairwise_info(robot_collision_vol, obstacle_collision_vol)
        d_self, v_self, normal_self, curv_self = safety_index.compute_pairwise_info(robot_collision_vol, robot_collision_vol) 
        phi_safe_mat_self = action_info.get("phi_safe_mat_self", None)
        for frame in robot_cfg.VisualizeSafeZone:
            largest_phi_idx = np.argmax(phi_safe_mat_env[frame])
            v = v_env[frame, largest_phi_idx]
            normal = normal_env[frame, largest_phi_idx]
            dmin = safety_index.min_distance["environment"]  # Minimum distance
            n = safety_index.n    # Exponent
            k = safety_index.k     # Scaling factor
            frame_1_world = robot_frames[frame]
            grid_size = 8   
            
            # space_size = (dmin**n + k * np.linalg.norm(v))**(1/n)
            space_size = (dmin**n + k * np.linalg.norm(v))**(1/n)
            # Generate grid values
            x_vals = np.linspace(-space_size, space_size, grid_size)
            try:
                phi = compute_phi_and_extract_best(dmin, n, k, v, normal, x_vals)
                dx = dy = dz = space_size * 2 / (grid_size - 1)  # Grid step size
                verts, faces, _, _ = measure.marching_cubes(phi, level=0.0)
                
                verts = verts * np.array([dx, dy, dz]) - space_size
                
                points_homogeneous = np.hstack([verts, np.ones((verts.shape[0], 1))])

                # Apply the transformation matrix T
                frame_1_world[:3, :3] = np.eye(3)
                transformed_points_homogeneous = frame_1_world @ points_homogeneous.T   # Matrix multiplication

                # Convert back to 3D coordinates (drop the homogeneous coordinate)
                transformed_verts = transformed_points_homogeneous.T[:, :3]
                render_surface_fn(transformed_verts[faces], VizColor.safe_zone)
            except:
                print("safe zone is not computable")
    
