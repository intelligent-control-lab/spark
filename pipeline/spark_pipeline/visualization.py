from spark_pipeline.base.base_pipeline import BasePipeline
from spark_utils import (
    COLLISION_CONTACT_TOLERANCE,
    VizColor,
    collision_volume_distance_color,
    compute_phi_and_extract_best,
)
import numpy as np
from spark_policy.safety.monitoring import BaseSafetyIndex
from spark_robot import RobotConfig, RobotKinematics
import matplotlib.pyplot as plt
from collections import deque
from skimage import measure


def render_critical_pairs(
    line_render_fn, frame_list_1, frame_list_2, mat, thres, mask, line_width, line_color
):
    """
    line_render_fn: function to render line segment
    frame_list_1: list of frames
    frame_list_2: list of frames
    mat: pairwise distance matrix

    connect frame_list_1[i] to frame_list_2[j] if mat[i,j] >= thres (or isnan if thres is None) and mask[i,j] == True
    """
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
            pos1=frame_list_1[i][:3, 3],
            pos2=frame_list_2[j][:3, 3],
            radius=line_width,
            color=line_color,
        )

    return indices_of_interest


def _has_active_collision(dist_robot_to_env, env_collision_mask, obstacle_id):
    """Return physical penetration status, independent of safety activation."""
    if dist_robot_to_env is None:
        return False
    dist = np.asarray(dist_robot_to_env, dtype=np.float64)
    if dist.ndim != 2 or obstacle_id >= dist.shape[1]:
        return False

    # ``env_collision_mask`` is intentionally ignored. A pair may be excluded
    # from the safety QP while still physically overlapping in the viewer.
    active = np.isfinite(dist[:, obstacle_id])
    if not np.any(active):
        return False
    # Physics engines maintain a small contact gap and collision-distance
    # calculations accumulate floating-point error.  Treat sub-millimetre
    # clearance as contact so a visibly touching pair does not remain grey.
    return bool(np.nanmin(dist[active, obstacle_id]) <= COLLISION_CONTACT_TOLERANCE)


def closest_collision_volume_distances(dist_robot_to_env, num_robot_volumes):
    """Return one signed surface clearance per robot collision volume."""
    closest = np.full(int(num_robot_volumes), np.inf, dtype=np.float64)
    if dist_robot_to_env is None:
        return closest
    distance = np.asarray(dist_robot_to_env, dtype=np.float64)
    if distance.ndim != 2 or distance.shape[0] == 0 or distance.shape[1] == 0:
        return closest
    rows = min(len(closest), distance.shape[0])
    for row in range(rows):
        finite = distance[row, np.isfinite(distance[row])]
        if finite.size:
            closest[row] = float(np.min(finite))
    return closest


def render_task_debug_trajectories(
    render_line_fn, trajectories, colors=None, radii=None, widths=None, use_pixel_width=False
):
    """Render trajectory distributions as line segments without frame glyphs."""

    if trajectories is None:
        return
    trajectories = np.asarray(trajectories, dtype=np.float64)
    if trajectories.ndim == 3 and trajectories.shape[-2:] == (4, 4):
        trajectories = trajectories.reshape(1, *trajectories.shape)
    if trajectories.ndim != 4 or trajectories.shape[-2:] != (4, 4):
        return

    default_color = np.array([0.392, 0.455, 0.545, 0.35], dtype=np.float64)
    colors_array = None if colors is None else np.asarray(colors, dtype=np.float64)
    radii_array = None if radii is None else np.asarray(radii, dtype=np.float64).reshape(-1)
    widths_array = None if widths is None else np.asarray(widths, dtype=np.float64).reshape(-1)

    for trajectory_index, trajectory in enumerate(trajectories):
        if colors_array is None:
            color = default_color
        elif colors_array.ndim == 1 and colors_array.shape[0] == 4:
            color = colors_array
        elif colors_array.ndim == 2 and trajectory_index < colors_array.shape[0]:
            color = colors_array[trajectory_index]
        else:
            color = default_color
        color = np.clip(np.asarray(color, dtype=np.float64).reshape(4), 0.0, 1.0)

        radius = 0.001
        if radii_array is not None and trajectory_index < radii_array.shape[0]:
            radius = float(np.clip(radii_array[trajectory_index], 0.0002, 0.01))
        width = 1.5
        if widths_array is not None and trajectory_index < widths_array.shape[0]:
            width = float(np.clip(widths_array[trajectory_index], 0.5, 10.0))

        positions = trajectory[:, :3, 3]
        for start, end in zip(positions[:-1], positions[1:]):
            if not (np.all(np.isfinite(start)) and np.all(np.isfinite(end))):
                continue
            if np.linalg.norm(end - start) <= 1e-8:
                continue
            if use_pixel_width:
                render_line_fn(start, end, width=width, color=color)
            else:
                render_line_fn(start, end, radius=radius, color=color)


def render_value_based_debug_info(
    render_sphere_fn,
    render_line_fn,
    render_box_fn,
    render_surface_fn,
    render_coordinate_frame_fn,
    agent_feedback,
    task_info,
    action_info,
    safety_index: BaseSafetyIndex,
    robot_frames,
    dist_robot_to_env,
    robot_cfg: RobotConfig,
    enable_safe_zone_render=False,
    render_trajectory_line_fn=None,
    render_goal_frames=True,
    render_task_debug_frames=True,
    render_action_debug_frames=True,
    render_robot_reference_frames=True,
    render_robot_collision_volumes=True,
    collision_volume_opacity_scale=1.0,
    collision_volume_opacity_floor=0.0,
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
        base_left_frame_world = goal_base_frame_world
        goal_list += [goal_base_frame_world]
        goal_sizes += [task_info.get("base_goal_size", 0.05)]

    if goal_right_frame_base is not None:
        goal_right_frame_world = goal_right_frame_base
        goal_list += [goal_right_frame_world]
        goal_sizes += [task_info.get("arm_goal_size", 0.05)]
    if goal_left_frame_base is not None:
        goal_left_frame_world = goal_left_frame_base
        goal_list += [goal_left_frame_world]
        goal_sizes += [task_info.get("arm_goal_size", 0.05)]
    for goal, goal_size in zip(goal_list, goal_sizes):
        if goal is not None and goal_size is not None:
            if goal.ndim == 3:
                for g in goal:
                    render_sphere_fn(g[:3, 3], g[:3, :3], goal_size * np.ones(3), VizColor.goal)
            else:
                render_sphere_fn(goal[:3, 3], goal[:3, :3], goal_size * np.ones(3), VizColor.goal)

    if "zmp" in action_info.keys():
        zmp = action_info["zmp"]
        if zmp is not None:
            render_sphere_fn(zmp, np.eye(3), 0.02 * np.ones(3), VizColor.unsafe)

    # ---------------------------- safety constraints ---------------------------- #

    obstacle_frames = task_info["obstacle"]["frames_world"]
    obstacle_geoms = task_info["obstacle"]["geom"]
    constraint_obstacle_frames = obstacle_frames
    selected_obstacles = action_info.get("safety_obstacle_indices", None)
    if selected_obstacles is not None:
        selected_obstacles = np.asarray(selected_obstacles, dtype=int)
        constraint_obstacle_frames = np.asarray(obstacle_frames)[selected_obstacles]

    # get collision constraint masks
    env_collision_mask = safety_index.env_collision_mask
    self_collision_mask = safety_index.self_collision_mask

    # display critical vol pairs (phi_hold)
    phi_hold_mat_env = action_info.get("phi_hold_mat_env", None)
    active_pairs_hold_env = render_critical_pairs(
        render_line_fn,
        robot_frames,
        constraint_obstacle_frames,
        phi_hold_mat_env,
        0.0,
        env_collision_mask,
        0.0005,
        VizColor.hold,
    )

    phi_hold_mat_self = action_info.get("phi_hold_mat_self", None)
    active_pairs_hold_self = render_critical_pairs(
        render_line_fn,
        robot_frames,
        robot_frames,
        phi_hold_mat_self,
        0.0,
        self_collision_mask,
        0.0005,
        VizColor.hold,
    )

    # display critical vol pairs (phi_safe)
    phi_safe_mat_env = action_info.get("phi_safe_mat_env", None)
    active_pairs_unsafe_env = render_critical_pairs(
        render_line_fn,
        robot_frames,
        constraint_obstacle_frames,
        phi_safe_mat_env,
        0.0,
        env_collision_mask,
        0.001,
        VizColor.unsafe,
    )

    phi_safe_mat_self = action_info.get("phi_safe_mat_self", None)
    active_pairs_unsafe_self = render_critical_pairs(
        render_line_fn,
        robot_frames,
        robot_frames,
        phi_safe_mat_self,
        0.0,
        self_collision_mask,
        0.001,
        VizColor.unsafe,
    )

    # display slack vars

    violation_mat_env = action_info.get("violation_mat_env", None)
    active_pairs_slack_env = render_critical_pairs(
        render_line_fn,
        robot_frames,
        constraint_obstacle_frames,
        violation_mat_env,
        1e-5,
        env_collision_mask,
        0.0015,
        VizColor.violation,
    )

    violation_mat_self = action_info.get("violation_mat_self", None)
    active_pairs_slack_self = render_critical_pairs(
        render_line_fn,
        robot_frames,
        robot_frames,
        violation_mat_self,
        1e-5,
        self_collision_mask,
        0.0015,
        VizColor.violation,
    )

    # display infeasible constraints
    if phi_safe_mat_env is not None and phi_safe_mat_self is not None:
        active_env_collision_mask = np.logical_and(env_collision_mask, phi_safe_mat_env >= 0.0)
        infeasible_pairs_env = render_critical_pairs(
            render_line_fn,
            robot_frames,
            constraint_obstacle_frames,
            violation_mat_env,
            None,
            active_env_collision_mask,
            0.05,
            VizColor.not_a_number,
        )
        active_self_collision_mask = np.logical_and(self_collision_mask, phi_safe_mat_self >= 0.0)
        infeasible_pairs_self = render_critical_pairs(
            render_line_fn,
            robot_frames,
            robot_frames,
            violation_mat_self,
            None,
            active_self_collision_mask,
            0.05,
            VizColor.not_a_number,
        )

    # Red is reserved for geometric contact/penetration, not
    # safety activation, QP participation, or positive slack.
    collision_obstacle_ids = set()
    if dist_robot_to_env is not None:
        distance = np.asarray(dist_robot_to_env, dtype=float)
        if distance.ndim == 2:
            collision_pairs = np.argwhere(
                np.isfinite(distance) & (distance <= COLLISION_CONTACT_TOLERANCE)
            )
            collision_obstacle_ids = {int(pair[1]) for pair in collision_pairs}
            if selected_obstacles is not None:
                collision_obstacle_ids = {
                    int(selected_obstacles[index])
                    for index in collision_obstacle_ids
                    if 0 <= index < len(selected_obstacles)
                }

    # ----------------------------- collision volumes ---------------------------- #
    # render robot collision volumes
    closest_distances = closest_collision_volume_distances(dist_robot_to_env, len(robot_frames))
    minimum_distance = float(safety_index.min_distance["environment"])
    for frame_id, frame_world in enumerate(robot_frames):
        geom = robot_cfg.CollisionVol[robot_cfg.Frames(frame_id)]
        color = collision_volume_distance_color(
            closest_distances[frame_id],
            minimum_distance,
            ignored=frame_id in safety_index.env_collision_vol_ignore,
            opacity_scale=collision_volume_opacity_scale,
            opacity_floor=collision_volume_opacity_floor,
        )

        # todo generalize this to handle different geometry types
        if render_robot_collision_volumes:
            if geom.type == "sphere":
                render_sphere_fn(
                    frame_world[:3, 3],
                    frame_world[:3, :3],
                    geom.attributes["radius"] * np.ones(3),
                    color,
                )
            elif geom.type == "box":
                render_box_fn(
                    frame_world[:3, 3],
                    frame_world[:3, :3],
                    np.array(
                        [
                            geom.attributes["length"],
                            geom.attributes["width"],
                            geom.attributes["height"],
                        ]
                    ),
                    color,
                )
            else:
                raise ValueError(f"Unknown geometry type: {geom.type}")
    for pair in active_pairs_unsafe_self:
        fname1 = robot_cfg.Frames(pair[0]).name
        fname2 = robot_cfg.Frames(pair[1]).name
        print(f"Unsafe self-collision between {fname1} and {fname2}")
        print(
            np.linalg.norm(robot_frames[pair[0]][:3, 3] - robot_frames[pair[1]][:3, 3])
            - robot_cfg.CollisionVol[pair[0]].attributes["radius"]
            - robot_cfg.CollisionVol[pair[1]].attributes["radius"]
        )
    # render obstacles
    visual_indices = np.arange(len(obstacle_frames))
    visual_budget = int(task_info.get("max_visualized_points", len(visual_indices)))
    if len(visual_indices) > visual_budget:
        visual_indices = np.linspace(0, len(visual_indices) - 1, visual_budget, dtype=int)
    for obstacle_id_task in visual_indices:
        frame_world = obstacle_frames[obstacle_id_task]
        geom = obstacle_geoms[obstacle_id_task]

        # obs id is obs id in task
        obstacle_id = obstacle_id_task

        # highlight if have collision
        if obstacle_id in collision_obstacle_ids:
            geom.color = VizColor.collision
        else:
            geom.color = VizColor.obstacle_task

        # todo generalize this to handle different geometry types
        if geom.type == "sphere":
            render_sphere_fn(
                frame_world[:3, 3],
                frame_world[:3, :3],
                geom.attributes["radius"] * np.ones(3),
                geom.color,
            )
        elif geom.type == "box":
            render_box_fn(
                frame_world[:3, 3],
                frame_world[:3, :3],
                np.array(
                    [geom.attributes["length"], geom.attributes["width"], geom.attributes["height"]]
                ),
                geom.color,
            )
        else:
            raise ValueError(f"Unknown geometry type: {geom.type}")

    # render safe zone
    if enable_safe_zone_render:
        # Define the parameters for the zero-level set
        robot_collision_vol, obstacle_collision_vol = safety_index.get_vol_info(
            agent_feedback["state"], task_info
        )
        d_env, v_env, normal_env, curv_env = safety_index.compute_pairwise_info(
            robot_collision_vol, obstacle_collision_vol
        )
        d_self, v_self, normal_self, curv_self = safety_index.compute_pairwise_info(
            robot_collision_vol, robot_collision_vol
        )
        phi_safe_mat_self = action_info.get("phi_safe_mat_self", None)
        for frame in robot_cfg.VisualizeSafeZone:
            largest_phi_idx = np.argmax(phi_safe_mat_env[frame])
            v = v_env[frame, largest_phi_idx]
            normal = normal_env[frame, largest_phi_idx]
            dmin = safety_index.min_distance["environment"]  # Minimum distance
            n = safety_index.n  # Exponent
            k = safety_index.k  # Scaling factor
            frame_1_world = robot_frames[frame]
            grid_size = 8

            # space_size = (dmin**n + k * np.linalg.norm(v))**(1/n)
            space_size = (dmin**n + k * np.linalg.norm(v)) ** (1 / n)
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
                transformed_points_homogeneous = (
                    frame_1_world @ points_homogeneous.T
                )  # Matrix multiplication

                # Convert back to 3D coordinates (drop the homogeneous coordinate)
                transformed_verts = transformed_points_homogeneous.T[:, :3]
                render_surface_fn(transformed_verts[faces], VizColor.safe_zone)
            except:
                print("safe zone is not computable")

    if "task_debug_trajectories" in task_info:
        line_fn = (
            render_trajectory_line_fn if render_trajectory_line_fn is not None else render_line_fn
        )
        render_task_debug_trajectories(
            line_fn,
            task_info["task_debug_trajectories"],
            colors=task_info.get("task_debug_trajectory_colors", None),
            radii=task_info.get("task_debug_trajectory_radii", None),
            widths=task_info.get("task_debug_trajectory_widths", None),
            use_pixel_width=render_trajectory_line_fn is not None,
        )
    if render_task_debug_frames and "task_debug_frames" in task_info:
        for frame in task_info["task_debug_frames"]:
            render_coordinate_frame_fn(frame, size=0.1)
    if render_action_debug_frames and "action_debug_frames" in action_info:
        for frame in action_info["action_debug_frames"]:
            render_coordinate_frame_fn(frame, size=0.1)
    if render_robot_reference_frames:
        try:
            R_ee = robot_frames[robot_cfg.Frames.R_ee]
            render_coordinate_frame_fn(R_ee, size=0.1)
        except:
            pass

        try:
            L_ee = robot_frames[robot_cfg.Frames.L_ee]
            render_coordinate_frame_fn(L_ee, size=0.1)
        except:
            pass

        try:
            render_coordinate_frame_fn(robot_base_frame, size=0.1)
        except:
            pass

    if render_goal_frames:
        try:
            for frame in goal_right_frame_world:
                render_coordinate_frame_fn(frame, size=0.1)
        except:
            pass

        try:
            for frame in goal_left_frame_world:
                render_coordinate_frame_fn(frame, size=0.1)
        except:
            pass

        try:
            for frame in base_left_frame_world:
                render_coordinate_frame_fn(frame, size=0.1)
        except:
            pass
