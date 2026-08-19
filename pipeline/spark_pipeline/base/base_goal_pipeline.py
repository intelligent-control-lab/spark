from spark_pipeline.base.base_pipeline import BasePipeline
from spark_pipeline.base.base_pipeline_config import BasePipelineConfig
from spark_utils import VizColor, Logger, DataBuffer, compute_masked_distance_matrix
from spark_pipeline import SPARK_PIPELINE_ROOT
import numpy as np
from collections import OrderedDict
from spark_pipeline.visualization import _has_active_collision, render_value_based_debug_info
import time
import os
import subprocess


class BaseGoalPipeline(BasePipeline[BasePipelineConfig]):
    def __init__(self, cfg):
        super().__init__(cfg)
        self.max_num_reset = self.cfg.max_num_reset
        self.logger = None
        self.profile_frequency = bool(getattr(self.cfg, "profile_frequency", False))
        self._runtime_timings = {
            name: [] for name in ("environment", "policy", "idle", "processing", "render", "cycle")
        }
        self._safety_trigger_steps = 0
        self._safety_observed_steps = 0
        self._safety_stage_timings = {"index": [], "solve": [], "constraints": []}
        self._safety_locomotion_corrections = []
        self._safety_upper_body_corrections = []
        self.policy_observability = {}
        self.env.agent.render_robot_collision_volumes = bool(
            getattr(self.cfg, "render_robot_collision_volumes", True)
        )
        self.initialize_log_buffer()

    def _refresh_policy_observability(self):
        provider = getattr(self.policy, "observability_context", None)
        context = provider() if callable(provider) else None
        self.policy_observability = dict(context or {})
        return self.policy_observability

    def post_physics_step(self, agent_feedback, task_info, action_info):
        self._prepare_post_physics_step(agent_feedback, task_info, action_info)

        # render simulation
        timing_start = time.perf_counter()
        render_every = max(1, int(getattr(self.cfg, "render_every", 1)))
        if self.pipeline_step % render_every == 0:
            self.render()
        self._record_runtime_timing("render", time.perf_counter() - timing_start)

    def _prepare_post_physics_step(self, agent_feedback, task_info, action_info):
        """Prepare one visualization/logging state before backend rendering."""
        self.task_info = task_info
        self.agent_feedback = agent_feedback
        self.action_info = action_info

        # common data processing and logging
        timing_start = time.perf_counter()
        self.process_and_save_data()
        self._record_runtime_timing("processing", time.perf_counter() - timing_start)

    def run(self, save_path=None):

        self.setup_logging(save_path)

        # reset environment
        agent_feedback, task_info = self.env.reset()
        self.policy.reset()
        self.num_reset = 1

        # initial action
        action, action_info = self.policy.act(agent_feedback, task_info)

        time_start = time.time()
        status_time = time.monotonic()
        status_step = self.pipeline_step
        next_cycle_deadline = time.perf_counter()
        print(
            "Pipeline control loop running "
            f"(target period "
            f"{self.cfg.env.agent.dt * self.cfg.env.agent.control_decimation:.3f}s). "
            "Press Ctrl-C to stop an unbounded run.",
            flush=True,
        )
        while self.pipeline_step < self.max_num_steps or self.max_num_steps < 0:
            if not self.env.agent.is_running():
                print("Agent backend stopped; terminating pipeline")
                break
            start_t = time.perf_counter()
            # reset if necessary
            if task_info["done"]:
                done_info = task_info.get("done_info", {})
                if done_info:
                    print(
                        "Pipeline done: "
                        f"reason={done_info.get('reason')}, "
                        f"episode_steps={done_info.get('episode_length')}, "
                        f"base_height={done_info.get('base_height')}, "
                        f"fallen={done_info.get('fallen')}, "
                        f"base_reached={done_info.get('base_goal_reached')}, "
                        f"base_error={done_info.get('base_goal_distance')}, "
                        f"base_yaw_error={done_info.get('base_goal_yaw_error')}, "
                        f"arm_reached={done_info.get('arm_goal_reached')}, "
                        f"left_arm_error={done_info.get('arm_goal_distance_left')}, "
                        f"right_arm_error={done_info.get('arm_goal_distance_right')}"
                    )
                else:
                    print("Pipeline done")
                if self.max_num_reset != -1 and self.num_reset >= self.max_num_reset:
                    break

                continuous_transition = bool(
                    getattr(self.cfg, "continuous_goal_transition", False)
                    and done_info.get("reason") == "goal_reached"
                )
                if continuous_transition:
                    agent_feedback, task_info = self.env.transition_task()
                    print("Continuous goal transition: preserved agent and policy state")
                else:
                    agent_feedback, task_info = self.env.reset()
                    self.policy.reset()
                self.num_reset += 1

                action, action_info = self.policy.act(agent_feedback, task_info)

            # environment step
            # s_next = env(s, a)
            timing_start = time.perf_counter()
            agent_feedback, task_info = self.env.step(action, action_info)
            self._record_runtime_timing("environment", time.perf_counter() - timing_start)

            # next action
            # a_next = policy(s_next)
            timing_start = time.perf_counter()
            action, action_info = self.policy.act(agent_feedback, task_info)
            self._safety_observed_steps += 1
            if bool(np.any(action_info.get("trigger_safe", False))):
                self._safety_trigger_steps += 1
                sport_ref = action_info.get("sport_u_ref", None)
                sport_safe = action_info.get("sport_u_safe", None)
                if sport_ref is not None and sport_safe is not None:
                    correction = np.asarray(sport_safe) - np.asarray(sport_ref)
                    if correction.size >= 20:
                        self._safety_upper_body_corrections.append(
                            float(np.linalg.norm(correction[:17]))
                        )
                        self._safety_locomotion_corrections.append(
                            float(np.linalg.norm(correction[17:20]))
                        )
            if "safety_index_seconds" in action_info:
                self._safety_stage_timings["index"].append(
                    float(action_info["safety_index_seconds"])
                )
                self._safety_stage_timings["solve"].append(
                    float(action_info.get("safety_solve_seconds", 0.0))
                )
                self._safety_stage_timings["constraints"].append(
                    int(action_info.get("safety_constraint_count", 0))
                )
            self._record_runtime_timing("policy", time.perf_counter() - timing_start)

            # post physics step (e.g., rendering, status publishing)
            self.pipeline_step += 1
            # Data logging historically records this pre-render processing
            # latency. Keep it available while the final paced cycle time is
            # assigned below.
            self.loop_time = time.perf_counter() - start_t
            self.post_physics_step(agent_feedback, task_info, action_info)

            # Pace the complete control + visualization-submission cycle.  The
            # previous ordering slept before rendering, so an inline MuJoCo
            # viewer ran at 20 ms plus render time while Isaac's asynchronous
            # submission remained close to 20 ms.
            idle_start = time.perf_counter()
            control_period = self.cfg.env.agent.dt * self.cfg.env.agent.control_decimation
            if getattr(self.cfg.env.agent, "real_time", True):
                # Pace against an absolute deadline. An occasional RTX frame
                # may exceed one control period, but following camera-free
                # updates can consume that slip and restore the target average.
                # Relative per-cycle sleeping permanently converted every
                # render overrun into a lower control frequency.
                next_cycle_deadline += control_period
                now = time.perf_counter()
                remaining = next_cycle_deadline - now
                if remaining > 0.0:
                    time.sleep(remaining)
                elif -remaining > 5.0 * control_period:
                    # Do not replay an unbounded backlog after a debugger,
                    # window drag, or another long interactive pause.
                    next_cycle_deadline = now
            else:
                next_cycle_deadline = time.perf_counter()
            self._record_runtime_timing("idle", time.perf_counter() - idle_start)
            self.loop_time = time.perf_counter() - start_t
            self._record_runtime_timing("cycle", time.perf_counter() - start_t)

            now = time.monotonic()
            if now - status_time >= 5.0:
                elapsed = max(now - status_time, 1.0e-9)
                qpos = np.asarray(
                    agent_feedback.get("body_qpos_fbk", agent_feedback.get("qpos_fbk", [])),
                    dtype=float,
                ).reshape(-1)
                base_height = f"{qpos[2]:.3f}m" if qpos.size >= 3 else "n/a"
                planner_command = action_info.get("sonic_planner_command", {})
                planner_mode = planner_command.get("mode", "n/a")
                sonic_ok = action_info.get("sonic_ok", "n/a")
                print(
                    "Pipeline alive: "
                    f"step={self.pipeline_step}, "
                    f"rate={(self.pipeline_step - status_step) / elapsed:.1f}Hz, "
                    f"base_height={base_height}, "
                    f"sonic_ok={sonic_ok}, planner_mode={planner_mode}",
                    flush=True,
                )
                status_time = now
                status_step = self.pipeline_step

            # print(f"Step: {self.pipeline_step} Frequencies: {1.0 / (time.time() - start_t): .3f} Hz")
            # print(f"Step: {self.pipeline_step} | Total Time: {time.time() - time_start:.1f} sec | Resets: {self.num_reset}", end="\r")

        # end of pipeline
        time_finish = time.time()
        print("Pipeline finished in ", time_finish - time_start, " seconds.")
        self._report_runtime_timings()
        if self._safety_observed_steps:
            print(
                "Safety filter activity: "
                f"{self._safety_trigger_steps}/{self._safety_observed_steps} "
                "control steps triggered"
            )
        if self._safety_stage_timings["index"]:
            index_time = np.asarray(self._safety_stage_timings["index"])
            solve_time = np.asarray(self._safety_stage_timings["solve"])
            constraints = np.asarray(self._safety_stage_timings["constraints"])
            print(
                "Safety stage profile: "
                f"index mean/p95={1e3 * index_time.mean():.2f}/"
                f"{1e3 * np.percentile(index_time, 95):.2f} ms, "
                f"solve mean/p95={1e3 * solve_time.mean():.2f}/"
                f"{1e3 * np.percentile(solve_time, 95):.2f} ms, "
                f"constraints mean/max={constraints.mean():.0f}/{constraints.max()}"
            )
        if self._safety_locomotion_corrections:
            locomotion = np.asarray(self._safety_locomotion_corrections)
            upper_body = np.asarray(self._safety_upper_body_corrections)
            print(
                "Safety correction norms on triggered steps: "
                f"locomotion mean/max={locomotion.mean():.4f}/{locomotion.max():.4f}, "
                f"upper-body mean/max={upper_body.mean():.4f}/{upper_body.max():.4f}"
            )
        self.save_results()
        self.env.agent.close_viewer()
        if self.cfg.enable_plotter:
            print("Terminating plotter process")
            self.plotter_process.terminate()
            self.plotter_process.wait()
        time.sleep(1)

    def _record_runtime_timing(self, name, duration):
        if self.profile_frequency:
            self._runtime_timings[name].append(float(duration))

    def _report_runtime_timings(self):
        if not self.profile_frequency or not self._runtime_timings["cycle"]:
            return
        count = len(self._runtime_timings["cycle"])
        warmup = min(10, max(0, count // 10))
        print(f"\nRuntime frequency profile ({count} cycles; first {warmup} excluded):")
        steady_cycle = np.asarray(self._runtime_timings["cycle"][warmup:], dtype=float)
        if not steady_cycle.size:
            steady_cycle = np.asarray(self._runtime_timings["cycle"], dtype=float)
        mean_cycle = float(steady_cycle.mean())
        control_period = float(self.cfg.env.agent.dt * self.cfg.env.agent.control_decimation)
        target_hz = 1.0 / control_period
        print(
            f"  effective: {1.0 / mean_cycle:.2f} Hz mean, "
            f"{1.0 / np.percentile(steady_cycle, 95):.2f} Hz p95-latency "
            f"(target {target_hz:.2f} Hz, simulated real-time factor "
            f"{control_period / mean_cycle:.2f}x)"
        )
        for name in ("policy", "environment", "processing", "render", "idle"):
            values = np.asarray(self._runtime_timings[name][warmup:], dtype=float)
            if not values.size:
                values = np.asarray(self._runtime_timings[name], dtype=float)
            print(
                f"  {name:11s}: mean {1e3 * values.mean():7.2f} ms, "
                f"p95 {1e3 * np.percentile(values, 95):7.2f} ms"
            )
        print()

    # ---------------------------------- helpers --------------------------------- #

    def setup_logging(self, save_path=None):
        """
        Save logger to {save_path}/log (clear if exists)
        Save data to {save_path}/{data}
        """

        self.save_path = save_path

        if self.save_path is None:
            return

        if os.path.exists(save_path):
            import shutil

            shutil.rmtree(save_path)

        os.makedirs(save_path)

        print("\nSAVING TO: ", self.save_path, "\n")
        self.logdir = os.path.join(self.save_path, "log")
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
                self.plotter_process = subprocess.Popen(
                    ["python3", plotter_path, self.logdir] + tags_to_plot
                )
        else:
            self.logger = None
            print("\nLOGGING DISABLED\n")

    def initialize_log_buffer(self):

        self.data_buffer = DataBuffer()

        for metric_name, flag in self.cfg.metric_selection.__dict__.items():
            if flag:
                self.data_buffer.add(metric_name)

    def prepare_render(self):
        should_render = bool(
            self.cfg.env.agent.enable_viewer
            or getattr(self.env.agent, "recording_enabled", False)
            or getattr(self.env.agent, "enable_camera", False)
        )
        if should_render:
            policy_observability = self._refresh_policy_observability()
            begin_render_frame = getattr(self.env.agent, "begin_render_frame", None)
            if callable(begin_render_frame):
                begin_render_frame()
            render_context = {}
            visualization_context = getattr(self.policy, "visualization_context", None)
            if callable(visualization_context):
                context = visualization_context(
                    agent_feedback=self.agent_feedback,
                    task_info=self.task_info,
                    action_info=self.action_info,
                    robot_frames=self.robot_frames_world,
                    dist_robot_to_env=self.dist_robot_to_env,
                    robot_cfg=self.robot_cfg,
                )
                if context is not None:
                    render_context.update(context)

            render_value_based_debug_info(
                render_sphere_fn=self.env.agent.render_sphere,
                render_line_fn=self.env.agent.render_line_segment,
                render_box_fn=self.env.agent.render_box,
                render_surface_fn=self.env.agent.render_surface,
                render_coordinate_frame_fn=self.env.agent.render_coordinate_frame,
                render_trajectory_line_fn=getattr(
                    self.env.agent, "render_pixel_line_segment", None
                ),
                agent_feedback=self.agent_feedback,
                task_info=self.task_info,
                action_info=render_context.get("action_info", self.action_info),
                safety_index=render_context.get(
                    "safety_index",
                    policy_observability.get("constraint_monitor"),
                ),
                robot_frames=render_context.get("robot_frames", self.robot_frames_world),
                dist_robot_to_env=render_context.get("dist_robot_to_env", self.dist_robot_to_env),
                robot_cfg=render_context.get("robot_cfg", self.robot_cfg),
                enable_safe_zone_render=self.cfg.enable_safe_zone_render,
                render_goal_frames=getattr(self.cfg, "render_goal_frames", True),
                render_task_debug_frames=getattr(self.cfg, "render_task_debug_frames", True),
                render_action_debug_frames=getattr(self.cfg, "render_action_debug_frames", True),
                render_robot_reference_frames=getattr(
                    self.cfg, "render_robot_reference_frames", True
                ),
                render_robot_collision_volumes=getattr(
                    self.env.agent,
                    "render_robot_collision_volumes",
                    getattr(self.cfg, "render_robot_collision_volumes", True),
                ),
                collision_volume_opacity_scale=getattr(
                    self.env.agent,
                    "collision_volume_opacity_scale",
                    1.0,
                ),
                collision_volume_opacity_floor=getattr(
                    self.env.agent,
                    "collision_volume_opacity_floor",
                    0.0,
                ),
            )

            # change debug obstacle color if in collision (agent will render them)
            for obstacle_id_debug, geom in enumerate(self.env.agent.obstacle_debug_geom):
                # obs id is obs id in debug + num_obstacle_task
                obstacle_id = obstacle_id_debug + self.env.task.num_obstacle_task

                # highlight if have collision
                if _has_active_collision(
                    self.dist_robot_to_env,
                    policy_observability.get("environment_constraint_mask"),
                    obstacle_id,
                ):
                    geom.color = VizColor.collision
                else:
                    geom.color = VizColor.obstacle_debug

    def flush_render(self):
        if (
            self.cfg.env.agent.enable_viewer
            or getattr(self.env.agent, "recording_enabled", False)
            or getattr(self.env.agent, "enable_camera", False)
        ):
            self.env.agent.render()

    def render(self):
        self.prepare_render()
        self.flush_render()

    def process_and_save_data(self):
        policy_observability = self._refresh_policy_observability()
        self.goal_teleop = self.task_info.get("goal_teleop", None)
        self.goal_left_frame_base = (
            self.goal_teleop.get("left", None) if self.goal_teleop is not None else None
        )
        self.goal_right_frame_base = (
            self.goal_teleop.get("right", None) if self.goal_teleop is not None else None
        )
        self.goal_base_frame_world = (
            self.goal_teleop.get("base", None) if self.goal_teleop is not None else None
        )

        self.robot_base_frame = self.agent_feedback["robot_base_frame"]

        # compute transformations of robot collision volumes
        x = self.agent_feedback["state"]
        dof_pos = self.robot_cfg.decompose_state_to_dof_pos(x)
        robot_frames = self.robot_kinematics.forward_kinematics(dof_pos)
        self.robot_frames_world = np.zeros_like(robot_frames)

        self.data_buffer.add("robot_frame", robot_frames)

        for i in range(len(robot_frames)):
            self.robot_frames_world[i, :, :] = self.robot_base_frame @ robot_frames[i, :, :]

        # ------------------------- self collision processing ------------------------ #

        self.dist_self, self.index_min_dist_self = compute_masked_distance_matrix(
            frame_list_1=self.robot_frames_world,
            geom_list_1=self.robot_cfg.CollisionVol.values(),
            frame_list_2=self.robot_frames_world,
            geom_list_2=self.robot_cfg.CollisionVol.values(),
            mask=policy_observability.get("self_constraint_mask"),
        )
        if self.cfg.metric_selection.dist_self:
            self.data_buffer.add("dist_self", self.dist_self)

        # ------------------------- env collision processing ------------------------ #

        # Keep physical overlap visualization independent from safety/QP pair
        # selection. Safety modules calculate their own masked distances.
        self.dist_robot_to_env, self.index_min_dist_env = compute_masked_distance_matrix(
            frame_list_1=self.robot_frames_world,
            geom_list_1=self.robot_cfg.CollisionVol.values(),
            frame_list_2=self.task_info["obstacle"]["frames_world"],
            geom_list_2=self.task_info["obstacle"]["geom"],
            mask=None,
        )

        if self.cfg.metric_selection.dist_robot_to_env:
            self.data_buffer.add("dist_robot_to_env", self.dist_robot_to_env)

        # --------------------------------- arm goal --------------------------------- #
        has_left_ee = hasattr(self.robot_cfg.Frames, "L_ee")
        has_right_ee = hasattr(self.robot_cfg.Frames, "R_ee")

        if self.goal_left_frame_base is not None and has_left_ee:
            self.dist_goal_left = np.linalg.norm(
                self.robot_frames_world[self.robot_cfg.Frames.L_ee, :3, 3]
                - self.goal_left_frame_base[0, :3, 3]
            )
        else:
            self.dist_goal_left = None

        if self.goal_right_frame_base is not None and has_right_ee:
            self.dist_goal_right = np.linalg.norm(
                self.robot_frames_world[self.robot_cfg.Frames.R_ee, :3, 3]
                - self.goal_right_frame_base[0, :3, 3]
            )
        else:
            self.dist_goal_right = None

        if self.cfg.metric_selection.dist_goal_arm:
            dist_arm_goal_list = []
            if self.dist_goal_left is not None:
                dist_arm_goal_list.append(self.dist_goal_left)
            if self.dist_goal_right is not None:
                dist_arm_goal_list.append(self.dist_goal_right)
            if len(dist_arm_goal_list) > 0:
                self.data_buffer.add("dist_goal_arm", np.array(dist_arm_goal_list))
            else:
                raise ValueError("Arm goal not set while dist_goal_arm is required.")

        # --------------------------------- base goal -------------------------------- #

        if self.goal_base_frame_world is not None:
            self.dist_goal_base = np.linalg.norm(
                self.robot_base_frame[:3, 3] - self.goal_base_frame_world[0, :3, 3]
            )
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
                mask = policy_observability.get("environment_constraint_mask")
                if mask is not None:
                    mat[~mask] = 0.0
                self.data_buffer.add("violation_env", mat)

            if "violation_mat_self" in self.action_info:
                mat = self.action_info["violation_mat_self"]
                mask = policy_observability.get("self_constraint_mask")
                if mask is not None:
                    mat[~mask] = 0.0
                self.data_buffer.add("violation_self", mat)

        # ------------------------------- other metrics ------------------------------ #
        if self.cfg.metric_selection.seed:
            self.data_buffer.add("seed", self.task_info["seed"])

        if self.cfg.metric_selection.done:
            self.data_buffer.add("done", self.task_info["done"])

        if self.cfg.metric_selection.loop_time:
            self.data_buffer.add("loop_time", self.loop_time)

        if self.cfg.metric_selection.trigger_safe_controller:
            self.data_buffer.add("trigger_safe", self.action_info["trigger_safe"])

        for key in (
            "safe_base_control_indices",
            "safe_base_trigger_safe",
            "safe_base_u_ref",
            "safe_base_u_safe",
            "safe_base_u_delta",
            "safe_base_motion_cmd",
            "safe_base_motion_cmd_norm",
            "safe_base_lg_max_abs",
            "safe_active_base_lg_max_abs",
        ):
            if key in self.action_info:
                self.data_buffer.add(key, np.asarray(self.action_info[key]))

    def log(self, agent_feedback, task_info, action_info):

        if self.logger is None:
            return

        logs = OrderedDict()
        if self.cfg.metric_selection.dof_pos:
            for dof in self.robot_cfg.DoFs:
                logs[f"tracking_pos/dof_{dof.name}"] = {
                    "cmd_pos": agent_feedback["dof_pos_cmd"][dof],
                    "fbk_pos": agent_feedback["dof_pos_fbk"][dof],
                }

        if self.cfg.metric_selection.dof_vel:
            for dof in self.robot_cfg.DoFs:
                logs[f"tracking_vel/dof_{dof.name}"] = {
                    "reference_vel": action_info["u_ref"][dof],
                    "safe_vel": action_info["u_safe"][dof],
                    "cmd_vel": agent_feedback["dof_vel_cmd"][dof],
                    "fbk_vel": agent_feedback["dof_vel_fbk"][dof],
                }

        if self.cfg.metric_selection.goal_pos:
            for side in ["left", "right"]:
                logs[f"goal/{side}"] = {
                    "goal_pos_x": self.task_info["goal_teleop"][side][0, 3],
                    "goal_pos_y": self.task_info["goal_teleop"][side][1, 3],
                    "goal_pos_z": self.task_info["goal_teleop"][side][2, 3],
                }

        if self.cfg.metric_selection.obstacle_pos:
            for obstacle_id, obstacle in enumerate(self.task_info["obstacle"]["frames_world"]):
                logs[f"obstacle/{obstacle_id}"] = {
                    "obstacle_pos_x": obstacle[0, 3],
                    "obstacle_pos_y": obstacle[1, 3],
                    "obstacle_pos_z": obstacle[2, 3],
                }

        logs["loop_time"] = self.loop_time
        logs["SSA"] = action_info["trigger_safe"]
        logs["phi_safe"] = action_info["phi_safe"].max()
        phi_safe_mat_self = action_info.get("phi_safe_mat_self", None)
        phi_safe_mat_env = action_info.get("phi_safe_mat_env", None)
        phi0_mat_env = action_info.get("phi0_mat_env", None)
        phi0dot_mat_env = action_info.get("phi0dot_mat_env", None)

        for frame in self.robot_cfg.VisualizePhiTraj:
            largest_phi_idx = np.argmax(phi_safe_mat_env[frame])
            logs[f"phi_env_{frame.name}"] = phi_safe_mat_env[frame, largest_phi_idx]
            logs[f"phi0_env_{frame.name}"] = phi0_mat_env[frame, largest_phi_idx]
            logs[f"phi0dot_env_{frame.name}"] = phi0dot_mat_env[frame, largest_phi_idx]
            logs[f"phi_k_env_{frame.name}"] = self.policy_observability.get(
                "constraint_gain", np.nan
            )
        # perform the logging
        for key, value in logs.items():
            if isinstance(value, dict):
                self.logger.log_scalars(value, key, step=self.logger.counter, phase=" ")
            else:
                self.logger.log_scalar(value, key)
        self.logger.flush()  # internal counter +1, force buffered data in

    def save_results(self):

        if self.save_path is None:
            print("No save path specified. Not saving data.")
            return

        # add final data
        policy_observability = self._refresh_policy_observability()
        env_mask = policy_observability.get("environment_constraint_mask")
        self_mask = policy_observability.get("self_constraint_mask")
        if env_mask is not None:
            self.data_buffer.add("env_collision_mask", env_mask)
        if self_mask is not None:
            self.data_buffer.add("self_collision_mask", self_mask)

        # save data
        self.data_buffer.save_npz(os.path.join(self.save_path, "data.npz"))
