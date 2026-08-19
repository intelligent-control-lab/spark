from spark_pipeline.base.base_pipeline_config import BasePipelineConfig
from spark_policy.composed_policy.safety_filtered import TeleopSafetyFilteredPolicyConfig


class TeleopPipelineConfig(BasePipelineConfig):
    """Base config for teleoperation pipelines (shared defaults)."""

    # ==========================================================================
    # Global pipeline options
    # ==========================================================================
    max_num_steps = -1
    max_num_reset = -1
    enable_logger = False
    enable_plotter = False
    enable_safe_zone_render = False
    render_robot_collision_volumes = True

    # ==========================================================================
    # Metric selection
    # ==========================================================================
    class metric_selection:
        # State / observation metrics
        dof_pos = False
        dof_vel = False
        goal_pos = False
        obstacle_pos = False

        # Distance metrics
        dist_self = False
        dist_robot_to_env = False
        dist_goal_arm = False
        dist_goal_base = False

        # Episode / runtime metrics
        seed = False
        done = False

        # Safety & performance metrics
        violation = True
        loop_time = True
        trigger_safe_controller = False

    # ==========================================================================
    # Robot configuration
    # ==========================================================================
    class robot(BasePipelineConfig.robot):
        class cfg(BasePipelineConfig.robot.cfg):
            pass

    # ==========================================================================
    # Environment configuration
    # ==========================================================================
    class env(BasePipelineConfig.env):
        class task(BasePipelineConfig.env.task):
            class_name = "TeleopTask"
            mode = "Velocity"
            num_obstacle_task = 0
            base_goal_enable = True
            arm_goal_enable = True

        class agent(BasePipelineConfig.env.agent):
            dt = 0.002
            use_sim_dynamics = True
            enable_keyboard_control = True
            control_decimation = 5
            enable_viewer = True
            viewer_show_simulation_info = False
            enable_camera = False
            camera_width = 640
            camera_height = 480
            camera_rate_hz = 30.0
            camera_display = False
            camera_config = None
            mujoco_model_path = None
            real_time = True
            # RTX presentation is intentionally slower than the 50 Hz control
            # loop. Ten visual updates per second leaves enough CPU/GPU slack
            # for the heaviest single-environment mobile manipulators while
            # the absolute-deadline pacer preserves their control frequency.
            viewer_hz = 10.0
            ignored_collision_body_names = []
            graspable_body_names = None
            grasp_exclude_body_names = []
            grasp_radius = 0.10

            # Backend-neutral view definition. MuJoCo consumes the spherical
            # camera fields directly; Isaac converts the same values to an
            # eye/target camera pose.
            viewer_config = dict(
                camera_lookat=(0.5, 0.0, 1.3),
                camera_distance=2.0,
                camera_azimuth=180.0,
                camera_elevation=-25.0,
                camera_vertical_fov=45.0,
                ground_style="grid",
                ground_color=(0.82, 0.82, 0.82),
                grid_color=(0.42, 0.42, 0.42),
                grid_spacing=0.5,
                grid_extent=5.0,
                grid_line_width=0.008,
            )

            # Obstacles perceived by the agent (MuJoCo keyboard-controlled)
            obstacle_debug = dict(
                num_obstacle=0,
                manual_movement_step_size=0.02,
            )

    # ==========================================================================
    # Executable policy configuration
    # ==========================================================================
    class policy(TeleopSafetyFilteredPolicyConfig):
        pass
