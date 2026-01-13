from spark_pipeline.base.base_pipeline_config import BasePipelineConfig


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
            base_goal_enable = False
            arm_goal_enable = False

        class agent(BasePipelineConfig.env.agent):
            dt = 0.002
            control_decimation = 5
            enable_viewer = True

            # Obstacles perceived by the agent (MuJoCo keyboard-controlled)
            obstacle_debug = dict(
                num_obstacle=0,
                manual_movement_step_size=0.1,
            )

    # ==========================================================================
    # Algorithm configuration
    # ==========================================================================
    class algo(BasePipelineConfig.algo):

        class policy:
            class_name = "TeleopPIDPolicy"

        class safe_controller:
            class_name = "BaseSafeController"

            class safety_index:
                class_name = "FirstOrderCollisionSafetyIndex"
                min_distance = {
                    "environment": 0.1,
                    "self": 0.01,
                }
                enable_self_collision = False

            class safe_algo:
                class_name = "ByPassSafeControl"
