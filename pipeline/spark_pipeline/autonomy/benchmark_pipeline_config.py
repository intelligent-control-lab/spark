from spark_pipeline.base.base_pipeline_config import BasePipelineConfig


class BenchmarkPipelineConfig(BasePipelineConfig):
    """Base config for benchmark pipelines (shared defaults)."""

    # ==========================================================================
    # Global pipeline options
    # ==========================================================================
    # One benchmark invocation covers 20 seconds at the shared 50 Hz policy
    # rate and permits repeated row-local episode resets. Runners consume these
    # values when their corresponding CLI options are omitted.
    max_num_steps = 1000
    max_num_reset = 10
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
            class_name = "BenchmarkTask"
            task_name = "BenchmarkTask"
            task_config = {}

        class agent(BasePipelineConfig.env.agent):
            dt = 0.002
            control_decimation = 5
            enable_viewer = True
            viewer_show_simulation_info = False

            # Obstacles perceived by the agent (MuJoCo keyboard-controlled)
            obstacle_debug = dict(
                num_obstacle=0,
                manual_movement_step_size=0.1,
            )

    # ==========================================================================
    # Executable policy configuration
    # ==========================================================================
    class policy(BasePipelineConfig.policy):
        class_name = "SafetyFilteredPolicy"

        class nominal_controller:
            class_name = "BenchmarkPIDPolicy"

        class safe_controller:
            class_name = None

            class safety_index:
                class_name = "FirstOrderCollisionSafetyIndex"
                min_distance = {
                    "environment": 0.1,
                    # Sphere self-collision is geometric overlap only.  Any
                    # desired clearance belongs to an explicit test/policy
                    # override, not the benchmark baseline.
                    "self": 0.0,
                }
                enable_self_collision = False

            class safe_algo:
                class_name = "ByPassSafeControl"
