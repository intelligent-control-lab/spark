from spark_pipeline.base.base_pipeline_config import BasePipelineConfig

class BenchmarkPipelineConfig(BasePipelineConfig):
    
    # ==========================================================================
    # Global pipeline options
    # ==========================================================================
    max_num_steps = 500
    max_num_reset = -1
    enable_logger = False
    enable_plotter = False
    enable_safe_zone_render = False
    
    # ==========================================================================
    # Metric selection
    # ==========================================================================
    class metric_selection:  
        dist_self         = False
        dist_robot_to_env = False
        dist_goal_arm     = False
        dist_goal_base    = False
        seed    = False
        done    = False

        violation = True
    
    class robot( BasePipelineConfig.robot ):
        
        class cfg( BasePipelineConfig.robot.cfg ):
            pass
    
    # ==========================================================================
    # Environment configuration
    # ==========================================================================
    
    class env( BasePipelineConfig.env ):
    
        class task(  BasePipelineConfig.env.task ):
            class_name          = "BenchmarkTask"
            task_name           = "BenchmarkTask" # e.g., will appear as ros node name
            task_config         = {}

        class agent( BasePipelineConfig.env.agent ):
            dt = 0.002
            control_decimation = 5
            enable_viewer = True
            # obstacles perceived by agent. For mujoco, we create some obstacles movable via keyboard
            obstacle_debug = dict(
                num_obstacle = 0,
                manual_movement_step_size = 0.1
            )
    
     # ==========================================================================
    # Algorithm configuration
    # ==========================================================================
    class algo( BasePipelineConfig.algo ):
    
        class policy:
            class_name = "BenchmarkPIDPolicy"

        class safe_controller:
            class_name = "BaseSafeController"

            class safety_index:
                class_name = "FirstOrderCollisionSafetyIndex"
                min_distance = {
                    "environment": 0.1,
                    "self": 0.01
                }
                
                enable_self_collision = False
            
            class safe_algo:
                class_name = "SafeSetAlgorithm"
                eta_ssa = 1.0
                safety_buffer = 0.1 # positive to allow hold state
                use_slack = True
                slack_weight = 1e3
                slack_regularization_order = 2
                control_weight = [
                    1.0, 1.0, 1.0, # waist
                    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, # left arm
                    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, # right arm
                    1.0, 1.0, 1.0 # locomotion
                ]

                # class_name = "SublevelSafeSetAlgorithm"
                # lambda_sss = 1.0
                # slack_weight = 1e3
                # control_weight = [
                #     1.0, 1.0, 1.0,
                #     1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                #     1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                #     1.0, 1.0, 1.0
                # ]
                
                # class_name = "ControlBarrierFunction"
                # lambda_cbf = 1
                # slack_weight = 1e3
                # control_weight = [
                #     1.0, 1.0, 1.0,
                #     1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                #     1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,
                #     1.0, 1.0, 1.0
                # ]
                
                # class_name = "PotentialFieldMethod"
                # c_pfm = 0.1
                
                # class_name = "SlidingModeAlgorithm"
                # c_sma = 1.0



