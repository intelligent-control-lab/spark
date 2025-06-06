from spark_pipeline.base.base_pipeline_config import BasePipelineConfig

class G1BenchmarkPipelineConfig(BasePipelineConfig):
    
    # benchmark config
    max_num_steps = 500
    max_num_reset = -1
    enable_logger = False
    enable_plotter = False
    enable_safe_zone_render = False
    
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
            class_name = "G1MobileBaseDynamic1Config"
        
        class kinematics( BasePipelineConfig.robot.kinematics ):
            pass
    
    class env( BasePipelineConfig.env ):
    
        class task(  BasePipelineConfig.env.task ):
            class_name          = "G1BenchmarkTask"
            task_name           = "G1BenchmarkTask" # e.g., will appear as ros node name
            task_config         = {}

        class agent( BasePipelineConfig.env.agent ):
            class_name = "G1MujocoAgent"
            dt = 0.002
            control_decimation = 5
            enable_viewer = True
            # obstacles perceived by agent. For mujoco, we create some obstacles movable via keyboard
            obstacle_debug = dict(
                num_obstacle = 0,
                manual_movement_step_size = 0.1
            )
    
    class algo( BasePipelineConfig.algo ):
    
        class policy( BasePipelineConfig.algo.policy ):
            class_name = "G1BenchmarkPIDPolicy"

        class safe_controller( BasePipelineConfig.algo.safe_controller ):
            
            # see G1BasicCollisionVelSSAConfig for full spec
            # when used in pipeline config, robot spec comes from pipeline config
            # robot spec in safe_controller config is ignored
            
            class safety_index( BasePipelineConfig.algo.safe_controller.safety_index ):
                class_name = "FirstOrderCollisionSafetyIndex"
                min_distance = {
                    "environment": 0.1,
                    "self": 0.01
                }
                
                enable_self_collision = False
            
            class safe_algo( BasePipelineConfig.algo.safe_controller.safe_algo ):
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



