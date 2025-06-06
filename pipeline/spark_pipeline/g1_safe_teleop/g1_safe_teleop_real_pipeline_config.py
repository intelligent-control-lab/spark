from spark_pipeline.base.base_pipeline_config import BasePipelineConfig
from spark_safe.safe_controller import G1BasicSafeControllerConfig

class G1SafeTeleopRealPipelineConfig(BasePipelineConfig):
    
    max_num_steps = -1
    enable_logger = False
    enable_plotter = False
    enable_safe_zone_render = False
    
    
    class metric_selection:
        
        dof_pos = False
        dof_vel = False
        goal_pos = False
        obstacle_pos = False
        
        dist_self         = False
        dist_robot_to_env = False
        dist_goal_arm     = False
        dist_goal_base    = False
        seed    = False
        done    = False

        violation = True
        loop_time = True
        trigger_safe_controller = True
    
    class robot( BasePipelineConfig.robot ):
        
        class cfg( BasePipelineConfig.robot.cfg ):
            class_name = "G1FixedBaseConfig"
        
        class kinematics( BasePipelineConfig.robot.kinematics ):
            pass
    
    class env( BasePipelineConfig.env ):
    
        class task(  BasePipelineConfig.env.task ):
            class_name          = "G1TeleopSimTask"
            task_name           = "G1TeleopSimTask" # e.g., will appear as ros node name
            enable_ros          = True
            
            ros_params = dict(
                robot_command_topic = "/g1_29dof/arm_position_controller/command",
                robot_state_topic   = "/g1_29dof/robot_state",
                robot_teleop_topic  = "/g1_29dof/robot_teleop",
                obstacle_topic      = "/g1_29dof/human_state",
                mssa_demo_obstacle_topic = "/g1_29dof/obstacle_state",
            )
            max_episode_length  = -1

        class agent( BasePipelineConfig.env.agent ):
            class_name = "G1RealAgent"
            dt = 0.002
            control_decimation = 5
            unitree_model = "g1"
            level = "high"
            send_cmd = False
            enable_viewer = False #TODO: add visulaization with real agent
    
    class algo( BasePipelineConfig.algo ):
    
        class policy( BasePipelineConfig.algo.policy ):
            class_name = "G1TeleopPIDPolicy"

        class safe_controller( BasePipelineConfig.algo.safe_controller ):
            
            class safety_index( BasePipelineConfig.algo.safe_controller.safety_index ):
                class_name = "BasicCollisionSafetyIndex"
                min_distance = {
                    "environment": 0.1,
                    "self": 0.01
                }
                
                enable_self_collision = True
            
            class safe_algo( BasePipelineConfig.algo.safe_controller.safe_algo ):
                # class_name = "ByPassSafeControl"
                class_name = "SafeSetAlgorithm"
                eta_ssa = 1.0
                safety_buffer = 0.1 # positive to allow hold state
                slack_weight = 1e3
                control_weight = [
                    1.0, 1.0, 1.0, # waist
                    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, # left arm
                    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, # right arm
                    10.0, 10.0, 10.0 # locomotion
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
    




