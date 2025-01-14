from spark_pipeline.base.base_pipeline_config import BasePipelineConfig
from spark_safe.safe_controller import G1BasicSafeControllerConfig

class G1BenchmarkPipelineConfig(BasePipelineConfig):
    
    max_num_steps = 500
    
    class robot( BasePipelineConfig.robot ):
        
        class cfg( BasePipelineConfig.robot.cfg ):
            class_name = "G1BasicConfig"
        
        class kinematics( BasePipelineConfig.robot.kinematics ):
            class_name = "G1BasicKinematics"
    
    class env( BasePipelineConfig.env ):
    
        class task(  BasePipelineConfig.env.task ):
            class_name          = "G1BenchmarkTask"
            task_name           = "G1BenchmarkTask" # e.g., will appear as ros node name
            enable_ros          = False
            max_episode_length  = 200

        class agent( BasePipelineConfig.env.agent ):
            class_name = "G1BasicMujocoAgent"
            mujoco_model = "g1/scene_29dof.xml"
            dt = 0.01
            # obstacles perceived by agent. For mujoco, we create some obstacles movable via keyboard
            obstacle_debug = dict(
                num_obstacle = 0,
                manual_movement_step_size = 0.1
            )
    
    class algo( BasePipelineConfig.algo ):
    
        class policy( BasePipelineConfig.algo.policy ):
            class_name = "G1TeleopPIDPolicy"

        class safe_controller( BasePipelineConfig.algo.safe_controller ):
            
            # see G1BasicCollisionVelSSAConfig for full spec
            # when used in pipeline config, robot spec comes from pipeline config
            # robot spec in safe_controller config is ignored
            
            class safety_index( BasePipelineConfig.algo.safe_controller.safety_index ):
                class_name = "BasicCollisionSafetyIndex"
                min_distance = {
                    "environment": 0.05,
                    "self": 0.05
                }
                
                # todo move to robot config
                enable_self_collision = True
                env_collision_vol_ignore = [
                    "pelvis_link_1",
                    "pelvis_link_2",
                    "pelvis_link_3",
                    "waist_yaw_joint",
                    "waist_roll_joint",
                    "waist_pitch_joint"
                ]
            
            class safe_algo( BasePipelineConfig.algo.safe_controller.safe_algo ):
                class_name = "SafeSetAlgorithm"
                eta = 1.0
                safety_buffer = 0.1 # positive to allow hold state
                slack_weight = 1e3
                control_weight = [
                    1.0, 1.0, 1.0, # waist
                    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, # left arm
                    1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, # right arm
                    10.0, 10.0, 10.0 # locomotion
                ]
    
                # class_name = "SublevelSafeSetAlgorithm"
                # lambda_SSS = 1.0
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
                # lambda_pfm = 0.1
                
                # class_name = "SlidingModeAlgorithm"
                # c_2 = 1.0



