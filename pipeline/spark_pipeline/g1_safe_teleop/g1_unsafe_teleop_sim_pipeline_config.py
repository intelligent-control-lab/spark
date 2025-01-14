from spark_pipeline.base.base_pipeline_config import BasePipelineConfig
from spark_safe.safe_controller import G1BasicSafeControllerConfig

class G1UnsafeTeleopSimPipelineConfig(BasePipelineConfig):
    
    max_num_steps = -1
    
    class robot( BasePipelineConfig.robot ):
        
        class cfg( BasePipelineConfig.robot.cfg ):
            class_name = "G1BasicConfig"
        
        class kinematics( BasePipelineConfig.robot.kinematics ):
            class_name = "G1BasicKinematics"
    
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
            )
            max_episode_length  = -1

        class agent( BasePipelineConfig.env.agent ):
            class_name = "G1BasicMujocoAgent"
            mujoco_model = "g1/scene_29dof.xml"
            dt = 0.01
            # obstacles perceived by agent. For mujoco, we create some obstacles movable via keyboard
            obstacle_debug = dict(
                num_obstacle = 2,
                manual_movement_step_size = 0.1
            )
    
    class algo( BasePipelineConfig.algo ):
    
        class policy( BasePipelineConfig.algo.policy ):
            class_name = "G1TeleopPIDPolicy"

        class safe_controller( BasePipelineConfig.algo.safe_controller ):
            
            class safety_index( BasePipelineConfig.algo.safe_controller.safety_index ):
                class_name = "BasicCollisionSafetyIndex"
                min_distance = {
                    "environment": 0.01,
                    "self": 0.01
                }
                
                # todo move to robot config
                enable_self_collision = True
                env_collision_vol_ignore = [
                    "pelvis_link_1",
                    "pelvis_link_2",
                    "pelvis_link_3",
                    "waist_yaw_joint",
                    "waist_roll_joint",
                    "waist_pitch_joint",
                    "torso_link_1",
                    "torso_link_2",
                    "torso_link_3"
                ]
            
            class safe_algo( BasePipelineConfig.algo.safe_controller.safe_algo ):
                class_name = "ByPassSafeControl"
    
        