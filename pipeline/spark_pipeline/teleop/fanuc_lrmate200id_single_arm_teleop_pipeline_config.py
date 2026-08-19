from spark_pipeline.teleop.teleop_pipeline_config import TeleopPipelineConfig


class FanucLRMate200iDSingleArmTeleopPipelineConfig(TeleopPipelineConfig):
    """FANUC LR Mate 200iD single-arm teleoperation config"""

    # ==========================================================================
    # Robot configuration
    # ==========================================================================
    class robot(TeleopPipelineConfig.robot):
        class cfg(TeleopPipelineConfig.robot.cfg):
            class_name = "FanucLRMate200iDSingleArmDynamic1Config"

    # ==========================================================================
    # Environment configuration
    # ==========================================================================
    class env(TeleopPipelineConfig.env):
        class task(TeleopPipelineConfig.env.task):
            use_dual_arm = False
            base_goal_enable = False
            goal_right_init = [0.4, 0.0, 0.5]
            ros_params = dict(
                robot_command_topic="robot_command",
                robot_state_topic="robot_state",
                robot_frame_topic="robot_frames",
                robot_teleop_topic="robot_teleop",
                robot_gripper_topic="robot_gripper",
                obstacle_topic="human_state",
                object_name_topic="object_names",
                object_position_topic="object_positions",
                robot_teleop_joint_topic="robot_teleop_joint",
                debug_frames_topic="debug_frames",
            )

        class agent(TeleopPipelineConfig.env.agent):
            class_name = "FanucLRMate200iDSingleArmAgent"
            dt = 0.005
            control_decimation = 4
