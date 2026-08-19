from spark_pipeline.teleop.teleop_pipeline_config import TeleopPipelineConfig


class AgiBotG1RightArmTeleopPipelineConfig(TeleopPipelineConfig):
    """AgiBot G1 right-arm teleoperation config."""

    class robot(TeleopPipelineConfig.robot):
        class cfg(TeleopPipelineConfig.robot.cfg):
            class_name = "AgiBotG1RightArmDynamic1Config"

    class env(TeleopPipelineConfig.env):
        class task(TeleopPipelineConfig.env.task):
            use_dual_arm = False
            arm_goal_init_from_current_ee = True
            base_goal_enable = False
            goal_right_init = [0.5, -0.25, 0.7]
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
                env_reset_topic="env_reset",
            )

        class agent(TeleopPipelineConfig.env.agent):
            class_name = "AgiBotG1RightArmAgent"
