from spark_pipeline.teleop.teleop_pipeline_config import TeleopPipelineConfig

class IIWA142fTeleopPipelineConfig(TeleopPipelineConfig):
    """IIWA14 (2F) teleoperation config: override robot/env/policy class names only."""

    # ==========================================================================
    # Robot configuration
    # ==========================================================================
    class robot(TeleopPipelineConfig.robot):

        class cfg(TeleopPipelineConfig.robot.cfg):
            class_name = "IIWA14SingleDynamic1Config"

    # ==========================================================================
    # Environment configuration
    # ==========================================================================
    class env(TeleopPipelineConfig.env):

        class task(TeleopPipelineConfig.env.task):
            use_dual_arm = False
            goal_right_init = [0.4, 0.0, 0.5]
            ros_params = dict(
                robot_command_topic = "/iiwa14_2f/arm_position_controller/command",
                robot_state_topic   = "/iiwa14_2f/robot_state",
                robot_teleop_topic  = "/iiwa14_2f/robot_teleop",
                obstacle_topic      = "/iiwa14_2f/human_state",
                mssa_demo_obstacle_topic = "/iiwa14_2f/obstacle_state",
                cart_traj_topic = "iiwa14_2f/cartesian_trajectory",
                agent_pos_topic = "iiwa14_2f/agent_position",
            )

        class agent(TeleopPipelineConfig.env.agent):
            class_name = "IIWA14MujocoFixedBaseAgent"