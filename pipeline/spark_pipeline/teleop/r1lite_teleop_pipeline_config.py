from spark_pipeline.teleop.teleop_pipeline_config import TeleopPipelineConfig

class R1LiteTeleopPipelineConfig(TeleopPipelineConfig):
    """R1Lite teleoperation config"""

    # ==========================================================================
    # Robot configuration
    # ==========================================================================
    class robot(TeleopPipelineConfig.robot):

        class cfg(TeleopPipelineConfig.robot.cfg):
            class_name = "R1LiteUpperDynamic1Config"

    # ==========================================================================
    # Environment configuration
    # ==========================================================================
    class env(TeleopPipelineConfig.env):

        class task(TeleopPipelineConfig.env.task):
            goal_left_init = [0.4, 0.4, 1.3]
            goal_right_init = [0.4, -0.4, 1.3]

        class agent(TeleopPipelineConfig.env.agent):
            class_name = "R1LiteMujocoFixedBaseAgent"