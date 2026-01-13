from spark_pipeline.teleop.teleop_pipeline_config import TeleopPipelineConfig

class G1TeleopPipelineConfig(TeleopPipelineConfig):
    """G1 teleoperation config"""

    # ==========================================================================
    # Robot configuration
    # ==========================================================================
    class robot(TeleopPipelineConfig.robot):

        class cfg(TeleopPipelineConfig.robot.cfg):
            class_name = "G1FixedBaseDynamic1Config"

    # ==========================================================================
    # Environment configuration
    # ==========================================================================
    class env(TeleopPipelineConfig.env):

        class task(TeleopPipelineConfig.env.task):
            goal_left_init = [0.25, 0.25, 0.0]
            goal_right_init = [0.25, -0.25, 0.0]

        class agent(TeleopPipelineConfig.env.agent):
            class_name = "G1MujocoFixedBaseAgent"