from spark_pipeline.teleop.teleop_pipeline_config import TeleopPipelineConfig

class Gen32fTeleopPipelineConfig(TeleopPipelineConfig):
    """Gen3 (2F) teleoperation config"""

    # ==========================================================================
    # Robot configuration
    # ==========================================================================
    class robot(TeleopPipelineConfig.robot):

        class cfg(TeleopPipelineConfig.robot.cfg):
            class_name = "Gen3SingleDynamic1Config"

    # ==========================================================================
    # Environment configuration
    # ==========================================================================
    class env(TeleopPipelineConfig.env):

        class task(TeleopPipelineConfig.env.task):
            use_dual_arm = False
            goal_right_init = [0.4, 0.0, 0.3]
            ros_params = dict()

        class agent(TeleopPipelineConfig.env.agent):
            class_name = "Gen3MujocoFixedBaseAgent"