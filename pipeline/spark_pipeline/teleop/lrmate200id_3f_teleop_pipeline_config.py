from spark_pipeline.teleop.teleop_pipeline_config import TeleopPipelineConfig

class LRMate200iD3fTeleopPipelineConfig(TeleopPipelineConfig):
    """LRMate200iD3 (3F) teleoperation config"""

    # ==========================================================================
    # Robot configuration
    # ==========================================================================
    class robot(TeleopPipelineConfig.robot):

        class cfg(TeleopPipelineConfig.robot.cfg):
            class_name = "LRMate200iD3fSingleDynamic1Config"

    # ==========================================================================
    # Environment configuration
    # ==========================================================================
    class env(TeleopPipelineConfig.env):

        class task(TeleopPipelineConfig.env.task):
            use_dual_arm = False
            goal_right_init = [0.4, 0.0, 0.3]
            ros_params = dict()

        class agent(TeleopPipelineConfig.env.agent):
            class_name = "LRMate200iD3fMujocoSingleAgent"