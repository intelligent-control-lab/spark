from spark_pipeline.autonomy.benchmark_pipeline_config import BenchmarkPipelineConfig


class FanucLRMate200iDSingleArmBenchmarkPipelineConfig(BenchmarkPipelineConfig):
    """FANUC LR Mate 200iD single-arm benchmark config"""

    # ==========================================================================
    # Robot configuration
    # ==========================================================================

    class robot(BenchmarkPipelineConfig.robot):
        class cfg(BenchmarkPipelineConfig.robot.cfg):
            class_name = "FanucLRMate200iDSingleArmDynamic1Config"

    # ==========================================================================
    # Environment configuration
    # ==========================================================================

    class env(BenchmarkPipelineConfig.env):
        class task(BenchmarkPipelineConfig.env.task):
            use_dual_arm = False

        class agent(BenchmarkPipelineConfig.env.agent):
            class_name = "FanucLRMate200iDSingleArmAgent"
