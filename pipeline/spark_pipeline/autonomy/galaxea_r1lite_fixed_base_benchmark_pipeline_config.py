from spark_pipeline.autonomy.benchmark_pipeline_config import BenchmarkPipelineConfig


class GalaxeaR1LiteFixedBaseBenchmarkPipelineConfig(BenchmarkPipelineConfig):
    """Galaxea R1 Lite fixed-base benchmark config."""

    # ==========================================================================
    # Robot configuration
    # ==========================================================================

    class robot(BenchmarkPipelineConfig.robot):
        class cfg(BenchmarkPipelineConfig.robot.cfg):
            class_name = "GalaxeaR1LiteFixedBaseDynamic1Config"

    # ==========================================================================
    # Environment configuration
    # ==========================================================================

    class env(BenchmarkPipelineConfig.env):
        class agent(BenchmarkPipelineConfig.env.agent):
            class_name = "GalaxeaR1LiteFixedBaseAgent"
