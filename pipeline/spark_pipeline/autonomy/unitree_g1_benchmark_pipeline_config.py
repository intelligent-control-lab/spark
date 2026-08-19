from spark_pipeline.autonomy.benchmark_pipeline_config import BenchmarkPipelineConfig


class UnitreeG1BenchmarkPipelineConfig(BenchmarkPipelineConfig):
    """G1 Benchmark config"""

    # ==========================================================================
    # Robot configuration
    # ==========================================================================

    class robot(BenchmarkPipelineConfig.robot):
        class cfg(BenchmarkPipelineConfig.robot.cfg):
            class_name = "UnitreeG1MobileBaseDynamic1Config"

    # ==========================================================================
    # Environment configuration
    # ==========================================================================

    class env(BenchmarkPipelineConfig.env):
        class agent(BenchmarkPipelineConfig.env.agent):
            class_name = "UnitreeG1MobileBaseMujocoAgent"
