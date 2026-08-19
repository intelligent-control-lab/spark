from spark_pipeline.autonomy.benchmark_pipeline_config import BenchmarkPipelineConfig


class AgiBotG1BenchmarkPipelineConfig(BenchmarkPipelineConfig):
    """AgiBot G1 benchmark config"""

    class robot(BenchmarkPipelineConfig.robot):
        class cfg(BenchmarkPipelineConfig.robot.cfg):
            class_name = "AgiBotG1FixedBaseDynamic1Config"

    class env(BenchmarkPipelineConfig.env):
        class agent(BenchmarkPipelineConfig.env.agent):
            class_name = "AgiBotG1FixedBaseAgent"
