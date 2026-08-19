from spark_pipeline.autonomy.benchmark_pipeline_config import BenchmarkPipelineConfig


class KinovaGen3DualArmBenchmarkPipelineConfig(BenchmarkPipelineConfig):
    """Gen3 (2F dual-arm) benchmark config"""

    class robot(BenchmarkPipelineConfig.robot):
        class cfg(BenchmarkPipelineConfig.robot.cfg):
            class_name = "KinovaGen3DualArmDynamic1Config"

    class env(BenchmarkPipelineConfig.env):
        class task(BenchmarkPipelineConfig.env.task):
            use_dual_arm = True

        class agent(BenchmarkPipelineConfig.env.agent):
            class_name = "KinovaGen3DualArmAgent"
