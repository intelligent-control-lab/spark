from spark_pipeline.autonomy.benchmark_pipeline_config import BenchmarkPipelineConfig


class FanucLRMate200iDDualArmBenchmarkPipelineConfig(BenchmarkPipelineConfig):
    """FANUC LR Mate 200iD dual-arm benchmark config"""

    class robot(BenchmarkPipelineConfig.robot):
        class cfg(BenchmarkPipelineConfig.robot.cfg):
            class_name = "FanucLRMate200iDDualArmDynamic1Config"

    class env(BenchmarkPipelineConfig.env):
        class task(BenchmarkPipelineConfig.env.task):
            use_dual_arm = True

        class agent(BenchmarkPipelineConfig.env.agent):
            class_name = "FanucLRMate200iDDualArmAgent"
