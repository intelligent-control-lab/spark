from spark_pipeline.autonomy.benchmark_pipeline_config import BenchmarkPipelineConfig

class Gen32fBenchmarkPipelineConfig(BenchmarkPipelineConfig):
    """Gen3 (2F) Benchmark config"""

    # ==========================================================================
    # Robot configuration
    # ==========================================================================

    class robot(BenchmarkPipelineConfig.robot ):
        
        class cfg( BenchmarkPipelineConfig.robot.cfg ):
            class_name = "Gen3SingleDynamic1Config"
        
    # ==========================================================================
    # Environment configuration
    # ==========================================================================
    
    class env(BenchmarkPipelineConfig.env ):

        class task(BenchmarkPipelineConfig.env.task):
            use_dual_arm = False

        class agent(BenchmarkPipelineConfig.env.agent ):
            class_name = "Gen3MujocoFixedBaseAgent"            