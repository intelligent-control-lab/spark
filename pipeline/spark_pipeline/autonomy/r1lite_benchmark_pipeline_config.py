from spark_pipeline.autonomy.benchmark_pipeline_config import BenchmarkPipelineConfig

class R1LiteBenchmarkPipelineConfig(BenchmarkPipelineConfig):
    """R1Lite Benchmark config"""

    # ==========================================================================
    # Robot configuration
    # ==========================================================================

    class robot(BenchmarkPipelineConfig.robot ):
        
        class cfg( BenchmarkPipelineConfig.robot.cfg ):
            class_name = "R1LiteUpperDynamic1Config"
        
    # ==========================================================================
    # Environment configuration
    # ==========================================================================
    
    class env(BenchmarkPipelineConfig.env ):

        class agent(BenchmarkPipelineConfig.env.agent ):
            class_name = "R1LiteMujocoFixedBaseAgent"
            