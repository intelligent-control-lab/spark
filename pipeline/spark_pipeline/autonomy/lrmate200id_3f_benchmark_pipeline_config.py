from spark_pipeline.autonomy.benchmark_pipeline_config import BenchmarkPipelineConfig

class LRMate200iD3fBenchmarkPipelineConfig(BenchmarkPipelineConfig):
    """LRMate200iD3 (3F) Benchmark config"""

    # ==========================================================================
    # Robot configuration
    # ==========================================================================

    class robot(BenchmarkPipelineConfig.robot ):
        
        class cfg( BenchmarkPipelineConfig.robot.cfg ):
            class_name = "LRMate200iD3fSingleDynamic1Config"
        
    # ==========================================================================
    # Environment configuration
    # ==========================================================================
    
    class env(BenchmarkPipelineConfig.env ):

        class task(BenchmarkPipelineConfig.env.task):
            use_dual_arm = False

        class agent(BenchmarkPipelineConfig.env.agent ):
            class_name = "LRMate200iD3fMujocoSingleAgent"
            