from spark_pipeline import BenchmarkPipeline as Pipeline
from spark_pipeline import G1BenchmarkPipelineConfig as PipelineConfig

def config_task_module(cfg: PipelineConfig, **kwargs):
    """Configure task-related settings."""
    cfg.env.task.class_name = "CorridorTask"
    return cfg

def config_agent_module(cfg: PipelineConfig, **kwargs):
    """Configure agent-related settings."""
    cfg.env.agent.class_name = "G1MujocoSportModeAgent"
    cfg.env.agent.enable_viewer = True
    cfg.env.agent.use_sim_dynamics = False
    return cfg

def config_policy_module(cfg: PipelineConfig, **kwargs):
    """Configure policy-related settings."""
    # Use the default policy configuration
    return cfg

def config_safety_module(cfg: PipelineConfig, **kwargs):
    """Configure safety-related settings."""
    # --------------------- Config Safe Control Algorithm --------------------- #
    safe_algo = kwargs.get("safe_algo", "bypass")  # Default to 'bypass' if not provided
    match safe_algo:
        case "bypass":
            cfg.algo.safe_controller.safe_algo.class_name = "ByPassSafeControl"
        
        case "ssa":
             cfg.algo.safe_controller.safe_algo.class_name = "BasicSafeSetAlgorithm"
             cfg.algo.safe_controller.safe_algo.eta_ssa = 1.0
             cfg.algo.safe_controller.safe_algo.control_weight = [
                 1.0, 1.0, 1.0,  # waist
                 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  # left arm
                 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  # right arm
                 1.0, 1.0, 1.0,
             ]

    # --------------------- Config Safe Control Index --------------------- #
    safety_index = kwargs.get("safety_index", "si2a")  # Default to 'si1' if not provided
    match safety_index:
        case "si2a":
            cfg.algo.safe_controller.safety_index.class_name = "SecondOrderCollisionSafetyIndexApprox"
            cfg.algo.safe_controller.safety_index.phi_n = 1.0
            cfg.algo.safe_controller.safety_index.phi_k = 0.5
        case 'si2nn':
            cfg.algo.safe_controller.safety_index.class_name = "SecondOrderNNCollisionSafetyIndex"
            cfg.algo.safe_controller.safety_index.phi_n = 1.0,
            cfg.algo.safe_controller.safety_index.phi_k = 1.0,
            cfg.algo.safe_controller.safety_index.phi_nn_path = "n_2_scalar.onnx"

    return cfg

def config_pipeline(cfg: PipelineConfig, **kwargs):
    """Configure pipeline settings."""
    cfg.robot.cfg.class_name = "G1SportModeDynamic2Config"
    cfg.max_num_steps = 10000
    cfg.max_num_reset = 1
    cfg.enable_logger = False
    cfg.enable_safe_zone_render = False

    cfg.metric_selection.dist_goal_base = True

    # Configure metrics to track by directly modifying the attributes
    cfg.metric_selection.dist_self = True
    cfg.metric_selection.dist_robot_to_env = True
    cfg.metric_selection.dist_goal_arm = True
    cfg.metric_selection.seed = True
    cfg.metric_selection.done = True
    
    return cfg
    
def run( **kwargs):
    """Main execution block to run the benchmark pipeline."""
    # Generate initial pipeline configuration
    cfg = PipelineConfig()
    
    # Apply configuration modules in order
    cfg = config_pipeline(cfg, **kwargs)
    cfg = config_task_module(cfg, **kwargs)
    cfg = config_agent_module(cfg, **kwargs)
    cfg = config_policy_module(cfg, **kwargs)
    cfg = config_safety_module(cfg, **kwargs)
    
    # Run the pipeline
    pipeline = Pipeline(cfg)
    pipeline.run(save_path = kwargs.get("save_path", None))
    
    return

if __name__ == "__main__":
    
    run(safe_algo = "ssa",
        safety_index = "si2a",
        save_path = "file/")
