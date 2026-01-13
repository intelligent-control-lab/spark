from spark_pipeline import BenchmarkPipeline as Pipeline
from spark_pipeline import R1LiteBenchmarkPipelineConfig as PipelineConfig
from spark_pipeline import generate_benchmark_test_case

def config_task_module(cfg: PipelineConfig, **kwargs):
    """Configure task-related settings."""
    cfg.env.task.max_episode_length = 500
    cfg.env.task.seed = 20    
    cfg.env.task.left_arm_goal_range = [(0.1, 0.4), (0.1, 0.4), (1.0, 1.0)]
    cfg.env.task.right_arm_goal_range = [(0.1, 0.4), (-0.4, -0.1), (1.0, 1.0)]
    return cfg

def config_agent_module(cfg: PipelineConfig, **kwargs):
    """Configure agent-related settings."""
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
            cfg.algo.safe_controller.safe_algo.eta_ssa = 0.1
            cfg.algo.safe_controller.safe_algo.control_weight = [
                1.0, 1.0, 1.0,  # waist
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  # left arm
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  # right arm
                1.0, 1.0, 1.0,
            ]
        
        case "rssa":
            cfg.algo.safe_controller.safe_algo.class_name = "RelaxedSafeSetAlgorithm"
            cfg.algo.safe_controller.safe_algo.eta_ssa = 0.1
            cfg.algo.safe_controller.safe_algo.slack_weight = 1e3
            cfg.algo.safe_controller.safe_algo.control_weight = [
                1.0, 1.0, 1.0,  # waist
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  # left arm
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  # right arm
                1.0, 1.0, 1.0  # locomotion
            ]
        
        case "sss":
            cfg.algo.safe_controller.safe_algo.class_name = "BasicSublevelSafeSetAlgorithm"
            cfg.algo.safe_controller.safe_algo.lambda_sss = 10.0
            cfg.algo.safe_controller.safe_algo.control_weight = [
                1.0, 1.0, 1.0,  # waist
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  # left arm
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  # right arm
                1.0, 1.0, 1.0  # locomotion
            ]
            
        case "rsss":
            cfg.algo.safe_controller.safe_algo.class_name = "RelaxedSublevelSafeSetAlgorithm"
            cfg.algo.safe_controller.safe_algo.lambda_sss = 10.0
            cfg.algo.safe_controller.safe_algo.slack_weight = 1e3
            cfg.algo.safe_controller.safe_algo.control_weight = [
                1.0, 1.0, 1.0,  # waist
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  # left arm
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  # right arm
                1.0, 1.0, 1.0  # locomotion
            ]
        
        case "cbf":
            cfg.algo.safe_controller.safe_algo.class_name = "BasicControlBarrierFunction"
            cfg.algo.safe_controller.safe_algo.lambda_cbf = 10.0
            cfg.algo.safe_controller.safe_algo.control_weight = [
                1.0, 1.0, 1.0,  # waist
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  # left arm
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  # right arm
                1.0, 1.0, 1.0  # locomotion
            ]
            
        case "rcbf":
            cfg.algo.safe_controller.safe_algo.class_name = "RelaxedControlBarrierFunction"
            cfg.algo.safe_controller.safe_algo.lambda_cbf = 10.0
            cfg.algo.safe_controller.safe_algo.slack_weight = 1e3
            cfg.algo.safe_controller.safe_algo.control_weight = [
                1.0, 1.0, 1.0,  # waist
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  # left arm
                1.0, 1.0, 1.0, 1.0, 1.0, 1.0,  # right arm
                1.0, 1.0, 1.0  # locomotion
            ]
        
        case "pfm":
            cfg.algo.safe_controller.safe_algo.class_name = "BasicPotentialFieldMethod"
            cfg.algo.safe_controller.safe_algo.c_pfm = 1.0
        
        case "sma":
            cfg.algo.safe_controller.safe_algo.class_name = "BasicSlidingModeAlgorithm"
            cfg.algo.safe_controller.safe_algo.c_sma = 1.0

    if cfg.algo.safe_controller.safe_algo.class_name != "ByPassSafeControl":
        if "Upper" in cfg.robot.cfg.class_name:
            cfg.algo.safe_controller.safe_algo.control_weight = cfg.algo.safe_controller.safe_algo.control_weight[3:]
        elif "RightArm" in cfg.robot.cfg.class_name:
            cfg.algo.safe_controller.safe_algo.control_weight = cfg.algo.safe_controller.safe_algo.control_weight[3:10]

    # --------------------- Config Safe Control Index --------------------- #
    safety_index = kwargs.get("safety_index", "si1")  # Default to 'si1' if not provided
    match safety_index:
        case "si1":
            cfg.algo.safe_controller.safety_index.class_name = "FirstOrderCollisionSafetyIndex"
        case "si1a":
            cfg.algo.safe_controller.safety_index.class_name = "FirstOrderCollisionSafetyIndexApprox"
        case "si2":
            cfg.algo.safe_controller.safety_index.class_name = "SecondOrderCollisionSafetyIndex"
            cfg.algo.safe_controller.safety_index.phi_n = 1.0
            cfg.algo.safe_controller.safety_index.phi_k = 1.0
        case "si2a":
            cfg.algo.safe_controller.safety_index.class_name = "SecondOrderCollisionSafetyIndexApprox"
            cfg.algo.safe_controller.safety_index.phi_n = 1.0
            cfg.algo.safe_controller.safety_index.phi_k = 1.0
        case 'si2nn':
            cfg.algo.safe_controller.safety_index.class_name = "SecondOrderNNCollisionSafetyIndex"
            cfg.algo.safe_controller.safety_index.phi_n = 2.0,
            cfg.algo.safe_controller.safety_index.phi_k = 1.0,
            cfg.algo.safe_controller.safety_index.phi_nn_path = "n_2_scalar.onnx"

    cfg.algo.safe_controller.safety_index.enable_self_collision = False
    cfg.algo.safe_controller.safety_index.min_distance['environment'] = 0.01

    return cfg

def config_pipeline(cfg: PipelineConfig, **kwargs):
    """Configure pipeline settings."""
    cfg = generate_benchmark_test_case(cfg, kwargs.get("test_case_name"))
    cfg.max_num_steps = 5000
    cfg.max_num_reset = -1
    cfg.enable_logger = False
    cfg.enable_safe_zone_render = False

    # Task-specific configurations using pattern matching
    if "Mobile" in cfg.env.task.task_name:
        cfg.metric_selection.dist_goal_base = True
    else:
        cfg.metric_selection.dist_goal_base = False

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
    # List of test cases to choose from
    TASK_CASE_LIST = [
        "R1LiteUpper_D1_AG_SO_v0",
        "R1LiteUpper_D2_AG_SO_v0",
    ]
    run(test_case_name = "R1LiteUpper_D2_AG_SO_v0",
        safe_algo = "rssa",
        safety_index = "si2")
